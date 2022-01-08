import argparse
import os.path as ops
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf

from lanenet_model import lanenet
from lanenet_model import my_postprocess
from local_utils.config_utils import parse_config_utils
from local_utils.log_util import init_logger
from tools import IPM_Image_Computation, Undistortion

CFG = parse_config_utils.lanenet_cfg
LOG = init_logger.get_logger(log_file_name_prefix='lanenet_test')


def init_args():
    """

    :return:
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--image_path', type=str, help='The image path or the src image save dir')
    parser.add_argument('--weights_path', type=str, help='The model weights path')

    return parser.parse_args()


def args_str2bool(arg_value):
    """

    :param arg_value:
    :return:
    """
    if arg_value.lower() in ('yes', 'true', 't', 'y', '1'):
        return True

    elif arg_value.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Unsupported value encountered.')


def minmax_scale(input_arr):
    """

    :param input_arr:
    :return:
    """
    min_val = np.min(input_arr)
    max_val = np.max(input_arr)

    output_arr = (input_arr - min_val) * 255.0 / (max_val - min_val)

    return output_arr


def get_M_and_Minv():
    """
    计算一些后处理需要的参数，放在外面，避免每张图片都计算一次
    暂时没有输入，但后续相机的内参这些会是输入进来的

    :return: 转成IPM图的转换矩阵，车前距离Zm ~ Zn，以及对应的图像像素坐标
    """
    # 左目，4号相机，图像去畸变后的相机内参
    undistort_camera_matrix = Undistortion.undistortion_4_without_img()

    # # 右目，2号相机，图像去畸变后的相机内参
    # undistort_camera_matrix = Undistortion.undistortion_2_without_img()

    # 实际相机的外参
    mat_real2cam_t = np.array(CFG.ACTUAL.cameraMatrix_T)
    # 去畸变后不再有畸变参数
    distCoeffs_undistort = np.array([0.0, 0.0, 0.0, 0.0])
    src_h = CFG.ACTUAL.SRC_IMG_H
    src_w = CFG.ACTUAL.SRC_IMG_W
    dst_h = CFG.ACTUAL.DST_IMG_H
    dst_w = CFG.ACTUAL.DST_IMG_W
    # 缩放后的相机内参
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(undistort_camera_matrix, distCoeffs_undistort,
                                                               (src_w, src_h), 0, (dst_w, dst_h))

    # # 仿真环境，相机外参、内参
    # mat_real2cam_t = np.array(CFG.ACTUAL.cameraMatrix_T_emulation)
    # new_camera_matrix = np.array(CFG.ACTUAL.cameraMatrix_K_emulation)

    new_camera_matrix[1, 1] = -new_camera_matrix[1, 1]

    # 实测数据
    X_min, X_max = CFG.ACTUAL.X_min, CFG.ACTUAL.X_max
    Z_min, Z_max = CFG.ACTUAL.Z_min, CFG.ACTUAL.Z_max

    A = IPM_Image_Computation.IPM_mat_computer(
        pitch=CFG.ACTUAL.cameraMatrix_R[0], yaw=CFG.ACTUAL.cameraMatrix_R[1],
        roll=CFG.ACTUAL.cameraMatrix_R[2], mat_real2cam_t=CFG.ACTUAL.cameraMatrix_T,
        mat_cam_k=new_camera_matrix, alpha_u=CFG.ACTUAL.alpha_u, alpha_v=CFG.ACTUAL.alpha_v,
        X_min=X_min, X_max=X_max, Z_min=Z_min, Z_max=Z_max, src_img_shape=(dst_h, dst_w))
    # M是去畸变，并进行缩放后的图像计算得到的
    M, Minv = A.IPM_mat()

    # # 这个XZ2uv是到正视图，但是你计算距离是在鸟瞰图，所以还要计算点坐标的投影变换
    # # 接下来不再采用计算恒定纵向距离点位置的方法
    # dis_real_zong = np.array([5, 6, 7, 8, 9, 10, 11])
    # dis_pix_y = []
    # for Z in dis_real_zong:
    #     u_src, v_src = A.XZ2uv(0, Z)
    #     mat_tmp = np.matmul(M, np.array([u_src, v_src, 1]))
    #     # 归一化
    #     point_IPM = [mat_tmp[0] / mat_tmp[2], mat_tmp[1] / mat_tmp[2]]
    #     dis_pix_y.append(point_IPM[1])

    return M, Minv, new_camera_matrix


def test_lanenet(src_dir, weights_path):
    """

    :param src_dir:
    :param weights_path:
    :return:
    """
    assert ops.exists(src_dir), '{:s} not exist'.format(src_dir)

    input_tensor = tf.placeholder(dtype=tf.float32, shape=[1, CFG.ACTUAL.DST_IMG_H, CFG.ACTUAL.DST_IMG_W, 3], name='input_tensor')

    net = lanenet.LaneNet(phase='test')
    binary_seg_ret, instance_seg_ret = net.inference(input_tensor=input_tensor, name='LaneNet')

    # 图像去畸变，缩放后的内参、投影变换矩阵
    M, Minv, new_camera_matrix = get_M_and_Minv()

    # 相机高度，新成像中心，新焦距，单目计算距离要用
    H = -np.array(CFG.ACTUAL.cameraMatrix_T)[1]
    new_center = (new_camera_matrix[0, 2], new_camera_matrix[1, 2])
    new_f_x = new_camera_matrix[0, 0]
    new_f_y = -new_camera_matrix[1, 1]
    postprocessor = my_postprocess.PostProcessor(M, Minv, H, new_center, new_f_x, new_f_y)

    # Set sess configuration
    sess_config = tf.ConfigProto()
    sess_config.gpu_options.per_process_gpu_memory_fraction = CFG.GPU.GPU_MEMORY_FRACTION
    sess_config.gpu_options.allow_growth = CFG.GPU.TF_ALLOW_GROWTH
    sess_config.gpu_options.allocator_type = 'BFC'

    sess = tf.Session(config=sess_config)

    # define saver
    saver = tf.train.Saver()

    with sess.as_default():
        saver.restore(sess=sess, save_path=weights_path)
        avg_time_cost = []

        # binary_dir = '/home/st/图片/与VO的实验/binary_left'
        for i in range(1 , 10000):
            # image_path = ops.join(src_dir,'frame0'+str(i)+'.jpg')
            image_path = ops.join(src_dir, str(i) + '.png')
            # print(image_path)
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            # 因为上面后处理的M矩阵是去畸变后的图像得到的，这里的图像同样去畸变
            image, new_camera_matrix = Undistortion.undistortion_4(image)
            image_vis = image
            image = cv2.resize(image, (CFG.ACTUAL.DST_IMG_W, CFG.ACTUAL.DST_IMG_H), interpolation=cv2.INTER_LINEAR)
            image_vis = cv2.resize(image_vis, (CFG.ACTUAL.DST_IMG_W, CFG.ACTUAL.DST_IMG_H), interpolation=cv2.INTER_LINEAR)
            image = image / 127.5 - 1.0

            time1 = time.time()
            binary_seg_image, instance_seg_image = sess.run(
                [binary_seg_ret, instance_seg_ret],
                feed_dict={input_tensor: [image]}
            )

            # binary_seg_result = np.array(binary_seg_image[0] * 255, dtype=np.uint8)
            # binary_output_path = ops.join(binary_dir, str(i) + '.png')
            # cv2.imwrite(binary_output_path, binary_seg_result)
            # cv2.imshow('binary', binary_seg_result)
            # cv2.waitKey(10)

            time2 = time.time()
            ret = postprocessor.postprocess(binary_seg_image[0], image_vis, CFG.POSTPROCESS.IPM_DIVIDED_NUMS)
            time3 = time.time()
            print('1   ', time3 - time1)


if __name__ == '__main__':
    """
    test code
    """
    # init args
    args = init_args()
    args.image_path = '/media/st/8CEA-E915/xy_gc/right'
    args.weights_path = '/media/st/8CEA-E915/Weights/CU_xy_gc_ml_900/tusimple_train.ckpt'

    test_lanenet(args.image_path, args.weights_path)