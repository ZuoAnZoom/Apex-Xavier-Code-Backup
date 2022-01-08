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
from tools import IPM_Image_Computation

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


def get_M_and_dis_pix_y():
    """
    计算一些后处理需要的参数，放在外面，避免每张图片都计算一次
    暂时没有输出，但后续相机的内参这些会是输入进来的

    :return: 转成IPM图的转换矩阵，车前距离Zm - Zn，以及对应的图像像素坐标
    """
    mat_real2cam_t = np.array([[0], [-1], [0]])
    mat_cam_k = np.array([[1312.961, 0.0,      640.0],
                          [0.0,      -1312.961, 360.0],
                          [0.0,      0.0,      1.0]])
    distCoeffs = np.array([0.0, 0.0, 0.0, 0.0])
    new_camera_matrix_2, roi_2 = cv2.getOptimalNewCameraMatrix(mat_cam_k, distCoeffs, (720, 480), 0, (512, 256))

    X_min, X_max = -1.0, 1.0
    Z_min, Z_max = 5.0, 20.0
    src_image = cv2.imread('/media/st/8CEA-E915/camera_data_200928/frame0050.jpg')
    src_image = cv2.resize(src_image, (512, 256))
    src_image_shape = src_image.shape
    A = IPM_Image_Computation.IPM_mat_computer(0, 0, 0, mat_real2cam_t, new_camera_matrix_2,
                                               1.0, 1.0, X_min, X_max, Z_min, Z_max, src_image_shape)
    M = A.IPM_mat()

    # 这个XZ2uv是到正视图，但是你计算距离是在鸟瞰图，所以还要计算点坐标的投影变换
    dis_real_zong = np.array([5, 6, 7, 8, 9, 10, 11])
    dis_pix_y = []
    for Z in dis_real_zong:
        u_src, v_src = A.XZ2uv(0, Z)
        mat_tmp = np.matmul(M, np.array([u_src, v_src, 1]))
        # 归一化
        point_IPM = [mat_tmp[0] / mat_tmp[2], mat_tmp[1] / mat_tmp[2]]
        dis_pix_y.append(point_IPM[1])

    return M, dis_real_zong, dis_pix_y


def test_lanenet(src_dir, weights_path):
    """

    :param src_dir:
    :param weights_path:
    :return:
    """
    assert ops.exists(src_dir), '{:s} not exist'.format(src_dir)

    input_tensor = tf.placeholder(dtype=tf.float32, shape=[1, 256, 512, 3], name='input_tensor')

    net = lanenet.LaneNet(phase='test')
    binary_seg_ret, instance_seg_ret = net.inference(input_tensor=input_tensor, name='LaneNet')

    M, dis_real_zong, dis_pix_y= get_M_and_dis_pix_y()
    postprocessor = my_postprocess.PostProcessor(M, dis_real_zong, dis_pix_y)

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

        for i in range(100, 600):
            image_path = ops.join(src_dir,'frame0'+str(i)+'.jpg')
            # image_path = ops.join(src_dir, str(i) + '.jpg')
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            image_vis = image
            image = cv2.resize(image, (512, 256), interpolation=cv2.INTER_LINEAR)
            image_vis = cv2.resize(image_vis, (512, 256), interpolation=cv2.INTER_LINEAR)
            image = image / 127.5 - 1.0

            time1 = time.time()
            binary_seg_image, instance_seg_image = sess.run(
                [binary_seg_ret, instance_seg_ret],
                feed_dict={input_tensor: [image]}
            )
            time2 = time.time()
            ret = postprocessor.postprocess(binary_seg_image[0], image_vis, 20)
            time3 = time.time()
            print('1   ', time2-time1)

if __name__ == '__main__':
    """
    test code
    """
    # init args
    args = init_args()
    args.image_path = '/media/st/8CEA-E915/camera_data_200928'
    args.weights_path = '/media/st/8CEA-E915/Weights/tusimple_simulation/tusimple_train.ckpt'

    test_lanenet(args.image_path, args.weights_path)