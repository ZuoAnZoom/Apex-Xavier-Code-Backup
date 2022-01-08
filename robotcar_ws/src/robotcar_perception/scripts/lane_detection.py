#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/h/catkin_ws/src/robotcar_perception/scripts')
sys.path.append('/home/h/catkin_ws/src/robotcar_perception')

import numpy as np
import cv2
import rospy
import tensorflow as tf
import time
# from tools.getdata import getdata
from cv_bridge import CvBridge
from robotcar_map.msg import lanes
from lanenet_model import lanenet
from lanenet_model import my_postprocess
from sensor_msgs.msg import Image
from tools import IPM_Image_Computation, Undistortion

from local_utils.config_utils import parse_config_utils
CFG = parse_config_utils.lanenet_cfg


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
    # undistort_camera_matrix = np.array(CFG.ACTUAL.undis_cameraMatrix_K_4)

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


if __name__ == '__main__':
    rospy.init_node('robotcar_perception')
    pub = rospy.Publisher("/perception/lanes", lanes, queue_size=10)

    # weights_path = '/home/nvidia/catkin_ws/src/robotcar_perception/scripts/Weights/tusimple_simulation/tusimple_train.ckpt'
    weights_path = '/home/nvidia/workspace/robotcar_ws/src/robotcar_perception/scripts/Weights/CU_xy_gc_ml_900/tusimple_train.ckpt'

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

        while 1:
            # time_1 = time.time()
            transform_result = lanes()
            msg = rospy.wait_for_message("/miivii_gmsl_ros_A/camera1", Image, timeout=None)
            transform_result.stamp = msg.header.stamp

            bridge = CvBridge()
            data = bridge.imgmsg_to_cv2(msg, 'bgr8')

            image = np.reshape(data, (720, 1280, 3))
            # image = cv2.imread('/home/st/Tusimple测试/图片/2.jpg', cv2.IMREAD_COLOR)
            # 因为上面后处理的M矩阵是去畸变后的图像得到的，这里的图像同样去畸变
            image, new_camera_matrix = Undistortion.undistortion_4(image)
            # 传入的图像已经去畸变

            image_vis = image
            image = cv2.resize(image, (CFG.ACTUAL.DST_IMG_W, CFG.ACTUAL.DST_IMG_H), interpolation=cv2.INTER_LINEAR)
            image_vis = cv2.resize(image_vis, (CFG.ACTUAL.DST_IMG_W, CFG.ACTUAL.DST_IMG_H), interpolation=cv2.INTER_LINEAR)
            image = image / 127.5 - 1.0

            # time_2 = time.time()
            binary_seg_image, instance_seg_image = sess.run(
                [binary_seg_ret, instance_seg_ret],
                feed_dict={input_tensor: [image]})

            # time_3 = time.time()
            # mask_image = postprocessor.cluster(
            #     binary_seg_result=binary_seg_image[0],
            #     instance_seg_result=instance_seg_image[0]
            # )
            #
            # if not np.any(mask_image):
            #     transform_result.num = 0
            #     pub.publish(transform_result)
            #     continue
            # time_4 = time.time()
            ret = postprocessor.postprocess(binary_seg_image[0], image_vis, 30)

            result = ret['positions']
            transform_result.num = len(result)
            for i in range(min(len(result), 4)):
                for j in range(len(result[i])):
                    exec('transform_result.y_{}.append(-1 * result[{}][{}][1])'.format(i, i, j))
                    exec('transform_result.x_{}.append(result[{}][{}][0])'.format(i, i, j))
            pub.publish(transform_result)
            print(result)

            # BEV_image = postprocess_result['BEV_image']
            # cv2.imshow("BEV_image", BEV_image)
            # cv2.waitKey(10)
            # time_5 = time.time()
            # print(time_2 - time_1)
            # print(time_3 - time_2)
            # print(time_4 - time_3)
            # print(time_5 - time_4)
            print("\n")

            # print(rospy.Time.now() - transform_result.stamp)

    # sess.close()
