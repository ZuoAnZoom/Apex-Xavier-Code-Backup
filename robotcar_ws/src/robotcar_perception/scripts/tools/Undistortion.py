import cv2
import numpy as np
from local_utils.config_utils import parse_config_utils

CFG = parse_config_utils.lanenet_cfg


# 对二号相机去畸变
def undistortion_2(src_image_2):
    """

    :param src_image_2:
    :return:
    """
    # 二号相机的内参矩阵
    cameraMatrix_2 = np.array(CFG.ACTUAL.cameraMatrix_K_2)

    # 二号相机的畸变参数
    distCoeffs_2 = np.array(CFG.ACTUAL.distCoeffs_2)

    # src_image_2 = cv2.imread('/media/st/8CEA-E915/xy_gc/right/380.png', cv2.IMREAD_UNCHANGED)
    src_h = CFG.ACTUAL.SRC_IMG_H
    src_w = CFG.ACTUAL.SRC_IMG_W

    undistort_camera_matrix_2, roi_2 = cv2.getOptimalNewCameraMatrix(cameraMatrix_2, distCoeffs_2,
                                                                     (src_w, src_h), 0, (src_w, src_h))

    # map1_2, map2_2 = cv2.initUndistortRectifyMap(cameraMatrix_2, distCoeffs_2, None,
    #                                              undistort_camera_matrix_2, (w, h), cv2.CV_32FC1)
    # dst_2 = cv2.remap(src_image_2, map1_2, map2_2, cv2.INTER_LINEAR)

    # 多张图不建议用，多次求转换矩阵浪费时间
    # 最后如果用原相机矩阵内参，则会得到有黑边的校正图像，使用求得的新内参，则会裁剪掉黑边
    dst_2 = cv2.undistort(src_image_2, cameraMatrix_2, distCoeffs_2, None, undistort_camera_matrix_2)

    # cv2.imshow('undistortion2', dst_2)
    # cv2.waitKey()

    return dst_2, undistort_camera_matrix_2


def undistortion_2_without_img():
    """
    只返回去畸变后的相机内参，不需要输入图片
    :return:
    """
    cameraMatrix_2 = np.array(CFG.ACTUAL.cameraMatrix_K_2)

    # 四号相机的畸变参数
    distCoeffs_2 = np.array(CFG.ACTUAL.distCoeffs_2)

    src_h = CFG.ACTUAL.SRC_IMG_H
    src_w = CFG.ACTUAL.SRC_IMG_W

    undistort_camera_matrix_2, roi_2 = cv2.getOptimalNewCameraMatrix(cameraMatrix_2, distCoeffs_2,
                                                               (src_w, src_h), 0, (src_w, src_h))

    return undistort_camera_matrix_2






# 对四号相机去畸变
def undistortion_4(src_image_4):
    """

    :param src_image_4:
    :return:
    """
    cameraMatrix_4 = np.array(CFG.ACTUAL.cameraMatrix_K_4)

    # 四号相机的畸变参数
    distCoeffs_4 = np.array(CFG.ACTUAL.distCoeffs_4)

    # src_image_4 = cv2.imread('/home/st/下载/4.png', cv2.IMREAD_UNCHANGED)
    src_h = CFG.ACTUAL.SRC_IMG_H
    src_w = CFG.ACTUAL.SRC_IMG_W
    # 去畸变同时缩放会有问题
    # dst_h = CFG.ACTUAL.DST_IMG_H
    # dst_w = CFG.ACTUAL.DST_IMG_W

    undistort_camera_matrix_4, roi_4 = cv2.getOptimalNewCameraMatrix(cameraMatrix_4, distCoeffs_4,
                                                               (src_w, src_h), 0, (src_w, src_h))

    dst_4 = cv2.undistort(src_image_4, cameraMatrix_4, distCoeffs_4, None, undistort_camera_matrix_4).astype(np.uint8)

    # cv2.imshow('undistortion4', dst_4)
    # cv2.waitKey()

    return dst_4, undistort_camera_matrix_4


def undistortion_4_without_img():
    """
    只返回去畸变后的相机内参，不需要输入图片
    :return:
    """
    cameraMatrix_4 = np.array(CFG.ACTUAL.cameraMatrix_K_4)

    # 四号相机的畸变参数
    distCoeffs_4 = np.array(CFG.ACTUAL.distCoeffs_4)

    src_h = CFG.ACTUAL.SRC_IMG_H
    src_w = CFG.ACTUAL.SRC_IMG_W

    undistort_camera_matrix_4, roi_4 = cv2.getOptimalNewCameraMatrix(cameraMatrix_4, distCoeffs_4,
                                                               (src_w, src_h), 0, (src_w, src_h))

    return undistort_camera_matrix_4


if __name__ == '__main__':
    src_image = cv2.imread('/media/st/8CEA-E915/xy_gc/left/380.png')
    dst_4, undistort_camera_matrix_4 = undistortion_4(src_image)
    undistort_matrix = undistortion_4_without_img()
    print("")