"""

"""

import numpy as np
import math as m
import cv2
import loguru as logger
from tools import Undistortion
from local_utils.config_utils import parse_config_utils

CFG = parse_config_utils.lanenet_cfg


class IPM_mat_computer(object):
    """

    """
    def __init__(self, pitch, yaw, roll, mat_real2cam_t, mat_cam_k, alpha_u, alpha_v,
                 X_min, X_max, Z_min, Z_max, src_img_shape):
        """

        :param pitch:
        :param yaw:
        :param roll:
        :param mat_real2cam_t  世界坐标系到相机坐标系的平移量，注意正负号
        :param mat_cam_k: 相机内参矩阵，从相机坐标系到图像坐标系，再到像素坐标，注意f_y的负号
        :param alpha_u: 相机水平视场角的一半，单位弧度
        :param alpha_v: 相机垂直视场角的一半，单位弧度
        :param X_min:
        :param X_max:
        :param Z_min:
        :param Z_max: 想要逆变换的路面范围，或者说用来计算src中四个点的路面范围
        :param src_img_shape: 原始图像的大小（H，W）
        """
        self.p = pitch
        self.y = yaw
        self.r = roll
        self.mat_real2cam_t = mat_real2cam_t
        self.mat_cam_k = mat_cam_k
        self.alpha_u = alpha_u
        self.alpha_v = alpha_v
        self.X_min = X_min
        self.X_max = X_max
        self.Z_min = Z_min
        self.Z_max = Z_max
        self.src_img_shape = src_img_shape

    def XZ2uv(self, X, Z):
        """
        从路平面坐标（X，Z）（单位m）转到图像像素坐标，即正投影变换过程
        :param X:
        :param Z:
        :return:
        """
        # 世界坐标系转相机坐标系的矩阵
        mat_pitch = np.array([[1, 0,              0],
                             [0, m.cos(self.p), -m.sin(self.p)],
                             [0, m.sin(self.p),  m.cos(self.p)]])
        mat_yaw = np.array([[ m.cos(self.y), 0, m.sin(self.y)],
                            [ 0,             1, 0],
                            [-m.sin(self.y), 0, m.cos(self.y)]])
        mat_roll = np.array([[m.cos(self.r), -m.sin(self.r), 0],
                             [m.sin(self.r),  m.cos(self.r), 0],
                             [0,              0,             1]])
        mat_real2cam_r = np.matmul(mat_yaw, mat_pitch)
        mat_real2cam = np.hstack((mat_real2cam_r, self.mat_real2cam_t))

        # 世界坐标系转图像像素坐标系，别忘了还有个因子 /Z_c，单目中是不能确定的
        mat_real2img_pix = np.matmul(self.mat_cam_k, mat_real2cam)

        # 同样是世界坐标系转图像坐标系
        # 我们不能从二维图像坐标反推三维坐标
        # 所以令Y= 0， 降维，在多高俯视路面没有意义
        mat_real2img_pix = np.delete(mat_real2img_pix, 1, axis=1)

        # 由于齐次式，可以将Z_c消除
        M_11, M_12, M_13 = mat_real2img_pix[0, 0], mat_real2img_pix[0, 1], mat_real2img_pix[0, 2]
        M_21, M_22, M_23 = mat_real2img_pix[1, 0], mat_real2img_pix[1, 1], mat_real2img_pix[1, 2]
        M_31, M_32, M_33 = mat_real2img_pix[2, 0], mat_real2img_pix[2, 1], mat_real2img_pix[2, 2]

        u = (M_11 * X + M_12 * Z + M_13) / (M_31 * X + M_32 * Z + M_33)
        v = (M_21 * X + M_22 * Z + M_23) / (M_31 * X + M_32 * Z + M_33)

        return u, v

    def vp(self, W, H):
        """
        计算消失点的像素坐标，我们进行逆变换测路面范围，投射到图像范围，不能超过消失点
        :param W: 图像横轴像素数量  #1280
        :param H: 图像纵轴像素数量  #720
        :return:
        """
        x = (1.0 - (m.tan(self.y) / m.tan(self.alpha_u))) * W / 2.0
        y = (1.0 - (m.tan(self.p) / m.tan(self.alpha_v))) * H / 2.0

        # 不能超出图像范围
        if x <= 1 :
            x = 1
        elif x >= W:
            x = W
        if y <= 1:
            y = 1
        elif y >= H:
            y = H

        # 没有选择四舍五入，因为变换范围可以小一点，但不能超出
        return int(x), int(y)

    def src_points(self):
        """
        计划逆变换的路面范围，转到原图上的四个点
        :param vp: 消失点坐标
        :return:
        """
        W = self.src_img_shape[1]
        H = self.src_img_shape[0]
        vp = self.vp(W, H)
        u_1, v_1 = self.XZ2uv(self.X_min, self.Z_min)
        if u_1 <= 0 or v_1 >= H or v_1 <= vp[1]:
            raise ValueError("The first point of road plane is out of range, please check it!")
        else:
            point1 = [u_1, v_1]

        u_2, v_2 = self.XZ2uv(self.X_min, self.Z_max)
        if u_2 <= 0 or v_2 >= H or v_2 <= vp[1]:
            raise ValueError("The second point of road plane is out of range, please check it!")
        else:
            point2 = [u_2, v_2]

        u_3, v_3 = self.XZ2uv(self.X_max, self.Z_max)
        if u_3 >=W or v_3 >= H or v_3 <= vp[1]:
            raise ValueError("The third point of road plane is out of range, please check it!")
        else:
            point3 = [u_3, v_3]

        u_4, v_4 = self.XZ2uv(self.X_max, self.Z_min)
        if u_4 >=W or v_4 >= H or v_4 <= vp[1]:
            raise ValueError("The forth point of road plane is out of range, please check it!")
        else:
            point4 = [u_4, v_4]

        return point1, point2, point3, point4

    def IPM_mat(self):
        """

        :param src_image:
        :return:
        """
        point1, point2, point3, point4 = self.src_points()
        src = np.float32([point1, point2, point3, point4])
        # 由于距离设置得离相机比较近，目标对应点也设置离图像中心近一点
        # 就不容易计算超出范围
        # 实测数据
        dst = np.array(CFG.ACTUAL.dst_pixels, dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        return M, Minv


if __name__ == '__main__':
    src_image = cv2.imread('/media/st/8CEA-E915/xy_gc/left/380.png')
    # src_image = cv2.imread('/home/st/图片/jm/5_79.png')
    # 去畸变后的新内参
    image_undistort, undistort_camera_matrix = Undistortion.undistortion_4(src_image)

    undistort_camera_matrix[1, 1] = -undistort_camera_matrix[1, 1]

    # 既然图像是去畸变的，“新相机”就不该有畸变参数
    distCoeffs_undistort = np.array([0.0, 0.0, 0.0, 0.0])
    src_h = CFG.ACTUAL.SRC_IMG_H
    src_w = CFG.ACTUAL.SRC_IMG_W
    dst_h = CFG.ACTUAL.DST_IMG_H
    dst_w = CFG.ACTUAL.DST_IMG_W
    # 缩放后的新内参
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(undistort_camera_matrix, distCoeffs_undistort,
                                                               (src_w, src_h), 0, (dst_w, dst_h))

    # 实测数据
    X_min, X_max = CFG.ACTUAL.X_min, CFG.ACTUAL.X_max
    Z_min, Z_max = CFG.ACTUAL.Z_min, CFG.ACTUAL.Z_max

    image_undistort = cv2.resize(image_undistort, (dst_w, dst_h))
    image_undistort_shape = image_undistort.shape
    A = IPM_mat_computer(pitch=CFG.ACTUAL.cameraMatrix_R[0], yaw=CFG.ACTUAL.cameraMatrix_R[1],
                         roll=CFG.ACTUAL.cameraMatrix_R[2], mat_real2cam_t=CFG.ACTUAL.cameraMatrix_T,
                         mat_cam_k=new_camera_matrix, alpha_u=CFG.ACTUAL.alpha_u, alpha_v=CFG.ACTUAL.alpha_v,
                         X_min=X_min, X_max=X_max, Z_min=Z_min, Z_max=Z_max, src_img_shape=image_undistort_shape)
    M, Minv = A.IPM_mat()
    BEV_img = cv2.warpPerspective(image_undistort, M, image_undistort.shape[1::-1], flags=cv2.INTER_LINEAR)

    # img1 = cv2.resize(src_image, (640, 360))
    # img2 = cv2.resize(BEV_img, (640, 360))
    hmerge = np.hstack((image_undistort, BEV_img))
    cv2.imshow('img', hmerge)
    cv2.waitKey(0)












