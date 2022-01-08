#pragma once

#ifndef CAMERA_H
#define CAMERA_H

#include "myslam/CommonInclude.h"

namespace myslam
{
    class Camera
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        enum eType
        {
            MONOCULAR = 1, // 单目
            RGBD = 2,      // RGBD
            STEREO = 3     // 双目
        };
        static eType Type_; // 传感器类型

        static int width_, height_; // 图像的宽和高

        static float fx_, fy_, cx_, cy_; // 相机内参
        static Mat33 K_;                 // 相机内参数矩阵
        static cv::Mat cvK_;
        /*
        |fx_  0  cx_|
        |0   fx_ cy_| = K
        |0   0   1  |
        */

        static float k1_, k2_, k3_, p1_, p2_;        // 畸变系数
        static cv::Mat D_;                           // 畸变系数矩阵
        static float min_X_, max_X_, min_Y_, max_Y_; // 图像去畸变后四个角点的坐标

        static cv::Mat K_l_, K_r_; // 双目相机各自的内参
        static cv::Mat D_l_, D_r_; // 双目相机各自的畸变系数矩阵
        static float base_;        // 双目基线长度（米）
        static float base_fx_;     // 双目基线长度（米）*fx_

        static int fps_; // 帧率

        static float depth_factor_; // 深度比例（仅RGBD相机）
    };
} // namespace myslam
#endif // CAMERA_H
