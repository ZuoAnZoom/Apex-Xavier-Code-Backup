#pragma once

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "myslam/CommonInclude.h"
#include "myslam/Camera.h"

namespace myslam
{
    class Transform
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 坐标转换：世界、相机、像素
        static Vec3 world2camera(const Vec3 &p_w, const SE3 &T_w_c);
        static Vec3 world2camera(const Point3f &p_w, const SE3 &T_w_c);
        static Mat world2camera(const Mat &p_w, const Mat &T_w_c);

        static Vec3 camera2world(const Vec3 &p_c, const SE3 &T_w_c);
        static Vec3 camera2world(const Point3f &p_c, const SE3 &T_w_c);

        static Vec2 camera2pixel(const Mat33 &K, const Vec3 &p_c);
        static Vec2 camera2pixel(const Mat33 &K, const Point3f &p_c);
        static Point2f camera2pixel(const Mat &K, const Mat &p_c);

        static Vec3 pixel2camera(const Mat33 &K, const Vec2 &p_p, const double depth);
        static Vec3 pixel2camera(const Mat33 &K, const Point2f &p_p, const double depth);

        static Vec3 pixel2world(const Mat33 &K, const Vec2 &p_p, const SE3 &T_w_c, const double depth);
        static Vec3 pixel2world(const Mat33 &K, const Point2f &p_p, const SE3 &T_w_c, const double depth);

        static Vec2 world2pixel(const Mat33 &K, const Vec3 &p_w, const SE3 &T_w_c);
        static Point2f world2pixel(const Mat &K, const Mat &p_w, const Mat &T_w_c);

        static Vec3 pixel2homo(const Mat33 &K, const Vec2 p_p);
        static Vec3 pixel2homo(const Mat33 &K, const Point2f p_p);

        static SE3 cvT2SE3(const Mat &R, const Mat &t);
    };

    class Func
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // 计算相对平移与旋转
        static void calcReltvRt(const SE3 &T1, const SE3 &T2, float &R_reltv, float &t_reltv);

        // 计算当前帧的基础矩阵
        static void calcFundmental(const SE3 &T1, const SE3 &T2, cv::Mat &F_21);

        // 判断前进或后退
        static void judgeForOrBackward(const SE3 &T_last, const SE3 &T_curr, bool &isForward, bool &isBackward);

        // 计算两个像素的距离
        static float calcDistence(const cv::Point2f &p0, const cv::Point2f &p1);

        // 判断一个点是否在图像边界内
        static bool inBorder(const cv::Point2f &pt, int border_size = 1);
        static bool inBorder(const float img_x, const float img_y, int border_size = 1);
        static bool inBorder(const float left, const float right, const float top, const float bot, int border_size = 1);

        // 创建一幅图像的金字塔
        static void createPyramids(const int &numlayers, const double pyramid_scale, const Mat &img1, vector<Mat> &pyr);

        // 获取像素灰度值(双线性插值)
        static float getPixelValue(const cv::Mat &img, const float x, const float y);

        // 计算两个矩形的交并比
        static float calcIOU(const cv::Point2f &R1LeftUp, const cv::Point2f &R1RightDown, const cv::Point2f &R2LeftUp, const cv::Point2f &R2RightDown);

        // 图像仿射块变换
        static void getWarpAffine(const SE3 &cam_ref, const SE3 &cam_cur, const Mat &patch, const Vec2 &px_ref, const double &depth, Mat &dst_warp);

        // 三角化
        static void triangulatePoint(const Mat34 &P_1, const Mat34 &P_2, const cv::Point2f &p_ref, const cv::Point2f &p_cur, Vec3 &p_esti);
        static void triangulatePoint(const Mat &P_1, const Mat &P_2, const cv::Point2f &p_ref, const cv::Point2f &p_cur, Mat &p_esti);

        // 计算两个像素块的NCC（归一化相似度）
        static float calcPatchNcc(const Mat &p1, const Mat &p2);
    };
} // namespace myslam
#endif