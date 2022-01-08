#pragma once

#ifndef FEATURE_H
#define FEATURE_H

#include "myslam/CommonInclude.h"

namespace myslam
{
    class Feature
    {
    public:
        typedef std::shared_ptr<Feature> Ptr;

        float x_;         // x坐标
        float y_;         // y坐标
        cv::Point2f pt_;  // 图像坐标
        float x_r_;       // 右目的x坐标
        float y_r_;       // 右目的y坐标
        float disparity_; // 视差（x_-x_r_）
        float depth_;     // 深度
        float response_;  // 强度
        int octave_;      // 所在图像金字塔的层数
        Mat descriptor_;  // 描述子

        int object_id_; // 所属目标（0为不属于任何目标）

        bool dyna_ = false;   // 是否是动点
        bool stereo_ = false; // 双目：是否左右目匹配且深度合理；RGBD：深度合理；单目：与上一帧特征匹配

        float view_cos_; // 视角余弦（单目初始化时记录）
        Vec3 camcoor_;   // 相机坐标（0：弃用）
        Vec3 wldcoor_;   // 世界坐标（0：弃用）

    public:
        Feature() : camcoor_(Vec3(0, 0, 0)), wldcoor_(Vec3(0, 0, 0))
        {
            descriptor_.create(1, 32, CV_8UC1);
        }
        ~Feature() {}
    };
} // namespace myslam
#endif