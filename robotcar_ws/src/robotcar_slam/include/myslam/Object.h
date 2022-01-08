#pragma once

#ifndef OBJECT_MOD_H
#define OBJECT_MOD_H

#include "myslam/CommonInclude.h"

namespace myslam
{
    class Object
    {
    public:
        typedef shared_ptr<Object> Ptr;

        // 在图像中的位置
        int left_;
        int right_;
        int top_;
        int bot_;

        // 分数
        float score_;

        // 掩码
        Mat mask_;
        int area_; // 面积

        // 标签
        int class_id_;  // 所属类别
        string label_;  // 类别标签
        int r_, g_, b_; // 颜色（用于显示）

        // 角点
        vector<cv::Point2f> corners_; // 角点坐标
        vector<float> cordepths_;     // 角点深度

        // 位置
        Vec3 wld_coor_;               // 世界坐标
        vector<Vec3> trajectory_;     // 运动轨迹（未计算出坐标则为0）
        vector<Vec3> corns_wld_coor_; // 目标边框四个角点的世界坐标
        float depth_;                 // 深度

        // 与帧的匹配关系
        vector<int> obs_kfms_; // 被观测到的帧的ID
        int num_unmatched_;    // 连续未被匹配的次数

        bool matched_by_curr_ = false; // 是否被当前帧匹配
        bool out_view_ = false;        // 是否移出相机视野
        bool error_ = false;           // 是否可用（如果在该目标上未提取到角点、目标完全超出图像边界为true）
        bool dyna_ = false;            // 是否是动态目标

    public:
        Object() : num_unmatched_(0) {}
        ~Object() {}
    };
} // namespace myslam
#endif