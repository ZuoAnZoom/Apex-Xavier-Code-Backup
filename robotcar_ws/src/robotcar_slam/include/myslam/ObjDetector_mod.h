#pragma once

#ifndef OBJDETECTOR_MOD_H
#define OBJDETECTOR_MOD_H

#include "myslam/CommonInclude.h"
#include "myslam/Param.h"
#include "myslam/Frame.h"
#include "myslam/Object.h"

namespace myslam
{
    class Detector
    {
    public:
        typedef shared_ptr<Detector> Ptr;

        vector<String> outlayers_; // 网络输出层名字

        Net net_;

        vector<string> classes_;                             // 目标标签
        int movable_things[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; // 移动目标标签

    public:
        Detector();
        ~Detector();

        void loadNetWork(); // 加载网络模型和权重

        void detectObject(Frame::Ptr curr); // 网络的前向传播

        void postProcess(Frame::Ptr curr, const vector<Mat> &outs); // 后处理

        void makeObjMask(Frame::Ptr curr); // 制作当前帧的目标掩码
    };
} // namespace myslam

#endif