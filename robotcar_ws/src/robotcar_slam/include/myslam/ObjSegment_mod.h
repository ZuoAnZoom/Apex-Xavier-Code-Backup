#pragma once

#ifndef OBJSEGMENT_MOD_H
#define OBJSEGMENT_MOD_H

#include "myslam/CommonInclude.h"
#include "myslam/Frame.h"
#include "myslam/Object.h"

namespace myslam
{
    class Frame;

    class Segment
    {
    public:
        typedef shared_ptr<Segment> Ptr;

        Net net_;

        vector<string> classes_;                             // 目标标签
        int movable_things[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; // 移动目标标签

    public:
        Segment();
        ~Segment();

        void loadNetWork(); // 加载网络模型和权重

        void segmentObject(Frame::Ptr curr); // 实例分割的主函数

        void postprocess(Frame::Ptr curr, const vector<Mat> &outs); // 后处理

        void maskOBjMask(Frame::Ptr curr); // 制作当前帧目标掩码
    };
} // namespace myslam

#endif