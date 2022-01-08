#pragma once

#ifndef LOOPCLOSING_MOD_H
#define LOOPCLOSING_MOD_H

#include "myslam/CommonInclude.h"
#include "myslam/Map.h"

namespace myslam
{
class Map;

class LoopClosing
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef shared_ptr<LoopClosing> Ptr;
    
    vector<Frame::Ptr> candidates_; // 闭环候选关键帧

    Frame::Ptr curr_; //当前关键帧

public:
    LoopClosing();
    ~LoopClosing();

    void closeLoop(Frame::Ptr kframe, Map::Ptr global_map); // 闭环检测，在全局地图中查找与传入的关键帧中相似的关键帧，LoopClosing的主函数

    /*闭环检测相关函数 */
    bool detectCandiDates(); // 检测是否存在闭环候选帧

    void computeRelativePose(); // 计算新关键帧与候选帧之间的相对位姿

    void correctLoop(); // 根据相对位姿关系，将环上的关键帧的位姿改正过来，使环闭合
};
} // namespace myslam
#endif