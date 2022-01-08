#include "myslam/LoopClosing_mod.h"

namespace myslam
{
LoopClosing::LoopClosing() {}

LoopClosing::~LoopClosing() {}
/**
 * @brief 闭环检测，在全局地图中查找与传入的关键帧中的场景相似的关键帧，LoopClosing的主函数
 * 
 * @param kframe 
 * @param global_map 
*/
void LoopClosing::closeLoop(Frame::Ptr kframe, Map::Ptr global_map)
{
    // if (!kframe->isKFrame())
    // {
    //     return;
    // }
    // else
    // {
    //     if (detectCandiDates())
    //     {
    //         curr_ = kframe;
    //         computeRelativePose();
    //         correctLoop();
    //     }
    // }
}
/**
 * @brief 检测是否存在闭环候选帧
 * 
 * @return true 
 * @return false 
*/
bool LoopClosing::detectCandiDates()
{
}
/**
 * @brief 计算新关键帧与候选帧之间的相对位姿
 * 
*/
void LoopClosing::computeRelativePose()
{
}
/**
 * @brief 根据相对位姿关系，将环上的关键帧的位姿改正过来，使环闭合
 * 
*/
void LoopClosing::correctLoop()
{
}
} // namespace myslam