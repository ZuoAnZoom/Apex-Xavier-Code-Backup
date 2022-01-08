#pragma once

#ifndef MAPPING_MOD_H
#define MAPPING_MOD_H

#include "myslam/CommonInclude.h"
#include "myslam/Param.h"
#include "myslam/Map.h"
#include "myslam/Frame.h"
#include "myslam/MapPoint.h"
#include "myslam/Odometry_mod.h"
#include "myslam/DirectTracker_mod.h"

namespace myslam
{
    class Frame;
    class MapPoint;
    class Map;

    class Mapping
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<Mapping> Ptr;

        Frame::Ptr curr_; // 当前关键帧

        unordered_set<int> del_fm_id_; // 被剔除的关键帧的ID
        unordered_set<int> del_mp_id_; // 被剔除的地图点的ID

        clock_t t1_, t2_;

    public:
        Mapping();
        ~Mapping(){};

        void buildMap(Frame::Ptr curr, Frame::Ptr last, Map::Ptr local_map); // 前端采用特征点法建图

        void insertFrame(Map::Ptr local_map);                          // 加入当前帧
        void insertMapPoints(Map::Ptr local_map);                      // 加入地图点或更新地图点属性
        void updateObjects(Frame::Ptr last, Map::Ptr local_map);       // 更新局部地图中的目标
        void eraseFrames(Map::Ptr local_map);                          // 剔除冗余关键帧
        void eraseMapPoints(Map::Ptr local_map);                       // 剔除冗余地图点
        void updateConnection(Map::Ptr local_map);                     // 更新关键帧与地图点之间的共视关系
        void trianMapPointCoor(MapPoint::Ptr mp, int idx);             // 三角化地图点坐标
        void updateGlobalMap(Map::Ptr local_map, Map::Ptr global_map); // 将处理完的局部地图更新全局地图
    };
} // namespace myslam

#endif // MAPPING_H
