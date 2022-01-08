#pragma once

#ifndef MAP_H
#define MAP_H

#include "myslam/CommonInclude.h"
#include "myslam/Frame.h"
#include "myslam/MapPoint.h"
#include "myslam/Object.h"

namespace myslam
{
    class Frame;
    class MapPoint;
    class Object;

    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<Map> Ptr;

        int num_mpoints_; // 地图点数量
        int num_frames_;  // 关键帧数量
        int num_objects_; // 目标数量

        list<shared_ptr<MapPoint>> map_points_; // 地图点
        list<shared_ptr<Frame>> frames_;        // 关键帧
        list<shared_ptr<Object>> objects_;      // 目标

    public:
        Map();
        ~Map();

        // 得到最新的帧
        Frame::Ptr getLastFrame();

        // 得到所有地图点的坐标
        vector<Vec3> getAllMapPointsCoors();

        // 得到所有帧的位姿
        vector<SE3> getAllFramesPoses();

        // 得到所有目标的世界坐标
        vector<vector<Vec3>> getAllObjectsCoors();
        
        // 得到所有目标的运行轨迹
        vector<vector<Vec3>> getAllObjectsTrajs();
    };
} // namespace myslam

#endif // MAP_H
