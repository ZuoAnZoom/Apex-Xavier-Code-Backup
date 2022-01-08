#pragma once

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/CommonInclude.h"
#include "myslam/Frame.h"

namespace myslam
{
    class Frame;

    class Sort
    {
    public:
        bool operator()(const weak_ptr<Frame> _A, const weak_ptr<Frame> _B) const;
    };

    class MapPoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<MapPoint> Ptr;
        static int factory_id_;
        int id_; // 地图点id

        Vec3 coor_;      // 世界坐标系下的坐标
        float max_dist_; // 可观测到该点的最远距离
        float min_dist_; // 可观测到该点的最近距离

        deque<Mat> descrs_; // 该地图点在所有帧中的描述子
        Mat best_descr_;    // 最佳描述子

        deque<Vec3> viewagls_; // 观测方向
        Vec3 mean_viewagl_;    // 平均观测方向
        float min_viewcos_;    // 最小观测视角余弦

        // 与关键帧之间的关联
        list<weak_ptr<Frame>> obs_kfms_;              // 观测到的帧
        map<weak_ptr<Frame>, int, Sort> obs_kfms_id_; // 观测到的帧及其在该帧中的索引

        int matched_times_; // 被匹配到的次数
        int visible_times_; // 被看到的次数

        bool dyna_ = false; // 是否是动态地图点

    public:
        MapPoint(int id, const Vec3 &coor = Vec3(0, 0, 0), const Vec3 &best_viewagl = Vec3(0, 0, 0), const Mat &best_descr = Mat(), double max_d = 50, double min_d = 3, double min_viewcos = 1.0);

        static MapPoint::Ptr createMapPoint(shared_ptr<Frame> curr, int i);

        void computeBestDescr(); // 计算最佳描述子

        void computeMeanViewAgl(); // 计算平均观测方向
    };
} // namespace myslam

#endif // MAPPOINT_H
