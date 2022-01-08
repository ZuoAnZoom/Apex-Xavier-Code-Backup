#pragma once

#ifndef FRAME_H
#define FRAME_H

#include "myslam/CommonInclude.h"
#include "myslam/ORBextractor_mod.h"
#include "myslam/Feature.h"
#include "myslam/Object.h"
#include "myslam/MapPoint.h"
#include "myslam/Transform.h"

namespace myslam
{
    class ORBextractor;
    class Feature;
    class Object;
    class MapPoint;

    class Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;
        static int factory_id_; // 当前帧ID
        int id_;                // 帧ID
        double time_stamp_;     // 时间戳

        static float min_depth_, max_depth_; // 特征点的最小最大深度

        SE3 T_wc_d_;  // 直接法解算得到的位姿
        SE3 T_wc_f_;  // 特征点法解算得到的位姿
        SE3 T_wc_;    // 前端跟踪得到的位姿
        SE3 T_truth_; // 真值位姿

        Mat left_, right_; // 左图,右图
        Mat mask_;         // 目标掩码（背景为0）
        Mat depth_;        // RGBD相机深度图
        Mat scimg_;        // 缩放图

        /*目标*/
        vector<shared_ptr<Object>> objects_;       // 当前帧目标
        vector<shared_ptr<Object>> mobjs_matched_; // 匹配的局部地图中的目标（nullptr为未与局部地图目标匹配上，视为当前帧新检测到的目标；与当前帧目标对应）

        /*特征点*/
        vector<shared_ptr<Feature>> features_;        // 当前帧特征点
        shared_ptr<ORBextractor> orbleft_, orbright_; // 左右目ORB特征
        vector<cv::Point2f> features_tracked_;        // 上一帧特征点通过直接法跟踪得到的在当前帧中的位置（0为未跟踪到，与上一帧特征点对应）
        vector<cv::Point2f> features_matched_;        // 上一帧或关键帧特征点匹配到的在当前帧的位置（0为未匹配上，与上一帧或关键帧特征点对应）

        /*地图点*/
        vector<shared_ptr<MapPoint>> mpoints_matched_; // 匹配的地图点（nullptr为未匹配上，与当前帧特征点对应）

        int num_features_; // 当前帧特征点的数量（左右目匹配上且深度合理的点）
        int num_matched_;  // 匹配的特征点的数量（等于features_matched_中不为0的数量）
        int num_tracked_;  // 跟踪的特征点的数量（等于features_tracked_中不为0的数量）
        int num_mpoints_;  // 匹配的地图点的数量（等于mpoints_matched_中不为nullptr的数量）

        bool is_good_ = false;   // 估计的位姿是否合理
        bool is_key_ = false;    // 是否是关键帧的标志
        bool just_init_ = false; // 是否刚完成初始化
        bool match_last_ = true; // 与上一帧匹配还是与关键帧匹配的标志

    public:
        Frame(int id);
        Frame(){};
        ~Frame(){};

        static Frame::Ptr createFrame(); // 创建一帧

        /*主要函数 */
        void extractKeyPoints(); // 提取关键点

        void undistKeyPoints(); // 根据相机畸变参数校正关键点坐标

        void calcDescriptors(); // 计算描述子

        void matchFromRGBD(); // RGBD相机生成左目深度

        void matchFromeStereo(); // 双目相机左右目特征点匹配

        void createFeatures(); // 生成当前帧的特征点

        bool matchFeaturesForTrian(Frame::Ptr frame, vector<int> &index); // 单目沿极线匹配

        bool matchFeaturesForMonoInit(Frame::Ptr frame, vector<int> &index, int th); // 单目初始化匹配

        bool matchFeatures(Frame::Ptr frame, const SE3 &initPose, int th); // 与传入的Frame进行特征匹配

        bool matchFeaturesByProjection(Frame::Ptr frame, const SE3 &initPose, int th); // 投影匹配

        bool matchFeaturesByBruteForce(Frame::Ptr frame, const SE3 &initPose, int th); // 暴力匹配

        bool matchMapPoints(list<shared_ptr<MapPoint>> map_points, const float th); // 与地图点进行匹配

        void calcFeaturesCamCoors(); // 计算特征点的相机坐标

        void calcFeaturesWldCoors(); // 计算特征点的世界坐标

        bool infrustum(shared_ptr<MapPoint> pMP, float &projx, float &projy, float &projz, int &prelevel, float &viewcos);

        bool reconstructPose(Frame::Ptr frame, const Mat &E, vector<Vec3> &pts_3D); // 从本质矩阵恢复位姿

        int checkRt(Frame::Ptr frame, const Mat &R, const Mat &t, vector<Vec3> &pts_3D, float &parallax); // 单目初始化时判断Rt是否合理

    private:
        vector<cv::KeyPoint> keypoints_l_; // 左目中的所有关键点（去除畸变后）
        vector<cv::KeyPoint> keypoints_r_; // 右目中的所有关键点
        Mat descriptors_l_;                // 左目所有关键点的描述子
        Mat descriptors_r_;                // 右目所有关键点的描述子
        vector<float> keypts_depth_;       // RGBD相机中每个特征点对应的深度
        vector<float> left_to_right_;      // 左目特征点对应的右目特征点
    };

} // namespace myslam

#endif // FRAME_H
