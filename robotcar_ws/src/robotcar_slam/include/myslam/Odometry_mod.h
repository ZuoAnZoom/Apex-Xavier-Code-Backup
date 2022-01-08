#pragma once

#ifndef ODOMETRY_MOD_H
#define ODOMETRY_MOD_H

#include "myslam/CommonInclude.h"
#include "myslam/Param.h"
#include "myslam/Frame.h"
#include "myslam/Map.h"
#include "myslam/MapPoint.h"
#include "myslam/DirectTracker_mod.h"
#include "myslam/MotionClassifier_mod.h"

namespace myslam
{
    class Frame;
    class Map;

    class Odometry
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<Odometry> Ptr;

        // 里程计状态
        enum OdomState
        {
            NOINIT = -1,
            OK = 1,
            LOST = 0
        };

        static OdomState curr_state_; // 当前帧VO状态

        // 前后帧位姿估计方式
        enum CalcMethod
        {
            FEATURES = 1,
            DIRECT = 2,
        };

        static CalcMethod calc_method_;

        // 匹配方法
        enum MatchMethod
        {
            LAST_FRAME = 1,
            KEY_FRAME = 2,
            LOCAL_MAP = 3,
        };
        MatchMethod match_method_;

        Frame::Ptr ref_;  // 参考关键帧
        Frame::Ptr curr_; // 当前帧
        Frame::Ptr last_; // 上一帧

        DirectTracker::Ptr direct_; // 直接法跟踪

        int num_lost_; // 匹配失败的次数

        SE3 T_reltv_; // 前后两帧的相对位姿
        SE3 T_model_; // 匀速模型计算出的位姿

        // 合理的解算位姿需要满足的条件
        static float normalpose_max_t_; // 与匀速模型的最大位移差
        static float normalpose_max_R_; // 与匀速模型的最大旋转差

        // 成为关键帧需要满足的条件
        static float key_max_t_; // 与参考帧的最大位移差
        static float key_max_R_; // 与参考帧的最大旋转差

        bool has_motion_model_; // 是否有恒速模型计算出位姿初值

        clock_t t1, t2;

    public: // functions
        Odometry();
        ~Odometry();

        void trackFrame(Frame::Ptr frame, Map::Ptr local_map); // 根据状态对每一帧进行处理，主要函数

        // 判断解算出的位姿是否合理
        bool isNormalPose(bool cmp_model, const SE3 &cur_T, const SE3 &ref_T = SE3());
        // 判断是否是个关键帧
        bool isKeyFrame(Map::Ptr local_map);
        // 判断是否已经跟踪丢失
        bool isLost();

        void updateLastFrame(); // 更新上一帧
        void updateRefFrame();  // 更新参考帧

        void calcFeaturesCoors(); // 计算特征点坐标

        /*初始化相关函数 */
        bool initOdometry(Map::Ptr local_map); // 初始化
        void processCurr();                    // 当前帧预处理
        void setPoseInit();                    // 设置位姿估计初值(匀速模型)

        /*重定位相关函数 */
        bool relocateOdometry(); // 重定位

        /*匹配相关函数 */
        bool matchLastFrame();                  // 匹配上一帧
        bool matchKeyFrame();                   // 匹配关键帧
        bool matchLocalMap(Map::Ptr local_map); // 匹配局部地图

        /*位姿估计相关函数 */
        void calcPoseByDirect();                // 直接法估计两帧相对位姿
        bool calcPoseByPnP();                   // PnP计算位姿
        void eraseOutliers(const Mat &inliers); // 剔除RANSACPnP计算中的误匹配
    };
} // namespace myslam

#endif
