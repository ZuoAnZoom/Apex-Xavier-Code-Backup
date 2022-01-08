#pragma once

#ifndef SYSTEM_H
#define SYSTEM_H

#include "myslam/CommonInclude.h"
#include "myslam/Param.h"
#include "myslam/Frame.h"
#include "myslam/Map.h"
#include "myslam/Odometry_mod.h"
#include "myslam/ObjDetector_mod.h"
#include "myslam/ObjSegment_mod.h"

#ifdef _ROS_
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <solo/img.h>
#include <solo/target.h>
#include <cv_bridge/cv_bridge.h>
#endif

namespace myslam
{
    class Frame;
    class Map;

    class System
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<System> Ptr;

        enum FileType // 文件类型
        {
            IMAGE = 0,
            VIDEO = 1
        };
        FileType file_type_;

        enum DatasetType // 数据集类性
        {
            KITTI = 0,
            EUROC = 1,
            TUM = 2,
            VIRTUAL = 3,
            OTHER = 4
        };
        DatasetType dataset_type_;

        enum ModelType // 深度学习模型
        {
            NONE = 0,
            YOLOv3 = 1,
            MASKRCNN = 2,
            SOLO = 3
        };
        ModelType model_type_;

        ifstream associate_;    // 关联文件
        ifstream groundtruth_;  // 真值文件
        ifstream gt_timestamp_; // 真值文件对应的时间戳文件（暂用）
        string dataset_dir_;    // 数据集路径
        bool has_truth_ = true; // 真值文件存在的标志

        vector<string> img0_files_, img1_files_;
        vector<double> time_stamps_;

        Mat m1l_, m2l_, m1r_, m2r_; // 左右目重映射矩阵
        SE3 T_BS_;                  // 相机相对于载体的位姿关系矩阵

        Mat left_, right_; // 左右目
        Mat depth_;        // RGBD相机深度图
        Frame::Ptr curr_;  // 当前帧
        Frame::Ptr last_;  // 上一帧
        Frame::Ptr key_;   // 关键帧

        float total_dist_; // 总行驶里程

        // 用于绘图、误差计算及结果输出
        SE3 init_truth_T_;               // 初始真值位姿
        SE3 curr_T_;                     // 当前帧位姿
        SE3 truth_T_;                    // 校验帧位姿
        vector<SE3> curr_vecT_;          // 估计的位姿序列
        vector<SE3> key_vecT_;           // 关键帧位姿序列
        vector<SE3> truth_vecT_;         // 校验帧位姿序列
        vector<SE3> mframes_vecT_;       // 局部地图帧位姿序列
        vector<Vec3> map_points_;        // 当前帧观测到的路标点
        vector<vector<Vec3>> obj_coors_; // 地图目标世界坐标（包含该目标的颜色）
        vector<vector<Vec3>> obj_trajs_; // 地图目标运动轨迹
        mutex mute_T_;                   // 位姿锁

        int num_lost_;           // 里程计跟踪失败的帧数
        vector<int> lost_frame_; // 里程计跟踪失败的帧序号

        clock_t t1_, t2_; // 计时

        string workspace_abs_dir_;

// ROS收发消息相关
#ifdef _ROS_
        ros::Publisher img_pub_;
        ros::Subscriber target_sub_;
        bool receive_target_ = false;
#endif

        int index_;

    public:
        System();
        ~System();

        bool initSystem();
        bool initConfigParameter();
        bool initImageBounds();
        bool initImageDataset();
        bool initVideoInterface();
        bool initNetWork(Detector::Ptr detector, Segment::Ptr segment);
        bool needRecalibStereo();
        void recalibStereo();
        bool isVideoOpen();
        bool readImage(int num, Frame::Ptr curr, std::vector<unsigned char> left, std::vector<unsigned char> right);
        bool readImageEach(int i, Frame::Ptr curr, std::vector<unsigned char> left, std::vector<unsigned char> right);
        bool readImageEach(Frame::Ptr curr, std::vector<unsigned char> left, std::vector<unsigned char> right);
#ifdef _ROS_
        void publishImgMsg(const string &img_path);
        void subscribeTargets(const solo::target &msg);
#endif
        void runNetWork(Detector::Ptr detector, Segment::Ptr segment, Frame::Ptr curr);
        void readTruth(Frame::Ptr curr);
        void readTruthData();
        void readTruthFromKitti(Frame::Ptr curr);
        void readTruthFromEuroc(Frame::Ptr curr);
        void readTruthFromTUM(Frame::Ptr curr);
        void calcCurrError(Frame::Ptr curr);
        void updateGUI(Frame::Ptr curr, Map::Ptr map);
        void outputResult();
        void calcTotalRMSE();
        void updateFrame(Frame::Ptr curr);
    };

#ifdef _ROS_
    void msgSpin();
#endif

} // namespace myslam
#endif