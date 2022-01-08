#pragma once

#ifndef MOTIONESTIMATOR_MOD_H
#define MOTIONESTIMATOR_MOD_H

#include "myslam/CommonInclude.h"
#include "Param.h"
#include "myslam/Frame.h"
#include "myslam/Map.h"

namespace myslam
{
    class Frame;
    class Map;

    class MotionClassifier
    {
    public:
        typedef shared_ptr<MotionClassifier> Ptr;

        int num_dyna_obj_; // 当前帧动态目标数量

        // 几何约束法、外观约束法、位置约束法对当前帧目标的动态标记
        vector<bool> dyna_g_;
        vector<bool> dyna_a_;
        vector<bool> dyna_p_;

        // 剔除动态目标后剩余的3D、2D点
        vector<Point3f> points_3d_;
        vector<Point2f> pixels_2d_;

        static int min_area_;            // 漏检目标的最小面积
        static int max_area_;            // 漏检目标的最大面积
        static float dyna_epipolar_dis_; // 点到极线的距离阈值
        static float dyna_ratio_;        // 动静点比例

    public:
        MotionClassifier();
        ~MotionClassifier();

        // 检测动态物体
        void detectDynaObjects(Frame::Ptr frame, Frame::Ptr curr, Map::Ptr local_map);

        // 将当前帧目标中恢复出深度的角点投影到frame帧
        void prjPointsToFrame(const SE3 &cur_T, const SE3 &pre_T, vector<Object::Ptr> &curr_obj, vector<cv::Point2f> &cors_pre, vector<cv::Point2f> &cors_cur);

        // 通过光流找到当前帧角点在frame帧中的位置
        void trackPointsInFrame(Frame::Ptr frame, Frame::Ptr curr, const vector<cv::Point2f> &cors_pre, const vector<cv::Point2f> &cors_cur, vector<cv::Point2f> &pts_pre, vector<cv::Point2f> &pts_cur);

        // 将地图目标投影到当前帧
        bool prjObjectsToCurr(Object::Ptr obj, const SE3 &curr_T, Vec2 &lefttop, Vec2 &rightbot);

        // 处理漏检
        void processMissedDetection(Map::Ptr local_map, Frame::Ptr curr);

        // 目标匹配
        void matchObject(Map::Ptr local_map, Frame::Ptr curr);

        // 目标定位
        void locateObject(Frame::Ptr curr);

        // 动态物体筛选
        void selectDynaObjects(Frame::Ptr frame, Frame::Ptr curr, Map::Ptr local_map);

        // 筛选动态物体的三种方法
        void selectByGeometry(Frame::Ptr frame, Frame::Ptr curr, const vector<cv::Point2f> &pts_pre, const vector<cv::Point2f> &pts_curr, const cv::Mat &F); // 几何约束

        void selectByAppearance(Frame::Ptr frame, Frame::Ptr curr, const vector<cv::Point2f> &cors_pre, const vector<cv::Point2f> &cors_curr); // 外观一致性

        void selectByPosition(Map::Ptr local_map, Frame::Ptr curr); // 深度一致性

        // 剔除动态目标上的特征掉
        void rejectDynaObjectPts(Frame::Ptr curr);

        // 重新计算当前帧位姿
        void recalcCurrPose(Frame::Ptr curr);
    };
} // namespace myslam

#endif