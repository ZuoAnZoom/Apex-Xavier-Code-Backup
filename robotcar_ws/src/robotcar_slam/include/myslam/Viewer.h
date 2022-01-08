#pragma once

#ifndef VIEWER_H
#define VIEWER_H

#include "myslam/CommonInclude.h"
#include "myslam/System.h"
#include "myslam/Frame.h"
#include "myslam/Map.h"

namespace myslam
{
    class System;
    class Frame;
    class Map;

    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef shared_ptr<Viewer> Ptr;

        // 相机大小
        const float w = 0.0001 * Camera::width_;
        const float h = 0.0001 * Camera::height_;
        const float z = w * 0.5;

    public:
        Viewer();
        ~Viewer();

        // 画一个相机
        void drawCamera(float red, float green, float blue);

        // 画原点坐标系
        void drawCoordination();

        // 画出当前帧位姿
        void drawCurrFrame(pangolin::OpenGlRenderState &s_cam, pangolin::OpenGlMatrix &Twc, System::Ptr system);

        // 画出校验帧位姿
        void drawTruthFrames(pangolin::OpenGlMatrix &Twc, System::Ptr system);

        // 画出关键帧位姿
        void drawKeyFrames(pangolin::OpenGlMatrix &Twc, System::Ptr system);

        // 画局部地图帧
        void drawMapFrames(pangolin::OpenGlMatrix &Tcw, System::Ptr system);

        // 画出轨迹
        void drawTranslation(System::Ptr system);
        
        // 画目标
        void drawObjects(System::Ptr system);

        // 画出地图点
        void drawMapPoints(System::Ptr system);

        // 画地图目标运动轨迹
        void drawObjetTrajs(System::Ptr system);

        // 显示当前帧图像
        void drawCurrImage(System::Ptr system);

        // GUI主函数
        void displayGUI(System::Ptr system);

        // 图像显示主函数
        void displayImage(System::Ptr);

        // 显示当前帧检测到的物体
        void displayDetectedObjects(Frame::Ptr curr);

        // 显示左右目匹配的特征点
        void displayLeftRightMatchPoints(Frame::Ptr curr);

        // 显示当前帧跟踪的点
        void displayCurrTrackPoints(Frame::Ptr curr, Frame::Ptr last);

        // 显示当前帧匹配的点
        void displayCurrMatchPoints(bool match_last, Frame::Ptr curr, Frame::Ptr last);

        // 显示当前帧匹配上的局部地图点
        void displayCurrMatchMapPoints(Frame::Ptr curr);
    };
} // namespace myslam

#endif