#include "myslam/Viewer.h"

namespace myslam
{
    Viewer::Viewer() {}

    Viewer::~Viewer() {}

    void Viewer::drawCoordination()
    {
        glLineWidth(7);
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0); // 红色X轴
        glVertex3f(0, 0, 0);
        glVertex3f(3, 0, 0);
        glColor3f(0.0, 1.0, 0.0); // 绿色Y轴
        glVertex3f(0, 0, 0);
        glVertex3f(0, 3, 0);
        glColor3f(0.0, 0.0, 1.0); // 蓝色Z轴
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 3);
        glEnd();
    }

    void Viewer::drawCamera(float red, float green, float blue)
    {
        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(red, green, blue);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
    }

    void Viewer::drawCurrFrame(pangolin::OpenGlRenderState &s_cam, pangolin::OpenGlMatrix &Tcw, System::Ptr system)
    {
        unique_lock<std::mutex> lck(system->mute_T_);

        s_cam.Follow(Tcw);
        Tcw = system->curr_T_.matrix();
        glPushMatrix();
        glMultMatrixd(Tcw.m);
        drawCamera(1, 0, 1); // 品红
        glPopMatrix();
    }

    void Viewer::drawKeyFrames(pangolin::OpenGlMatrix &Tcw, System::Ptr system)
    {
        if (system->key_vecT_.size() == 0)
            return;
        unique_lock<std::mutex> lck1(system->mute_T_);
        for (int i = 0; i < system->key_vecT_.size(); i++)
        {
            Tcw = system->key_vecT_[i].matrix();

            glPushMatrix();
            glMultMatrixd(Tcw.m);
            glLineWidth(2);
            drawCamera(1, 0, 1); // 品红
            glPopMatrix();
        }
    }

    void Viewer::drawTruthFrames(pangolin::OpenGlMatrix &Tcw, System::Ptr system)
    {
        if (system->truth_vecT_.size() == 0 || !system->has_truth_)
            return;
        unique_lock<std::mutex> lck1(system->mute_T_);
        Tcw = system->truth_vecT_.back().matrix();
        glPushMatrix();
        glMultMatrixd(Tcw.m);
        glLineWidth(2);
        drawCamera(0, 0, 1); // 蓝色
        glPopMatrix();
    }

    void Viewer::drawMapFrames(pangolin::OpenGlMatrix &Tcw, System::Ptr system)
    {
        unique_lock<std::mutex> lck(system->mute_T_);
        if (system->mframes_vecT_.size() == 0)
            return;
        for (int i = 0; i < system->mframes_vecT_.size(); i++)
        {
            Tcw = system->mframes_vecT_[i].matrix();

            glPushMatrix();
            glMultMatrixd(Tcw.m);
            glLineWidth(2);
            drawCamera(1, 0, 1); // 品红
            glPopMatrix();
        }
    }

    void Viewer::drawTranslation(System::Ptr system)
    {
        if (system->curr_vecT_.size() <= 1)
            return;
        glLineWidth(3);
        glBegin(GL_LINES);
        glColor3f(0.f, 1.f, 0.f); // 绿色
        unique_lock<std::mutex> lck1(system->mute_T_);
        // 画出估计的位姿轨迹
        for (int i = 0; i < system->curr_vecT_.size() - 1; i++)
        {
            Vec3 last_t = system->curr_vecT_[i].translation();
            Vec3 curr_t = system->curr_vecT_[i + 1].translation();

            glVertex(last_t);
            glVertex(curr_t);
        }
        // 画出校验帧轨迹
        glColor3f(0.0f, 0.f, 1.f); // 蓝色
        if (system->has_truth_)
        {
            for (int i = 0; i < system->truth_vecT_.size() - 1; i++)
            {
                Vec3 last_t = system->truth_vecT_[i].translation();
                Vec3 curr_t = system->truth_vecT_[i + 1].translation();

                glVertex(last_t);
                glVertex(curr_t);
            }
        }
        glEnd();
    }

    void Viewer::drawObjects(System::Ptr system)
    {
        unique_lock<std::mutex> lck(system->mute_T_);
        if (system->obj_coors_.size() == 0)
            return;
        for (int i = 0; i < system->obj_coors_.size(); i++)
        {
            glLineWidth(2);
            glBegin(GL_LINES);

            glColor3f(system->obj_coors_[i][4](0) / 255.f, system->obj_coors_[i][4](1) / 255.f, system->obj_coors_[i][4](2) / 255.f);
            glVertex(system->obj_coors_[i][0]);
            glVertex(system->obj_coors_[i][1]);

            glVertex(system->obj_coors_[i][1]);
            glVertex(system->obj_coors_[i][3]);

            glVertex(system->obj_coors_[i][3]);
            glVertex(system->obj_coors_[i][2]);

            glVertex(system->obj_coors_[i][2]);
            glVertex(system->obj_coors_[i][0]);
            glEnd();
        }
    }

    void Viewer::drawMapPoints(System::Ptr system)
    {
        unique_lock<std::mutex> lck(system->mute_T_);
        if (system->map_points_.size() == 0)
            return;
        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(1.f, 0.f, 0.f); // 红色
        for (auto iterr : system->map_points_)
        {
            if (iterr == Vec3(0, 0, 0))
                continue;
            glVertex(iterr);
        }
        glEnd();
    }

    void Viewer::drawObjetTrajs(System::Ptr system)
    {
        unique_lock<std::mutex> lck(system->mute_T_);
        if (system->obj_trajs_.size() == 0)
            return;
        glLineWidth(3);
        glBegin(GL_LINES);
        glColor3f(250. / 255.f, 128. / 255.f, 10. / 255.f); // 橙色
        for (int i = 0; i < system->obj_trajs_.size(); i++)
        {
            for (int j = 0; j < system->obj_trajs_[i].size() - 1; j++)
            {
                glVertex(system->obj_trajs_[i][j]);
                glVertex(system->obj_trajs_[i][j + 1]);
            }
        }
        glEnd();
    }

    /**
 * @brief 显示当前帧影像
 * 
 */
    void Viewer::drawCurrImage(System::Ptr system)
    {
        unique_lock<std::mutex> lck(system->mute_T_);
        if (system->curr_->left_.empty())
            return;
        Mat curr_left = system->curr_->left_.clone(); // FIXME: 这里经常崩溃
        // if (curr_left.channels() == 1)
        // {
        //     cvtColor(curr_left, curr_left, COLOR_GRAY2RGB);
        // }

        putText(curr_left, to_string(system->curr_->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID

        // // 显示目标掩码（用于图像分割）
        // Mat coloredMask(Camera::height_, Camera::width_, CV_8UC3, 0.7 * Scalar(0, 255, 255));
        // coloredMask += curr_left;
        // coloredMask.copyTo(curr_left, curr->mask_);

        // // 显示目标边框
        // for (auto obj : curr->objects_)
        // {
        //     Rect box(obj->left_, obj->top_, obj->right_ - obj->left_, obj->bot_ - obj->top_);

        //     rectangle(curr_left, box, Scalar(0, 255, 0), 2); // 绿色

        //     putText(curr_left, obj->label_, Point(box.x, box.y), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 红色
        // }
        cv::imshow("CurrLeft", curr_left);
        cv::waitKey(1);
    }

    /**
 * @brief GUI的主函数
 * 
 * @param system 存储着当前帧估计的位姿, 过往帧估计的位姿序列, 以及当前帧估计的点的相机坐标或世界坐标, 当前校验帧位姿, 过往校验帧位姿序列
 * @param GlobalMap 全局地图(有关键帧和全局地图点)
 */
    void Viewer::displayGUI(System::Ptr system)
    {
        //生成一个gui界面，定义大小
        pangolin::CreateWindowAndBind("Main", 1024, 768);
        //进行深度测试，保证某视角下像素只有一种颜色，不混杂
        glEnable(GL_DEPTH_TEST);

        //放置一个相机
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -2, -3, 0, 0, 0, 0, -1, 0)); // eye lookat up

        //创建视角窗口
        pangolin::Handler3D handler(s_cam);
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                                    .SetHandler(&handler);

        // 当前相机的位姿变换矩阵
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        while (!pangolin::ShouldQuit())
        {
            //清除颜色缓冲和深度缓冲
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);

            //背景置为白色
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            //==============================================画起始坐标轴
            drawCoordination();

            // =============================================画点云
            drawMapPoints(system);

            // =============================================画局部地图帧
            drawMapFrames(Twc, system);

            // =============================================画校验帧
            drawTruthFrames(Twc, system);

            // =============================================画关键帧
            // drawKeyFrames(Twc, system);

            // =============================================画当前帧
            drawCurrFrame(s_cam, Twc, system);

            // =============================================画轨迹
            drawTranslation(system);

            // =============================================画目标
            drawObjects(system);

            // =============================================画目标运动轨迹
            // drawObjetTrajs(system);

            // =============================================显示当前帧影像
            // drawCurrImage(system);

            //交换帧和并推进事件
            pangolin::FinishFrame();
        }
    }
    /**
 * @brief 显示当前帧左右目或前后帧匹配
 * 
 */
    void Viewer::displayImage(System::Ptr system)
    {
        // 显示检测到的物体
        // displayDetectedObjects(system->curr_);

        // 显示左右目匹配
        // displayLeftRightMatchPoints(system->curr_);

        // 显示前后帧匹配或关键帧匹配
        if (system->curr_->match_last_)
            displayCurrMatchPoints(true, system->curr_, system->last_);
        else
            displayCurrMatchPoints(false, system->curr_, system->key_);

        // 显示直接法跟踪的点
        // displayCurrTrackPoints(system->curr_, system->last_);

        // 显示匹配到的地图点
        // displayCurrMatchMapPoints(system->curr_);
    }

    /**
 * @brief 显示当前帧检测到的物体
 * 
 * @param curr 
 */
    void Viewer::displayDetectedObjects(Frame::Ptr curr)
    {
        if (curr.get() == nullptr)
            return;
        Mat curr_left = curr->left_.clone();
        if (curr_left.channels() == 1)
        {
            cvtColor(curr_left, curr_left, COLOR_GRAY2RGB);
        }
        // putText(curr_left, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID

        // // 显示目标掩码
        // for (int i = 0; i < curr->objects_.size(); i++)
        // {
        //     int r = curr->objects_[i]->r_;
        //     int g = curr->objects_[i]->g_;
        //     int b = curr->objects_[i]->b_;
        //     Mat coloredMask(Camera::height_, Camera::width_, CV_8UC3, 0.7 * Scalar(b, g, r));
        //     coloredMask += curr_left;
        //     Mat mask_cmp(Camera::height_, Camera::width_, CV_8UC1, Scalar(i + 1));
        //     Mat mask = curr->mask_ == mask_cmp;
        //     coloredMask.copyTo(curr_left, mask);
        // }

        // 显示目标边界框
        for (auto obj : curr->objects_)
        {
            Rect box(obj->left_, obj->top_, fabs(obj->left_ - obj->right_), fabs(obj->top_ - obj->bot_));

            rectangle(curr_left, box, Scalar(0, 255, 0), 2); // 绿色

            putText(curr_left, obj->label_, Point(box.x, box.y), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 红色
        }
        imshow("curr_Objects", curr_left);
        cv::waitKey(0);
    }

    /**
 * @brief 显示左右目匹配的特征点
 * 
 * @param curr 
 */
    void Viewer::displayLeftRightMatchPoints(Frame::Ptr curr)
    {
        if (curr.get() == nullptr || curr->features_.size() == 0)
            return;
        if (Camera::Type_ == Camera::eType::RGBD)
        {
            cout << "RGBD camera has no right img" << endl;
            return;
        }

        Mat curr_left = curr->left_.clone();
        Mat curr_right = curr->right_.clone();
        Mat left_right;
        left_right.push_back(curr_left);
        left_right.push_back(curr_right);
        if (left_right.channels() == 1)
        {
            cvtColor(left_right, left_right, COLOR_GRAY2RGB);
        }
        putText(left_right, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID

        cv::Point2f start_pos, end_pos;
        for (auto ft : curr->features_)
        {
            int r = rand() % (255 + 1);
            int g = rand() % (255 + 1);
            int b = rand() % (255 + 1);
            start_pos = cv::Point2f(ft->x_, ft->y_);
            // 画出当前帧的特征点
            cv::circle(left_right, start_pos, 5, cv::Scalar(r, g, b), 1);
            // 画出左右目匹配
            if (ft->stereo_)
            {
                end_pos = cv::Point(ft->x_r_, ft->y_ + Camera::height_);
                cv::circle(left_right, end_pos, 5, cv::Scalar(r, g, b), 1);
                cv::line(left_right, start_pos, end_pos, cv::Scalar(r, g, b), 1);
            }
        }
        cv::imshow("LeftRight_Matched", left_right);
        cv::waitKey(0);
    }

    /**
 * @brief 画出光流或直接法跟踪到的上一帧的特征点在当前帧中的位置
 * 
 * @param img 
 * @param curr 
 * @param last 
 */
    void Viewer::displayCurrTrackPoints(Frame::Ptr curr, Frame::Ptr last)
    {
        if (curr.get() == nullptr || last.get() == nullptr || curr->features_tracked_.size() == 0)
            return;
        Mat curr_left = curr->left_.clone();
        if (curr_left.channels() == 1)
        {
            cvtColor(curr_left, curr_left, COLOR_GRAY2RGB);
        }
        putText(curr_left, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID

        int i = 0;
        for (auto kp : curr->features_tracked_)
        {
            if (kp != cv::Point2f(0, 0))
            {
                cv::Point2f start_pos = last->features_[i]->pt_;
                cv::Point2f end_pos = kp;
                cv::line(curr_left, start_pos, end_pos, cv::Scalar(0, 255, 0));
                cv::circle(curr_left, end_pos, 2, cv::Scalar(0, 250, 0), 2);
            }
            i++;
        }
        cv::imshow("Curr_Tracked", curr_left);
        cv::waitKey(0);
    }
    /**
 * @brief 显示前后帧匹配的点
 * 
 * @param img 
 * @param curr 
 * @param last 
 */
    void Viewer::displayCurrMatchPoints(bool match_last, Frame::Ptr curr, Frame::Ptr frame)
    {
        if (curr.get() == nullptr || frame.get() == nullptr || curr->id_ == 0 || curr->features_matched_.size() == 0)
            return;
        Mat frame_and_curr;
        frame_and_curr.push_back(frame->left_);
        frame_and_curr.push_back(curr->left_);
        if (frame_and_curr.channels() == 1)
        {
            cvtColor(frame_and_curr, frame_and_curr, COLOR_GRAY2RGB);
        }

        // // 画出frame帧特征点
        // for (int i = 0; i < frame->features_.size(); i++)
        // {
        //     cv::Point2f kpt(frame->features_[i]->pt_);
        //     cv::circle(frame_and_curr, kpt, 5, cv::Scalar(0, 255, 0));
        // }
        // // 画出当前帧特征点
        // for (int i = 0; i < curr->features_.size(); i++)
        // {
        //     cv::Point2f kpt(curr->features_[i]->pt_.x, curr->features_[i]->pt_.y + Camera::height_);
        //     cv::circle(frame_and_curr, kpt, 5, cv::Scalar(0, 255, 0));
        // }
        // 画出两帧匹配上的特征点
        for (int i = 0; i < curr->features_matched_.size(); i++)
        {
            if (curr->features_matched_[i] != Point2f(0, 0))
            {
                Point2f start_pos = frame->features_[i]->pt_;
                Point2f end_pos = curr->features_matched_[i];
                end_pos.y += Camera::height_;
                int r = rand() % (255 + 1);
                int g = rand() % (255 + 1);
                int b = rand() % (255 + 1);
                cv::line(frame_and_curr, start_pos, end_pos, cv::Scalar(r, g, b), 1);
                cv::circle(frame_and_curr, start_pos, 5, cv::Scalar(r, g, b));
                cv::circle(frame_and_curr, end_pos, 5, cv::Scalar(r, g, b));
            }
        }

        if (match_last)
        {
            cv::putText(frame_and_curr, "LastFrame", cv::Point2f(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255));
        }
        else
        {
            cv::putText(frame_and_curr, "KeyFrame", cv::Point2f(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255));
        }
        cv::putText(frame_and_curr, to_string(frame->id_), cv::Point2f(50, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255));
        cv::putText(frame_and_curr, "CurrFrame", cv::Point2f(50, 50 + Camera::height_), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255));
        cv::putText(frame_and_curr, to_string(curr->id_), cv::Point2f(50, 70 + Camera::height_), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255));

        cv::imshow("Curr_Matched", frame_and_curr);
        cv::waitKey(0);
    }

    /**
 * @brief 显示当前帧匹配上的局部地图点
 * 
 * @param curr 当前帧
 */
    void Viewer::displayCurrMatchMapPoints(Frame::Ptr curr)
    {
        if (curr.get() == nullptr || curr->id_ == 0)
            return;

        Mat curr_left = curr->left_.clone();
        if (curr_left.channels() == 1)
        {
            cvtColor(curr_left, curr_left, COLOR_GRAY2RGB);
        }
        putText(curr_left, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID

        for (int i = 0; i < curr->mpoints_matched_.size(); i++)
        {
            if (curr->mpoints_matched_[i] != nullptr)
            {
                cv::circle(curr_left, curr->features_[i]->pt_, 5, Scalar(0, 255, 0));
            }
        }

        cv::imshow("Curr_Matched_MapPoints", curr_left);
        cv::waitKey(0);
    }

} // namespace myslam