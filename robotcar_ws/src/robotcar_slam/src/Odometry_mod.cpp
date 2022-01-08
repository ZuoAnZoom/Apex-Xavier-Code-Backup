#include "myslam/Odometry_mod.h"

namespace myslam
{
    Odometry::CalcMethod Odometry::calc_method_ = Odometry::CalcMethod::FEATURES;
    Odometry::OdomState Odometry::curr_state_ = Odometry::OdomState::NOINIT;
    float Odometry::normalpose_max_t_ = 0;
    float Odometry::normalpose_max_R_ = 0;
    float Odometry::key_max_t_ = 0;
    float Odometry::key_max_R_ = 0;

    Odometry::Odometry()
        : num_lost_(0)
    {
        curr_.reset(new myslam::Frame);
        last_.reset(new myslam::Frame);
        ref_.reset(new myslam::Frame);

        direct_.reset(new myslam::DirectTracker);
    }

    Odometry::~Odometry() {}
    /**
 * @brief 里程计的主函数，对curr_操作
 * 对当前帧影像进行处理processCurr:当前帧左右目影像提取关键点、关键点去畸变、计算描述子、左右目匹配、计算特征点的相机坐标
 * 
 * 设置位姿估计的初值setPoseInit:根据GNSS、IMU观测数据或者常速运动模型来对当前帧位姿赋一个初值
 * 
 * 如果没有初始化，则需要初始化initOdometry：把第一帧影像作为关键帧插入到地图中，并将第一帧的所有特征点作为初始地图点加入到全局和局部地图中
 * 
 * 通过光流法计算一个初始位姿calcOpticalFlow：由于光流法计算的位姿精度不高，所以仅将计算结果作为一个初值，加速后续特征点的匹配以及优化收敛时间
 * 
 * 根据当前位姿初值选择特征匹配的方法chooseMethod：如果当前帧位姿距离参考关键帧较近（偏移较少），则与上一帧匹配matchLastFrame；若距离较远，则与参考帧匹配matchKeyFrame（纠偏）；若距离更远，则与地图点匹配matchLocalMap（进一步纠偏）
 * 
 * 当前帧与上一帧或参考关键帧或地图点匹配后，通过PnP估计位姿calcPnP
 * 
 * 对估计的位姿进行合理性检验isNormalPose
 * 
 * 判断当前帧是否是个关键帧isKeyFrame
 * 
 * 更新局部地图updateLocalMap：维持一个局部地图的滑动窗口,该窗口内有参考关键帧及最近N个关键帧对应的地图点。将curr_的特征点计算出世界坐标后加入到局部地图中，删除N帧之前的地图点
 * 
 * 输入:含有一帧影像left_和right_的Frame类,含有全局地图的Map类
 * 输出:计算出当前帧位姿的Frame
 * @param frame 
 * @param map 
 * @return true 
 * @return false 
*/
    void Odometry::trackFrame(Frame::Ptr frame, Map::Ptr local_map)
    {
        // 处理当前帧
        curr_ = frame;
        processCurr();

        // 根据状态作相应处理
        switch (curr_state_)
        {
        case NOINIT: // 初始化
        {
            if (initOdometry(local_map))
            {
                curr_state_ = OK;
                curr_->just_init_ = true;
                updateRefFrame();
            }
            break;
        }
        case OK: // 正常
        {
            setPoseInit();

            switch (calc_method_) // 前后帧匹配与位姿估计
            {
            case FEATURES: // 特征点法
            {
                // 匹配上一帧或关键帧
                if ((matchLastFrame() && calcPoseByPnP() && isNormalPose(true, curr_->T_wc_f_)) || (matchKeyFrame() && calcPoseByPnP() && isNormalPose(true, curr_->T_wc_f_)))
                {
                    curr_->is_good_ = true;
                    num_lost_ = 0;

                    SE3 match_T = curr_->T_wc_f_;
                    // 匹配局部地图点
                    if (matchLocalMap(local_map) && calcPoseByPnP() && isNormalPose(false, curr_->T_wc_f_, match_T))
                    {
                        curr_->T_wc_ = curr_->T_wc_f_;
                    }
                    else
                    {
                        curr_->T_wc_ = match_T;
                    }
                }
                else
                {
                    num_lost_++;
                    curr_->is_good_ = false;
                    curr_->T_wc_ = T_model_;
                    cout << "当前帧匹配失败！" << endl;
                }
                break;
            }
            case DIRECT: // 直接法
            {
                calcPoseByDirect();
                if (isNormalPose(true, curr_->T_wc_d_))
                {
                    num_lost_ = 0;
                    curr_->is_good_ = true;
                    curr_->T_wc_ = curr_->T_wc_d_;
                }
                else
                {
                    num_lost_++;
                    curr_->is_good_ = false;
                    curr_->T_wc_ = T_model_;
                    cout << "当前帧跟踪失败！" << endl;
                }
                break;
            }
            }
            break;
        }
        case LOST: // 丢失
        {
            if (relocateOdometry())
                num_lost_ = 0;
            curr_state_ = OK;
            break;
        }
        }
        // 判断是否跟踪失败
        if (num_lost_ > Param::max_lost_)
        {
            cout << "里程计跟踪失败！" << endl;
            // state_ = LOST;
        }

        // 计算当前帧特征点世界坐标
        calcFeaturesCoors();

        // 判断当前帧是否是关键帧
        if (isKeyFrame(local_map))
            updateRefFrame();

        // 更新上一帧
        updateLastFrame();

        return;
    }
    /**
 * @brief 位姿估计结果检验，防止位姿跳变
 * // TODO：需要设计更合理的位姿检验算法
 */
    bool Odometry::isNormalPose(bool cmp_model, const SE3 &cur_T, const SE3 &ref_T)
    {
        if (!has_motion_model_ && cmp_model)
            return true;

        float t_reltv, R_reltv;
        // 通过与匀速模型检验
        if (cmp_model)
            Func::calcReltvRt(T_model_, cur_T, R_reltv, t_reltv);
        // 与帧匹配位姿检验
        else
            Func::calcReltvRt(ref_T, cur_T, R_reltv, t_reltv);

        if (isnan(t_reltv) || isnan(R_reltv) || t_reltv > normalpose_max_t_ || R_reltv > normalpose_max_R_)
        {
            switch (match_method_)
            {
            case LAST_FRAME:
                cout << "跟踪上一帧估计的位姿结果有误！";
                break;
            case KEY_FRAME:
                cout << "跟踪关键帧估计的位姿结果有误！";
                break;
            case LOCAL_MAP:
                cout << "跟踪局部地图估计的位姿结果有误！";
                break;
            }
            cout << "位移差：" << t_reltv << " > " << normalpose_max_t_ << " 旋转差：" << R_reltv << " > " << normalpose_max_R_ << endl;
            return false;
        }
        else
            return true;
    }
    /**
 * @brief 关键帧判断
 * 输入:通过PnP算出位姿的curr_
 * 输出:curr_的is_key_
 * 不能在相机静止时疯狂插入关键帧，也不能相机走远时不插关键帧
 */
    bool Odometry::isKeyFrame(Map::Ptr local_map)
    {
        bool insert = false;
        if (curr_->id_ == 0)
        {
            insert = true;
        }
        else if (curr_->is_good_)
        {
            // 判断相机移动的距离是否过小
            // float t_c_r, t_c_l, R_c_r, R_c_l;
            // Func::calcReltvRt(curr_->T_cw_, ref_->T_cw_, R_c_r, t_c_r);
            // bool c_dist = t_c_r > key_max_t_;
            // bool c_rot = R_c_r > key_max_R_;

            // 判断当前帧与局部地图最近的关键帧的距离是否大于一定阈值
            float min_dist = 10000.0, min_rot = 10000.0;
            for (auto it = local_map->frames_.begin(); it != local_map->frames_.end(); it++)
            {
                float dist = (it->get()->T_wc_.translation() - curr_->T_wc_.translation()).norm();
                float rot = (it->get()->T_wc_.so3().inverse() * curr_->T_wc_.so3()).log().norm();
                if (dist < min_dist)
                {
                    min_dist = dist;
                }
                if (rot < min_rot)
                {
                    min_rot = rot;
                }
            }
            bool c_dist = min_dist > key_max_t_;
            bool c_rot = min_rot > key_max_R_;

            // 比上一关键帧过去了5帧
            bool c1 = (curr_->id_ - ref_->id_) > 5; /// 阈值

            // // 匹配的地图点少于参考帧的75%（参考帧的地图点与当前帧的地图点可能会交集不大）
            bool c2 = false;
            // if (ref_->id_ == 0)
            //     c2 = curr_->num_mpoints_ < (local_map->num_mpoints_ * 0.75);
            // else
            //     c2 = curr_->num_mpoints_ < (ref_->num_mpoints_ * 0.4);
            // cout << curr_->num_mpoints_ << "  " << ref_->num_mpoints_ * 0.4 << endl;

            // 计算当前帧匹配上的地图点数与可用特征点数之比
            float ratio = (float)curr_->num_mpoints_ / curr_->num_features_;
            // cout << ratio << endl;
            bool c3 = ratio < 0.5;

            if ((c_dist || c_rot) && (c1 || c2 || c3))
                insert = true;
        }

        if (insert)
        {
            cout << "当前帧是关键帧" << endl;
            return true;
        }
        else
            return false;
        // return true;
    }
    /**
 * @brief 判断是否已经丢失
 * 
 * @return true 丢失
 * @return false 未丢失
*/
    bool Odometry::isLost()
    {
    }
    /**
 * @brief 更新上一帧
 * 输入:curr_
 * 输出:让curr_赋值给last_
*/
    void Odometry::updateLastFrame()
    {
        // 如果是第一帧、未完成初始化、初始化刚完成都无法计算相对姿态
        if (curr_->id_ != 0 && curr_state_ != OdomState::NOINIT && !curr_->just_init_)
        {
            has_motion_model_ = true;
            // 计算这两帧的相对位移和相对旋转
            T_reltv_ = last_->T_wc_.inverse() * curr_->T_wc_;
        }

        last_ = curr_;
    }
    /**
 * @brief 以当前帧生成新的关键帧，并将参考帧设为该关键帧
 * 输入:curr_
 * 输出:让curr_赋值给ref_
*/
    void Odometry::updateRefFrame()
    {
        curr_->is_key_ = true;
        ref_ = curr_;
    }

    /**
 * @brief 初始化
 * 将第一帧双目影像的恢复深度后的特征点加入到局部地图中，更新参考帧
 * 输入:第一帧curr_
 * 输出:插入第一帧和第一帧的特征点的全局和局部地图
 * @param frame 当前帧
 * @param map 地图
*/
    bool Odometry::initOdometry(Map::Ptr local_map)
    {
        switch (Camera::Type_)
        {
        case Camera::eType::STEREO:
        case Camera::eType::RGBD:
        {
            if (curr_->id_ == 0)
            {
                curr_->is_good_ = true;
                for (auto ft : curr_->features_)
                {
                    ft->wldcoor_ = ft->camcoor_;
                }
            }
            else
            {
                // 其他帧根据一直传递下来的匀速模型计算的位姿计算特征点世界坐标
                curr_->calcFeaturesWldCoors();
            }
        }
        break;
        case Camera::eType::MONOCULAR:
        {
            if (curr_->id_ == 0)
                return false;

            // 当前帧与参考帧特征匹配
            vector<int> index; // 当前帧的第i个特征与参考帧的第几个特征对应
            if (!curr_->matchFeaturesForMonoInit(ref_, index, 100))
            {
                cout << "与上一帧匹配的特征点数量过少，无法初始化" << endl;
                return false;
            }
            cout << "与初始帧匹配上的特征点的数量：" << curr_->num_matched_ << endl;
            Mat img;
            img.push_back(ref_->left_);
            img.push_back(curr_->left_);
            cvtColor(img, img, COLOR_GRAY2BGR);
            putText(img, to_string(ref_->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1);                    // 显示当前帧ID
            putText(img, to_string(curr_->id_), cv::Point2i(10, 30 + Camera::height_), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID
            for (int i = 0; i < curr_->features_.size(); i++)
            {
                int idx = index[i];
                if (idx == -1)
                    continue;
                int r = rand() % (255 + 1);
                int g = rand() % (255 + 1);
                int b = rand() % (255 + 1);
                cv::Point2f start(ref_->features_[idx]->pt_);
                cv::Point2f end(curr_->features_[i]->x_, curr_->features_[i]->y_ + Camera::height_);
                cv::circle(img, start, 5, Scalar(b, g, r));
                cv::circle(img, end, 5, Scalar(b, g, r));
                cv::line(img, start, end, Scalar(b, g, r));
            }
            cv::imshow("img", img);
            cv::waitKey(0);

            // 计算本质矩阵E
            vector<cv::Point2f> pix_ref;
            vector<cv::Point2f> pix_cur;
            for (int i = 0; i < curr_->features_matched_.size(); i++)
            {
                cv::Point2f pt = curr_->features_matched_[i];
                if (pt != Point2f(0, 0))
                {
                    pix_ref.push_back(ref_->features_[i]->pt_);
                    pix_cur.push_back(pt);
                }
            }
            Mat inliers;
            Mat E = cv::findEssentialMat(pix_ref, pix_cur, Camera::cvK_, RANSAC, 0.999, 1.0, inliers);

            // 剔除误匹配
            int id = 0;
            for (int i = 0; i < curr_->features_matched_.size(); i++)
            {
                if (curr_->features_matched_[i] != Point2f(0, 0))
                {
                    if (inliers.ptr<int>(id)[0] == 0)
                    {
                        curr_->features_matched_[i] = Point2f(0, 0);
                        curr_->num_matched_--;
                    }
                    id++;
                }
            }
            cout << curr_->num_matched_ << endl;

            // 从本质矩阵恢复位姿
            vector<Vec3> pts_3D; // 与参考帧特征对应
            if (!curr_->reconstructPose(ref_, E, pts_3D))
            {
                cout << "暂无法从当前帧恢复位姿" << endl;
                return false;
            }

            // 计算特征属性：世界坐标、相机坐标、视角余弦
            curr_->T_wc_ = curr_->T_truth_; ///////////暂时使用真值位姿
            cout << curr_->T_wc_.matrix3x4() << endl;
            Mat34 P_ref = Camera::K_ * ref_->T_wc_.inverse().matrix3x4();
            Mat34 P_cur = Camera::K_ * curr_->T_wc_.inverse().matrix3x4();
            for (int i = 0; i < curr_->features_.size(); i++)
            {
                if (index[i] != -1)
                {

                    // 三角化点的世界坐标
                    Vec3 coor;
                    Func::triangulatePoint(P_ref, P_cur, ref_->features_[index[i]]->pt_, curr_->features_[i]->pt_, coor);
                    if (coor(2) < 0)
                    {
                        curr_->features_matched_[index[i]] = Point2f(0, 0);
                        index[i] = -1;
                        continue;
                    }

                    curr_->features_[i]->wldcoor_ = coor;
                    cout << curr_->features_[i]->wldcoor_ << endl;

                    // 计算相机坐标
                    curr_->features_[i]->camcoor_ = Transform::world2camera(coor, curr_->T_wc_);

                    curr_->features_[i]->stereo_ = true;

                    // 计算视角余弦
                    Vec3 v_last = curr_->features_[i]->wldcoor_;
                    Vec3 v_curr = curr_->features_[i]->wldcoor_ - curr_->T_wc_.translation();
                    curr_->features_[i]->view_cos_ = v_last.dot(v_curr) / (v_last.norm() * v_curr.norm());
                }
            }

            // // 尺度归一化
            // vector<float> depth;
            // for (int i = 0; i < pts_3D.size(); i++)
            // {
            //     if (pts_3D[i] == Vec3(0, 0, 0))
            //         continue;
            //     depth.push_back(pts_3D[i](2));
            // }
            // sort(depth.begin(), depth.end());
            // float medianDepth = depth[depth.size() / 2]; // 取点云深度的中值

            // curr_->T_cw_.translation() /= medianDepth; // 平移
            // for (int i = 0; i < pts_3D.size(); i++)
            // {
            //     if (pts_3D[i] == Vec3(0, 0, 0))
            //         continue;
            //     curr_->features_[i]->stereo_ = true;
            //     curr_->features_[i]->wldcoor_ = (pts_3D[i] / medianDepth); // 点坐标

            //     // 计算视角余弦
            //     Vec3 v_last = curr_->features_[i]->wldcoor_;
            //     Vec3 v_curr = curr_->features_[i]->wldcoor_ - curr_->T_cw_.translation();
            //     curr_->features_[i]->view_cos_ = v_last.dot(v_curr) / (v_last.norm() * v_curr.norm());
            // }
        }
        break;
        }

        return true;
    }

    /**
 * @brief 对当前帧curr_图象进行处理(提取特征点,特征点去畸变,计算描述子,左右目特征点匹配)
 * 输入:当前帧curr_
 * 输出:处理后的当前帧curr_
*/
    void Odometry::processCurr()
    {
        // 转换为灰度图
        if (curr_->left_.channels() == 3)
        {
            cvtColor(curr_->left_, curr_->left_, COLOR_RGB2GRAY);
        }
        if (Camera::Type_ == Camera::eType::STEREO)
        {
            if (curr_->right_.channels() == 3)
            {
                cvtColor(curr_->right_, curr_->right_, COLOR_RGB2GRAY);
            }
        }

        curr_->extractKeyPoints();
        curr_->calcDescriptors();

        if (Camera::Type_ == Camera::eType::STEREO)
            curr_->matchFromeStereo();
        else if (Camera::Type_ == Camera::eType::RGBD)
            curr_->matchFromRGBD();

        curr_->undistKeyPoints();

        curr_->createFeatures();

        // 初始化匹配地图点
        curr_->mpoints_matched_ = vector<MapPoint::Ptr>(curr_->features_.size(), nullptr);

        cout << "提取的特征点的数量：" << curr_->features_.size() << endl;
        cout << "可用特征点数量：" << curr_->num_features_ << endl;
    }

    /**
 * @brief 设置位姿估计的初值（第一帧、常速运动、上一帧或惯导积分、GNSS定位）
 * 在特征匹配之前,需要先将curr_的位姿赋一个初值,以加速特征匹配以及后续位姿优化的收敛时间
 * 如果是第一帧:可以考虑将GNSS的定位结果以及惯导初始对准的结果作为位姿初值
 * 如果是正常帧:可以考虑用常速运动模型,认为这两帧的位姿变化与上两帧的位姿变化相同,也可以考虑直接把上一帧的位姿作为当前帧的位姿初值,如果有惯导数据和GNSS数据,也可以加以利用
 * 输入:处理后的curr_
 * 输出:加上初值后的curr_
*/
    void Odometry::setPoseInit()
    {
        // 如果没有匀速模型计算出位姿，就用上一帧的位姿作为当前帧的初值
        if (!has_motion_model_)
        {
            switch (calc_method_)
            {
            case FEATURES:
            {
                curr_->T_wc_f_ = last_->T_wc_;
                break;
            }
            case DIRECT:
            {
                curr_->T_wc_d_ = last_->T_wc_;
                break;
            }
            }
            return;
        }

        // 其他帧则通过匀速模型计算位姿初值
        T_model_ = last_->T_wc_ * T_reltv_;
        switch (calc_method_)
        {
        case FEATURES:
        {
            curr_->T_wc_f_ = T_model_;
            break;
        }
        case DIRECT:
        {
            curr_->T_wc_d_ = T_model_;
            break;
        }
        }
    }

    /**
 * @brief 重定位
 * 输入:处理后的当前帧curr_和全局地图
 * 输出:估计出位姿的当前帧curr_
 * @return true 
 * @return false 
*/
    bool Odometry::relocateOdometry()
    {
        curr_->T_wc_ = last_->T_wc_;
        return true;
    }

    /**
     * @brief 计算特征点相机坐标和世界坐标
     * 
     */
    void Odometry::calcFeaturesCoors()
    {
        switch (Camera::Type_)
        {
        case Camera::eType::MONOCULAR:
        {
            if (curr_state_ == Odometry::OdomState::NOINIT || ref_->id_ == curr_->id_) // 还未完成初始化或刚初始化完
                return;

            // 与上一帧极线特征匹配
            vector<int> index; // 上一帧的第i个特征点对应当前帧第几个
            curr_->matchFeaturesForTrian(last_, index);
            cout << "与上一帧三角化匹配上的特征点数量：" << curr_->num_matched_ << endl;

            Mat img;
            img.push_back(ref_->left_);
            img.push_back(curr_->left_);
            cvtColor(img, img, COLOR_GRAY2BGR);
            putText(img, to_string(ref_->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1);                    // 显示当前帧ID
            putText(img, to_string(curr_->id_), cv::Point2i(10, 30 + Camera::height_), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID
            for (int i = 0; i < curr_->features_matched_.size(); i++)
            {
                if (curr_->features_matched_[i] != cv::Point2f(0, 0))
                {
                    int r = rand() % (255 + 1);
                    int g = rand() % (255 + 1);
                    int b = rand() % (255 + 1);
                    cv::Point2f start(ref_->features_[i]->pt_);
                    // cv::Point2f end(curr_->features_matched_[i].x, curr_->features_matched_[i].y + Camera::height_);
                    cv::Point2f end(curr_->features_[index[i]]->x_, curr_->features_[index[i]]->y_ + Camera::height_);
                    cv::circle(img, start, 5, Scalar(b, g, r));
                    cv::circle(img, end, 5, Scalar(b, g, r));
                    cv::line(img, start, end, Scalar(b, g, r));
                }
            }
            cv::imshow("img", img);
            cv::waitKey(0);

            // 计算两帧的投影矩阵
            Mat34 P_last = Camera::K_ * last_->T_wc_.inverse().matrix3x4();
            Mat34 P_curr = Camera::K_ * curr_->T_wc_.inverse().matrix3x4();

            // 三角化特征点
            for (int i = 0; i < curr_->features_matched_.size(); i++)
            {
                if (curr_->features_matched_[i] != cv::Point2f(0, 0))
                {
                    Func::triangulatePoint(P_last, P_curr, last_->features_[i]->pt_, curr_->features_matched_[i], curr_->features_[index[i]]->wldcoor_);

                    curr_->features_[index[i]]->camcoor_ = Transform::world2camera(curr_->features_[index[i]]->wldcoor_, curr_->T_wc_);
                }
            }
            break;
        }
        case Camera::eType::STEREO:
        case Camera::eType::RGBD:
        {
            curr_->calcFeaturesCamCoors();
            curr_->calcFeaturesWldCoors();
            break;
        }
        }
        // 单目与上一帧极线特征匹配
    }

    /**
 * @brief 匹配上一帧
 * 输入:curr_的Mat descriptors_和上一帧的Mat descriptors_
 * 输出:vector<cv::DMatch> matchs
 * 
*/
    bool Odometry::matchLastFrame()
    {
        match_method_ = LAST_FRAME;
        curr_->match_last_ = true;
        curr_->matchFeatures(last_, T_model_, 7);
        cout << "与上一帧匹配上的特征点的数量： " << curr_->num_matched_ << endl;

        // Mat img;
        // img.push_back(last_->left_);
        // img.push_back(curr_->left_);
        // cvtColor(img, img, COLOR_GRAY2BGR);
        // putText(img, to_string(last_->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1);                   // 显示当前帧ID
        // putText(img, to_string(curr_->id_), cv::Point2i(10, 30 + Camera::height_), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID
        // for (int i = 0; i < curr_->features_matched_.size(); i++)
        // {
        //     if (curr_->features_matched_[i] == Point2f(0, 0))
        //         continue;
        //     int r = rand() % (255 + 1);
        //     int g = rand() % (255 + 1);
        //     int b = rand() % (255 + 1);
        //     cv::Point2f start(last_->features_[i]->pt_);
        //     cv::Point2f end(curr_->features_matched_[i].x, curr_->features_matched_[i].y + Camera::height_);
        //     cv::circle(img, start, 5, Scalar(b, g, r));
        //     cv::circle(img, end, 5, Scalar(b, g, r));
        //     cv::line(img, start, end, Scalar(b, g, r));
        // }
        // cv::imshow("img", img);
        // cv::waitKey(0);
        return true;
    }
    /**
 * @brief 匹配关键帧
 * 输入:curr_的Mat descriptors_和参考关键帧的Mat descriptors_
 * 输出:vector<cv::DMatch> matchs赋值
 *
*/
    bool Odometry::matchKeyFrame()
    {
        match_method_ = KEY_FRAME;
        curr_->match_last_ = false;
        curr_->matchFeatures(ref_, T_model_, 7);
        cout << "与关键帧匹配上的特征点的数量： " << curr_->num_matched_ << endl;
        return true;
    }
    /**
 * @brief 匹配局部地图
 * 根据局部地图点的最佳描述子和观测方向来加速匹配
 * 输入:curr_的Mat descriptors_和局部地图中所有地图点的Mat mpoints_descrips_
 * 输出:vector<MapPoint *> map_points_赋值
 * 
*/
    bool Odometry::matchLocalMap(Map::Ptr local_map)
    {
        match_method_ = LOCAL_MAP;
        int th;
        if (Camera::eType::RGBD)
            th = 3;
        else
            th = 1;

        if (curr_->matchMapPoints(local_map->map_points_, th))
        {
            cout << "与局部地图匹配上的地图点的数量：" << curr_->num_mpoints_ << endl;
            return true;
        }
        else
        {
            cout << "当前帧匹配上的局部地图点数量过少" << endl;
            return false;
        }
    }

    /**
 * @brief 直接法通过上一帧特征点估计两帧相对位姿
 * 
 */
    void Odometry::calcPoseByDirect()
    {
        VecVector2d pixels_ref_left;
        vector<double> depth_ref_left;

        /* 使用上一帧恢复出深度的特征点*/
        // for (int i = 0; i < last_->keypoints_l_.size(); i++)
        for (auto ft : last_->features_)
        {
            if (!ft->stereo_)
                continue;
            // 左目
            pixels_ref_left.push_back(Vec2(ft->x_, ft->y_));
            depth_ref_left.push_back(ft->camcoor_[2]);
        }

        /* 使用直接法计算相对位姿并跟踪上一帧特征点*/
        float camK[4] = {Camera::fx_, Camera::fy_, Camera::cx_, Camera::cy_};
        // SE3 T_relative_left = SE3(R_reltv_, t_reltv_).inverse();
        SE3 T_relative_left = curr_->T_wc_d_.inverse() * last_->T_wc_;
        vector<cv::Point2f> features_tracked;

        direct_->trackMultiLayer(camK, last_->left_, curr_->left_,
                                 pixels_ref_left, depth_ref_left, T_relative_left, features_tracked);

        /* 当前帧跟踪结果与上一帧特征点对应*/
        curr_->num_tracked_ = 0;
        int j = 0;
        curr_->features_tracked_.reserve(last_->features_.size());
        for (int i = 0; i < last_->features_.size(); i++)
        {
            if (!last_->features_[i]->stereo_)
            {
                curr_->features_tracked_[i] = cv::Point2f(0, 0);
            }
            else
            {
                curr_->features_tracked_[i] = cv::Point2f(features_tracked[j].x, features_tracked[j].y);
                curr_->num_tracked_++;
                j++;
            }
        }

        /* 更新当前帧位姿*/
        curr_->T_wc_d_ = last_->T_wc_ * T_relative_left.inverse();
        cout << "当前帧直接法与上一帧跟踪上的特征点的数量：" << curr_->num_tracked_ << endl;
    }

    /**
 * @brief PnP估计当前帧位姿并剔除误匹配
 * 输入:已知位姿的Frame中的vector<cv::Point3f> features_wldcoors_成员，curr_的匹配的特征点的像素坐标
 * 输出:SE3 T_cw_f
*/
    bool Odometry::calcPoseByPnP()
    {
        /*
  // 落在目标框内的匹配视为动态匹配，加以剔除
  // 取出静态匹配的3d点和2d点
  int num_static_match = 0; // 静态匹配的数量
  vector<Point3f> points_3d;
  vector<Point2f> pixels_2d;
  Frame::Ptr fm;
  if (match_method_ == LAST_FRAME)
  {
    fm = last_;
  }
  else
  {
    fm = ref_;
  }
  switch (match_method_)
  {
  case LAST_FRAME:
  case KEY_FRAME:
  {
    int i = 0;
    for (auto fm_ft : fm->features_)
    {
      if (curr_->features_matched_[i] == Point2f(0, 0)) // 未匹配上
      {
        curr_->static_match_.push_back(false);
        i++;
        continue;
      }
      float cur_x = curr_->features_matched_[i].x;
      float cur_y = curr_->features_matched_[i].y;
      // 如果在上一帧是静点，当前帧也是静点，则加入解算队列
      if (!fm_ft->dyna_ && !curr_->mask_.ptr<unsigned char>((int)cur_y)[(int)cur_x]) // 静态匹配
      {
        num_static_match++;
        curr_->static_match_.push_back(true);
        Vec3 fm_3d = fm_ft->camcoor_;
        points_3d.push_back(Point3f(fm_3d(0, 0), fm_3d(1, 0), fm_3d(2, 0)));
        pixels_2d.push_back(Point2f(cur_x, cur_y));
      }
      else // 动态匹配
      {
        curr_->static_match_.push_back(false);
      }
      i++;
    }
    if (num_static_match < 15)
    {
      cout << "与上一帧的静态匹配数量过少，无法通过PnP计算位姿" << endl;
      return false;
    }
    break;
  }
  case LOCAL_MAP:
  {
    if (curr_->num_mpoints_ < 15)
    {
      cout << "地图点匹配数量过少，无法通过PnP计算位姿" << endl;
      return false;
    }
    int i = 0;
    for (auto mp : curr_->mpoints_matched_)
    {
      if (mp != nullptr)
      {
        Vec3 map_3d = mp->coor_;
        points_3d.push_back(Point3f(map_3d(0, 0), map_3d(1, 0), map_3d(2, 0)));
        pixels_2d.push_back(curr_->features_[i]->pt_);
      }
      i++;
    }
    break;
  }
  }
  */
        switch (match_method_)
        {
        case LAST_FRAME:
        {
            if (curr_->num_matched_ < 15)
            {
                cout << "与上一帧匹配的特征点数量过少，无法计算位姿" << endl;
                return false;
            }
            break;
        }
        case KEY_FRAME:
        {
            if (curr_->num_matched_ < 15)
            {
                cout << "与关键帧匹配的特征点数量过少，无法计算位姿" << endl;
                return false;
            }
            break;
        }
        case LOCAL_MAP:
        {
            if (curr_->num_mpoints_ < 15)
            {
                cout << "与地图点匹配数量过少，无法计算位姿" << endl;
                return false;
            }
            break;
        }
        }

        // 取出匹配上的3d点和当前帧的2d点
        vector<Point3f> points_3d;
        vector<Point2f> pixels_2d;
        switch (match_method_)
        {
        case LAST_FRAME:
        {
            int i = 0;
            for (auto pt : curr_->features_matched_)
            {
                if (pt != Point2f(0, 0))
                {
                    Vec3 last_3d = last_->features_[i]->camcoor_;
                    points_3d.push_back(Point3f(last_3d(0, 0), last_3d(1, 0), last_3d(2, 0)));
                    pixels_2d.push_back(pt);
                }
                i++;
            }
            break;
        }
        case KEY_FRAME:
        {
            int i = 0;
            for (auto pt : curr_->features_matched_)
            {
                if (pt != Point2f(0, 0))
                {
                    Vec3 ref_3d = ref_->features_[i]->camcoor_;
                    points_3d.push_back(Point3f(ref_3d(0, 0), ref_3d(1, 0), ref_3d(2, 0)));
                    pixels_2d.push_back(pt);
                }
                i++;
            }
            break;
        }
        case LOCAL_MAP:
        {
            int i = 0;
            for (auto mp : curr_->mpoints_matched_)
            {
                if (mp != nullptr)
                {
                    Vec3 map_3d = mp->coor_;
                    points_3d.push_back(Point3f(map_3d(0, 0), map_3d(1, 0), map_3d(2, 0)));
                    pixels_2d.push_back(curr_->features_[i]->pt_);
                }
                i++;
            }
            break;
        }
        }

        // 计算位姿
        Mat K, r, t, R, inliers;
        Mat33 R_eg;
        Vec3 t_eg;

        // cv::eigen2cv(Camera::K_, K);
        // cv::eigen2cv(T_model_.translation(), t);
        // cv::eigen2cv(T_model_.rotationMatrix(), r);
        // cv::Rodrigues(r, r);

        if (!cv::solvePnPRansac(points_3d, pixels_2d, Camera::cvK_, Mat(), r, t, false, 100, 4.0, 0.98999, inliers))
        {
            cout << "特征匹配数量过少，无法计算位姿" << endl;
            return false;
        }
        cv::Rodrigues(r, R);
        cv::cv2eigen(R, R_eg);
        cv::cv2eigen(t, t_eg);

        // 更新当前帧位姿
        switch (match_method_)
        {
        case LAST_FRAME:
        {
            curr_->T_wc_f_ = last_->T_wc_ * SE3(R_eg, t_eg).inverse();
            break;
        }
        case KEY_FRAME:
        {
            curr_->T_wc_f_ = ref_->T_wc_ * SE3(R_eg, t_eg).inverse();
            break;
        }
        case LOCAL_MAP:
        {
            curr_->T_wc_f_ = SE3(R_eg, t_eg).inverse();
            break;
        }
        }
        // cout << curr_->T_cw_f_.matrix3x4() << endl;
        // 剔除误匹配
        eraseOutliers(inliers);

        return true;
    }

    /**
 * @brief 剔除RANSACPnP计算中的误匹配
 * solvePnPRansac会返回一个Mat(inliers),该Mat中存放着所有匹配内点序号,需要从中找出外点序号
 *
 * @param org_size 进行解算的原始点对数量
 * @param inliers solvePnPRansac返回的内点矩阵
 */
    void Odometry::eraseOutliers(const Mat &inliers)
    {
        int j = 0;
        int count = 0;
        switch (match_method_)
        {
        case LAST_FRAME:
        case KEY_FRAME:
        {
            for (int i = 0; i < curr_->features_matched_.size(); i++)
            {
                if (curr_->features_matched_[i] != Point2f(0, 0))
                {
                    if (j >= inliers.rows || count != inliers.ptr<int>(j)[0])
                    {
                        curr_->features_matched_[i] = Point2f(0, 0);
                        curr_->num_matched_--;
                    }
                    else
                        j++;
                    count++;
                }
            }
            break;
        }
        case LOCAL_MAP:
        {
            for (int i = 0; i < curr_->mpoints_matched_.size(); i++)
            {
                if (curr_->mpoints_matched_[i] != nullptr)
                {
                    if (j >= inliers.rows || count != inliers.ptr<int>(j)[0])
                    {
                        curr_->mpoints_matched_[i] = nullptr;
                        curr_->num_mpoints_--;
                    }
                    else
                        j++;
                    count++;
                }
            }
            break;
        }
        }
        return;
    }
} // namespace myslam
