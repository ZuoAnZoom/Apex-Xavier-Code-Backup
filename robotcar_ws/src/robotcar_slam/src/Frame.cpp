#include "myslam/Frame.h"

namespace myslam
{
    int Frame::factory_id_ = 0;

    float Frame::min_depth_ = 0;
    float Frame::max_depth_ = 0;

    Frame::Frame(int id)
        : id_(id),
          time_stamp_(-1), num_features_(0), num_matched_(0), num_mpoints_(0), num_tracked_(0)
    {
        mask_ = Mat::zeros(Camera::height_, Camera::width_, CV_8UC1);
        orbleft_.reset(new myslam::ORBextractor);
        orbright_.reset(new myslam::ORBextractor);
    }

    Frame::Ptr Frame::createFrame()
    {
        return Frame::Ptr(new Frame(factory_id_++));
    }

    /**
 * @brief 对一帧左右目图像提取关键点
 * 输入:Mat left_, right_;
 * 输出:vector<cv::KeyPoint> keypoints_l_, keypoints_r_
 */
    void Frame::extractKeyPoints()
    {
        // 提取ORB特征点
        // TODO: 将描述子的计算部分移到calcDescriptors()函数里
        thread orbleft(&ORBextractor::extractORB, orbleft_, left_, std::ref(keypoints_l_), std::ref(descriptors_l_));
        thread orbright(&ORBextractor::extractORB, orbright_, right_, std::ref(keypoints_r_), std::ref(descriptors_r_));
        orbleft.join();
        orbright.join();

        // 图像金字塔，只保存两层（MotionEstimator中用到）
        scimg_ = orbleft_->mvImagePyramid[1];

        // 把特征点分配到网格中以加速匹配
        ORBextractor::AssignfeatoGrid(keypoints_l_, orbleft_);
    }
    /**
 * @brief 关键点去畸变
 * 该函数只在单目相机或RGBD相机会用到，双目相机已经对每张图像校正过了
 * 输入:vector<cv::KeyPoint> keypoints_l_和Camera类中的D_成员
 * 输出:去畸变后的vector<cv::KeyPoint> keypoints_l_
 * 
*/
    void Frame::undistKeyPoints()
    {
        if (!Camera::k1_)
        {
            return;
        }

        // 构建关键点坐标数组
        int num_kp = keypoints_l_.size();
        cv::Mat kpts(num_kp, 2, CV_32F);
        for (int i = 0; i < num_kp; i++)
        {
            kpts.ptr<float>(i)[0] = keypoints_l_[i].pt.x;
            kpts.ptr<float>(i)[1] = keypoints_l_[i].pt.y;
        }

        // 计算去畸变后的关键点坐标
        cv::undistortPoints(kpts, kpts, Camera::cvK_, Camera::D_, cv::Mat(), Camera::cvK_);

        // 坐标赋值
        for (int i = 0; i < num_kp; i++)
        {
            keypoints_l_[i].pt.x = kpts.ptr<float>(i)[0];
            keypoints_l_[i].pt.y = kpts.ptr<float>(i)[1];
        }
    }
    /**
 * @brief 计算描述子
 * 输入:去畸变后的vector<cv::KeyPoint> keypoints_l_, keypoints_r_
 * 输出:Mat descriptors_l_和descriptors_r_
 */
    void Frame::calcDescriptors()
    {
        // ORBfeatureLeft->calcDescriptors(keypoints_l_, descriptors_l_);
        // ORBfeatureRight->calcDescriptors(keypoints_r_, descriptors_r_);
    }

    /**
 * @brief 在深度图中读取对应特征点的深度
 * 注意因未对深度图去畸变，所以特征点的坐标使用去除畸变前的坐标
 */
    void Frame::matchFromRGBD()
    {
        keypts_depth_.reserve(keypoints_l_.size());
        for (int i = 0; i < keypoints_l_.size(); i++)
        {
            cv::KeyPoint kp = keypoints_l_[i];
            int x = std::round(kp.pt.x), y = std::round(kp.pt.y);
            float depth = depth_.ptr<float>(y)[x];
            // float depth = Func::getPixelValue(depth_, kp.pt.x, kp.pt.y);
            keypts_depth_[i] = depth;
        }
    }

    /**
 * @brief 双目相机左右目特征点匹配
 * 输入:Mat descriptors_l_和descriptors_r_
 * 输出:vector<double> left_to_right_
*/
    void Frame::matchFromeStereo()
    {
        matchLeftRight(keypoints_l_, keypoints_r_, left_to_right_, descriptors_l_, descriptors_r_, orbleft_, orbright_); //// 有没有防止右目点在左目点右边的
    }

    /**
 * @brief 生成当前帧的特征点
 * 
 */
    void Frame::createFeatures()
    {
        num_features_ = 0;
        features_.reserve(keypoints_l_.size());
        for (int i = 0; i < keypoints_l_.size(); i++)
        {
            Feature::Ptr ft(new Feature);
            // 坐标、描述子
            ft->x_ = keypoints_l_[i].pt.x;
            ft->y_ = keypoints_l_[i].pt.y;
            ft->pt_ = cv::Point2f(ft->x_, ft->y_);
            ft->octave_ = keypoints_l_[i].octave;
            ft->response_ = keypoints_l_[i].response;
            ft->descriptor_ = descriptors_l_.row(i);

            // 视差
            switch (Camera::Type_)
            {
            case Camera::eType::STEREO:
            {
                if (left_to_right_[i] == 0)
                {
                    ft->stereo_ = false;
                }
                else
                {
                    ft->stereo_ = true;
                    ft->x_r_ = left_to_right_[i];
                    ft->disparity_ = ft->x_ - ft->x_r_;
                    num_features_++;
                }
                break;
            }
            case Camera::eType::RGBD:
            {
                if (keypts_depth_[i] <= 0)
                {
                    ft->stereo_ = false;
                }
                else
                {
                    ft->x_r_ = ft->x_ - Camera::base_fx_ / keypts_depth_[i]; // RGBD相机bf哪来的
                    if (ft->x_r_ > 0)
                    {
                        ft->stereo_ = true;
                        ft->disparity_ = ft->x_ - ft->x_r_;
                        num_features_++;
                    }
                    else
                    {
                        ft->stereo_ = false;
                    }
                }
                break;
            }
            case Camera::eType::MONOCULAR:
            {
                num_features_++;
            }
            }

            // // 所属目标ID
            // int x = std::round(ft->x_), y = std::round(ft->y_);
            // int object_id = mask_.ptr<unsigned char>(y)[x]; // FIXME：偶尔会崩
            // if (object_id != 0)
            // {
            //     ft->object_id_ = object_id;
            //     // ft->dyna_ = true;
            // }
            features_.push_back(ft);
        }
    }

    /**
 * @brief 计算该帧图像中所有特征点的相机坐标
*/
    void Frame::calcFeaturesCamCoors()
    {
        int i = 0;
        for (auto ft : features_)
        {
            if (!ft->stereo_)
            {
                ft->camcoor_ = Vec3(0, 0, 0);
                ft->depth_ = 0;
            }
            else
            {
                switch (Camera::Type_)
                {
                case Camera::eType::STEREO:
                {
                    ft->depth_ = Camera::base_fx_ / ft->disparity_;
                    break;
                }
                case Camera::eType::RGBD:
                {
                    ft->depth_ = keypts_depth_[i];
                    break;
                }
                }

                if (ft->depth_ < min_depth_ || ft->depth_ > max_depth_) // 深度不合理
                {
                    ft->stereo_ = false;
                    ft->camcoor_ = Vec3(0, 0, 0);
                    num_features_--;
                }
                else
                {
                    ft->stereo_ = true;
                    Vec2 kp(ft->x_, ft->y_);
                    ft->camcoor_ = Transform::pixel2camera(Camera::K_, kp, ft->depth_);
                }
            }
            i++;
        }
    }
    /**
 * @brief 计算该帧图像中所有特征点的相世界坐标
*/
    void Frame::calcFeaturesWldCoors()
    {
        for (auto ft : features_)
        {
            if (!ft->stereo_)
            {
                ft->wldcoor_ = Vec3(0, 0, 0);
                continue;
            }
            else
            {
                ft->wldcoor_ = Transform::camera2world(ft->camcoor_, T_wc_);
            }
        }
    }

    /**
 * @brief 当前帧与上一帧或关键帧进行特征匹配
 * 上一帧中stereo_为true的特征与当前帧所有特征进行匹配
 * 关键帧中stereo_为true的特征与当前帧所有特征进行暴力匹配
 * 当前帧的初始位姿用匀速模型计算出的
 * @param frame 上一帧或关键帧
 * @param th 阈值
 */
    bool Frame::matchFeatures(Frame::Ptr frame, const SE3 &initPose, int th)
    {
        num_matched_ = 0;
        features_matched_.clear();
        features_matched_ = vector<cv::Point2f>(frame->features_.size(), Point2f(0, 0));

        if (matchFeaturesByProjection(frame, initPose, th))
            return true;
        else if (matchFeaturesByBruteForce(frame, initPose, th))
            return true;
        else
            return false;
        return true;
    }

    bool Frame::matchFeaturesForMonoInit(Frame::Ptr ref, vector<int> &index_cur, int thr)
    {

        features_matched_.clear();
        features_matched_ = vector<cv::Point2f>(ref->features_.size(), Point2f(0, 0));
        num_matched_ = 0;

        int N = ref->keypoints_l_.size();
        vector<cv::Point2f> fea_mat(N, Point2f(0, 0));
        vector<int> index_ref(N, -1);                  // 参考帧的第i个特征与当前帧的第几个特征对应
        index_cur = vector<int>(features_.size(), -1); // 当前帧的第i个特征与参考帧的第几个特征对应

        const float factor = ORBextractor::HISTO_LENGTH / 360.0f;
        vector<int> rotHist[ORBextractor::HISTO_LENGTH]; // 旋转直方图（检查旋转一致性）
        for (int i = 0; i < ORBextractor::HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        for (int i = 0; i < N; i++)
        {
            int nLastOctave = ref->keypoints_l_[i].octave;
            // if (nLastOctave > 0)
            //     continue;

            cv::KeyPoint kp = ref->keypoints_l_[i];
            vector<size_t> vIndices2 = orbleft_->GetFeaturesInArea(keypoints_l_, kp.pt.x, kp.pt.y, thr, nLastOctave, nLastOctave);

            if (vIndices2.empty())
                continue;

            const cv::Mat dMP = ref->descriptors_l_.row(i);

            // 遍历满足条件的特征点
            int bestDist = INT_MAX;
            int bestDist2 = INT_MAX;
            int bestIdx2 = -1;
            for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end(); vit != vend; vit++)
            {
                const size_t i2 = *vit;

                const cv::Mat &d = descriptors_l_.row(i2);

                const int dist = ORBextractor::DescriptorDistance(dMP, d);
                if (dist < bestDist)
                {
                    bestDist2 = bestDist;
                    bestDist = dist;
                    bestIdx2 = i2;
                }
                else if (dist < bestDist2)
                    bestDist2 = dist;
            }

            // 判断匹配点的相对旋转量和平均旋转量的差异，若较大则为错误匹配
            if (bestDist <= ORBextractor::TH_LOW)
            {
                if (bestDist < (float)bestDist2 * 0.6)
                {
                    if (fea_mat[i] != cv::Point2f(0, 0))
                        num_matched_--;
                    fea_mat[i] = keypoints_l_[bestIdx2].pt;
                    index_ref[i] = bestIdx2;
                    index_cur[bestIdx2] = i;
                    num_matched_++;

                    float rot = ref->keypoints_l_[i].angle - keypoints_l_[bestIdx2].angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = std::round(rot * factor);
                    if (bin == ORBextractor::HISTO_LENGTH)
                        bin = 0;
                    assert(bin >= 0 && bin < ORBextractor::HISTO_LENGTH);
                    rotHist[bin].push_back(bestIdx2);
                }
            }
        }

        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        ORBextractor::ComputeThreeMaxima(rotHist, ORBextractor::HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < ORBextractor::HISTO_LENGTH; i++)
        {
            if (i != ind1 && i != ind2 && i != ind3)
            {
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    for (int c = 0; c < N; c++) // TODO：与源码不同
                    {
                        if (fea_mat[c] == keypoints_l_[rotHist[i][j]].pt)
                        {
                            fea_mat[c] = Point2f(0, 0);
                            index_cur[index_ref[c]] = -1;
                            index_ref[c] = -1;
                            num_matched_--;
                        }
                    }
                }
            }
        }

        features_matched_ = fea_mat;

        // // 如果匹配数量过少，则返回false
        // if (num_matched_ < 0.1 * frame->num_features_)
        //     return false;
        // else
        //     return true;
        return true;
    }

    bool Frame::matchFeaturesForTrian(Frame::Ptr frame, vector<int> &index)
    {
        features_matched_.clear();
        features_matched_ = vector<cv::Point2f>(frame->features_.size(), Point2f(0, 0));
        num_matched_ = 0;

        // 计算当前帧的相机光心在frame帧投影的位置
        Vec3 cam_coor = Transform::world2camera(T_wc_.translation(), frame->T_wc_);
        Vec2 pix_coor = Transform::camera2pixel(Camera::K_, cam_coor);

        // 计算当前帧到frame的基础矩阵F
        Mat F_fc;
        Func::calcFundmental(T_wc_, frame->T_wc_, F_fc);

        // 计算当前帧的点在frame帧上的极线
        vector<cv::Point2f> pts_cur;
        for (int i = 0; i < features_.size(); i++)
        {
            if (mpoints_matched_[i] != nullptr)
                continue;
            pts_cur.push_back(features_[i]->pt_);
        }
        vector<cv::Vec<float, 3>> epilines;
        cv::computeCorrespondEpilines(pts_cur, 1, F_fc, epilines);

        // 光流法找到当前帧特征点在上一帧图像上的位置
        vector<cv::Point2f> pts_pre;
        vector<unsigned char> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(left_, frame->left_, pts_cur, pts_pre, status, err, cv::Size(10, 10), 3);

        // 计算跟踪误差的均值和方差

        // 遍历当前帧特征点，找到frame中对应的特征点，并判断是否符合极限约束
        int N = frame->features_.size();
        const float factor = ORBextractor::HISTO_LENGTH / 360.0f;
        vector<int> rotHist[ORBextractor::HISTO_LENGTH]; // 旋转直方图（检查旋转一致性）
        vector<cv::Point2f> fea_mat(N, Point2f(0, 0));
        index = vector<int>(N, -1); // frame帧的第i个特征点对应当前帧第几个
        int count = -1;
        for (int i = 0; i < features_.size(); i++)
        {
            if (mpoints_matched_[i] != nullptr)
                continue;

            count++;

            if (status[count] == 0 || !Func::inBorder(pts_pre[count]))
                continue;

            // 在跟踪位置附近找到frame帧的特征点
            vector<size_t> indices;
            int curr_octave = features_[i]->octave_;
            indices = frame->orbleft_->GetFeaturesInArea(frame->keypoints_l_, pts_pre[count].x, pts_pre[count].y, 5, curr_octave - 1, curr_octave + 1);
            if (indices.empty())
                continue;

            // 遍历找到的特征点，找出最佳特征点
            int min_dist = INT_MAX;
            int min_idx = -1;
            const cv::Mat &cur_d = descriptors_l_.row(i);
            for (int i = 0; i < indices.size(); i++)
            {
                const size_t idx = indices[i];

                const cv::Mat &pre_d = frame->descriptors_l_.row(idx);

                const int dist = ORBextractor::DescriptorDistance(cur_d, pre_d);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_idx = idx;
                }
            }

            // 判断最佳特征点是否满足极线约束
            if (min_dist <= ORBextractor::TH_HIGH) // TODO：如何选择TH_HIGH, TH_LOW
            {
                // 计算到极点的距离
                float d_ex = pix_coor(0) - pts_pre[count].x;
                float d_ey = pix_coor(1) - pts_pre[count].y;
                float d_e = d_ex * d_ex + d_ey * d_ey;
                int octave = frame->features_[min_idx]->octave_;
                if (d_e < 100 * ORBextractor::mvScaleFactor[octave])
                    continue;

                // 计算到极线的距离
                float A = epilines[count][0], B = epilines[count][1], C = epilines[count][2];
                float x = pts_pre[count].x, y = pts_pre[count].y;
                float d_sqr = (A * x + B * y + C) * (A * x + B * y + C) / (A * A + B * B);
                if (d_sqr > 3.84 * frame->features_[min_idx]->octave_)
                    continue;

                fea_mat[min_idx] = keypoints_l_[i].pt;
                index[min_idx] = i;

                float rot = keypoints_l_[i].angle - frame->keypoints_l_[min_idx].angle;
                if (rot < 0.0)
                    rot += 360.0f;
                int bin = std::round(rot * factor);
                if (bin == ORBextractor::HISTO_LENGTH)
                    bin = 0;
                assert(bin >= 0 && bin < ORBextractor::HISTO_LENGTH);
                rotHist[bin].push_back(min_idx);
            }
        }

        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        ORBextractor::ComputeThreeMaxima(rotHist, ORBextractor::HISTO_LENGTH, ind1, ind2, ind3);
        for (int i = 0; i < ORBextractor::HISTO_LENGTH; i++)
        {
            if (i != ind1 && i != ind2 && i != ind3)
            {
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    for (int c = 0; c < N; c++) // TODO：与源码不同
                    {
                        if (fea_mat[c] == keypoints_l_[rotHist[i][j]].pt)
                        {
                            fea_mat[c] = Point2f(0, 0);
                            index[c] = -1;
                        }
                    }
                }
            }
        }
        for (int i = 0; i < N; i++)
        {
            if (index[i] == -1)
            {
                features_matched_[i] = Point2f(0, 0);
                continue;
            }
            features_[index[i]]->stereo_ = true;
            features_matched_[i] = fea_mat[i];
            num_matched_++;
        }

        return true;
    }

    /**
 * @brief 将Frame帧的特征投影到当前帧进行匹配
 * 
 * @param frame 上一帧
 * @param initPose 
 * @param th 
 * @return true 
 * @return false 与第一帧匹配，或者匹配数量过少
 */
    bool Frame::matchFeaturesByProjection(Frame::Ptr frame, const SE3 &initPose, int th)
    {
        // 如果是第一帧，则直接暴力匹配
        if (frame->id_ == 0)
            return false;

        int N = frame->keypoints_l_.size();

        // 判断是前进还是后退
        bool isForward, isBackward;
        Func::judgeForOrBackward(T_wc_f_, frame->T_wc_, isForward, isBackward);

        // 将Frame中的点投影到当前帧
        vector<cv::Point2f> prjPos(N, cv::Point2f(0, 0));
        vector<float> invzc(N, 0);
        for (int i = 0; i < frame->features_.size(); i++)
        {
            // 如果上一帧的特征点匹配到了地图点，就将该地图点投影到当前帧
            // 否则就用上一帧双目三角化出的坐标
            Vec3 lwld, x3Dc;
            MapPoint::Ptr mp = frame->mpoints_matched_[i];
            if (mp != nullptr)
                lwld = mp->coor_;
            else if (frame->features_[i]->wldcoor_ != Vec3(0, 0, 0))
                lwld = frame->features_[i]->wldcoor_;
            else
                continue;
            x3Dc = Transform::world2camera(lwld, T_wc_f_);
            if (x3Dc(2) < 1.0)
                continue;

            invzc[i] = 1.0 / x3Dc(2);

            Vec2 pixeluv = Transform::camera2pixel(Camera::K_, x3Dc);
            float u = pixeluv(0);
            float v = pixeluv(1);
            if (!Func::inBorder(u, v))
                continue;

            prjPos[i] = cv::Point2f(u, v);

            // if (id_ == 5)
            // {
            //     Mat img;
            //     img.push_back(frame->left_);
            //     img.push_back(left_);
            //     cvtColor(img, img, COLOR_GRAY2BGR);
            //     cv::circle(img, frame->features_[i]->pt_, 5, Scalar(0, 255, 0));
            //     cv::circle(img, cv::Point2f(u, v + Camera::height_), 5, Scalar(0, 255, 0));
            //     cv::imshow("img", img);
            //     cv::waitKey(0);
            // }
        }

        // 与Frame帧特征匹配
        const float factor = ORBextractor::HISTO_LENGTH / 360.0f;
        vector<int> rotHist[ORBextractor::HISTO_LENGTH]; // 旋转直方图（检查旋转一致性）
        vector<cv::Point2f> fea_mat(N, Point2f(0, 0));
        vector<int> fea_order(N, -1);
        for (int i = 0; i < ORBextractor::HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        for (int idex = 0; idex < 2; idex++) // 两遍匹配，第二遍阈值调大
        {
            for (int i = 0; i < N; i++)
            {
                if (!frame->features_[i]->stereo_)
                {
                    continue;
                }
                if (fea_mat[i] != Point2f(0, 0))
                    continue;

                int nLastOctave = frame->keypoints_l_[i].octave;

                // Search in a window. Size depends on scale
                float radius = 0;
                if (idex == 0)
                    radius = th * orbleft_->mvScaleFactor[nLastOctave];
                else
                    radius = 2 * th * orbleft_->mvScaleFactor[nLastOctave];
                vector<size_t> vIndices2;

                // 根据前进还是后退在不同尺度上搜索特征点
                // NOTE 尺度越大,图像越小
                float u = prjPos[i].x, v = prjPos[i].y;
                if (prjPos[i] == Point2f(0, 0))
                    continue;
                if (isForward)
                {
                    vIndices2 = orbleft_->GetFeaturesInArea(keypoints_l_, u, v, radius, nLastOctave);
                }
                else if (isBackward)
                {
                    vIndices2 = orbleft_->GetFeaturesInArea(keypoints_l_, u, v, radius, 0, nLastOctave);
                }
                else
                {
                    vIndices2 = orbleft_->GetFeaturesInArea(keypoints_l_, u, v, radius, nLastOctave - 1, nLastOctave + 1);
                }

                if (vIndices2.empty())
                    continue;

                const cv::Mat dMP = frame->descriptors_l_.row(i);

                int bestDist = 256;
                int bestIdx2 = -1;

                // 遍历满足条件的特征点
                for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end();
                     vit != vend; vit++)
                {
                    const size_t i2 = *vit;
                    // if (!features_[i2]->stereo_)
                    //     continue;

                    if (features_[i2]->stereo_)
                    {
                        const float ur = u - Camera::base_fx_ * invzc[i];
                        const float er = fabs(ur - features_[i2]->x_r_);
                        if (er > radius)
                            continue;
                    }

                    const cv::Mat &d = descriptors_l_.row(i2);

                    const int dist = ORBextractor::DescriptorDistance(dMP, d);
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                if (bestDist <= ORBextractor::TH_HIGH)
                {
                    fea_mat[i] = keypoints_l_[bestIdx2].pt;
                    fea_order[i] = bestIdx2;
                    num_matched_++;

                    float rot = frame->keypoints_l_[i].angle - keypoints_l_[bestIdx2].angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = std::round(rot * factor);
                    if (bin == ORBextractor::HISTO_LENGTH)
                        bin = 0;
                    assert(bin >= 0 && bin < ORBextractor::HISTO_LENGTH);
                    rotHist[bin].push_back(bestIdx2);
                }
            }

            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;
            ORBextractor::ComputeThreeMaxima(rotHist, ORBextractor::HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < ORBextractor::HISTO_LENGTH; i++)
            {
                if (i != ind1 && i != ind2 && i != ind3)
                {
                    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                    {
                        for (int c = 0; c < N; c++) // TODO：与源码不同
                        {
                            if (fea_mat[c] == keypoints_l_[rotHist[i][j]].pt)
                            {
                                fea_mat[c] = Point2f(0, 0);
                                fea_order[c] = -1;
                                num_matched_--;
                            }
                        }
                    }
                }
            }
            if (num_matched_ >= 0.5 * frame->num_features_)
                break;
        }
        features_matched_ = fea_mat;

        // 如果匹配数量过少，则返回false
        if (num_matched_ < 0.5 * frame->num_features_)
            return false;
        else
            return true;
    }

    /**
 * @brief 与Frame帧的特征点进行暴力匹配
 * 当投影匹配数量过少时，或与第一帧匹配时，才会进行暴力匹配
 * 保留投影匹配，对投影匹配未匹配上的进行暴力匹配
 * @param frame 关键帧
 * @param initPose 
 * @param th 
 * @return true 
 * @return false 匹配数量过少
 */
    bool Frame::matchFeaturesByBruteForce(Frame::Ptr frame, const SE3 &initPose, int th)
    {
        /*特征点匹配*/
        vector<DMatch> matches;
        vector<vector<DMatch>> knnMatches;
        cv::Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming(2)"); // BF匹配
        // 取出frame中可以匹配的特征
        Mat descriptors_1;
        for (int i = 0; i < frame->features_.size(); i++)
        {
            if (!frame->features_[i]->stereo_)
                continue;
            else if (id_ != 1 && features_matched_[i] != Point2f(0, 0))
                continue;
            else // 取出可用点
            {
                descriptors_1.push_back(frame->features_[i]->descriptor_);
            }
        }
        if (!descriptors_1.rows)
        {
            return false;
        }

        // 匹配
        Mat descriptors_2 = descriptors_l_;
        matcher->match(descriptors_1, descriptors_2, matches); ////mask
        // matcher->knnMatch(descriptors_1, descriptors_2, knnMatches, 2); // knn匹配

        /*匹配点对筛选*/
        // knn匹配比率测试
        // vector<bool> mark;
        // for (size_t i = 0; i < knnMatches.size(); i++)
        // {
        //     const DMatch &bestMatch = knnMatches[i][0];
        //     const DMatch &betterMatch = knnMatches[i][1];

        //     float distanceRatio = bestMatch.distance / betterMatch.distance;
        //     if (distanceRatio < 1.f / 1.5f)
        //     {
        //         matches.push_back(bestMatch);
        //         mark.push_back(true);
        //     }
        //     else
        //     {
        //         mark.push_back(false);
        //     }
        // }

        // 计算最小距离
        float min_dis = std::min_element(
                            matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; })
                            ->distance;

        // 匹配结果进一步筛选
        int j = 0;
        int count = 0;
        features_matched_.reserve(frame->features_.size());
        for (int i = 0; i < frame->features_.size(); i++)
        {
            if (!frame->features_[i]->stereo_ || features_matched_[i] != Point2f(0, 0))
                continue;
            else
            {
                cv::DMatch &m = matches[j];
                int queryIdx = m.queryIdx; // frame帧
                int trainIdx = m.trainIdx; // 当前帧
                if (m.distance < max<float>(min_dis * 2, 30.0))
                {
                    count++;
                    num_matched_++;
                    features_matched_[i] = keypoints_l_[trainIdx].pt;
                }
                j++;
            }
        }

        // 如果匹配数量过少，则返回false
        if (num_matched_ < 0.5 * frame->num_features_)
            return false;
        else
            return true;
    }

    /**
 * @brief 当前帧与局部地图点进行匹配
 * 所有局部地图点与当前帧可用特征进行匹配
 * @param map_points 局部地图点
 * @param th 阈值
 * @return true 
 * @return false 
 */
    bool Frame::matchMapPoints(list<shared_ptr<MapPoint>> map_points, const float th)
    {
        num_mpoints_ = 0;
        int cN = keypoints_l_.size();
        vector<int> mpoints_bestdist(cN, 256);

        for (auto it = map_points.begin(); it != map_points.end(); it++)
        {
            MapPoint::Ptr pmp = *it;
            bool trackview = false;
            float projx = 0.0;
            float projy = 0.0;
            float projz = 0.0;
            int prelevel = -1;
            float viewcos = 0;

            // 判断地图点是否在当前帧视野
            if (!infrustum(pmp, projx, projy, projz, prelevel, viewcos))
                continue;
            pmp->visible_times_++;

            vector<size_t> vIndices;
            float r = viewcos > 0.998 ? 2.5 : 4.0; // RadiusByViewingCos
            r *= th;
            float radius = r * orbleft_->mvScaleFactor[prelevel];
            vIndices = orbleft_->GetFeaturesInArea(keypoints_l_, projx, projy, radius, prelevel - 1, prelevel);
            if (vIndices.empty())
                continue;

            const cv::Mat d_mp = pmp->best_descr_;

            int bestdist = 256;
            int bestlevel = -1;
            int bestdist2 = 256;
            int bestlevel2 = -1;
            int bestidx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
            {
                const size_t i2 = *vit;
                // if (!features_[i2]->stereo_)
                //     continue;
                if (i2 >= descriptors_l_.rows)
                    continue;

                float left_to_right = features_[i2]->x_r_;
                if (left_to_right > 0)
                {
                    const float ur = projx - Camera::base_fx_ / projz;
                    const float er = fabs(ur - left_to_right);
                    if (er > radius)
                        continue;
                }
                const cv::Mat &d_ft = descriptors_l_.row(i2);
                const int dist = ORBextractor::DescriptorDistance(d_mp, d_ft);

                // 记录最优匹配和次优匹配
                if (dist < bestdist)
                {
                    bestdist2 = bestdist;
                    bestdist = dist;
                    bestlevel2 = bestlevel;
                    bestlevel = keypoints_l_[i2].octave;
                    bestidx = i2;
                }
                else if (dist < bestdist2)
                {
                    bestlevel2 = keypoints_l_[i2].octave;
                    bestdist2 = dist;
                }
            }

            if (bestdist <= ORBextractor::TH_HIGH)
            {
                if (bestlevel == bestlevel2 && bestdist > 0.8 * bestdist2)
                    continue;
                if (bestdist <= mpoints_bestdist[bestidx])
                {
                    mpoints_bestdist[bestidx] = bestdist;
                    mpoints_matched_[bestidx] = pmp;
                }
            }
        }

        for (auto mp : mpoints_matched_)
        {
            if (mp != nullptr)
            {
                mp->matched_times_++;
                num_mpoints_++;
            }
        }
        return true;
    }

    bool Frame::infrustum(shared_ptr<MapPoint> pMP, float &projx, float &projy, float &projz, int &prelevel, float &viewcos)
    {
        //pMP->bTrackInView = false;
        Vec3 P = pMP->coor_;
        const Vec3 Pc = Transform::world2camera(P, T_wc_f_);
        projz = Pc(2);
        if (projz < 1.0)
            return false;
        const Vec2 pixeluv = Transform::camera2pixel(Camera::K_, Pc);
        float u = pixeluv(0);
        float v = pixeluv(1);

        // 将MapPoint投影到当前帧，判断是否在图像里
        if (!Func::inBorder(u, v))
            return false;
        const double maxdis = pMP->max_dist_;
        const double mindis = pMP->min_dist_;

        // 当前帧视角与该地图点的平均观测方向夹角，若大于60度则抛弃
        const Vec3 viewangl = pMP->mean_viewagl_;
        const Vec3 PO = P - T_wc_f_.translation();
        const double dist = PO.norm();
        const float cos = PO.dot(viewangl) / dist;
        if (cos < 0.5)
            return false;

        // 最远最近距离
        if (dist > maxdis || dist < mindis)
            return false;

        // 预测尺度
        const double ratio = maxdis / dist;
        int nScale = ceil(log(ratio) / log(Param::scale_));
        if (nScale < 0)
            nScale = 0;
        else if (nScale > Param::nlevels_ - 1)
            nScale = Param::nlevels_ - 1;

        prelevel = nScale;
        projx = u;
        projy = v;
        viewcos = cos;
        return true;
    }

    /**
     * @brief 从本质矩阵恢复位姿
     * 
     * @param frame 参考帧
     * @param E 本质矩阵
     * @param T 当前帧位姿
     * @return true 
     * @return false 
     */
    bool Frame::reconstructPose(Frame::Ptr frame, const Mat &E, vector<Vec3> &pts_3D)
    {
        // 分解本质矩阵
        cv::Mat R1, R2, t;
        cv::decomposeEssentialMat(E, R1, R2, t);
        cv::Mat t1 = t;
        cv::Mat t2 = -t;
        R1.convertTo(R1, CV_32FC1);
        R2.convertTo(R2, CV_32FC1);
        t1.convertTo(t1, CV_32FC1);
        t2.convertTo(t2, CV_32FC1);

        // 对每个Rt进行检验，顺便三角化匹配点
        vector<Vec3> pts_3D1, pts_3D2, pts_3D3, pts_3D4;
        pts_3D1.resize(features_matched_.size(), Vec3(0, 0, 0));
        pts_3D2.resize(features_matched_.size(), Vec3(0, 0, 0));
        pts_3D3.resize(features_matched_.size(), Vec3(0, 0, 0));
        pts_3D4.resize(features_matched_.size(), Vec3(0, 0, 0));
        float palx1, palx2, palx3, palx4;
        int good1 = checkRt(frame, R1, t1, pts_3D1, palx1);
        int good2 = checkRt(frame, R1, t2, pts_3D2, palx2);
        int good3 = checkRt(frame, R2, t1, pts_3D3, palx3);
        int good4 = checkRt(frame, R2, t2, pts_3D4, palx4);
        cout << good1 << "  " << good2 << "  " << good3 << "  " << good4 << endl;

        // 判断最优结果是否明显
        int max_good = max(good1, max(good2, max(good3, good4)));
        int min_good = max((int)(0.9 * num_matched_), 50);

        int similar = 0;
        if (good1 > 0.7 * max_good)
            similar++;
        if (good2 > 0.7 * max_good)
            similar++;
        if (good3 > 0.7 * max_good)
            similar++;
        if (good4 > 0.7 * max_good)
            similar++;

        if (max_good < min_good || similar > 1)
            return false;

        // 判断最优结果的视差是否较大
        if (max_good == good1 && palx1 > 1.0)
        {
            T_wc_ = Transform::cvT2SE3(R1, t1).inverse();
            pts_3D = pts_3D1;
            return true;
        }
        else if (max_good == good2 && palx2 > 1.0)
        {
            T_wc_ = Transform::cvT2SE3(R1, t2).inverse();
            pts_3D = pts_3D2;
            return true;
        }
        else if (max_good == good3 && palx3 > 1.0)
        {
            T_wc_ = Transform::cvT2SE3(R2, t1).inverse();
            pts_3D = pts_3D3;
            return true;
        }
        else if (max_good == good4 && palx4 > 1.0)
        {
            T_wc_ = Transform::cvT2SE3(R2, t2).inverse();
            pts_3D = pts_3D4;
            return true;
        }
        return false;
    }

    /**
     * @brief 单目初始化时判断Rt是否合理
     * 
     * @param frame 参考帧
     * @param R E分解出的一个R
     * @param t E分解出的一个t
     * @param pts_3D 三角化后的点坐标
     * @param parallax 视差中值
     * @return int 内点数
     */
    int Frame::checkRt(Frame::Ptr frame, const Mat &R, const Mat &t, vector<Vec3> &pts_3D, float &parallax)
    {
        // 恢复两个相机的投影矩阵和光心
        cv::Mat P1(3, 4, CV_32F, cv::Scalar(0)); // 相机1的投影矩阵
        Camera::cvK_.copyTo(P1.rowRange(0, 3).colRange(0, 3));

        cv::Mat P2(3, 4, CV_32F, cv::Scalar(0)); // 相机2的投影矩阵
        R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        t.copyTo(P2.rowRange(0, 3).col(3));
        P2 = Camera::cvK_ * P2;

        cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32FC1); // 相机1的光心的世界坐标
        cv::Mat O2 = -R.t() * t;                     // 相机2的光心的世界坐标（注意：R,t算的是1->2）

        vector<float> vCosPalx;
        int num_good = 0;
        for (int i = 0; i < features_matched_.size(); i++)
        {
            if (features_matched_[i] == cv::Point2f(0, 0))
                continue;

            cv::Point2f pt1(frame->features_[i]->pt_);
            cv::Point2f pt2(features_matched_[i].x, features_matched_[i].y);

            // 三角化恢复三维点
            Mat p3D1;
            Func::triangulatePoint(P1, P2, pt1, pt2, p3D1);
            if (!isfinite(p3D1.at<float>(0)) || !isfinite(p3D1.at<float>(1)) || !isfinite(p3D1.at<float>(2)))
                continue;

            // 计算视差角余弦
            Mat normal1 = p3D1 - O1;
            Mat normal2 = p3D1 - O2;
            float cosParlx = normal1.dot(normal2) / (cv::norm(normal1) * cv::norm(normal2));

            // 判断深度是否为正
            if (p3D1.at<float>(2) <= 0 && cosParlx < 0.99998)
                continue;

            Mat p3D2 = R * p3D1 + t;
            if (p3D2.at<float>(2) <= 0 && cosParlx < 0.99998)
                continue;

            // 判断重投影误差是否过大
            cv::Point2f pix1 = Transform::camera2pixel(Camera::cvK_, p3D1);
            float squareError1 = (pix1.x - pt1.x) * (pix1.x - pt1.x) + (pix1.y - pt1.y) * (pix1.y - pt1.y);

            if (squareError1 > 4.0 * 1.0)
                continue;

            cv::Point2f pix2 = Transform::camera2pixel(Camera::cvK_, p3D2);
            float squareError2 = (pix2.x - pt2.x) * (pix2.x - pt2.x) + (pix2.y - pt2.y) * (pix2.y - pt2.y);

            if (squareError2 > 4.0 * 1.0)
                continue;

            Vec3 p3D;
            cv::cv2eigen(p3D1, p3D);
            pts_3D[i] = p3D;
            vCosPalx.push_back(cosParlx);
            num_good++;
        }
        // 选择视差的中值
        if (num_good > 0)
        {
            sort(vCosPalx.begin(), vCosPalx.end());
            parallax = acos(vCosPalx[vCosPalx.size() / 2]) * 180 / PI;
        }
        else
            parallax = 0;
        return num_good;
    }
} // namespace myslam
