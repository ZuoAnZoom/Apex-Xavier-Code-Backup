#include "myslam/MotionClassifier_mod.h"

namespace myslam
{
    int MotionClassifier::min_area_ = 0;
    int MotionClassifier::max_area_ = 0;
    float MotionClassifier::dyna_epipolar_dis_ = 0;
    float MotionClassifier::dyna_ratio_ = 0;

    MotionClassifier::MotionClassifier()
    {
    }

    MotionClassifier::~MotionClassifier() {}

    /**
     * @brief 
     * 
     * @param frame 
     * @param curr 
     * @param local_map 
     */
    void MotionClassifier::detectDynaObjects(Frame::Ptr frame, Frame::Ptr curr, Map::Ptr local_map)
    {

        // 当前帧目标和局部地图目标进行匹配
        matchObject(local_map, curr);

        // 处理漏检
        processMissedDetection(local_map, curr);

        // 当前帧目标定位
        locateObject(curr);

        // 筛选动态物体
        selectDynaObjects(frame, curr, local_map);

        // 剔除当前帧中动态目标上的特征点
        rejectDynaObjectPts(curr);

        // 重新计算位姿
        recalcCurrPose(curr);
    }

    /**
 * @brief 筛选出当前帧上的动态目标
 * 主要方法：
 * １.几何约束：对在当前帧的每个目标上提取的角点，使用光流法找到在frame帧的对应位置，计算其到对应极线的距离，若距离大于阈值则视为动点。（极线约束）
 * 2.外观一致性假设：对在当前帧的每个目标上提取的角点，将其投影至frame帧，计算在两帧上的像素块的NCC或描述子，判断是否一致，若不一致则有可能是存在动态物体或者是视角改变导致的物体外观的变化
 * 3.目标位置一致性假设：通过当前帧目标与局部地图目标匹配得到地图目标的世界坐标，如果某个目标的位置发生较大变化则有可能是动态目标

 * 主要待解决问题：
 * 1. 漏检：某个目标一直存在视野中且能被检测到，但在当前帧未检测到时，需要有漏检补偿
 * 2. 误检：突然在该帧检测到一个新目标，但实际上并不是该类型的物体，需要将该目标从局部地图中剔除。漏检误检的解决都是为了提高视频序列时序一致性。漏检比误检更严重！
 * 3. 当物体沿极线运动时，前两种方法都不是很有效，使用方法三来弥补
 * 4. 部分阈值只适合行车环境，需要在各种场景下大量测试
 * 5. 只是检测了能够运动的物体，如车辆、行人，但是对于可以运动的物体没有检测，如椅子、书等，可以只针对行车环境来考虑，也可以加入IMU

 * @param curr 
 */
    void MotionClassifier::selectDynaObjects(Frame::Ptr frame, Frame::Ptr curr, Map::Ptr local_map)
    {
        if (curr->id_ == 0 || !curr->objects_.size())
            return;

        // 计算基础矩阵F
        cv::Mat F_cv;
        Func::calcFundmental(curr->T_wc_, frame->T_wc_, F_cv);

        // 将当前帧的目标中恢复出深度的角点投影到frame帧
        vector<cv::Point2f> cors_pre, cors_cur;
        prjPointsToFrame(curr->T_wc_, frame->T_wc_, curr->objects_, cors_pre, cors_cur);
        if (!cors_pre.size())
            return;

        // 通过光流找到当前帧角点在frame帧中的位置
        vector<cv::Point2f> pts_pre, pts_cur;
        trackPointsInFrame(frame, curr, cors_pre, cors_cur, pts_pre, pts_cur);
        if (!pts_pre.size())
            return;

        dyna_g_ = vector<bool>(curr->objects_.size(), false);
        dyna_a_ = vector<bool>(curr->objects_.size(), false);
        dyna_p_ = vector<bool>(curr->objects_.size(), false);

        // ==================================法一：几何约束（极线约束）
        selectByGeometry(frame, curr, pts_pre, pts_cur, F_cv);

        // ==================================法二：外观一致性
        // selectByAppearance(frame, curr, cors_pre, cors_cur);

        // ==================================法三：目标位置一致性
        selectByPosition(local_map, curr);

        // 综合三种方法判断动态物体
        num_dyna_obj_ = 0;
        for (int i = 0; i < curr->objects_.size(); i++)
        {
            Object::Ptr cur_obj = curr->objects_[i];
            if (dyna_g_[i] || dyna_a_[i] || dyna_p_[i] || cur_obj->class_id_ == 0)
            // if (dyna_g_[i] || dyna_a_[i] || dyna_p_[i])
            {
                cur_obj->dyna_ = true;
                num_dyna_obj_++;
            }
            else
                cur_obj->dyna_ = false;
        }
        // 画图
        {
            cv::Mat img;
            img.push_back(curr->left_);
            cvtColor(img, img, COLOR_GRAY2RGB);
            putText(img, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID
            for (int i = 0; i < curr->objects_.size(); i++)
            {
                Object::Ptr obj = curr->objects_[i];
                cv::Rect box(obj->left_, obj->top_, obj->right_ - obj->left_, obj->bot_ - obj->top_);
                if (obj->dyna_) // 动目标
                {
                    rectangle(img, box, Scalar(0, 0, 255), 2); // 红色
                }
                else // 静目标
                {
                    rectangle(img, box, Scalar(0, 255, 0), 2); // 绿色
                }
            }
            cv::imshow("img_DynaObjects", img);
            cv::waitKey(1);
        }
    }

    /**
 * @brief 将当前帧目标中恢复出深度的角点投影到frame帧
 * 
 * @param T_fc 两帧间相对位姿
 * @param objects 当前帧的目标
 * @param cors_pre frame帧角点
 * @param cors_cur 对应的当前帧角点
 */
    void MotionClassifier::prjPointsToFrame(const SE3 &cur_T, const SE3 &pre_T, vector<Object::Ptr> &curr_obj, vector<cv::Point2f> &cors_pre, vector<cv::Point2f> &cors_cur)
    {
        SE3 T_fc = cur_T.inverse() * pre_T;
        for (auto obj : curr_obj)
        {
            for (int i = 0; i < obj->corners_.size(); i++)
            {
                cv::Point2f cur_pix(obj->corners_[i]);
                float depth = obj->cordepths_[i];
                Vec3 cur_camcoor = Transform::pixel2camera(Camera::K_, cur_pix, depth);
                Vec2 pre_pixcoor = Transform::world2pixel(Camera::K_, cur_camcoor, T_fc);
                if (pre_pixcoor == Vec2(-1, -1))
                    continue;
                cv::Point2f pre_pix(pre_pixcoor(0), pre_pixcoor(1));
                if (!Func::inBorder(pre_pix))
                    continue;
                cors_cur.push_back(cur_pix);
                cors_pre.push_back(pre_pix);
            }
        }
    }

    /**
     * @brief 将地图目标投影到当前帧
     * 
     * @param obj_lt 地图目标边框左上角点世界坐标
     * @param obj_rb 地图目标边框右下角点世界坐标
     * @param obj_c 地图目标中心点的世界坐标
     * @param curr_T 当前帧位姿
     * @param lefttop 地图目标投影在当前帧的左上角点像素坐标
     * @param rightbot 地图目标投影在当前帧的右下角点像素坐标
     * @return true 投影在当前帧视野内
     * @return false 投影不在当前帧视野内
     */
    bool MotionClassifier::prjObjectsToCurr(Object::Ptr obj, const SE3 &curr_T, Vec2 &lefttop, Vec2 &rightbot)
    {
        // 只投影左上角点与右下角点
        Vec3 camcoor_lt = Transform::world2camera(obj->corns_wld_coor_[0], curr_T);
        Vec3 camcoor_rb = Transform::world2camera(obj->corns_wld_coor_[3], curr_T);

        // 判断是否在相机前方
        if (camcoor_lt(2) < 1.0 || camcoor_rb(2) < 1.0)
            return false;

        // 投影到当前帧
        lefttop = Transform::camera2pixel(Camera::K_, camcoor_lt);
        rightbot = Transform::camera2pixel(Camera::K_, camcoor_rb);

        // 判断是否移出相机视野
        if (!Func::inBorder(lefttop(0), lefttop(1)) || !Func::inBorder(rightbot(0), rightbot(1)))
            obj->out_view_ = true;

        // 判断是否超出图像边界
        if (!Func::inBorder(lefttop(0), rightbot(0), lefttop(1), rightbot(1)))
            return false;

        if (lefttop(0) < 0) // x
            lefttop(0) = 0;
        if (rightbot(0) > Camera::width_)
            rightbot(0) = Camera::width_;

        if (lefttop(1) < 0) // y
            lefttop(1) = 0;
        if (rightbot(1) > Camera::height_)
            rightbot(1) = Camera::height_;

        return true;
    }

    /**
     * @brief 通过光流找到当前帧角点在frame帧中的位置
     * 
     * @param cors_pre 
     * @param cors_cur 
     * @param pts_pre 
     * @param pts_cur 
     */
    void MotionClassifier::trackPointsInFrame(Frame::Ptr frame, Frame::Ptr curr, const vector<cv::Point2f> &cors_pre, const vector<cv::Point2f> &cors_cur, vector<cv::Point2f> &pts_pre, vector<cv::Point2f> &pts_cur)
    {
        // 光流跟踪
        vector<unsigned char> status;
        vector<float> err;
        cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
        cv::calcOpticalFlowPyrLK(curr->left_, frame->left_, cors_cur, cors_pre, status, err, cv::Size(10, 10), 0, criteria, OPTFLOW_USE_INITIAL_FLOW);

        // 光流跟踪结果筛选
        for (int i = 0; i < status.size(); i++)
        {
            if (status[i] != 0)
            {
                int x = std::round(cors_cur[i].x), y = std::round(cors_cur[i].y);
                if (!Func::inBorder(Point2f(x, y)))
                    continue;
                else
                {
                    pts_pre.push_back(cors_pre[i]);
                    pts_cur.push_back(cors_cur[i]);
                }
            }
        }
    }

    /**
     * @brief 处理漏检情况
     * 遍历局部地图中的每个目标，查看每个目标是否匹配上当前帧目标。
     * 若否，则判断该目标是否投影在该帧图像的边缘区域，若否则说明发生了漏检。
     * 若发生了漏检，则根据匀速运动模型假设，计算该地图目标在当前帧的位置，并投影到当前帧。
     * 以投影区域作为一个新的目标加入到当前帧中，继续进行后续解算。
     * 要真正区分漏检、目标移出视野、目标被遮挡
     * @param curr 当前帧
     */
    void MotionClassifier::processMissedDetection(Map::Ptr local_map, Frame::Ptr curr)
    {
        if (local_map->objects_.size() == 0)
            return;

        vector<int> new_classIds;
        vector<float> new_confidences;
        vector<cv::Point2i> new_boxes_lt;
        vector<cv::Point2i> new_boxes_rb;
        vector<Object::Ptr> new_obj; // 因为漏检新添加的目标(用于显示)
        for (auto iter = local_map->objects_.begin(); iter != local_map->objects_.end(); iter++)
        {
            // 判断是否发生漏检
            Object::Ptr map_obj = *iter;
            if (map_obj->error_)
                continue;
            else if (map_obj->matched_by_curr_)
                continue;
            else if (map_obj->out_view_)
                continue;

            // 预测目标边框在当前帧中的位置
            int sz = map_obj->trajectory_.size();
            if (sz < 5)
                continue;
            int end = min(sz, 10);
            Vec3 mean_move(0, 0, 0);
            for (int i = 2; i <= end; i++) // 计算过去end帧平均移动向量
            {
                mean_move += (map_obj->trajectory_[sz - i + 1] - map_obj->trajectory_[sz - i]);
            }
            mean_move /= ((end - 1) == 0 ? 1 : (end - 1));

            map_obj->corns_wld_coor_[0] += mean_move;
            map_obj->corns_wld_coor_[3] += mean_move;

            // 判断是否移出相机视野
            Vec2 lefttop, rightbot;
            if (!prjObjectsToCurr(map_obj, curr->T_wc_, lefttop, rightbot))
                continue;

            // 判断是否被遮挡
            int left = lefttop(0), right = rightbot(0), top = lefttop(1), bot = rightbot(1);
            Rect rect(left, top, right - left, bot - top);
            Mat mask = (curr->mask_(rect) == 0);
            int area = cv::countNonZero(mask);
            if (area < min_area_ || (sz < 5 && area > max_area_) || (float)area / rect.area() < Param::min_area_ratio_)
                continue;

            new_classIds.push_back(map_obj->class_id_);
            new_confidences.push_back(map_obj->score_);
            new_boxes_lt.push_back(cv::Point2i(left, top));
            new_boxes_rb.push_back(cv::Point2i(right, bot));

            // 生成一个新目标加入到当前帧中
            Object::Ptr obj(new myslam::Object);
            obj->left_ = left;
            obj->right_ = right;
            obj->top_ = top;
            obj->bot_ = bot;
            obj->score_ = map_obj->score_;
            obj->area_ = area;
            obj->class_id_ = map_obj->class_id_;
            obj->label_ = map_obj->label_;
            obj->r_ = map_obj->r_;
            obj->g_ = map_obj->g_;
            obj->b_ = map_obj->b_;

            int curr_num_obj = curr->objects_.size();
            obj->mask_ = Mat(bot - top, right - left, CV_8UC1, curr_num_obj + 1);
            obj->mask_.copyTo(curr->mask_(rect), mask);
            curr->objects_.push_back(obj);
            curr->mobjs_matched_.push_back(map_obj);
            new_obj.push_back(obj);
        }

        // 画图
        if (Param::drawMissDetect_)
        {
            Mat img;
            img.push_back(curr->left_);
            cvtColor(img, img, COLOR_GRAY2RGB);
            // putText(img, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID
            // 画出因为漏检新加入的目标
            for (int i = 0; i < new_obj.size(); i++)
            {
                Object::Ptr obj = new_obj[i];
                cv::Point2f lt = Point2f(obj->left_, obj->top_);
                cv::Point2f rb = Point2f(obj->right_, obj->bot_);
                cv::Rect prj_box(lt, rb);
                cv::rectangle(img, prj_box, Scalar(0, 0, 255), 2);
                putText(img, obj->label_, lt, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 红色
            }
            cv::imshow("img_MissedDetection", img);
            cv::waitKey(0);
        }
    }

    /**
 * @brief 通过几何约束来筛选动态物体
 * 将当前帧恢复出深度的点投影到frame帧后，在投影位置做光流跟踪
 * 计算当前帧的点在frame帧的极线
 * 计算frame帧的点到对应极线的距离，并判断该点是否是动点
 * 计算目标内动静点的比例并判断该目标是否是动态目标
 */
    void MotionClassifier::selectByGeometry(Frame::Ptr frame, Frame::Ptr curr, const vector<cv::Point2f> &pts_pre, const vector<cv::Point2f> &pts_cur, const cv::Mat &F)
    {
        // 计算极线
        vector<cv::Vec<float, 3>> epilines;
        cv::computeCorrespondEpilines(pts_cur, 1, F, epilines);

        for (int i = 0; i < pts_cur.size(); i++)
        {
            Mat img;
            img.push_back(frame->left_);
            img.push_back(curr->left_);
            cvtColor(img, img, COLOR_GRAY2RGB);
            putText(img, to_string(frame->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // frame帧ID

            putText(img, to_string(curr->id_), cv::Point2i(10, 30 + Camera::height_), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 当前帧ID
            cv::circle(img, pts_pre[i], 5, Scalar(0, 255, 0));
            cv::circle(img, pts_cur[i] + Point2f(0, Camera::height_), 5, Scalar(0, 255, 0));

            cv::line(img, cv::Point2f(0, -epilines[i][2] / epilines[i][1]), cv::Point2f(img.cols, -(epilines[i][2] + epilines[i][0] * img.cols) / epilines[i][1]), Scalar(0, 255, 0));
            cv::imshow("img", img);
            cv::waitKey(0);
        }

        // 计算frame帧点到极线的距离并判断是否是动点
        vector<bool> dyna_pts;
        for (int i = 0; i < pts_pre.size(); i++)
        {
            float A = epilines[i][0], B = epilines[i][1], C = epilines[i][2];
            float x = pts_pre[i].x, y = pts_pre[i].y;
            float d_2 = (A * x + B * y + C) * (A * x + B * y + C) / (A * A + B * B);
            if (d_2 < dyna_epipolar_dis_)
            {
                dyna_pts.push_back(false);
            }
            else
            {
                dyna_pts.push_back(true);
            }
        }

        // 统计目标内动静点的个数
        int num_obj = curr->objects_.size();
        vector<pair<int, int>> dyna_obj(num_obj, make_pair(0, 0));
        for (int i = 0; i < pts_cur.size(); i++)
        {
            int x = std::round(pts_cur[i].x), y = std::round(pts_cur[i].y);
            int obj_id = curr->mask_.ptr<unsigned char>(y)[x];
            if (dyna_pts[i]) // 动点
            {
                dyna_obj[obj_id - 1].first++;
            }
            else // 静点
            {
                dyna_obj[obj_id - 1].second++;
            }
        }

        // 计算目标内动静点的比例并判断该目标是否是动态目标
        for (int i = 0; i < num_obj; i++)
        {
            Object::Ptr obj = curr->objects_[i];
            int num_dypt = dyna_obj[i].first;
            int num_stpt = dyna_obj[i].second;
            if (num_dypt == 0 && num_stpt == 0)
            {
                dyna_g_[i] = false;
            }
            else if ((float)num_dypt / (num_dypt + num_stpt) > dyna_ratio_)
            {
                dyna_g_[i] = true;
            }
            else
            {
                dyna_g_[i] = false;
            }
        }

        // 画图
        if (Param::drawGeometry_)
        {
            cv::Mat img;
            // img.push_back(frame->left_);
            img.push_back(curr->left_);
            cvtColor(img, img, COLOR_GRAY2RGB);
            // putText(img, to_string(frame->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1);                  // frame帧ID
            putText(img, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 当前帧ID

            for (int i = 0; i < pts_cur.size(); i++)
            {
                cv::Point2f start_pos(pts_cur[i].x, pts_cur[i].y);
                cv::Point2f end_pos(pts_pre[i].x, pts_pre[i].y);

                //     // 显示极线
                //     // int r = rand() % (255 + 1);
                //     // int g = rand() % (255 + 1);
                //     // int b = rand() % (255 + 1);
                //     // cv::circle(img, end_pos, 5, Scalar(b, g, r));
                //     // cv::line(img, cv::Point2f(0, -epilines[i][2] / epilines[i][1]), cv::Point2f(img.cols, -(epilines[i][2] + epilines[i][0] * img.cols) / epilines[i][1]), Scalar(b, g, r));

                //     // 显示光流跟踪
                //     // cv::circle(img, pts_pre[i], 5, Scalar(0, 255, 0), 1);
                //     // cv::circle(img, end_pos, 5, Scalar(0, 255, 0), 1);
                //     // cv::line(img, start_pos, end_pos, Scalar(0, 255, 255));

                // 显示动静点
                cv::Point2f pt(start_pos.x, start_pos.y);
                if (dyna_pts[i])
                {
                    cv::circle(img, pt, 5, Scalar(0, 0, 255), 1); // 动点
                }
                else
                {
                    cv::circle(img, pt, 5, Scalar(255, 255, 0), 1); // 静点
                }
            }
            int k = 0;
            for (auto obj : curr->objects_)
            {
                // Rect box(obj->left_, obj->top_ + Camera::height_, fabs(obj->left_ - obj->right_), fabs(obj->top_ - obj->bot_));
                Rect box(obj->left_, obj->top_, fabs(obj->left_ - obj->right_), fabs(obj->top_ - obj->bot_));
                if (dyna_g_[k]) // 动态目标
                {
                    rectangle(img, box, Scalar(0, 0, 255), 2); // 红色
                }
                else // 静态目标
                {
                    rectangle(img, box, Scalar(0, 255, 0), 2); // 绿色
                }
                k++;
            }
            cv::imshow("img_Geometry", img);
            cv::waitKey(0);
        }
    }

    /**
 * @brief 通过当前帧的点在frame帧对应位置处的像素块外观差异来判断是否是动态物体
 * 如果只是两个patch的像素灰度差作为判断是否是动点的话，太不鲁棒，稍有移动，灰度差就会很大，尝试通过计算NCC或描述子的方法判断是否一致
 * 统计一下，计算每个目标上patch块像素差的均值和标准差，动态物体的均值和标准差都偏大，或许可以以此来判断是否是动态物体
 * 判断相机是前进还是后退，以此在图像金字塔的不同层上提取像素块
 * 
 * @param frame 
 * @param curr 
 * @param cors_pre
 * @param cors_cur
 */
    // void MotionClassifier::selectByAppearance(Frame::Ptr frame, Frame::Ptr curr, const vector<cv::Point2f> &cors_pre, const vector<cv::Point2f> &cors_cur)
    // {
    //     int num_obj = curr->objects_.size();
    //     vector<float> obj_ncc[num_obj];                           // 每个目标的像素块相似度
    //     vector<pair<int, int>> obj_pts(num_obj, make_pair(0, 0)); // 每个目标的动静点个数
    //     vector<Point2f> dyna_pts, static_pts;                     // 动点和静点（用于显示）

    //     // 通过像素块归一化相似度NCC判断每个点是否是动点
    //     int dx[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
    //     int dy[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
    //     for (int i = 0; i < cors_cur.size(); i++)
    //     {
    //         // 根据前进或后退在当前帧的不同缩放图中取出像素块
    //         // 前进：frame原图，curr缩小
    //         // 后退：frame缩小，curr原图
    //         int x0_cur = std::round(cors_cur[i].x - Param::halfpatch_size_);
    //         int y0_cur = std::round(cors_cur[i].y - Param::halfpatch_size_);
    //         if (!Func::inBorder(x0_cur, y0_cur) || !Func::inBorder(x0_cur + Param::halfpatch_size_ * 2, y0_cur + Param::halfpatch_size_ * 2))
    //         {
    //             continue;
    //         }
    //         Mat patch_cur;
    //         if (isForward_)
    //         {
    //             patch_cur = curr->scimg_(Rect(x0_cur / Param::scale_, y0_cur / Param::scale_, Param::halfpatch_size_ * 2, Param::halfpatch_size_ * 2));
    //         }
    //         else
    //         {
    //             patch_cur = curr->left_(Rect(x0_cur, y0_cur, Param::halfpatch_size_ * 2, Param::halfpatch_size_ * 2));
    //         }
    //         Scalar mean_cur = cv::mean(patch_cur);
    //         patch_cur.convertTo(patch_cur, CV_32F, 1.0, -mean_cur[0]); // 去掉均值
    //         int x0_pre = std::round(cors_pre[i].x - Param::halfpatch_size_);
    //         int y0_pre = std::round(cors_pre[i].y - Param::halfpatch_size_);
    //         // 在投影位置的邻域移动像素块并计算NCC的最大值
    //         float max_ncc = 0;
    //         for (int j = 0; j < 9; j++)
    //         {
    //             // 根据前进或后退在frame的不同缩放图中取出像素块
    //             x0_pre += dx[j];
    //             y0_pre += dy[j];
    //             if (!Func::inBorder(x0_pre, y0_pre) || !Func::inBorder(x0_pre + 2 * Param::halfpatch_size_, y0_pre + 2 * Param::halfpatch_size_))
    //             {
    //                 continue;
    //             }
    //             Mat patch_pre;
    //             if (isBackward_)
    //             {
    //                 patch_pre = frame->scimg_(Rect(x0_pre / Param::scale_, y0_pre / Param::scale_, Param::halfpatch_size_ * 2, Param::halfpatch_size_ * 2));
    //             }
    //             else
    //             {
    //                 patch_pre = frame->left_(Rect(x0_pre, y0_pre, Param::halfpatch_size_ * 2, Param::halfpatch_size_ * 2));
    //             }
    //             Scalar mean_pre = cv::mean(patch_pre);
    //             patch_pre.convertTo(patch_pre, CV_32F, 1.0, -mean_pre[0]); // 去掉均值

    //             // 计算像素块相似度NCC
    //             float ncc = Func::calcPatchNcc(patch_pre, patch_cur);
    //             if (ncc > max_ncc)
    //             {
    //                 max_ncc = ncc;
    //             }
    //         }

    //         // 判断该点是否是动点并将该点加入到对应的目标数组中
    //         int x = std::round(cors_cur[i].x), y = std::round(cors_cur[i].y);
    //         int obj_id = curr->mask_.ptr<unsigned char>(y)[x];
    //         obj_ncc[obj_id - 1].push_back(max_ncc);
    //         if (max_ncc < Param::dyna_patch_ncc_)
    //         {
    //             obj_pts[obj_id - 1].first++; // 动点
    //             dyna_pts.push_back(cors_cur[i]);
    //         }
    //         else
    //         {
    //             obj_pts[obj_id - 1].second++; // 静点
    //             static_pts.push_back(cors_cur[i]);
    //         }
    //     }

    //     // 判断目标是否是动态目标
    //     for (int i = 0; i < num_obj; i++)
    //     {
    //         Object::Ptr obj = curr->objects_[i];
    //         int num_dypt = obj_pts[i].first;
    //         int num_stpt = obj_pts[i].second;
    //         if (num_dypt == 0 && num_stpt == 0)
    //         {
    //             dyna_a_[i] = false;
    //             continue;
    //         }

    //         // 计算动静点比例
    //         float pro = (float)num_dypt / (num_dypt + num_stpt);

    //         // 计算像素块灰度差的均值和标准差
    //         Scalar mean, std;
    //         cv::meanStdDev(obj_ncc[i], mean, std);

    //         if (pro > dyna_ratio_ && std[0] > Param::dyna_patch_std_)
    //         {
    //             dyna_a_[i] = true;
    //         }
    //         else
    //         {
    //             dyna_a_[i] = false;
    //         }
    //     }

    //     // 画图
    //     if (Param::drawAppearance_)
    //     {
    //         cv::Mat img;
    //         img.push_back(curr->left_);
    //         cvtColor(img, img, COLOR_GRAY2RGB);
    //         putText(img, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID

    //         for (auto pt : dyna_pts) // 动点
    //         {
    //             cv::circle(img, pt, 5, Scalar(0, 0, 255)); // 红色
    //         }
    //         for (auto pt : static_pts) // 静点
    //         {
    //             cv::circle(img, pt, 5, Scalar(0, 255, 0)); // 绿色
    //         }
    //         int i = 0;
    //         for (auto obj : curr->objects_)
    //         {
    //             Rect box(obj->left_, obj->top_, obj->right_ - obj->left_, obj->bot_ - obj->top_);
    //             if (dyna_a_[i]) // 动目标
    //             {
    //                 rectangle(img, box, Scalar(0, 0, 255), 2); // 红色
    //             }
    //             else // 静目标
    //             {
    //                 rectangle(img, box, Scalar(0, 255, 0), 2); // 绿色
    //             }
    //             putText(img, to_string(i), Point(box.x, box.y), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 红色
    //             i++;
    //         }
    //         cv::imshow("img_Appearance", img);
    //         cv::waitKey(0);
    //     }
    // }

    /**
 * @brief 目标位置一致性约束
 * locateObject已经算出了当前帧中每个目标的世界坐标
 * matchObject已经将当前帧目标和上一帧目标进行了匹配
 * 需要将当前帧目标和局部地图中的目标对应起来（这一步是在matchObj函数中已经完成）
 * 遍历地图目标，判断每个目标的位置是否与其平均位置发生了较大的移动，若是则有可能是动态物体
 * 该方法只能局部地图中已有的目标判断是否为动态物体，对当前帧新检测到的物体无法进行判断
 * // TODO：考虑一个非常长的物体（如火车），在一张单目影像中盛不下来，虽然是运动的但是在图像中的方框一直未改变，导致计算的物体的位置一直未改变而被误认为静态的
 * @param frame 
 * @param curr 
 */
    void MotionClassifier::selectByPosition(Map::Ptr local_map, Frame::Ptr curr)
    {
        if (local_map->objects_.size() == 0)
            return;

        for (auto iter = local_map->objects_.begin(); iter != local_map->objects_.end(); iter++)
        {
            Object::Ptr map_obj = *iter;
            map_obj->dyna_ = false;
            if (map_obj->error_)
                continue;

            // 计算过去20帧的平均位置
            int sz = map_obj->trajectory_.size();
            if (sz < 5) // 说明目标位置较不确定
                continue;
            Vec3 mean_pos(0, 0, 0);
            int end = min(sz, 10);
            for (int i = 2; i <= end; i++)
            {
                mean_pos += map_obj->trajectory_[sz - i];
            }
            mean_pos /= (end - 1);

            // 判断当前位置是否发生较大变动
            if ((map_obj->wld_coor_ - mean_pos).norm() > Param::dyna_pos_change_ / sz)
                map_obj->dyna_ = true;
        }

        // 找到地图目标对应的当前帧目标
        for (int i = 0; i < curr->objects_.size(); i++)
        {
            if (curr->mobjs_matched_[i] != nullptr)
                dyna_p_[i] = curr->mobjs_matched_[i]->dyna_;
            else
            {
                dyna_p_[i] = false;
            }
        }

        // 画图
        if (Param::drawPosition_)
        {
            cv::Mat img;
            img.push_back(curr->left_);
            cvtColor(img, img, COLOR_GRAY2RGB);
            // putText(img, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID
            for (int i = 0; i < curr->objects_.size(); i++)
            {
                Object::Ptr obj = curr->objects_[i];
                cv::Rect box(obj->left_, obj->top_, obj->right_ - obj->left_, obj->bot_ - obj->top_);
                if (dyna_p_[i]) // 动目标
                {
                    rectangle(img, box, Scalar(0, 0, 255), 2); // 红色
                }
                else // 静目标
                {
                    rectangle(img, box, Scalar(0, 255, 0), 2); // 绿色
                }
            }
            cv::imshow("img_Position", img);
            cv::waitKey(0);
        }
    }

    /**
 * @brief 对图中检测到的目标定位
 * 对当前帧的每个目标的深度赋值，每个目标存储其上提取的角点及其深度
 */
    void MotionClassifier::locateObject(Frame::Ptr curr)
    {
        if (!curr->objects_.size())
        {
            return;
        }

        /* 重新从当前帧上提取角点并进行光流左右目匹配*/

        // // 在当前帧的目标框内提取角点
        // vector<cv::Point2f> cors_left;
        // cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
        // cv::goodFeaturesToTrack(curr->left_, cors_left, num_corners_, 0.01, 8, curr->mask_);
        // if (cors_left.size() == 0)
        //     return;
        // cv::cornerSubPix(curr->left_, cors_left, cv::Size(10, 10), cv::Size(-1, -1), criteria);

        // // 通过光流法在当前帧右目找到对应点
        // vector<unsigned char> status;
        // vector<float> error;
        // vector<Point2f> cors_right(cors_left);
        // switch (Camera::Type_)
        // {
        // case Camera::eType::STEREO:
        // {
        //     cv::calcOpticalFlowPyrLK(curr->left_, curr->right_, cors_left, cors_right, status, error, cv::Size(50, 50), 4, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
        //     break;
        // }
        // case Camera::eType::RGBD:
        // {
        //     for (int i = 0; i < cors_left.size(); i++)
        //     {
        //         cv::Point2f pt = cors_left[i];
        //         int x = std::round(pt.x), y = std::round(pt.y);
        //         float depth = curr->depth_.ptr<float>(y)[x];
        //         if (depth == 0)
        //             status.push_back(0);
        //         else
        //         {
        //             cors_right[i] = cv::Point2f(pt.x - Camera::base_fx_ / depth, pt.y);
        //             status.push_back(1);
        //         }
        //     }
        //     break;
        // }
        // }

        // // 匹配结果进行筛选
        // int num_obj = curr->objects_.size();
        // vector<cv::Point2f> obj_pts[num_obj];
        // vector<cv::Point2f> obj_pts_r[num_obj]; // 用于显示
        // vector<cv::Mat> depth_pts(num_obj);
        // for (int i = 0; i < cors_left.size(); i++)
        // {
        //     cv::Point2f pt_left = cors_left[i];
        //     cv::Point2f pt_right = cors_right[i];
        //     if (!status[i] || (pt_left.x - pt_right.x) <= 0 || fabs(pt_left.y - pt_right.y) > 2 || !Func::inBorder(pt_right))
        //         continue;
        //     else
        //     {
        //         int x = std::round(pt_left.x), y = std::round(pt_left.y);
        //         int obj_id = curr->mask_.ptr<unsigned char>(y)[x];
        //         if (obj_id != 0)
        //         {
        //             float depth = Camera::base_fx_ / (pt_left.x - pt_right.x);
        //             obj_pts[obj_id - 1].push_back(pt_left);
        //             obj_pts_r[obj_id - 1].push_back(pt_right);
        //             depth_pts[obj_id - 1].push_back(depth);
        //         }
        //     }
        // }

        /*使用前端左右目匹配的特征点*/

        int num_obj = curr->objects_.size();
        vector<cv::Point2f> obj_pts[num_obj];
        vector<cv::Point2f> obj_pts_r[num_obj]; // 用于显示
        vector<cv::Mat> depth_pts(num_obj);
        for (int i = 0; i < curr->features_.size(); i++)
        {
            Feature::Ptr ft = curr->features_[i];
            int x = std::round(ft->x_), y = std::round(ft->y_);
            int obj_id = curr->mask_.ptr<unsigned char>(y)[x];
            if (ft->stereo_ && obj_id)
            {
                obj_pts[obj_id - 1].push_back(ft->pt_);
                obj_pts_r[obj_id - 1].push_back(cv::Point2f(ft->x_r_, ft->y_));
                depth_pts[obj_id - 1].push_back(ft->depth_);
            }
        }

        // 目标角点筛选
        // 统计该目标上一共有多少个角点，取其深度前0.8的点，计算中值作为目标深度
        int k = 0;
        for (auto obj : curr->objects_)
        {
            int num_pts = obj_pts[k].size();
            if ((float)num_pts / obj->area_ > Param::numpts_area_ratio_ || num_pts > 5)
            {
                obj->error_ = false;

                // 角点深度排序
                cv::Mat depth = depth_pts[k];
                cv::Mat sort_idx;
                cv::sortIdx(depth, sort_idx, cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);

                // 目标角点及其深度赋值
                int num_keeppts = std::round((float)num_pts * Param::keeppts_ratio_); // 仅保留0.8的角点来计算目标深度
                obj->corners_.reserve(num_keeppts);
                obj->cordepths_.reserve(num_keeppts);
                for (int j = 0; j < num_keeppts; j++)
                {
                    int id = sort_idx.ptr<int>(j)[0];
                    float cordepth = depth_pts[k].ptr<float>(id)[0];
                    obj->corners_.push_back(obj_pts[k][id]);
                    obj->cordepths_.push_back(cordepth);
                }

                // 计算目标角点深度的中位数作为目标深度
                // TODO：尝试使用滤波滤除外点，或者直方图统计
                float med_depth;
                int med = num_keeppts / 2;
                if (num_keeppts % 2 == 0)
                    med_depth = (obj->cordepths_[med - 1] + obj->cordepths_[med]) / 2;
                else
                    med_depth = obj->cordepths_[med];

                // 目标的深度
                obj->depth_ = med_depth;

                // 计算目标的世界坐标
                obj->wld_coor_ = Transform::pixel2world(Camera::K_, cv::Point2f((obj->left_ + obj->right_) * 0.5, (obj->top_ + obj->bot_) * 0.5), curr->T_wc_, obj->depth_);
                obj->trajectory_.push_back(obj->wld_coor_);

                // 计算目标四个角点的世界坐标
                obj->corns_wld_coor_.push_back(Transform::pixel2world(Camera::K_, cv::Point2f(obj->left_, obj->top_), curr->T_wc_, obj->depth_));
                obj->corns_wld_coor_.push_back(Transform::pixel2world(Camera::K_, cv::Point2f(obj->right_, obj->top_), curr->T_wc_, obj->depth_));
                obj->corns_wld_coor_.push_back(Transform::pixel2world(Camera::K_, cv::Point2f(obj->left_, obj->bot_), curr->T_wc_, obj->depth_));
                obj->corns_wld_coor_.push_back(Transform::pixel2world(Camera::K_, cv::Point2f(obj->right_, obj->bot_), curr->T_wc_, obj->depth_));
            }
            else
            {
                obj->error_ = true;
            }
            k++;
        }

        // 更新地图目标error属性
        for (int i = 0; i < curr->objects_.size(); i++)
            if (curr->mobjs_matched_[i] != nullptr)
                curr->mobjs_matched_[i]->error_ = curr->objects_[i]->error_;

        // 画图
        if (Param::drawLocateObj_)
        {
            cv::Mat img;
            img.push_back(curr->left_);
            // img.push_back(curr->right_);
            cvtColor(img, img, COLOR_GRAY2RGB);
            putText(img, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID
            // // 显示目标掩码
            // Mat coloredMask(Camera::height_, Camera::width_, CV_8UC3, 0.2 * Scalar(0, 255, 255));
            // cv::Mat roi = img(Rect(0, 0, Camera::width_, Camera::height_));
            // coloredMask += roi;
            // coloredMask.copyTo(roi, curr->mask_);
            // 显示提取的角点
            for (int i = 0; i < num_obj; i++)
            {
                for (int j = 0; j < obj_pts[i].size(); j++)
                {
                    cv::line(img, obj_pts[i][j], obj_pts_r[i][j], Scalar(0, 255, 0));
                    cv::circle(img, obj_pts[i][j], 5, Scalar(0, 255, 255));
                }
            }
            // 显示边界框
            for (auto obj : curr->objects_)
            {
                if (!obj->error_)
                {
                    Rect box(obj->left_, obj->top_, fabs(obj->left_ - obj->right_), fabs(obj->top_ - obj->bot_));
                    rectangle(img, box, Scalar(0, 255, 0), 2); // 绿色

                    putText(img, to_string(obj->depth_).substr(0, 4), Point(obj->left_, obj->top_ - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2); // 红色
                }
                else
                {
                    Rect box(obj->left_, obj->top_, fabs(obj->left_ - obj->right_), fabs(obj->top_ - obj->bot_));
                    rectangle(img, box, Scalar(255, 0, 0), 2); // 蓝色
                }
            }
            cv::imshow("img_LocateObject", img);
            cv::waitKey(0);
        }
    }

    /**
 * @brief 与局部地图中的目标进行匹配
 * 计算当前帧目标与局部地图中的目标的交并比IOU，选择交并比最大的作为匹配结果
 * 
 */
    void MotionClassifier::matchObject(Map::Ptr local_map, Frame::Ptr curr)
    {
        if (local_map->objects_.size() == 0)
        {
            return;
        }
        else if (curr->objects_.size() == 0)
        {
            // 需要将地图目标投影到当前帧、并更新地图目标的匹配关系
        }

        vector<Object::Ptr> vecObj;
        vector<int> obj_classid;
        for (auto iter = local_map->objects_.begin(); iter != local_map->objects_.end(); iter++)
        {
            vecObj.push_back(*iter);
            obj_classid.push_back(iter->get()->class_id_);
        }

        // 将局部地图中的每个目标边框投影到当前帧
        vector<vector<cv::Point2f>> prjborders(local_map->objects_.size(), vector<cv::Point2f>(2, cv::Point2f(-1, -1)));
        int i = 0;
        for (auto iter = local_map->objects_.begin(); iter != local_map->objects_.end(); iter++)
        {
            Object::Ptr map_obj = *iter;
            if (map_obj->error_)
            {
                i++;
                continue;
            }

            // 将地图目标投影在当前帧
            Vec2 lefttop, rightbot;
            if (!prjObjectsToCurr(map_obj, curr->T_wc_, lefttop, rightbot))
            {
                map_obj->error_ = true;
                map_obj->left_ = map_obj->right_ = map_obj->top_ = map_obj->bot_ = 0;
                i++;
                continue;
            }

            // 更新地图目标在当前帧的投影位置
            map_obj->left_ = lefttop(0);
            map_obj->top_ = lefttop(1);
            map_obj->right_ = rightbot(0);
            map_obj->bot_ = rightbot(1);

            vector<cv::Point2f> coors;
            coors.push_back(cv::Point2f(lefttop(0), lefttop(1)));
            coors.push_back(cv::Point2f(rightbot(0), rightbot(1)));
            prjborders[i] = coors;

            i++;
        }

        // 计算每个投影边框与当前帧每个obj边框之间的IOU
        vector<vector<float>> iou_mark(curr->objects_.size(), vector<float>(local_map->objects_.size(), 0));
        for (int i = 0; i < curr->objects_.size(); i++)
        {
            Object::Ptr obj_curr = curr->objects_[i];
            cv::Point2f R1LeftUp(obj_curr->left_, obj_curr->top_);
            cv::Point2f R1RightDown(obj_curr->right_, obj_curr->bot_);
            for (int j = 0; j < prjborders.size(); j++)
            {
                if (prjborders[j][0] != cv::Point2f(-1, -1) && obj_curr->class_id_ == obj_classid[j]) // 同一class的物体才被匹配
                {
                    cv::Point2f R2LeftUp = prjborders[j][0];
                    cv::Point2f R2RightDown = prjborders[j][1];
                    float IOU = Func::calcIOU(R1LeftUp, R1RightDown, R2LeftUp, R2RightDown);
                    iou_mark[i][j] = IOU;
                }
                else
                    iou_mark[i][j] = 0;
            }
        }

        // 目标找到最佳匹配
        unordered_set<int> MatchedID;
        for (int i = 0; i < iou_mark.size(); i++)
        {
            int matchedID = -1;
            float maxIOU = 0;
            for (int j = 0; j < iou_mark[i].size(); j++)
            {
                if (iou_mark[i][j] > maxIOU && MatchedID.find(j) == MatchedID.end()) // 大于最大的交并比且未被匹配过
                {
                    maxIOU = iou_mark[i][j];
                    matchedID = j;
                }
            }
            if (maxIOU > Param::match_IOU_)
            {
                curr->mobjs_matched_[i] = vecObj[matchedID];
                MatchedID.insert(matchedID);
            }
            else
            {
                curr->mobjs_matched_[i] = nullptr;
            }
        }
        // 更新地图目标与当前帧目标的匹配关系
        int obj_id = 0;
        for (auto iter = local_map->objects_.begin(); iter != local_map->objects_.end(); iter++)
        {
            if (MatchedID.find(obj_id) != MatchedID.end())
            {
                iter->get()->matched_by_curr_ = true;
            }
            else
            {
                iter->get()->matched_by_curr_ = false;
            }
            obj_id++;
        }

        // 画图
        if (Param::drawMatchObj_)
        {
            cv::Mat img;
            img.push_back(curr->left_);
            img.push_back(curr->left_);
            cvtColor(img, img, COLOR_GRAY2RGB);
            putText(img, to_string(curr->id_), cv::Point2i(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1); // 显示当前帧ID

            // 画出当前帧目标和地图目标匹配关系
            for (int i = 0; i < curr->objects_.size(); i++)
            {
                Object::Ptr curr_obj = curr->objects_[i];
                cv::Point2f lu(curr_obj->left_, curr_obj->top_ + Camera::height_);
                cv::Point2f rd(curr_obj->right_, curr_obj->bot_ + Camera::height_);
                cv::Rect curr_box(lu, rd);

                Object::Ptr map_obj = curr->mobjs_matched_[i];
                if (map_obj != nullptr) // 如果当前帧匹配上了地图目标
                {
                    cv::Point2f lt(map_obj->left_, map_obj->top_);
                    cv::Point2f rb(map_obj->right_, map_obj->bot_);
                    cv::Rect prj_box(lt, rb);
                    cv::rectangle(img, prj_box, Scalar(map_obj->b_, map_obj->g_, map_obj->r_), 2);
                    cv::rectangle(img, curr_box, Scalar(map_obj->b_, map_obj->g_, map_obj->r_), 2);
                    putText(img, curr_obj->label_, lt, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 红色
                }
                else
                {
                    cv::rectangle(img, curr_box, Scalar(255, 255, 255), 1);
                }
                putText(img, curr_obj->label_, lu, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 红色
            }

            // 画出未被匹配上的地图目标
            for (auto iter = local_map->objects_.begin(); iter != local_map->objects_.end(); iter++)
            {
                Object::Ptr map_obj = *iter;
                if (!map_obj->error_ && !map_obj->matched_by_curr_)
                {
                    cv::Point2f lt = Point2f(map_obj->left_, map_obj->top_);
                    cv::Point2f rb = Point2f(map_obj->right_, map_obj->bot_);
                    cv::Rect prj_box(lt, rb);
                    cv::rectangle(img, prj_box, Scalar(255, 255, 255), 1);
                    putText(img, map_obj->label_, lt, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // 红色
                }
            }
            cv::imshow("img_MatchObject", img);
            cv::waitKey(0);
        }
    }

    /**
     * @brief 剔除动态目标上的特征点
     * 
     */
    void MotionClassifier::rejectDynaObjectPts(Frame::Ptr curr)
    {
        if (num_dyna_obj_ == 0 || !curr->objects_.size() || curr->id_ == 0)
            return;

        points_3d_.clear();
        pixels_2d_.clear();
        for (int i = 0; i < curr->features_.size(); i++)
        {
            // 判断当前帧特征点是否是动态特正点
            Feature::Ptr ft = curr->features_[i];
            int x = std::round(ft->x_), y = std::round(ft->y_);
            int obj_id = curr->mask_.ptr<unsigned char>(y)[x];
            if (obj_id == 0 || !curr->objects_[obj_id - 1]->dyna_)
            {
                ft->dyna_ = false;
            }
            else
            {
                ft->dyna_ = true;
            }

            // 取静态特征点的匹配坐标
            MapPoint::Ptr mp = curr->mpoints_matched_[i];
            if (ft->dyna_)
            {
                if (mp != nullptr)
                {
                    mp->dyna_ = true;
                }
                mp = nullptr;
            }
            else // ft->dyna_==false
            {
                if (mp != nullptr)
                {
                    Vec3 map_3d = mp->coor_;
                    pixels_2d_.push_back(ft->pt_);
                    points_3d_.push_back(Point3f(map_3d(0, 0), map_3d(1, 0), map_3d(2, 0)));
                }
            }
        }
    }

    /**
     * @brief 重新计算当前帧位姿
     * 
     * @param curr 
     */
    void MotionClassifier::recalcCurrPose(Frame::Ptr curr)
    {
        if (num_dyna_obj_ == 0 || !curr->objects_.size() || curr->id_ == 0)
            return;

        if (points_3d_.size() < 10)
            return;

        Mat K, r, t, R, inliers;
        Mat33 R_eg;
        Vec3 t_eg;

        cv::eigen2cv(Camera::K_, K);
        cv::eigen2cv(curr->T_wc_.rotationMatrix(), r);
        cv::eigen2cv(curr->T_wc_.translation(), t);
        cv::Rodrigues(r, r);

        if (!cv::solvePnPRansac(points_3d_, pixels_2d_, K, Mat(), r, t, true, 100, 4.0, 0.98999, inliers))
        // if (!cv::solvePnP(points_3d_, pixels_2d_, K, Mat(), r, t, true))
        {
            cout << "特征匹配数量过少，无法计算位姿" << endl;
            return;
        }

        cv::Rodrigues(r, R);
        cv::cv2eigen(R, R_eg);
        cv::cv2eigen(t, t_eg);

        // curr->T_cw_f_ = SE3(R_eg, t_eg).inverse();
        curr->T_wc_ = SE3(R_eg, t_eg).inverse();
    }
} // namespace myslam