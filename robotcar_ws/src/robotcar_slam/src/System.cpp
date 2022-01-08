#include "myslam/System.h"
#include <thread>
#include <ros/ros.h>
#include <ros/node_handle.h>

namespace myslam
{

    System::System()
        : file_type_(IMAGE), model_type_(NONE), left_(Mat()), right_(Mat()), T_BS_(SE3()), num_lost_(0), total_dist_(0.)
    {
#ifdef _ROS_
        ros::NodeHandle nh;
        img_pub_ = nh.advertise<solo::img>("image", 1);
        target_sub_ = nh.subscribe("solo", 1, &System::subscribeTargets, this);

        thread spinTask(msgSpin);
        spinTask.detach();
#endif
        ros::NodeHandle nh;
        nh.getParam("workspace_abs_dir", workspace_abs_dir_);
        workspace_abs_dir_ += "robotcar_slam/";
    }

    System::~System() {}

#ifdef _ROS_
    void msgSpin()
    {
        while (ros::ok())
        {
            ros::spin();
        }
    }
#endif

    bool System::initSystem()
    {
        if (!initConfigParameter())
            return false;

        switch (file_type_)
        {
        case myslam::System::FileType::IMAGE:
        {
            if (!initImageDataset())
                return false;
            break;
        }
        case myslam::System::FileType::VIDEO:
        {
            if (!initVideoInterface())
            {
                cerr << "initiate video interface failed!" << endl;
                return false;
            }
            break;
        }
        }
        return true;
    }
    /**
 * @brief 初始化每个类中的静态成员参数
 * 
 * @return true 
 * @return false 
 */
    bool System::initConfigParameter()
    {
        // 打开default.yaml并判断数据集类型
        FileStorage fs0(workspace_abs_dir_ + "config/default.yaml", FileStorage::READ);
        if (!fs0.isOpened())
        {
            cerr << "default.yaml文件路径错误或文件不存在！" << endl;
            return false;
        }
        string cfgfile_name = fs0["camera_file"];
        if (cfgfile_name.empty())
        {
            cerr << "请在default.yaml中填入相机配置文件路径！" << endl;
            return false;
        }

        if (cfgfile_name == "config/KITTI00-02.yaml" || cfgfile_name == "config/KITTI03.yaml" || cfgfile_name == "config/KITTI04-12.yaml" || cfgfile_name == "config/KITTI05.yaml" || cfgfile_name == "config/KITTI00-02_Mono.yaml")
        {
            dataset_type_ = KITTI;
        }
        else if (cfgfile_name == "config/EuRoc.yaml")
        {
            dataset_type_ = EUROC;
        }
        else if (cfgfile_name == "config/TUM1.yaml" || cfgfile_name == "config/TUM2.yaml" || cfgfile_name == "config/TUM3.yaml")
        {
            dataset_type_ = TUM;
        }
        else if (cfgfile_name == "config/virtual.yaml")
        {
            dataset_type_ = VIRTUAL;
        }
        else
        {
            cout << "数据集类型未知！" << endl;
            dataset_type_ = OTHER;
        }

        /*读取相机参数*/
        cv::FileStorage fs1(workspace_abs_dir_ + cfgfile_name, FileStorage::READ);
        if (!fs1.isOpened())
        {
            cerr << "相机配置文件路径错误或文件不存在！" << endl;
            return false;
        }

        // 相机类型
        fs1["Camera.type"] >> Camera::Type_;
        if (!Camera::Type_)
        {
            cerr << "请在配置文件中填入相机类型：单目1，RGBD2，双目3！" << endl;
            return false;
        }

        // 相机内参及畸变参数
        fs1["Camera.width"] >> Camera::width_;
        fs1["Camera.height"] >> Camera::height_;
        fs1["Camera.fx"] >> Camera::fx_;
        fs1["Camera.fy"] >> Camera::fy_;
        fs1["Camera.cx"] >> Camera::cx_;
        fs1["Camera.cy"] >> Camera::cy_;
        fs1["Camera.k1"] >> Camera::k1_;
        fs1["Camera.k2"] >> Camera::k2_;
        fs1["Camera.k3"] >> Camera::k3_;
        fs1["Camera.p1"] >> Camera::p1_;
        fs1["Camera.p2"] >> Camera::p2_;
        fs1["Camera.bf"] >> Camera::base_fx_;
        fs1["Camera.fps"] >> Camera::fps_;
        if (Camera::width_ == 0 || Camera::height_ == 0 || Camera::fx_ == 0 || Camera::fy_ == 0 || Camera::cx_ == 0 || Camera::cy_ == 0 || (Camera::Type_ == Camera::eType::STEREO || Camera::Type_ != Camera::eType::MONOCULAR) && Camera::base_fx_ == 0)
        {
            cerr << "请在配置文件中填入相机参数！" << endl;
            return false;
        }

        // 构建相机内参数矩阵和畸变系数矩阵
        Camera::base_ = Camera::base_fx_ / Camera::fx_;
        Camera::K_ << Camera::fx_, 0, Camera::cx_,
            0, Camera::fy_, Camera::cy_,
            0, 0, 1;
        cv::eigen2cv(Camera::K_, Camera::cvK_);
        Camera::cvK_.convertTo(Camera::cvK_, CV_32FC1);
        if (Camera::k3_)
        {
            Camera::D_ = (cv::Mat_<float>(5, 1) << Camera::k1_, Camera::k2_, Camera::p1_, Camera::p2_, Camera::k3_);
        }
        else
        {
            Camera::D_ = (cv::Mat_<float>(4, 1) << Camera::k1_, Camera::k2_, Camera::p1_, Camera::p2_);
        }

        // 图像边界坐标初始化
        initImageBounds();

        switch (Camera::Type_)
        {
        // 双目相机
        case Camera::eType::STEREO:
        {
            // 双目校正参数（kitti数据集已经去除畸变所以没有该项，EuRoc数据集未去畸变需包含该项）
            cv::Mat P_l, P_r, R_l, R_r;
            fs1["LEFT.K"] >> Camera::K_l_;
            fs1["RIGHT.K"] >> Camera::K_r_;

            fs1["LEFT.P"] >> P_l;
            fs1["RIGHT.P"] >> P_r;

            fs1["LEFT.R"] >> R_l;
            fs1["RIGHT.R"] >> R_r;

            fs1["LEFT.D"] >> Camera::D_l_;
            fs1["RIGHT.D"] >> Camera::D_r_;

            int rows_l = fs1["LEFT.height"];
            int cols_l = fs1["LEFT.width"];
            int rows_r = fs1["RIGHT.height"];
            int cols_r = fs1["RIGHT.width"];

            // 构建双目重映射矩阵
            if (!Camera::K_l_.empty())
            {
                cv::initUndistortRectifyMap(Camera::K_l_, Camera::D_l_, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, m1l_, m2l_);
                cv::initUndistortRectifyMap(Camera::K_r_, Camera::D_r_, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, m1r_, m2r_);
            }
            break;
        }
        // RGBD相机
        case Camera::eType::RGBD:
        {
            // 深度比例转换
            fs1["Camera.depth_factor"] >> Camera::depth_factor_;
            if (Camera::depth_factor_ == 0)
            {
                cerr << "请在配置文件中填入深度比例因子！" << endl;
                return false;
            }
            if (fabs(Camera::depth_factor_) < 1e-5)
                Camera::depth_factor_ = 1;
            else
                Camera::depth_factor_ = 1.0f / Camera::depth_factor_;
            break;
        }
        case Camera::eType::MONOCULAR:
        {
            break;
        }
        }

        // 相机相对于载体的位姿关系矩阵
        Mat cv_T_BS;
        Mat44 T_BS;
        fs1["T_BS"] >> cv_T_BS;
        if (cv_T_BS.empty())
            fs1["Left_T_BS"] >> cv_T_BS;
        if (!cv_T_BS.empty())
            cv::cv2eigen(cv_T_BS, T_BS);

        Eigen::Quaterniond q(T_BS.block<3, 3>(0, 0));
        q.normalized();
        T_BS_ = SE3(q, T_BS.block<3, 1>(0, 3));
        if (dataset_type_ == VIRTUAL)
        {
            Eigen::AngleAxisd rot_vecZ(M_PI / 2, Vec3(0, 0, -1));
            Eigen::AngleAxisd rot_vecX(M_PI / 2, Vec3(-1, 0, 0));
            Mat33 T_rot = rot_vecZ.matrix() * rot_vecX.matrix();
            Eigen::Quaterniond q(T_rot);
            q.normalized();
            T_BS_ = SE3(q, Vec3(0, 0, 0));
        }

        /*读取模块参数*/
        cv::FileStorage fs2(workspace_abs_dir_ + "config/Param.yaml", FileStorage::READ);
        if (!fs2.isOpened())
        {
            cerr << "模块配置文件路径错误或文件不存在！" << endl;
            return false;
        }
        // ORBextractor类
        fs2["ORBextractor.scaleFactor"] >> Param::scale_;
        fs2["ORBextractor.nLevels"] >> Param::nlevels_;
        fs2["ORBextractor.iniThFAST"] >> Param::iniThFast_;
        fs2["ORBextractor.minThFAST"] >> Param::minThFast_;
        ORBextractor::initStaticParam();

        // Odometry类
        fs2["Odometry.LastCurrMethod"] >> Param::calc_method_;
        fs2["Odometry.MaxNumLost"] >> Param::max_lost_;
        Odometry::calc_method_ = Odometry::CalcMethod(Param::calc_method_);
        switch (Camera::Type_)
        {
        case Camera::eType::STEREO:
        case Camera::eType::RGBD:
        {
            // Odometry::normalpose_max_t_ = Camera::base_;
            Odometry::normalpose_max_t_ = 0.5;
            Odometry::key_max_t_ = Camera::base_;
        }
        break;
        case Camera::eType::MONOCULAR:
        {
            Odometry::normalpose_max_t_ = 0.5; // TODO：考虑这些参数如何设
            Odometry::key_max_t_ = 0.3;
        }
        break;
        }
        Odometry::normalpose_max_R_ = 15 * PI / 180.0;
        int min_val = std::min(Camera::width_, Camera::height_);
        Odometry::key_max_R_ = 0.2 * atan2(4 * Camera::fx_ * min_val, 4 * Camera::fx_ * Camera::fx_ - min_val * Camera::width_); // 视场角的20%

        // DirectTracker类
        fs2["DirectTracker.pyramids"] >> Param::pyramids_;
        fs2["DirectTracker.pyramid_scale"] >> Param::pyramid_scale_;
        fs2["DirectTracker.half_patch_size"] >> Param::half_patch_;
        float scale = 1;
        for (int i = 0; i < Param::pyramids_; i++)
        {
            DirectTracker::scales_.push_back(scale);
            scale *= Param::pyramid_scale_;
        }

        // Mapping类
        fs2["Mapping.LocalMapSize"] >> Param::map_size_;

        // System类
        fs2["System.ModelType"] >> Param::model_type_;
        model_type_ = System::ModelType(Param::model_type_);

        // ObjDetector类
        fs2["Detector.cfgfile"] >> Param::detector_config_;
        fs2["Detector.weightfile"] >> Param::detector_weight_;
        fs2["Detector.namesfile"] >> Param::detector_names_;
        fs2["Detector.thresh"] >> Param::detector_thr_;
        fs2["Detector.nms_thresh"] >> Param::detector_nmsthr_;
        fs2["Detector.inp_width"] >> Param::detector_inpwidth_;
        fs2["Detector.inp_height"] >> Param::detector_inpheight_;

        // ObjSegment类
        fs2["Segment.cfgfile"] >> Param::segment_config_;
        fs2["Segment.weightfile"] >> Param::segment_weight_;
        fs2["Segment.namesfile"] >> Param::segment_names_;
        fs2["Segment.thresh"] >> Param::segment_thr_;
        fs2["Segment.nms_thresh"] >> Param::segment_nmsthr_;
        fs2["Segment.mask_thresh"] >> Param::segment_maskthr_;

        // MotionClassifier类
        fs2["Classifier.min_area_ratio"] >> Param::min_area_ratio_;
        fs2["Classifier.numpts_area_ratio"] >> Param::numpts_area_ratio_;
        fs2["Classifier.halfpatch_size"] >> Param::halfpatch_size_;
        fs2["Classifier.dyna_patch_ncc"] >> Param::dyna_patch_ncc_;
        fs2["Classifier.dyna_patch_std"] >> Param::dyna_patch_std_;
        fs2["Classifier.dyna_pos_change"] >> Param::dyna_pos_change_;
        fs2["Classifier.keeppts_ratio"] >> Param::keeppts_ratio_;
        fs2["Classifier.match_IOU"] >> Param::match_IOU_;
        fs2["Classifier.drawMatchObject"] >> Param::drawMatchObj_;
        fs2["Classifier.drawlocateObject"] >> Param::drawLocateObj_;
        fs2["Classifier.drawMissdDetect"] >> Param::drawMissDetect_;
        fs2["Classifier.drawGeometry"] >> Param::drawGeometry_;
        fs2["Classifier.drawAppearance"] >> Param::drawAppearance_;
        fs2["Classifier.drawPosition"] >> Param::drawPosition_;
        MotionClassifier::min_area_ = Camera::height_ * Camera::width_ * 0.002;
        MotionClassifier::max_area_ = Camera::height_ * Camera::width_ * 0.4;
        MotionClassifier::dyna_epipolar_dis_ = 10.0 / Camera::fps_;
        MotionClassifier::dyna_ratio_ = 0.5;

        // 计算特征点的最小最大深度
        switch (Camera::Type_)
        {
        case Camera::eType::STEREO:
        case Camera::eType::RGBD:
        {
            Frame::min_depth_ = Camera::base_fx_ / (Camera::max_X_ - Camera::min_X_);
            Frame::max_depth_ = Camera::base_fx_;
        }
        break;
        case Camera::eType::MONOCULAR:
        {
            Frame::min_depth_ = 0; // TODO：考虑这些参数如何设
            Frame::max_depth_ = 400;
        }
        }
        return true;
    }

    /**
 * @brief 计算去畸变后图像四个角点的坐标
 * 
 * @return true
 * @return false 
 */
    bool System::initImageBounds()
    {
        if (!Camera::k1_)
        {
            Camera::min_X_ = Camera::min_Y_ = 0;
            Camera::max_X_ = Camera::width_;
            Camera::max_Y_ = Camera::height_;
            return true;
        }

        // 构建图像四个角点坐标数组
        // 矫正前四个边界点：(0,0) (cols,0) (0,rows) (cols,rows)
        cv::Mat mat(4, 2, CV_32F);
        mat.ptr<float>(0)[0] = 0.0; //左上
        mat.ptr<float>(0)[1] = 0.0;
        mat.ptr<float>(1)[0] = Camera::width_; //右上
        mat.ptr<float>(1)[1] = 0.0;
        mat.ptr<float>(2)[0] = 0.0; //左下
        mat.ptr<float>(2)[1] = Camera::height_;
        mat.ptr<float>(3)[0] = Camera::width_; //右下
        mat.ptr<float>(3)[1] = Camera::height_;

        // 角点去畸变
        cv::undistortPoints(mat, mat, Camera::cvK_, Camera::D_, cv::Mat(), Camera::cvK_);

        // 选取最小和最大的边界坐标
        Camera::min_X_ = min(mat.ptr<float>(0)[0], mat.ptr<float>(2)[0]); //左上和左下横坐标最小的
        Camera::max_X_ = max(mat.ptr<float>(1)[0], mat.ptr<float>(3)[0]); //右上和右下横坐标最大的
        Camera::min_Y_ = min(mat.ptr<float>(0)[1], mat.ptr<float>(1)[1]); //左上和右上纵坐标最小的
        Camera::max_Y_ = max(mat.ptr<float>(2)[1], mat.ptr<float>(3)[1]); //左下和右下纵坐标最小的
    }

    /**
     * @brief 初始化神经网络
     * 
     * @param detector yolov3
     * @param segment maskrcnn
     */
    bool System::initNetWork(Detector::Ptr detector, Segment::Ptr segment)
    {
        switch (model_type_)
        {
        case NONE:
            break;
        case YOLOv3:
            detector->loadNetWork();
            break;
        case MASKRCNN:
            segment->loadNetWork();
            break;
        case SOLO:
#ifndef _ROS_
            cerr << "未使用ROS编译，不能使用SOLO！" << endl;
            return false;
#endif
            break;
        }
        return true;
    }

    /**
 * @brief 读取associate文件中每张图象的路径及时间戳，读取verificate检验文件
 * 为System的dataset_dir_,left_times_,right_times_,left_files_,right_files_赋值, 并打开associate_文件
 * 输入:config_file_文件
 * 输出:是否成功打开associate_以及是否有左目和右目图象的时间戳和相应路径
*/
    bool System::initImageDataset()
    {
        FileStorage fs1(workspace_abs_dir_ + "default.yaml", FileStorage::READ);
        fs1["dataset_dir"] >> dataset_dir_;
        if (dataset_dir_.empty())
        {
            cerr << "请在default.yaml文件中填入数据集路径！" << endl;
            return false;
        }
        associate_.open(dataset_dir_ + "/associate.txt");
        groundtruth_.open(dataset_dir_ + "/groundtruth.txt");
        if (!associate_)
        {
            cerr << "associate.txt文件不存在！" << endl;
            // return false;
        }
        if (!groundtruth_)
        {
            // has_truth_ = false;
            // cout << "真值文件不存在！" << endl;
        }

        // 读取所有帧的两张图片的路径
        while (dataset_dir_.back() != '/')
        {
            dataset_dir_.pop_back();
        }
        cout << "dataset: " << dataset_dir_ << endl;
        while (!associate_.eof() && associate_.good())
        {
            string time_stamp, img0_file, img1_file;
            associate_ >> time_stamp >> img0_file >> img1_file;
            if (time_stamp.size() == 0)
                break;
            time_stamps_.push_back(atof(time_stamp.c_str()));
            img0_files_.push_back(dataset_dir_ + img0_file);
            img1_files_.push_back(dataset_dir_ + img1_file);
        }

        if (img0_files_.empty() || img1_files_.empty())
        {
            // cerr << "左目图像或右目图像不存在！" << endl;
            // return false;
        }
        else
        {
            cout << "共读取" << img0_files_.size() << " 张图像" << endl;
            return true;
        }
    }

    /**
 * @brief 初始化视频接口
 * 输入:
 * 输出:是否成功初始化视频接口的
 * 可有可无,如无必要可以删掉
*/
    bool System::initVideoInterface()
    {
    }

    bool System::readImage(int num, Frame::Ptr frame, std::vector<unsigned char> left, std::vector<unsigned char> right)
    {
        curr_ = frame;

        switch (file_type_)
        {
        case IMAGE:
        {
            if (!readImageEach(num, frame, left, right))
                return false;
            break;
        }
        case VIDEO:
        {
            if (!readImageEach(frame, left, right))
            {
                cerr << "read image from video failed!" << endl;
                return false;
            }
            break;
        }
        }
        return true;
    }

    /**
     * @brief 判断是否需要重新校正
     * 
     */
    bool System::needRecalibStereo()
    {
        if (index_ == 0)
            return false;
        if (last_->num_features_ < 0.25 * ORBextractor::nfeatures)
            return true;
    }

    /**
     * @brief 重新校正双目外参
     * 
     */
    void System::recalibStereo()
    {
        // 计算左右目的基础矩阵
        vector<cv::Point2f> left_pts, right_pts;
        for (int i = 0; i < last_->features_.size(); i++)
        {
            Feature::Ptr ft = last_->features_[i];
            if (ft->stereo_)
            {
                left_pts.push_back(ft->pt_);
                right_pts.push_back(Point2f(ft->x_r_, ft->y_));
            }
        }
        Mat F = cv::findFundamentalMat(Mat(left_pts), Mat(right_pts));

        // 双目重标定
        cv::Size imgSize(Camera::width_, Camera::height_);
        Mat H_l, H_r;
        stereoRectifyUncalibrated(Mat(left_pts), Mat(right_pts), F, imgSize, H_l, H_r, 3);

        // 计算外参
        Mat R_l, R_r, P_l, P_r;
        R_l = Camera::K_l_.inv() * H_l * Camera::K_l_;
        R_r = Camera::K_r_.inv() * H_r * Camera::K_r_;
        P_l = Camera::K_l_;
        P_r = Camera::K_r_;

        // 重新构建双目重映射矩阵
        cv::initUndistortRectifyMap(Camera::K_l_, Camera::D_l_, R_l, P_l, imgSize, CV_32F, m1l_, m2l_);
        cv::initUndistortRectifyMap(Camera::K_r_, Camera::D_r_, R_r, P_r, imgSize, CV_32F, m1r_, m2r_);
    }

    bool System::isVideoOpen()
    {
    }

    /**
 * @brief 从文件中读取每张图像
 * 根据initImageDataset()中的left_files_,right_files_和left_times_读取每张图象,并赋给Frame类中的left_,right_和time_stamp_
 * 输入:第i张, Frame类
 * 输出:是否存在相应图象
 * @param i 
 * @param frame 
*/
    bool System::readImageEach(int i, Frame::Ptr frame, std::vector<unsigned char> left, std::vector<unsigned char> right)
    {
        // index_ = i;
        // left_ = cv::imread(img0_files_[i], IMREAD_UNCHANGED);
        cv::Mat left_(left), img_split[4], img_merge[3];
        left_ = left_.reshape(4, 720).clone();
        cv::split(left_, img_split);
        img_merge[0] = img_split[0];
        img_merge[1] = img_split[1];
        img_merge[2] = img_split[2];

        cv::Mat image;
        cv::merge(img_merge, 3, image);
        left_ = image;

        if (left_.data == nullptr)
        {
            cerr << "read left image: " << i << " failed!" << endl;
            return false;
        }
        // cout << img0_files_[i] << endl;

#ifdef _ROS_
        // 发送图像路径
        if (model_type_ == SOLO)
            publishImgMsg(img0_files_[i]);
#endif

        switch (Camera::Type_)
        {
        // 双目相机
        case Camera::eType::STEREO:
        {
            // right_ = cv::imread(img1_files_[i], IMREAD_UNCHANGED);
            cv::Mat right_(right), img_split[4], img_merge[3];
            right_ = right_.reshape(4, 720).clone();

            cv::split(right_, img_split);
            img_merge[0] = img_split[0];
            img_merge[1] = img_split[1];
            img_merge[2] = img_split[2];

            cv::Mat image;
            cv::merge(img_merge, 3, image);
            right_ = image;

            if (right_.data == nullptr)
            {
                cerr << "read right image: " << i << " failed!" << endl;
                return false;
            }

            // 判断是否需要重新校正
            // if (needRecalibStereo())
            // {
            //     recalibStereo();
            // }

            // 双目校正
            if (!m1l_.empty())
            {
                // cv::remap(left_, left_, m1l_, m2l_, cv::INTER_LINEAR);
                // cv::remap(right_, right_, m1r_, m2r_, cv::INTER_LINEAR);
                // cv::imshow("left", left_);
                // cv::imshow("right", right_);
                // cv::waitKey(0);
            }
            frame->right_ = right_;
            break;
        }
        // RGBD相机
        case Camera::eType::RGBD:
        {
            depth_ = cv::imread(img1_files_[i], IMREAD_UNCHANGED); // TUM的深度图是16bit，读取时按原格式读取
            if (depth_.data == nullptr)
            {
                cerr << "read depth image: " << i << " failed!" << endl;
                return false;
            }

            // 将视差图转为深度图
            if ((fabs(Camera::depth_factor_ - 1.0f) > 1e-5) || depth_.type() != CV_32F)
            {
                depth_.convertTo(depth_, CV_32F, Camera::depth_factor_);
            }
            frame->depth_ = depth_;
            break;
        }
        case Camera::eType::MONOCULAR:
        {
            break;
        }
        }

        // TODO：对图像进行直方图均衡化
        frame->left_ = left_;
        // frame->time_stamp_ = time_stamps_[i];
        return true;
    }

    /**
 * @brief 从视频接口读取每张图像
 * 从初始化后的视频接口中读取一帧图象以及对应的时间戳并赋给Frame类中的left_,right_和time_stamp_
 * 输入:Frame类
 * 输出:是否成功从视频接口中读取一帧图象
 * @param frame 
*/

    bool System::readImageEach(Frame::Ptr frame, std::vector<unsigned char> left, std::vector<unsigned char> right)
    {
    }

#ifdef _ROS_
    /**
     * @brief 将图像路径发送出去
     * 
     */
    void System::publishImgMsg(const string &img_path)
    {
        solo::img msg;
        msg.name = img_path;
        img_pub_.publish(msg);
    }
    /**
     * @brief 接收SOLO的分割结果，并生成新的目标加入到当前帧中
     * 
     */
    void System::subscribeTargets(const solo::target &msg)
    {
        // 生成新目标
        int num = msg.num;
        for (int i = 0; i < num; i++)
        {
            Object::Ptr obj(new myslam::Object);
            obj->left_ = msg.left[i];
            obj->right_ = msg.right[i];
            obj->top_ = msg.top[i];
            obj->bot_ = msg.bot[i];
            obj->score_ = msg.score[i];
            obj->area_ = msg.area[i];
            obj->class_id_ = msg.class_id[i];
            obj->label_ = msg.label[i];
            obj->r_ = rand() % (255 + 1);
            obj->b_ = rand() % (255 + 1);
            obj->g_ = rand() % (255 + 1);
            curr_->objects_.push_back(obj);
        }
        curr_->mobjs_matched_ = vector<Object::Ptr>(curr_->objects_.size(), nullptr);

        // 当前帧目标掩码
        uint8_t mask[msg.mask.size()];
        memcpy(mask, &msg.mask[0], msg.mask.size() * sizeof(msg.mask[0]));
        curr_->mask_ = Mat(Camera::height_, Camera::width_, CV_8UC1, mask, (size_t)Camera::width_ * sizeof(uint8_t));

        // cv::imshow("mask", (curr_->mask_ != 0));
        // cv::waitKey(0);
        if (msg.name == img0_files_[index_])
            receive_target_ = true;
    }
#endif

    /**
     * @brief 运行神经网络
     * 
     * @param detector 
     * @param segment 
     * @param curr
     */
    void System::runNetWork(Detector::Ptr detector, Segment::Ptr segment, Frame::Ptr curr)
    {
        switch (model_type_)
        {
        case NONE:
            break;
        case YOLOv3:
            detector->detectObject(curr);
            break;
        case MASKRCNN:
            segment->segmentObject(curr);
            break;
        case SOLO:
#ifdef _ROS_
            while (receive_target_ == false)
                ;
            receive_target_ = false;
#endif
            break;
        }
    }

    /**
 * @brief 读取校验文件
 * 
 */
    void System::readTruth(Frame::Ptr curr)
    {
        if (!has_truth_ || groundtruth_.eof())
            return;

        switch (dataset_type_)
        {
        case KITTI:
        {
            readTruthFromKitti(curr);
            break;
        }
        case EUROC:
        {
            readTruthFromEuroc(curr);
            break;
        }
        case TUM:
        {
            readTruthFromTUM(curr);
            break;
        }
        case VIRTUAL:
        {
            readTruthData();
            break;
        }
        case OTHER:
            break;
        }

        // 载体的真实位姿转换到相机系下
        truth_T_ = truth_T_ * T_BS_;

        // 如果是第一帧，则保存第一帧位姿
        if (curr->id_ == 0)
        {
            init_truth_T_ = truth_T_.inverse();
        }

        // 真实位姿转换到原点
        truth_T_ = init_truth_T_ * truth_T_;

        curr->T_truth_ = truth_T_;
    }

    void System::readTruthData()
    {
        float tx, ty, tz, qx, qy, qz, qw;
        groundtruth_ >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalized();
        Vec3 t(tx, ty, tz);
        truth_T_ = SE3(q, t);
        return;
    }

    /**
 * @brief 读取kitti数据集的真值文件
 * 
 */
    void System::readTruthFromKitti(Frame::Ptr curr)
    {
        /*不带惯导数据的真值文件*/
        double temp[12];
        groundtruth_ >> temp[0] >> temp[1] >> temp[2] >> temp[3] >>
            temp[4] >> temp[5] >> temp[6] >> temp[7] >>
            temp[8] >> temp[9] >> temp[10] >> temp[11];

        Mat34 T;
        T << temp[0], temp[1], temp[2], temp[3],
            temp[4], temp[5], temp[6], temp[7],
            temp[8], temp[9], temp[10], temp[11];
        Eigen::Quaterniond q(T.block<3, 3>(0, 0));
        q.normalized();
        unique_lock<std::mutex> lck1(mute_T_);
        truth_T_ = SE3(q, T.col(3));

        // /*带惯导数据的真值文件*/
        // // 找到时间上最接近的真值
        // string curr_fline, last_fline;
        // double timestamp;
        // double min_diff = 1000000;
        // double last_diff = 1000000;
        // getline(groundtruth_, curr_fline);
        // while (!groundtruth_.eof())
        // {
        //     timestamp = stod(curr_fline.substr(0, 13));
        //     float curr_diff = abs(timestamp - curr->time_stamp_);
        //     if (curr_diff < min_diff)
        //         min_diff = curr_diff;
        //     if (curr_diff > last_diff)
        //         break;

        //     last_diff = curr_diff;
        //     last_fline = curr_fline;
        //     getline(groundtruth_, curr_fline);
        // }
        // // cout << last_fline << endl;

        // // 分割字符串
        // stringstream ss(last_fline);
        // double temp[7]; // 时间戳 lat lon alt roll pitch yaw
        // for (int i = 0; i < 7; i++)
        // {
        //     ss >> temp[i];
        // }

        // // 欧拉角转四元数
        // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(temp[4], Vec3::UnitX()));
        // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(temp[5], Vec3::UnitY()));
        // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(temp[6], Vec3::UnitZ()));
        // Eigen::Quaterniond q;
        // q = yawAngle * pitchAngle * rollAngle;
        // q.normalized();

        // // 大地坐标转直角坐标
        // Vec3 XYZ = Transform::BLH2XYZ(Vec3(temp[1] * PI / 180.0, temp[2] * PI / 180.0, temp[3]));

        // unique_lock<std::mutex> lck1(mute_T_);
        // truth_T_ = SE3(q, XYZ);
    }

    /**
 * @brief 读取Euroc数据集的真值文件
 * 
 */
    void System::readTruthFromEuroc(Frame::Ptr curr)
    {
        string curr_fline, last_fline;
        long timestamp;
        long min_diff = LONG_MAX;
        long last_diff = LONG_MAX;

        // 去除头部
        getline(groundtruth_, curr_fline);
        while (curr_fline[0] == '#')
        {
            getline(groundtruth_, curr_fline);
        }

        // 找到时间上最接近的
        while (!groundtruth_.eof())
        {
            timestamp = stol(curr_fline.substr(0, 20)); //////
            long curr_diff = abs(timestamp - curr->time_stamp_);
            if (curr_diff < min_diff)
                min_diff = curr_diff;
            if (curr_diff > last_diff)
                break;

            last_diff = curr_diff;
            last_fline = curr_fline;
            getline(groundtruth_, curr_fline);
        }
        // cout << last_fline << endl;

        // 分割一行字符串
        string item;
        stringstream ss;
        ss.str(last_fline);
        vector<float> data;
        getline(ss, item, ',');
        timestamp = stol(item);
        for (int i = 0; i < 7; i++)
        {
            getline(ss, item, ',');
            data.push_back(stof(item));
        }

        // cout << timestamp << " " << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << "  " << data[6] << endl;
        Eigen::Quaterniond q(data[3], data[4], data[5], data[6]);
        q.normalized();
        Vec3 t(data[0], data[1], data[2]);
        unique_lock<std::mutex> lck1(mute_T_);
        truth_T_ = SE3(q, t);
    }

    /**
 * @brief 读取TUM数据集的真值文件
 * 
 */
    void System::readTruthFromTUM(Frame::Ptr curr)
    {
        string curr_fline, last_fline;
        double timestamp;
        float min_diff = 1000000;
        float last_diff = 1000000;

        // 去除头部
        getline(groundtruth_, curr_fline);
        while (curr_fline[0] == '#')
        {
            getline(groundtruth_, curr_fline);
        }

        // 找到时间上最接近的真值数据
        while (!groundtruth_.eof())
        {
            timestamp = stod(curr_fline.substr(0, 16));

            float curr_diff = abs(timestamp - curr->time_stamp_);
            if (curr_diff < min_diff)
                min_diff = curr_diff;
            if (curr_diff > last_diff)
                break;

            last_diff = curr_diff;
            last_fline = curr_fline;
            getline(groundtruth_, curr_fline);
        }

        // 分割字符串，真值位姿赋值
        string item;
        stringstream ss;
        ss.str(last_fline);
        float tx, ty, tz, qx, qy, qz, qw;
        ss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        // cout << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " " << qw << endl;
        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalized();
        Vec3 t(tx, ty, tz);
        unique_lock<std::mutex> lck1(mute_T_);
        truth_T_ = SE3(q, t);
    }

    /**
 * @brief 更新GUI的参数
 * 
 * @param curr_ 
 */
    void System::updateGUI(Frame::Ptr curr, Map::Ptr local_map)
    {
        unique_lock<std::mutex> lck1(mute_T_);

        // 当前帧位姿
        curr_T_ = curr->T_wc_;
        curr_vecT_.push_back(curr_T_);

        // 关键帧位姿序列
        if (curr->is_key_)
        {
            key_vecT_.push_back(curr_T_);
            // TODO：实现更新优化后位姿
        }

        // 校验帧位姿序列
        truth_vecT_.push_back(truth_T_);

        // 局部地图帧位姿序列
        mframes_vecT_ = local_map->getAllFramesPoses();

        // 地图点坐标序列
        map_points_ = local_map->getAllMapPointsCoors();

        // 地图目标世界坐标
        obj_coors_ = local_map->getAllObjectsCoors();

        // 地图目标运动轨迹
        // obj_trajs_ = local_map->getAllObjectsTrajs();
    }

    /**
     * @brief 更新帧
     * 
     * @param curr 当前帧
     */
    void System::updateFrame(Frame::Ptr curr)
    {
        last_ = curr;
        if (curr->is_key_)
            key_ = curr;
    }

    /**
 * @brief 计算当前帧估计位姿结果与校验帧的误差
 * 
 */
    void System::calcCurrError(Frame::Ptr curr)
    {
        if (!curr->is_good_)
        {
            num_lost_++;
            lost_frame_.push_back(curr->id_);
        }

        if (has_truth_ && curr_->id_ != 0)
        {
            // 计算绝对误差
            float t_ate, R_ate;
            Func::calcReltvRt(curr_T_, truth_T_, R_ate, t_ate);

            cout << "ATE：" << t_ate << " 米" << endl;
        }

        // 输出当前里程
        int sz = truth_vecT_.size();
        if (sz != 1)
        {
            if (has_truth_)
            {
                total_dist_ += (truth_vecT_.back().translation() - truth_vecT_[sz - 2].translation()).norm();
            }
            else
            {
                total_dist_ += (curr_vecT_.back().translation() - curr_vecT_[sz - 2].translation()).norm();
            }
            cout << "当前里程：" << total_dist_ << "米" << endl;
        }
    }

    /**
 * @brief 估计结果输出
 * 
 */
    void System::outputResult()
    {
        cout << "*************************************" << endl;
        string path;
#ifdef _ROS_
        path = "src/VSLAM/config/";
#else
        path = "config/";
#endif
        cv::FileStorage fs(path + "default.yaml", FileStorage::READ);
        string result_dir = fs["result_dir"];
        cout << "result_dir: " << result_dir << endl;

        time_t now_time = time(NULL);
        tm *t_tm = localtime(&now_time);
        string time_now = asctime(t_tm);

        ofstream esti_result(result_dir + "/esti_result.txt");
        esti_result << "# estimate result" << endl;
        esti_result << "# dataset path: " + dataset_dir_ << endl;
        esti_result << "# time: " + time_now;
        esti_result << "# timestamp tx ty tz qx qy qz qw" << endl;
        for (int i = 0; i < curr_vecT_.size(); i++)
        {
            Vec3 t = curr_vecT_[i].translation();
            Eigen::Quaterniond q = curr_vecT_[i].unit_quaternion();
            esti_result << setiosflags(ios::fixed) << setprecision(4) << to_string(time_stamps_[i]) << " " << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        if (has_truth_)
        {
            ofstream truth_result(result_dir + "/truth_result.txt");
            truth_result << "# ground truth" << endl;
            truth_result << "# dataset path: " + dataset_dir_ << endl;
            truth_result << "# time: " + time_now;
            truth_result << "# timestamp tx ty tz qx qy qz qw" << endl;
            for (int i = 0; i < truth_vecT_.size(); i++)
            {
                Vec3 t = truth_vecT_[i].translation();
                Eigen::Quaterniond q = truth_vecT_[i].unit_quaternion();
                truth_result << setiosflags(ios::fixed) << setprecision(4) << to_string(time_stamps_[i]) << " " << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }
    }

    /**
 * @brief 计算全程误差
 * 
 */
    void System::calcTotalRMSE()
    {
        if (!has_truth_)
            cout << "没有真值位姿无法评估全程精度" << endl;

        t2_ = clock();

        float max_t_ate = 0, max_t_rpe = 0, t_total_ate = 0, R_total_ate = 0, t_total_rpe = 0, R_total_rpe = 0;
        float total_t = 0;
        float t_ate, R_ate, t_rpe, R_rpe, T_rpe;
        for (size_t i = 1; i < curr_vecT_.size(); i++)
        {
            if (has_truth_)
            {
                // 计算绝对误差
                Func::calcReltvRt(curr_vecT_[i], truth_vecT_[i], R_ate, t_ate);
                t_total_ate += t_ate * t_ate;
                R_total_ate += R_ate * R_ate;
                if (max_t_ate < t_ate)
                    max_t_ate = t_ate;

                // 计算相对误差
                SE3 esti_reltv = curr_vecT_[i - 1].inverse() * curr_vecT_[i];
                SE3 test_reltv = truth_vecT_[i - 1].inverse() * truth_vecT_[i];
                Func::calcReltvRt(esti_reltv, test_reltv, R_rpe, t_rpe);
                t_total_rpe += t_rpe * t_rpe;
                R_total_rpe += R_rpe * R_rpe;
                if (max_t_rpe < t_rpe)
                    max_t_rpe = t_rpe;

                total_t += (truth_vecT_[i].translation() - truth_vecT_[i - 1].translation()).norm();
            }
            else
            {
                total_t += (curr_vecT_[i].translation() - curr_vecT_[i - 1].translation()).norm();
            }
        }

        cout << "全程共 " << total_t << " 米" << endl;
        if (has_truth_)
        {
            t_total_ate /= (curr_vecT_.size() - 1);
            t_total_ate = sqrt(t_total_ate);

            R_total_ate /= (curr_vecT_.size() - 1);
            R_total_ate = sqrt(R_total_ate);

            t_total_rpe /= (curr_vecT_.size() - 1);
            t_total_rpe = sqrt(t_total_rpe);

            R_total_rpe /= (curr_vecT_.size() - 1);
            R_total_rpe = sqrt(R_total_rpe);

            cout << "最大绝对位置误差为：" << max_t_ate << " 米" << endl;
            cout << "全程的ATE_RMSE为：" << t_total_ate << endl;
            cout << "全程的RTE_RMSE为：" << t_total_rpe << endl;
        }

        cout << "里程计跟踪失败的帧数为：" << num_lost_;
        if (num_lost_ != 0)
        {
            cout << " 分别为：";
            for (auto iter : lost_frame_)
            {
                cout << iter << " ";
            }
        }
        cout << endl;

        double total_time = (double)(t2_ - t1_) / CLOCKS_PER_SEC;
        cout << "全程共用时：" << total_time << "秒    平均每帧" << total_time / (curr_vecT_.size() - 1) << "秒" << endl;
    }
} // namespace myslam