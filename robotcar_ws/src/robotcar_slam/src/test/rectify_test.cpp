#include "myslam/System.h"
#include "myslam/Camera.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotcar_rec");

    myslam::System::Ptr system(new myslam::System);

    system->initSystem();

    // 循环处理每帧图象
    int i = 2000;
    while ((system->file_type_ == myslam::System::FileType::IMAGE) ? i < system->img0_files_.size() : system->isVideoOpen())
    {
        if (i >= 4000)
            break;
        cout << "*********** Image: " << i << " ***********" << endl;

        // 读取一帧图像
        system->left_ = cv::imread(system->img0_files_[i], IMREAD_UNCHANGED);
        if (system->left_.data == nullptr)
        {
            cerr << "read left image: " << i << " failed!" << endl;
            return false;
        }
        cout << system->img0_files_[i] << endl;

        system->right_ = cv::imread(system->img1_files_[i], IMREAD_UNCHANGED);
        if (system->right_.data == nullptr)
        {
            cerr << "read right image: " << i << " failed!" << endl;
            return false;
        }

        // 双目校正
        cv::remap(system->left_, system->left_, system->m1l_, system->m2l_, cv::INTER_LINEAR);
        cv::remap(system->right_, system->right_, system->m1r_, system->m2r_, cv::INTER_LINEAR);

        // 检验校正结果
        Mat left_right;
        hconcat(system->left_, system->right_, left_right);
        for (int j = 0; j < left_right.rows; j += 20)
        {
            int r = rand() % (255 + 1);
            int g = rand() % (255 + 1);
            int b = rand() % (255 + 1);
            cv::line(left_right, Point(0, j), Point(left_right.cols, j), Scalar(b, g, r));
        }
        imshow("left_right", left_right);
        cv::waitKey(0);

        // 校正图像输出
        string path_left = "/home/lihua/Document/实测数据集/201223_VO/left_pro/" + to_string(i) + ".png";
        string path_right = "/home/lihua/Document/实测数据集/201223_VO/right_pro/" + to_string(i) + ".png";
        cv::imwrite(path_left, system->left_);
        cv::imwrite(path_right, system->right_);

        i++;
    }
    return 0;
}
