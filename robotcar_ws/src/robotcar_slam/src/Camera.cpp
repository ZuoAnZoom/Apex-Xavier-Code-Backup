#include "myslam/Camera.h"

namespace myslam
{
    Camera::eType Camera::Type_ = Camera::eType::STEREO;
    Mat33 Camera::K_ = Mat33::Identity();
    cv::Mat Camera::cvK_ = cv::Mat();
    cv::Mat Camera::D_ = cv::Mat();
    int Camera::height_ = 0;
    int Camera::width_ = 0;
    float Camera::fx_ = 0;
    float Camera::fy_ = 0;
    float Camera::cx_ = 0;
    float Camera::cy_ = 0;
    float Camera::k1_ = 0;
    float Camera::k2_ = 0;
    float Camera::k3_ = 0;
    float Camera::p1_ = 0;
    float Camera::p2_ = 0;
    float Camera::min_X_ = 0;
    float Camera::max_X_ = 0;
    float Camera::min_Y_ = 0;
    float Camera::max_Y_ = 0;
    cv::Mat Camera::K_l_=cv::Mat();
    cv::Mat Camera::K_r_=cv::Mat();
    cv::Mat Camera::D_l_=cv::Mat();
    cv::Mat Camera::D_r_=cv::Mat();
    float Camera::base_ = 0;
    float Camera::base_fx_ = 0;
    float Camera::depth_factor_ = 0;
    int Camera::fps_ = 0;

} // namespace myslam
