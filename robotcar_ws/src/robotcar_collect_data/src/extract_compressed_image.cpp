#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

cv::Mat imgCallback;
int i = 0;

static void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{   
  ROS_INFO("Get");
  try
  {
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    imgCallback = cv_ptr_compressed->image;

    string name = "/home/h/data/vo_test_data/left_camera/" + to_string(i) + ".jpg";
    cv::imwrite(name, imgCallback);

    cv::imshow("imgCallback",imgCallback);
    cv::waitKey(1);
    ROS_INFO("cv_ptr_compressed: %d h: %d\n", cv_ptr_compressed->image.cols, cv_ptr_compressed->image.rows);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Error\n");
  }
  i++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_compressed_image");
  ros::NodeHandle nh;
  ros::Subscriber image_sub;
  std::string image_topic = "/left_camera/image_raw/compressed";
  image_sub = nh.subscribe(image_topic,10,ImageCallback);

  ROS_INFO("ROS OK!");
  ros::spin();
  return 0;
}
