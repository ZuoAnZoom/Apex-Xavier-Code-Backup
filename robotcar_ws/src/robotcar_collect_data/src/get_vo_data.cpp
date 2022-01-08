#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>


using namespace std;
using namespace message_filters;

int i = 1;

void getPose(tf::Stamped<tf::Pose>& global_pose)
{
  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = "left_camera";
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();

  tf::TransformListener tf_(ros::Duration(10));
  
  while (1)
  {
    try
    {
      tf_.transformPose("odom", robot_pose, global_pose);
    }
    catch (tf::LookupException& ex)
    {
      continue;
    }
    catch (tf::ConnectivityException& ex)
    {
      continue;
    }
    catch (tf::ExtrapolationException& ex)
    {
      continue;
    }
    
    break;
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr &left_image, const sensor_msgs::ImageConstPtr &right_image)
{
  cv::Mat left_(left_image->data);
  left_ = left_.reshape(3, 720).clone();

  cv::Mat right_(right_image->data);
  right_ = right_.reshape(3, 720).clone();

  ros::Time time = left_image->header.stamp;

  string left_name = "/home/h/data_set/vo_test_data2/left_camera/" + to_string(i) + ".jpg";
  string right_name = "/home/h/data_set/vo_test_data2/right_camera/" + to_string(i) + ".jpg";

  cv::imwrite(left_name, left_);
  cv::imwrite(right_name, right_);


  tf::Stamped<tf::Pose> global_pose;
  getPose(global_pose);
  double x = global_pose.getOrigin().x();
  double y = global_pose.getOrigin().y();
  double z = global_pose.getOrigin().z();
  double q_x = global_pose.getRotation().x();
  double q_y = global_pose.getRotation().y();
  double q_z = global_pose.getRotation().z();
  double q_w = global_pose.getRotation().w();

  ofstream outFile;
  outFile.open("/home/h/data_set/vo_test_data2/true_value.txt", ios::app);
  outFile << x << "  " << y << "  "  << z << "  "  << q_x << "  "  << q_y << "  "  << q_z << "  "  << q_w << endl;
  outFile.close();

  i++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_vo_data");
  ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::Image> left_cam_sub(nh, "left_camera/image_raw", 1, ros::TransportHints().tcpNoDelay());
	message_filters::Subscriber<sensor_msgs::Image> right_cam_sub(nh, "right_camera/image_raw", 1, ros::TransportHints().tcpNoDelay());

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
  Synchronizer<syncPolicy> sync(syncPolicy(10), left_cam_sub, right_cam_sub);  

  sync.registerCallback(boost::bind(&imageCallback, _1, _2));
    
  ros::spin();

  return 0;
}