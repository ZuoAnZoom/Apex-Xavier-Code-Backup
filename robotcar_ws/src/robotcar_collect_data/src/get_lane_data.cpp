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
#include <geometry_msgs/TransformStamped.h>
#include <robotcar_map/lanes.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace message_filters;

int i = 1;
tf::TransformListener* tf_;

void lanesCallback(const robotcar_map::lanes msg)
{  
  ofstream outFile;
  outFile.open("/home/h/data/lane_test_data/lane_points.txt", ios::app);

  tf_->waitForTransform("odom", "left_camera", msg.stamp, ros::Duration(0.1));
  int num = msg.num;

  if (num == 0)
  {
    return;
  }

  if (num > 0)
  {
    for (int i = 0; i < msg.x_0.size(); i++)
    {
      tf::Stamped<tf::Vector3> point_c(tf::Vector3(msg.x_0[i], msg.y_0[i], 0), msg.stamp, "left_camera");
      tf::Stamped<tf::Vector3> point_w;
      tf_->transformPoint("odom", point_c, point_w);
      double x = point_w.getX();
      double y = point_w.getY();
      outFile << x << "," << y << " ";
    }
    outFile << endl;
  }

  if (num > 1)
  {
    for (int i = 0; i < msg.x_1.size(); i++)
    {
      tf::Stamped<tf::Vector3> point_c(tf::Vector3(msg.x_1[i], msg.y_1[i], 0), msg.stamp, "left_camera");
      tf::Stamped<tf::Vector3> point_w;
      tf_->transformPoint("odom", point_c, point_w);
      double x = point_w.getX();
      double y = point_w.getY();
      outFile << x << "," << y << " ";
    }
    outFile << endl;  
  }

  if (num > 2)
  {
    for (int i = 0; i < msg.x_2.size(); i++)
    {
      tf::Stamped<tf::Vector3> point_c(tf::Vector3(msg.x_2[i], msg.y_2[i], 0), msg.stamp, "left_camera");
      tf::Stamped<tf::Vector3> point_w;
      tf_->transformPoint("odom", point_c, point_w);
      double x = point_w.getX();
      double y = point_w.getY();
      outFile << x << "," << y << " ";
    }
    outFile << endl;  
  }

  if (num > 3)
  {
    for (int i = 0; i < msg.x_3.size(); i++)
    {
      tf::Stamped<tf::Vector3> point_c(tf::Vector3(msg.x_3[i], msg.y_3[i], 0), msg.stamp, "odom");
      tf::Stamped<tf::Vector3> point_w;
      tf_->transformPoint("odom", point_c, point_w);
      double x = point_w.getX();
      double y = point_w.getY();
      outFile << x << "," << y << " ";
    }
    outFile << endl;  
  }
  outFile << endl;  
  outFile.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_lane_data");
  ros::NodeHandle g_nh;
  tf_ = new tf::TransformListener(ros::Duration(10));

  ros::Subscriber lane_sub = g_nh.subscribe("/perception/lanes", 1, lanesCallback);
  sleep(5);
  ros::spin();

  return 0;
}