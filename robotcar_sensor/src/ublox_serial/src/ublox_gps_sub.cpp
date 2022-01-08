#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include "ublox_serial/gps.h"
#include "ublox_serial/ublox.h"

void gpsCallback(const ublox_serial::gps::ConstPtr &msg)
{
  ROS_INFO("header: %s", msg->header.c_str());
  ROS_INFO("gpsTime: %s", msg->gpsTime.c_str());
  ROS_INFO("status: %s", msg->status.c_str());
  ROS_INFO("Lat: %s", msg->lat.c_str());
  ROS_INFO("nsIndicator: %s", msg->nsIndicator.c_str());
  ROS_INFO("lon: %s", msg->lon.c_str());
  ROS_INFO("ewIndicator: %s", msg->ewIndicator.c_str());
  ROS_INFO("spd: %s", msg->spd.c_str());
  ROS_INFO("cog:%s", msg->cog.c_str());
  ROS_INFO("date:%s", msg->date.c_str());
  ROS_INFO("mv:%s", msg->mv.c_str());
  ROS_INFO("mvEW:%s", msg->mvEW.c_str());
  ROS_INFO("poseMode:%s", msg->poseMode.c_str());
  ROS_INFO("navStatus:%s", msg->navStatus.c_str());
  ROS_INFO("checksum:%s", msg->checksum.c_str());
  //ROS_INFO("CrLf:%s", msg->CrLf.c_str());
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "ublox_gps_sub"); //
  ros::NodeHandle n;                 //
  ros::Subscriber sub = n.subscribe("/gps_info", 1, gpsCallback); //创建subscriber
  ros::spin(); //反复调用当前可触发的回调函数，阻塞
  return 0;
}
