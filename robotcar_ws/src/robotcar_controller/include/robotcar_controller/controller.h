#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>
#include <mutex>
#include <time.h>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <robotcar_general/print_helper.h>

class Controller
{
public:
  Controller();

  ~Controller();

  // 根据轨迹信息生成并下发控制指令
  void ctrlRobotCar();

private:
  // 接收运动规划模块发送的轨迹信息
  void trajCallback(const trajectory_msgs::JointTrajectory& msg);

  // 话题订阅
  static void spinTopic();

  std::mutex mu_;

  tf::TransformListener tf_;
  
  geometry_msgs::Twist v_;
  ros::Subscriber traj_sub_;
  ros::Publisher v_pub_;

  trajectory_msgs::JointTrajectory traj_w_;

  std::ofstream of_;

  bool receive_traj_;
  bool record_;
  float rec_dt_;
  clock_t last_t_;
  double lookahead_dis_;
};

#endif /* CONTROLLER_H */
