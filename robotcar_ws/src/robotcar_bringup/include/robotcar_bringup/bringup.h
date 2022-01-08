#ifndef BRINGUP_H
#define BRINGUP_H

#include <mutex>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <robotcar_general/math_tools.h>
#include <robotcar_general/print_helper.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

// PID控制参数
struct PID 
{
  PID():set_goal(0), actual_value(0), err(0), err_last(0),
        kp(0), ki(0), kd(0), control(0), integral(0){}

  float set_goal;                           // 输入变量，即期望输出的变量值
  float actual_value;                       // 实际输出变量，即采样回来的输出变量
  float err;                                // 误差值
  float err_last;                           // 上一次误差值
  float kp, ki, kd;                         // 比例/积分/微分系数
  float control;                            // 定义控制值
  float integral;                           // 积分值
};

class BringUp
{
public:
  BringUp();

  ~BringUp();

  // 初始化
  bool init(std::string password);

private:
  // 填充油门控制帧
  void setThrottleCommand(int throttle_percent, struct can_frame &throttle_command_frame, bool give_up = false);

  // 填充刹车控制帧
  void setBrakeCommand(int brake_percent, struct can_frame &brake_command_frame, bool give_up = false);

  // 填充转向控制帧
  void setSteerCommand(float steer_angle, struct can_frame &steer_command_frame, bool give_up = false);

  // 填充换挡控制帧
  void setGearCommand(int gear, struct can_frame &gear_command_frame);

  // 单帧发送
  void sendCanFrame(struct can_frame send_frame);

  // 单帧处理
  void canFrameProcess(struct can_frame &recv_frame);

  // PID控制：输入目标速度，输出油门/刹车控制量
  std::vector<float> pidCalculate();

  // 订阅速度指令消息的回调函数
  void velCallback(const geometry_msgs::Twist& msg);

  // 订阅控制指令消息(油门/刹车/转向)的回调函数
  void cmdCallback(const std_msgs::Float32MultiArray& msg);

  // 开始接收消息
  static void msgSpin();

  // 构造上升沿，获取车辆控制权
  void getControl();

  // 读线程
  friend void readSpin(BringUp* robot);

  // 写线程
  friend void writeSpin(BringUp* robot);

  // 接收车辆上报帧，持续更新车辆状态信息
  void canRead();

  // 向车辆下发报文(PID控制)
  void canWritePID();

  // 向车辆下发报文(直接控制)
  void canWrite();

  // 车辆速度下发
  float goal_speed_;                            // 目标速度
  float goal_speed_th_;                         // 目标角速度
  float goal_steer_;                            // 目标转向
  float goal_throttle_;                         // 目标油门
  float goal_brake_;                            // 目标刹车
  int max_throttle_;                            // 最大油门
  bool emergency_brake_;                        // 紧急停车

  // 车辆状态上报
  int throttle_percent_;                        // 油门百分比
  int brake_percent_;                           // 刹车百分比
  float steer_angle_;                           // 行车角度（单位：度）
  int gear_;                                    // 档位
  float speed_;                                 // 当前车速（单位：m/s）
  float speed_th_;                              // 角速度
  float acc_;                                   // 当前加速度（单位：m^2/s）

  // 收到速度目标后触发标志位
  std::mutex msg_mu_;                           // 锁: 收取目标指令消息&CAN下发指令报文
  std::mutex serial_mu_;                        // 锁: 串口读&写
  std::mutex ecu_mu_;                           // 锁: ECU信息读取&使用

  // 发布tf和odom话题
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_vel_;
  ros::Subscriber sub_cmd_;
  tf::TransformBroadcaster odom_broadcaster_;

  // neo
  // 发布车辆上报的速度
  ros::Publisher odom_pub_;

  // Socket
  int s_;
  struct sockaddr_can addr_; 
  struct ifreq ifr_; 

  // PID
  int pid_enable_;
  PID pid_;
  float descend_rate_;
  
  // 写线程频率与时间记录
  float write_fq_;
  struct timeval last_time_write_speed_;
};

// 写线程
void writeSpin(BringUp* robot);

// 读线程
void readSpin(BringUp* robot);

#endif /* BRINGUP_H */

