#include <cmath>
#include <vector>
#include <thread>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <robotcar_bringup/bringup.h>

// CAN帧ID
#define THROTTLE_COMMAND_CAN_ID   0x0110    // 油门控制
#define BRAKE_COMMAND_CAN_ID      0x0111    // 刹车控制
#define STEER_COMMAND_CAN_ID      0x0112    // 转向控制
#define GEAR_COMMAND_CAN_ID       0x0114    // 档位控制
#define THROTTLE_STATUS_CAN_ID    0x0510    // 油门状态
#define BRAKE_STATUS_CAN_ID       0x0511    // 刹车状态
#define STEER_STATUS_CAN_ID       0x0512    // 转向状态
#define GEAR_STATUS_CAN_ID        0x0514    // 档位状态
#define ECU_STATUS_CAN_ID         0x0515    // 底盘车速和状态

#define PI                        3.1415926

using namespace std;

BringUp::BringUp():speed_(0), speed_th_(0), goal_speed_(0), goal_speed_th_(0),
                   throttle_percent_(0), brake_percent_(0), steer_angle_(0), emergency_brake_(false)
{
  // 从服务器读取参数
  ros::NodeHandle b_nh("bringup");
  b_nh.param("pid_enable", pid_enable_, 0);
  b_nh.param("kp", pid_.kp, 1.8f);
  b_nh.param("ki", pid_.ki, 0.001f);
  b_nh.param("kd", pid_.kd, 10.0f);
  b_nh.param("descend_rate", descend_rate_, 0.8f);
  b_nh.param("write_fq", write_fq_, 2.0f);
  b_nh.param("max_throttle", max_throttle_, 11);

  gettimeofday(&last_time_write_speed_, NULL);
}

BringUp::~BringUp()
{ 
  // 关闭Socket
  close(s_);
}

bool BringUp::init(std::string password)
{
  // 使能CAN0设备
  std::string cmd1 = "";
  if (!password.empty())
    cmd1 = "echo " + password + " | sudo -S modprobe can";
  else
    cmd1 = "sudo modprobe can";
  system(cmd1.c_str());
  system("sudo modprobe can_raw");
  system("sudo modprobe mttcan");
  system("sudo ip link set down can0");
  system("sudo ip link set can0 type can bitrate 500000 berr-reporting on fd off");
  system("sudo ip link set up can0");

  // 初始化Socket
  s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  // 指定CAN0设备
  strcpy(ifr_.ifr_name, "can0");
  ioctl(s_, SIOCGIFINDEX, &ifr_);
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;

  // 绑定Socket_can
  bind(s_, (struct sockaddr *)&addr_, sizeof(addr_)); 
  
  // 发布Odom话题
  pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

  // neo
  odom_pub_ = nh_.advertise<geometry_msgs::Twist>("odom_update", 1);

  // 订阅速度指令
  sub_vel_ = nh_.subscribe("cmd_vel", 1, &BringUp::velCallback, this);

  // 订阅控制指令
  sub_cmd_ = nh_.subscribe("cmd_chassis", 1, &BringUp::cmdCallback, this);

  // 获取车辆控制权
  getControl();

  // 开启话题消息监听线程，持续订阅话题消息并进入回调函数
  thread msg_spin(BringUp::msgSpin);
  msg_spin.detach();

  return true;
}

void BringUp::setThrottleCommand(int throttle_percent, struct can_frame &throttle_command_frame, bool give_up)
{
  if (throttle_percent < 0 || throttle_percent > 100)
  {
    printConfig();
    ROS_INFO("%d不是有效的油门控制比例！\n", throttle_percent);
  }  
  else
  {
    throttle_command_frame.can_id = THROTTLE_COMMAND_CAN_ID;
    throttle_command_frame.can_dlc = 2;
    give_up ? throttle_command_frame.data[0] = 0x00 : throttle_command_frame.data[0] = 0x01;
    throttle_command_frame.data[1] = throttle_percent;
  }
}

void BringUp::setBrakeCommand(int brake_percent, struct can_frame &brake_command_frame, bool give_up)
{
  if (brake_percent < 0 || brake_percent > 100)
  {
    printConfig();
    ROS_INFO("%d不是有效的刹车控制比例！\n", brake_percent);
  }
  else
  {
    brake_command_frame.can_id = BRAKE_COMMAND_CAN_ID;
    brake_command_frame.can_dlc = 2;
    give_up ? brake_command_frame.data[0] = 0x00 : brake_command_frame.data[0] = 0x01;
    brake_command_frame.data[1] = brake_percent;
  }
}

void BringUp::setSteerCommand(float steer_angle, struct can_frame &steer_command_frame, bool give_up)
{
  if (steer_angle > PI / 6 || steer_angle < PI / -6)
  {
    printConfig();
    ROS_INFO("%f度不是有效的转向角度！\n", steer_angle * 180 / PI);
  }
  else
  {
    steer_command_frame.can_id = STEER_COMMAND_CAN_ID;
    steer_command_frame.can_dlc = 3;
    give_up ? steer_command_frame.data[0] = 0x00 : steer_command_frame.data[0] = 0x01;

    steer_angle *= 1000;
    if (steer_angle < 0)
    {
      steer_angle += 0x10000;
    }

    steer_command_frame.data[2] = steer_angle / 256;
    steer_command_frame.data[1] = steer_angle - steer_command_frame.data[2] * 256;
  }
}

void BringUp::setGearCommand(int gear, struct can_frame &gear_command_frame)
{
  if (gear > 4 || gear < 1)
  {
    printConfig();
    ROS_INFO("%d不是有效的控制档位！\n", gear);
  }
  else
  {
    gear_command_frame.can_id = GEAR_COMMAND_CAN_ID;
    gear_command_frame.can_dlc = 1;
    gear_command_frame.data[0] = gear;
  }
}

void BringUp::sendCanFrame(struct can_frame send_frame)
{
  serial_mu_.lock();
  write(s_, &send_frame, sizeof(send_frame));
  serial_mu_.unlock();
}

void BringUp::canFrameProcess(struct can_frame &recv_frame)
{
  switch (recv_frame.can_id)
  {
    case STEER_STATUS_CAN_ID:
    {
      if ((recv_frame.data[2] * 256 + recv_frame.data[1]) * 2 <= 0xFFFF)
      {
        steer_angle_ = (float)(recv_frame.data[2]*256.0 + recv_frame.data[1]) / (1000.0);
      }
      else 
      {
        steer_angle_ = (float)(0xFFFF - recv_frame.data[2]*256.0 - recv_frame.data[1]) * (-1000.0);
      }
    }; break;

    case GEAR_STATUS_CAN_ID:
    {
      gear_ = recv_frame.data[0];
    }; break;

    ecu_mu_.lock();

    case BRAKE_STATUS_CAN_ID:
    {
      brake_percent_ = recv_frame.data[1];
    }; break;

    case THROTTLE_STATUS_CAN_ID:
    {
      throttle_percent_ = recv_frame.data[1];
    }; break;

    case ECU_STATUS_CAN_ID:
    {
      if ((recv_frame.data[1] * 256 + recv_frame.data[0]) * 2 <= 0xFFFF)
      {
        speed_ = (float)(recv_frame.data[1] * 256.0 + recv_frame.data[0]) / 100.0;
      }
      else
      {
        speed_ = (float)(0xFFFF - recv_frame.data[1] * 256.0 - recv_frame.data[0]) / -100.0;
      }
      if (gear_ == 2)
      {
        speed_ *= -1;
      }
      if ((recv_frame.data[3] * 256 + recv_frame.data[2]) * 2 <= 0xFFFF)
      {
        acc_ = (float)(recv_frame.data[3] * 256 + recv_frame.data[2]) / 1000.0;
      }
      else
      {
        acc_ = (float)(0xFFFF - recv_frame.data[3] * 256 + recv_frame.data[2]) / -1000.0;
      }
      // csy
      geometry_msgs::Twist odom_vel;
      odom_vel.linear.x = speed_;
      odom_vel.angular.z = speed_ * tan(steer_angle_) / 1.0;
      odom_pub_.publish(odom_vel);
    }; 

    // neo
    // geometry_msgs::Twist odom_vel;
    // odom_vel.linear.x = speed_;
    // odom_vel.angular.z = speed_ * tan(steer_angle_) / 1.0;
    // odom_pub_.publish(odom_vel);

    ecu_mu_.unlock();
    break;

    default: ;
  }
}

vector<float> BringUp::pidCalculate()
{
  msg_mu_.lock();
  pid_.set_goal = goal_speed_;
  msg_mu_.unlock();

  ecu_mu_.lock();
  pid_.actual_value = speed_;
  ecu_mu_.unlock();

  pid_.err = pid_.set_goal - pid_.actual_value;
  pid_.integral += pid_.err;
  pid_.control = pid_.kp * pid_.err + pid_.ki * pid_.integral + pid_.kd * (pid_.err - pid_.err_last);

  vector<float> res;
  res.push_back(pid_.control);
  res.push_back(pid_.kp * pid_.err);
  res.push_back(pid_.ki * pid_.integral);
  res.push_back(pid_.kd * (pid_.err - pid_.err_last));
  pid_.err_last = pid_.err;

  return res;
}

void BringUp::velCallback(const geometry_msgs::Twist& msg)
{
  msg_mu_.lock();

  if (msg.linear.y == 0)
  {
    goal_speed_ = msg.linear.x;
    goal_speed_th_ = msg.angular.z;
  }
  else
  {
    // 刹车急停
    if (msg.linear.y == 1)
    {
      emergency_brake_ = true;
      goal_speed_ = 0;
      goal_speed_th_ = 0;
    }
    // 解除刹车
    else if (msg.linear.y == 2)
    {
      emergency_brake_ = false;
    }
  }

  msg_mu_.unlock();
}

void BringUp::cmdCallback(const std_msgs::Float32MultiArray& msg)
{
  msg_mu_.lock();

  goal_steer_ = msg.data[0];
  goal_throttle_ = msg.data[1];
  goal_brake_ = msg.data[2];

  msg_mu_.unlock();
}

void BringUp::msgSpin()
{
  ros::spin();
}

void BringUp::getControl()
{
  struct can_frame throttle_give_up;
  setThrottleCommand(0, throttle_give_up, true);

  struct can_frame brake_give_up;
  setBrakeCommand(0, brake_give_up, true);

  struct can_frame steer_give_up;
  setSteerCommand(0, steer_give_up, true);

  struct can_frame throttle_get;
  setThrottleCommand(0, throttle_get);

  struct can_frame brake_get;
  setBrakeCommand(0, brake_get);

  struct can_frame steer_get;
  setSteerCommand(0, steer_get);

  // 先放弃线控权，再获取线控权
  int num = 1000;
  for (int i = 0; i < num; i++)
  {
    sendCanFrame(throttle_give_up);
    sendCanFrame(brake_give_up);
    sendCanFrame(steer_give_up);
  }

  for (int i = 0; i < num; i++)
  {
    sendCanFrame(throttle_get);
    sendCanFrame(brake_get);
    sendCanFrame(steer_get);
  }
}

void readSpin(BringUp* robot)
{
  while (1)
  {
    robot->canRead();
    usleep(0);
  }
}

void writeSpin(BringUp* robot)
{
  while (1)
  {
    if (robot->pid_enable_ == true)
    {
      robot->canWritePID();
    }
    else
    {
      robot->canWrite();
    }
    
    usleep(1.0e6 / 10);
  }
}

void BringUp::canRead()
{
  struct can_frame received_frame;

  serial_mu_.lock();
  int flag = read(s_, &received_frame, sizeof(received_frame));
  serial_mu_.unlock();

  if (flag > 0)
  {
    canFrameProcess(received_frame);
  }
}

void BringUp::canWritePID()
{
  msg_mu_.lock();
  float goal_speed = goal_speed_;
  float goal_speed_th = goal_speed_th_;
  bool emergency_brake = emergency_brake_;
  msg_mu_.unlock();

  // 紧急停车
  if (emergency_brake == true)
  {
    // 下发油门报文
    struct can_frame throttle_command_frame;
    setThrottleCommand(0, throttle_command_frame); 
    sendCanFrame(throttle_command_frame); 

    // 下发刹车报文
    struct can_frame brake_command_frame;
    setBrakeCommand(15, brake_command_frame);
    sendCanFrame(brake_command_frame);
    
    return;
  }

  // 速度控制(频率为write_fq_)
  struct timeval now;
  gettimeofday(&now, NULL);
  float dur = ((now.tv_sec - last_time_write_speed_.tv_sec) * 1000000 + (now.tv_usec - last_time_write_speed_.tv_usec)) / 1.0e6;
  if (dur >= 0.6/* 1.0 / write_fq_ */)
  {
    gettimeofday(&last_time_write_speed_, NULL);

    ecu_mu_.lock();
    float speed = speed_;
    int throttle_percent = throttle_percent_;
    int brake_percent = brake_percent_;
    ecu_mu_.unlock();

    if (speed * goal_speed < 0)
    {
      goal_speed = 0;
    }

    // 油门与刹车控制量
    vector<float> pid_res = pidCalculate();
    float res = pid_res[0];
    int throttle_cmd, brake_cmd;
    if (speed >= 0 && goal_speed >= 0)
    {
      // 加速
      if (res >= 0)
      {
        if (brake_percent == 0)
        {
          throttle_cmd = BOUND(throttle_percent + ROUND(res), 0, max_throttle_);
          brake_cmd = brake_percent;
        } 
        else
        {
          throttle_cmd = throttle_percent;
          brake_cmd = BOUND(brake_percent - ROUND(res), 0, 100);
        }
      }
      // 减速
      else
      {
        if (throttle_percent == 0)
        {
          throttle_cmd = throttle_percent;
          brake_cmd = BOUND(brake_percent - ROUND(res), 0, 100);
        }
        else
        {
          throttle_cmd = BOUND(throttle_percent + ROUND(descend_rate_ * res), 0, max_throttle_);
          brake_cmd = brake_percent;
        }
      }
      
      // 下发档位报文（前进档）
      struct can_frame gear_command_frame;
      setGearCommand(4, gear_command_frame);
      sendCanFrame(gear_command_frame);
    }
    else if (speed <= 0 && goal_speed <= 0)
    {
      // 加速
      if (res <= 0)
      {
        if (brake_percent == 0)
        {
          throttle_cmd = BOUND(throttle_percent - ROUND(res), 0, max_throttle_);
          brake_cmd = brake_percent;
        }
        else
        {
          throttle_cmd = throttle_percent;
          brake_cmd = BOUND(brake_percent + ROUND(res), 0, 100);
        }
      }
      // 减速
      else
      {
        if (throttle_percent == 0)
        {
          throttle_cmd = throttle_percent;
          brake_cmd = BOUND(brake_percent + ROUND(res), 0, 100);
        }
        else
        {
          throttle_cmd = BOUND(throttle_percent - ROUND(descend_rate_ * res), 0, max_throttle_);
          brake_cmd = brake_percent;
        }
      }

      // 下发档位报文（倒车档）
      struct can_frame gear_command_frame;
      setGearCommand(2, gear_command_frame);
      sendCanFrame(gear_command_frame);
    }

    // 下发油门报文
    struct can_frame throttle_command_frame;
    setThrottleCommand(throttle_cmd, throttle_command_frame); 
    sendCanFrame(throttle_command_frame); 

    // 下发刹车报文
    struct can_frame brake_command_frame;
    setBrakeCommand(brake_cmd, brake_command_frame);
    sendCanFrame(brake_command_frame);

    // 过程量输出到日志
    printConfig();
    ROS_INFO_STREAM("goal:" <<
                    setw(5) << left << goal_speed << 
                    "cur:" <<
                    setw(5) << left << speed <<
                    "p:" <<
                    setw(5) << left << pid_res[1] <<
                    "i:" <<
                    setw(5) << left << pid_res[2] <<
                    "d:" <<
                    setw(5) << left << pid_res[3] <<
                    "cur_th:" <<
                    setw(5) << left << throttle_percent <<
                    "cmd_th:" <<
                    setw(5) << left << throttle_cmd << endl);
  }

  // 转向控制(高频), 暂时用角速度直接作为转向角
  struct can_frame steer_command_frame;
  goal_speed_th = min(max(goal_speed_th, -10.0f), 10.0f);
  setSteerCommand(goal_speed_th * PI / 180, steer_command_frame);
  sendCanFrame(steer_command_frame);
}

void BringUp::canWrite()
{
  msg_mu_.lock();
  float goal_steer = goal_steer_;
  float goal_throttle = goal_throttle_;
  float goal_brake = goal_brake_;
  msg_mu_.unlock();

  struct can_frame steer_command_frame;
  setSteerCommand(goal_steer * PI / 180, steer_command_frame);
  sendCanFrame(steer_command_frame);

  if (goal_throttle_ >= 0)
  {
    struct can_frame gear_command_frame;
    setGearCommand(4, gear_command_frame);
    sendCanFrame(gear_command_frame);

    struct can_frame throttle_command_frame;
    setThrottleCommand(goal_throttle, throttle_command_frame);
    sendCanFrame(throttle_command_frame);
  }
  else
  {
    struct can_frame gear_command_frame;
    setGearCommand(2, gear_command_frame);
    sendCanFrame(gear_command_frame);

    struct can_frame throttle_command_frame;
    setThrottleCommand(-1 * goal_throttle, throttle_command_frame);
    sendCanFrame(throttle_command_frame);
  }

  struct can_frame brake_command_frame;
  setBrakeCommand(goal_brake, brake_command_frame);
  sendCanFrame(brake_command_frame);

  printConfig();
  ROS_INFO_STREAM("转向:" << setw(5) << left << goal_steer << 
                  "油门:" << setw(5) << left << goal_throttle <<
                  "刹车:" << setw(5) << left << goal_brake << endl);
}
