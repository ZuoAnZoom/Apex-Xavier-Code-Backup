#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <math.h>
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3.h>

#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <time.h>
#include <iostream>

#define HEADER_55                        0x55
#define HEADER_AA                        0xAA
#define FRAME_LEN                        78

#define DEG_TO_RAD                       (M_PI / 180.0f)
#define m_s2                             9.8

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

union float_4bytes_t
{
  uint8_t bytes[4];
  float float_val;
};

union uint32_4bytes_t
{
  uint8_t bytes[4];
  uint32_t uint32_val;
};

union uint64_8bytes_t
{
  uint8_t bytes[8];
  uint64_t uint64_val;
};

struct gyro_data_t{
  float x;
  float y;
  float z;
};

struct accl_data_t{
  float x;
  float y;
  float z;
};

//quarternion
struct quat_data_t{
  float x;
  float y;
  float z;
  float w;
};

//euler angles
struct euler_data_t{
  float roll;
  float pitch;
  float yaw;
  float reserve;
};

class ImuDriver
{
  private:
    bool init();
    void publish_data(void);
    void parse_data(uint8_t * data_buf);
    uint8_t check_sum(uint8_t* addr, uint16_t len);
    void parse_msg();

  public:
    ImuDriver();
    ~ImuDriver();

    void run();

  private:
    uint8_t rx_cnt;
    uint8_t state;
    bool parse_flag_;

    sensor_msgs::Imu sync_imu_data, ros_imu_data;

    boost::system::error_code ec_;
    boost::asio::io_service io_service_;
    serial_port_ptr port_;
    boost::mutex mutex_;

    ros::Publisher sync_imu_pub, ros_imu_pub;

    std::string port_name_;
    int baud_rate_;

    std::string base_frame_;

    struct accl_data_t accKal;
    struct gyro_data_t gyroCal;
    struct quat_data_t quat;
    struct euler_data_t euler;

    uint64_t realtime_us;

    bool pub_sync_time_topic_;
    bool pub_ros_time_topic_;
};

ImuDriver::ImuDriver():rx_cnt(0), state(0)
{}

ImuDriver::~ImuDriver()
{
  boost::mutex::scoped_lock look(mutex_);
  parse_flag_ = false;
  if(port_)
  {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
}

uint8_t ImuDriver::check_sum(uint8_t* addr, uint16_t len)
{
  uint8_t SumData = 0;
  while(len--)
  {
    SumData = SumData ^ *addr;
    addr++;
  }
  return SumData;
}

void ImuDriver::publish_data(void)
{
  uint64_t ns;
  ns = realtime_us*1000;
  sync_imu_data.header.stamp.fromNSec(ns);
  ros_imu_data.header.stamp = ros::Time::now();

  ros_imu_data.header.frame_id = sync_imu_data.header.frame_id = base_frame_;
  ros_imu_data.orientation.x = sync_imu_data.orientation.x = (double)quat.x;
  ros_imu_data.orientation.y = sync_imu_data.orientation.y = (double)quat.y;
  ros_imu_data.orientation.z = sync_imu_data.orientation.z = (double)quat.z;
  ros_imu_data.orientation.w = sync_imu_data.orientation.w = (double)quat.w;

  ros_imu_data.linear_acceleration.x = sync_imu_data.linear_acceleration.x = (double)accKal.x;
  ros_imu_data.linear_acceleration.y = sync_imu_data.linear_acceleration.y = (double)accKal.y;
  ros_imu_data.linear_acceleration.z = sync_imu_data.linear_acceleration.z = (double)accKal.z;

  ros_imu_data.angular_velocity.x = sync_imu_data.angular_velocity.x = (double)(gyroCal.x);
  ros_imu_data.angular_velocity.y = sync_imu_data.angular_velocity.y = (double)(gyroCal.y);
  ros_imu_data.angular_velocity.z = sync_imu_data.angular_velocity.z = (double)(gyroCal.z);

  ros_imu_data.orientation_covariance = sync_imu_data.orientation_covariance = { 0.001, 0, 0,
                                      0, 0.001, 0,
                                      0, 0, 0.001
                                      };

  ros_imu_data.angular_velocity_covariance = sync_imu_data.angular_velocity_covariance = { 0.001, 0, 0,
                                      0, 0.001, 0,
                                      0, 0, 0.001
                                      };

  ros_imu_data.linear_acceleration_covariance = sync_imu_data.linear_acceleration_covariance = { 0.001, 0, 0,
                                      0, 0.001, 0,
                                      0, 0, 0.001
                                      };
  if((pub_sync_time_topic_ == true)&&(pub_ros_time_topic_ == true))
  {
    sync_imu_pub.publish(sync_imu_data);
    ros_imu_pub.publish(ros_imu_data);
  }
  else if(pub_sync_time_topic_ == true)
  {
    sync_imu_pub.publish(sync_imu_data);
  }
  else
  {
    ros_imu_pub.publish(ros_imu_data);
  }

}


/*
 * parse data
*/
void ImuDriver::parse_data(uint8_t * data_buf)
{
  uint8_t idx = 0;
  uint64_8bytes_t time_union;
  float_4bytes_t float_4bytes_union;

  uint16_t year, m_sec, u_sec;
  uint8_t month, day, hour, min, sec;
  uint64_t ns;

  int64_t time_unix_nsec = ros::Time::now().toNSec();

  year = (data_buf[3]<<8) | data_buf[2];
  month = data_buf[4];
  day = data_buf[5];
  hour = data_buf[6];
  min = data_buf[7];
  sec = data_buf[8];
  m_sec = (data_buf[10]<<8) | data_buf[9];
  u_sec = (data_buf[12]<<8) | data_buf[11];

   //printf("now is: %d/%d/%d %02d:%02d:%02d.%d'%d\r\n" ,year, month, day, hour, min, sec, m_sec, u_sec);

    for(idx = 0; idx < 8; idx++)
    {
        time_union.bytes[idx] = data_buf[13 + idx];
    }
    realtime_us = time_union.uint64_val;

    ns = realtime_us*1000;

    printf("time from imu sync:       %lld\r\n", ns);
    printf("now unixtime is:          %lld\r\n", time_unix_nsec);


  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[21 + idx];
  }
  quat.w = float_4bytes_union.float_val;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[25 + idx];
  }
  quat.x = float_4bytes_union.float_val;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[29 + idx];
  }
  quat.y = float_4bytes_union.float_val;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[33 + idx];
  }
  quat.z = float_4bytes_union.float_val;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[37 + idx];
  }
  euler.roll = float_4bytes_union.float_val;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[41 + idx];
  }
  euler.pitch = float_4bytes_union.float_val;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[45 + idx];
  }
  euler.yaw = float_4bytes_union.float_val;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[49 + idx];
  }
  euler.reserve = float_4bytes_union.float_val;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[53 + idx];
  }
  accKal.x = float_4bytes_union.float_val*m_s2;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[57 + idx];
  }
  accKal.y = float_4bytes_union.float_val*m_s2;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[61 + idx];
  }
  accKal.z = float_4bytes_union.float_val*m_s2;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[65 + idx];
  }
  gyroCal.x = float_4bytes_union.float_val*DEG_TO_RAD;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[69 + idx];
  }
  gyroCal.y = float_4bytes_union.float_val*DEG_TO_RAD;

  for(idx = 0; idx < 4; idx++)
  {
    float_4bytes_union.bytes[idx] = data_buf[73 + idx];
  }
  gyroCal.z = float_4bytes_union.float_val*DEG_TO_RAD;
}


void ImuDriver::parse_msg()
{
  uint8_t buffer_data[100], sum_check;
  uint16_t i, sl_data_len, payload_len;

  int64_t time_unix_nsec;

  parse_flag_ = true;

  while(parse_flag_)
  {
    switch(state)
    {
      case 0:   //got HEADER_55
        {
          rx_cnt = 0;
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[rx_cnt],1), ec_);
          if(buffer_data[rx_cnt] == HEADER_55)
          {
            time_unix_nsec = ros::Time::now().toNSec();
            //printf("receive 0x55 unixtime is: %lld\r\n", time_unix_nsec);
            rx_cnt++;
            state = 1;
          }else
          {
            state = 0;
            printf("receive Header 55 error\r\n");
          }
          break;
        }
      case 1:   //got HEADER_AA
        {
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[rx_cnt], 1), ec_);
          if(buffer_data[rx_cnt] == HEADER_AA)
          {
            rx_cnt++;
            state = 2;
          }else
          {
            state = 0;
            printf("receive Header AA error\r\n");
          }
          break;
        }
      case 2:
        {
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[rx_cnt], 1), ec_);
          rx_cnt++;
          if(rx_cnt >= FRAME_LEN)
          {
            //got a whole frame	check sum
            sum_check = check_sum(buffer_data, (FRAME_LEN - 1));
            if(sum_check == buffer_data[FRAME_LEN - 1]) {
              time_unix_nsec = ros::Time::now().toNSec();
              //printf("receive 0xFF unixtime is: %lld\r\n", time_unix_nsec);
              parse_data(buffer_data);
              publish_data();
            }
            else{
              //ROS_WARN("check sum wrong!");
              printf("check sum is: %x, while byte_receive is: %x", sum_check, buffer_data[FRAME_LEN - 1]);
              ROS_INFO_STREAM("error:check sum is:" << sum_check << ", while byte_receive is:" << buffer_data[FRAME_LEN - 1]);
            }
            state = 0;
          }
          break;
        }
      default:
        {
          state = 0;
          break;
        }
    }
  }

}

bool ImuDriver::init()
{
  if(port_)
  {
    ROS_WARN("error : port is already opened...");
    return false;
  }
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, ec_);
  if (ec_)
  {
    ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
    return false;
  }

  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  return true;
}

void ImuDriver::run()
{
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  private_node.param<std::string>("port_name", port_name_, std::string("/dev/ttyUSB0"));
  private_node.param<std::string>("base_frame", base_frame_, std::string("gyro_link"));

  private_node.param<int>("baud_rate", baud_rate_, 115200);
  private_node.param<bool>("pub_sync_time_topic",pub_sync_time_topic_,true);
  private_node.param<bool>("pub_ros_time_topic",pub_ros_time_topic_,true);

  std::cout << "init: publish sync time topic ? "<< pub_sync_time_topic_<< " publish ros time topic ? "<< pub_ros_time_topic_ <<std::endl;

  if (init())
  {
    if((pub_sync_time_topic_ == true)&&(pub_ros_time_topic_ == true))
    {
      sync_imu_pub = node.advertise<sensor_msgs::Imu>("imu_sync", 1000);
      ros_imu_pub = node.advertise<sensor_msgs::Imu>("imu", 1000);
    }
    else if(pub_sync_time_topic_ == true)
    {
      sync_imu_pub = node.advertise<sensor_msgs::Imu>("imu_sync", 1000);
    }
    else
    {
      ros_imu_pub = node.advertise<sensor_msgs::Imu>("imu", 1000);
    }

    boost::thread parse_thread(boost::bind(&ImuDriver::parse_msg, this));
    ros::spin();
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ppsAtom_imu");
  ImuDriver driver;
  driver.run();
  return 0;
}

