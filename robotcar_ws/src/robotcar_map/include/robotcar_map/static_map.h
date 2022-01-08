#ifndef STATIC_MAP_H
#define STATIC_MAP_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <robotcar_map/map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>

class StaticMap : public Map
{
public:
  StaticMap(std::string name);

  ~StaticMap();
  
  // 获取参数，订阅静态地图话题
  void onInitialize();
  
  // 确保传入的范围不小于整张静态地图的范围
  void updateBounds(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y,
                    double* max_x, double* max_y);

  // 更新主地图数据
  void updateData(Map& main_map, int min_i, int min_j, int max_i, int max_j);

private:
  // 地图话题回调函数，根据接收到的静态地图重设主地图尺寸
  void topicCallback(const nav_msgs::OccupancyGridConstPtr& input_map);

  void fileCallback(std::string file, ros::NodeHandle* nh_ptr);

  // 当没有外源地图时，初始化空地图
  void setEmptyMap(ros::NodeHandle* nh_ptr);

  // 数值转换(地图源：话题)
  unsigned char interpretTopicValue(unsigned char value);

  // 数值转换(地图源：文件)
  unsigned char interpretRGBValue(int r, int g, int b);

  std::string main_map_frame_;        // 主地图坐标系
  std::string sub_map_frame_;         // 子地图坐标系，即本层静态地图坐标系
  bool map_received_;                 // 收到静态地图标志位
  bool use_static_map_bound_;         // 使用静态地图更新bound标志位
  bool use_static_map_data_;          // 使用静态地图更新数据标志位

  ros::Subscriber map_sub_;           // 订阅静态地图话题
};

#endif /* STATIC_MAP_H */
