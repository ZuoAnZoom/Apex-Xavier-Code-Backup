#ifndef SEMANTIC_MAP_H
#define SEMANTIC_MAP_H

#include <vector>
#include <list>
#include <string>

#include <ros/time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <robotcar_map/map.h>
#include <robotcar_map/lanes.h>
#include <robotcar_map/obstacles.h>
#include <robotcar_general/curve_fitter.h>

struct LidarObs
{
  geometry_msgs::Point origin_;

  // LaserScan类型的观测数据被转换格式后储存
  pcl::PointCloud<pcl::PointXYZ> cloud_;
};

class SemanticMap : public Map
{
public:
  SemanticMap(std::string name);

  ~SemanticMap();
  
  // 获取参数及主地图数据，开始订阅传感器话题
  void onInitialize();

  // 确保用到的点云数据被包含在地图更新范围内
  void updateBounds(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y,
                    double* max_x, double* max_y);
  
  // 地图更新
  void updateData(Map& main_map, int min_i, int min_j, int max_i, int max_j);

private:
  // 雷达回调函数
  void inputLaserData(const sensor_msgs::LaserScanConstPtr& msg);

  // 车道线回调函数
  void inputLanePerception(const robotcar_map::lanes& msg);

  // 障碍物回调函数
  void inputObstaclePerception(const robotcar_map::obstacles& msg);

  // 标记点云点为障碍物
  void markLaserObstacles(const LidarObs& observation);

  // 标记点云与雷达连线上的点为可行区域
  void markLaserFreespace(const LidarObs& observation, double* min_x, double* min_y, double* max_x, double* max_y);

  // 标记车道线
  void markVisualLanes(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y, double* max_x, double* max_y);

  // 标记障碍物
  void markVisualObstacles(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y, double* max_x, double* max_y);

  // 坐标转换
  void camera2Map(double camera_x, double camera_y, double& map_x, double& map_y, ros::Time stamp);
  void camera2Cell(double camera_x, double camera_y, MapCell& map_cell, ros::Time stamp);

  // 确保范围包含单点
  void includeBound(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

  // 将足迹转换至当下位置
  void transformFootprint(double robotcar_x, double robotcar_y, double robotcar_yaw);

  // 开启话题监听
  static void spinTopic();


  std::string main_map_frame_;                                // 主地图坐标系
  std::string lidar_frame_;                                   // 传感器坐标系，即障碍地图所在坐标系

  ros::Subscriber lidar_sub_;                                 // 订阅传感器话题
  ros::Subscriber lane_sub_;                                  // 订阅车道线话题
  ros::Subscriber obstacle_sub_;                              // 订阅障碍物话题
  std::list<LidarObs> observations_;                          // 各历元观测值缓冲池
  bool obs_empty_;                                            // 无观测值标志位

  boost::recursive_mutex lock_;                               // 观测值存取锁
  boost::recursive_mutex l_lock_;                             // 车道线存取锁
  boost::recursive_mutex o_lock_;                             // 障碍物存取锁

  double max_obstacle_height_;                                // 计入障碍地图的障碍物最大高度阈值
  double min_obstacle_height_;                                // 计入障碍地图的障碍物最小高度阈值
  double max_obstacle_distance_;                              // 计入障碍地图的障碍物最远距离阈值

  double fliter_length_;                                      // 滤波窗口的长
  double fliter_width_;                                       // 滤波窗口的宽（防止传感器扫描到车辆上的点，计入地图）

  std::vector<geometry_msgs::Point> transformed_footprint_;   // 机器人当下足迹

  std::vector<std::vector<double>> lanes_x, lanes_y;
  ros::Time lanes_stamp_, obstacles_stamp_;
  std::vector<double> obstacle_x_min_, obstacle_x_max_, obstacle_y_min_, obstacle_y_max_;

  CurveFitter cf_;
  tf::TransformListener tfl;

  bool first_time_;
  double slope_last_;

  ros::Time laser_stamp_;

  int lidar_enable_;
  int lane_perception_enable_;
  int obstacle_perception_enable_;
};

#endif /* SEMANTIC_MAP_H */
