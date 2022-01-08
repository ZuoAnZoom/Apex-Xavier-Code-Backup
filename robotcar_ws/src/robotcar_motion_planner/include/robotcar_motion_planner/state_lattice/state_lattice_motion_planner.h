#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <cmath>
#include <vector>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <robotcar_map/map.h>
#include <robotcar_general/curve_fitter.h>
#include <robotcar_motion_planner/state_lattice/bvp_solver.h>
#include <robotcar_motion_planner/state_lattice/frame_transformer.h>

struct Path
{
  std::vector<std::vector<double>> data;
  double k_sum;
};

class StateLatticeMotionPlanner
{
public:
  // 使用参数服务器上的参数来构造运动规划器，布尔参数无实际作用，只与上一个构造函数做区分
  StateLatticeMotionPlanner(bool use_param, Map* map, std::vector<geometry_msgs::Point> footprint);

  ~StateLatticeMotionPlanner();

  // 主函数，构建State Lattice并搜索最佳轨迹
  void findBestTraj(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> vel);

private:
  // 构建State Lattice
  bool constructLattice();

  // 线方程拟合
  bool lineCurveFit();

  // 生成顶点
  void selectVertex();

  // 获取约束
  void setConstraints();

  // 生成边
  void constructEdge();

  // 调用BVPSolver生成轨迹
  Path calTrajectory(int i, int j);

  // 对边表赋开销
  float evaluateEdge(std::vector<double> v0, std::vector<double> v1, Path result);

  // 碰撞检测
  float detectCollision(std::vector<double> edge_point);

  // 偏离道路中心线检测
  float detectDeviation(std::vector<std::vector<double>> edge);

  // 规划周期间连续性检测
  float detectContinuousity(std::vector<std::vector<double>> edge);

  // 图搜索，得到最优轨迹
  int searchTrajectory();

  // 最优轨迹发布
  void publishTrajectory(std::vector<int> v_on_traj);

  // 线方程拟合可视化
  void displayCurveFit();

  // 顶点可视化
  void displayVertex();

  // 边可视化
  void displayEdge();

  // 最优轨迹可视化
  void displayBestEdge(std::vector<int> v_on_traj);

  // 地图坐标系到局部坐标系的坐标转换
  void coordMap2Local(double mx, double my, double& lx, double& ly);

  // 局部坐标系到地图坐标系的坐标转换
  void coordLocal2Map(double lx, double ly, double& mx, double& my);

  // 局部坐标系到地图坐标系的角度转换
  double angleLocal2Map(double theta);

  // 角度归一化
  double angleNormalization(double theta);

private:
  Map* map_;                                          // 栅格地图
  std::vector<geometry_msgs::Point> footprint_;       // 车辆足迹

  tf::TransformListener tf_;                          // 坐标转换
  BVPSolver bvp_;                                     // BVP问题处理
  CurveFitter cf_;                                    // 曲线拟合
  FrameTransformer ftf_;                              // 坐标系转换

  tf::Stamped<tf::Pose> global_pose_;                 // 车体位姿
  tf::Stamped<tf::Pose> vel_;                         // 车体速度
  unsigned int mx_self_, my_self_;                    // 车体坐标(栅格坐标系)
  double wx_self_, wy_self_;                          // 车体坐标(地图坐标系)
  double yaw_;                                        // 车体朝向

  int layer_num_;                                     // 图-层数
  int point_num_per_layer_;                           // 图-单层点数
  std::vector<double> layer_dis_;                     // 图-层间距离
  std::vector<std::vector<double>> vertex;            // 顶点集(Cartesian坐标系)
  std::vector<std::vector<double>> vertex_f;          // 顶点集(Frenet坐标系)
  double** edge;                                      // 边集
  std::vector<std::vector<double>>** edge_info;       // 边集轨迹信息

  double x_ref_, y_ref_;                              // 车体到线方程投影点
  std::vector<double> line_cof_;                      // 线方程系数
  std::vector<double> yaw_lane_;                      // 每层的车道线朝向
  std::vector<double> k_constraint_;                  // 每层的车道线曲率

  trajectory_msgs::JointTrajectory last_traj_;        // 上一历元的规划结果

  ros::Publisher marker_pub;                          // 可视化发布器
  ros::Publisher traj_pub;                            // 轨迹结果发布器

  int axis_flag_;
  std::vector<double> sd_cof_l_;
  std::vector<double> sl_list_, dl_list_;
};

#endif /* MOTION_PLANNER_H */
