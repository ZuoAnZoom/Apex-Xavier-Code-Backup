#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <vector>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <robotcar_map/map.h>

struct Trajectory
{
  double v_, vth_;                  // 轨迹对应的速度/角速度控制
  std::vector<double> x_, y_, th_;  // 轨迹点
  double time_gap_;                 // 轨迹点间时间间隔
  int num_;                         // 轨迹点数量
  double cost_;                     // 轨迹代价
};

class DWAMotionPlanner
{
public:
  DWAMotionPlanner(Map* map,
                std::vector<geometry_msgs::Point> footprint,
                double acc_lim_v = 3.0, double acc_lim_vth = 1.0,
                double max_v = 2.0, double min_v = 0.1, double max_vth = 1.0, double min_vth = -1.0,
                double min_in_place_vth = 0.4, double backup_v = -0.1,
                int v_samples = 10, int vth_samples = 10,
                double sim_time = 3.0, double sim_period = 0.15,
                double dist_gap = 0.2, double ang_gap = 0.03,
                double pdist_weight = 4, double gdist_weight = 4, double occcost_weight = 5,
                double heading_lookahead = 0.325, double oscillation_reset_dist = 0.05,
                double escape_reset_dist = 0.10, double escape_reset_theta = M_PI_2);

  // 使用参数服务器上的参数来构造运动规划器，布尔参数无实际作用，只为与上一个构造函数做区分
  DWAMotionPlanner(bool use_param, Map* map, std::vector<geometry_msgs::Point> footprint);

  ~DWAMotionPlanner();

  // 传入全局规划路径，需保证轨迹连续
  void setPath(const std::vector<geometry_msgs::PoseStamped> new_path);

  // 运动规划核心函数，轨迹预测并打分，寻求最优轨迹对应的速度控制
  Trajectory getVelCmd(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
                       tf::Stamped<tf::Pose>& vel_cmd);

  // 轨迹生成、打分，找到最优轨迹
  Trajectory getBestTraj(double x, double y, double theta, double v, double vth, double acc_v, double acc_vth);

  // 推算单条轨迹，计算其代价
  void calcTraj(double x, double y, double theta, double v, double vth, double v_samp, double vth_samp,
                double acc_v, double acc_vth, Trajectory& traj);

  // 构造路径地图
  void constructPathMap();

  // 构造目标地图
  void constructGoalMap();

  // 地图传播
  void mapSpread(int* map_for_spread, std::vector<unsigned int> candidates);

  // 轨迹模拟时计算下一轨迹点的速度/角速度
  double computeNextVel(double v1, double v0, double acc, double dt);

  // 轨迹实时模拟情况可视化
  void displayTraj();
  
  // 参数设置：地图
  void setMap(Map* map) { map_ = map; 
                          goal_map_ = new int[map_->getSizeInCellsX() * map_->getSizeInCellsY()];
                          path_map_ = new int[map_->getSizeInCellsX() * map_->getSizeInCellsY()]; }

  // 参数设置：足迹
  void setFootprint(std::vector<geometry_msgs::Point> footprint) { footprint_ = footprint; }

  // 参数设置：仿真时间
  void setSimTime(double sim_time) { sim_time_ = sim_time; }

  // 参数设置：轨迹点间隔
  void setTrajPointDensity(double dist_gap, double ang_gap) { dist_gap_ = dist_gap;
                                                              ang_gap_ = ang_gap; }

  // 参数设置：采样数量
  void setSampleNum(int v_samples, int vth_samples) { v_samples_ = v_samples;
                                                      vth_samples_ = vth_samples; }

  // 参数设置：打分权重
  void setWeight(double pdist_weight, double gdist_weight, double occcost_weight) { pdist_weight_ = pdist_weight;
                                                                                    gdist_weight_ = gdist_weight;
                                                                                    occcost_weight_ = occcost_weight; }

  // 参数设置：速度阈值
  void setVelLim(double max_v, double min_v, double max_vth, double min_vth) { max_v_ = max_v;
                                                                               min_v_ = min_v;
                                                                               max_vth_ = max_vth;
                                                                               min_vth_ = min_vth; }


private:
  Map* map_;                                        // 栅格地图
  std::vector<geometry_msgs::Point> footprint_;     // 车辆足迹
  std::vector<geometry_msgs::PoseStamped> path_;    // 全局路径
  double final_goal_x_, final_goal_y_;              // 目标坐标
  bool final_goal_position_valid_;                  // 目标是否有效标志位

  double max_v_, min_v_, max_vth_, min_vth_;        // 用于轨迹模拟的速度/角速度阈值
  double acc_lim_v_, acc_lim_vth_;                  // 用于轨迹模拟的加速度/角加速度
  double min_in_place_vth_;                         // 原地自转最小角速度
  double backup_v_;                                 // 直线后退速度
  int v_samples_;                                   // 用于轨迹模拟的速度采样数
  int vth_samples_;                                 // 用于轨迹模拟的角速度采样数
  double sim_time_;                                 // 一段轨迹的仿真时间
  double sim_period_;                               // DWA窗口的仿真时间
  double dist_gap_;                                 // 轨迹点距离间隔
  double ang_gap_;                                  // 轨迹点朝向间隔
  Trajectory temp_traj_;                            // 记录实时轨迹模拟情况
  ros::Publisher temp_traj_pub_;                    // 发布实时轨迹模拟情况

  int* path_map_;                                   // 路径地图
  int* goal_map_;                                   // 目标地图

  double pdist_weight_;                             // 路径距离打分权重
  double gdist_weight_;                             // 目标距离打分权重
  double occcost_weight_;                           // 障碍代价打分权重
  double heading_lookahead_;                        // “前视”距离，用于评价原地自转轨迹

  bool stuck_left_, stuck_right_;                   // 震荡检测标志位
  bool rotating_left_, rotating_right_;             // 震荡检测标志位
  double prev_x_, prev_y_;                          // 震荡检测位置记录
  double oscillation_reset_dist_;                   // 震荡检测标志位重置的距离阈值

  bool escaping_;                                   // 逃逸状态标志位
  double escape_x_, escape_y_, escape_theta_;       // 逃逸状态位姿记录
  double escape_reset_dist_, escape_reset_theta_;   // 退出逃逸状态的距离/角距离阈值
};

#endif /* MOTION_PLANNER_H */