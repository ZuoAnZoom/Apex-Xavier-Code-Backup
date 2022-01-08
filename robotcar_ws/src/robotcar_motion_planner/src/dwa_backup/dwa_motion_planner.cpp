#include <robotcar_motion_planner/dwa/dwa_motion_planner.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>
#include <boost/algorithm/string.hpp>
#include <ros/console.h>
#include <eigen3/Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

using namespace std;

DWAMotionPlanner::DWAMotionPlanner(Map* map,
                                   vector<geometry_msgs::Point> footprint,
                                   double acc_lim_v, double acc_lim_vth,
                                   double max_v, double min_v, double max_vth, double min_vth,
                                   double min_in_place_vth, double backup_v,
                                   int v_samples, int vth_samples,
                                   double sim_time, double sim_period,
                                   double dist_gap, double ang_gap,
                                   double pdist_weight, double gdist_weight, double occcost_weight,
                                   double heading_lookahead, double oscillation_reset_dist,
                                   double escape_reset_dist, double escape_reset_theta)
{
  ros::NodeHandle nh("motion_planner");

  map_ = map;
  footprint_ = footprint;
  final_goal_position_valid_ = false;

  acc_lim_v_ = acc_lim_v;
  acc_lim_vth_ = acc_lim_vth;
  max_v_ = max_v;
  min_v_ = min_v;
  max_vth_ = max_vth;
  min_vth_ = min_vth;
  min_in_place_vth_ = min_in_place_vth;
  backup_v_ = backup_v;
  v_samples_ = v_samples;
  vth_samples_ = vth_samples;
  sim_time_ = sim_time;
  sim_period_ = sim_period;
  dist_gap_ = dist_gap;
  ang_gap_ = ang_gap;

  goal_map_ = new int[map_->getSizeInCellsX() * map_->getSizeInCellsY()];
  path_map_ = new int[map_->getSizeInCellsX() * map_->getSizeInCellsY()];

  pdist_weight_ = pdist_weight;
  gdist_weight_ = gdist_weight;
  occcost_weight_ = occcost_weight;
  heading_lookahead_ = heading_lookahead;

  stuck_left_ = false;
  stuck_right_ = false;
  rotating_left_ = false;
  rotating_right_ = false;
  oscillation_reset_dist_ = oscillation_reset_dist;

  escaping_ = false;
  escape_reset_dist_ = escape_reset_dist;
  escape_reset_theta_ = escape_reset_theta;

  temp_traj_pub_ = nh.advertise<nav_msgs::Path>("traj_candidates", 1);
}

DWAMotionPlanner::DWAMotionPlanner(bool use_param, Map* map, vector<geometry_msgs::Point> footprint)
{
  ros::NodeHandle nh("motion_planner");

  map_ = map;
  footprint_ = footprint;
  final_goal_position_valid_ = false;

  nh.param("acc_lim_v", acc_lim_v_, 3.0);
  nh.param("acc_lim_vth", acc_lim_vth_, 1.0);
  nh.param("max_v", max_v_, 2.0);
  nh.param("min_v", min_v_, 0.1);
  nh.param("max_vth", max_vth_, 1.0);
  nh.param("min_vth", min_vth_, -1.0);
  nh.param("min_in_place_vth", min_in_place_vth_, 0.4);
  nh.param("backup_v", backup_v_, -0.1);
  nh.param("v_samples", v_samples_, 10);
  nh.param("vth_samples", vth_samples_, 10);
  nh.param("sim_time", sim_time_, 3.0);
  nh.param("sim_period", sim_period_, 0.15);
  nh.param("dist_gap", dist_gap_, 0.2);
  nh.param("ang_gap", ang_gap_, 0.03);

  goal_map_ = new int[map_->getSizeInCellsX() * map_->getSizeInCellsY()];
  path_map_ = new int[map_->getSizeInCellsX() * map_->getSizeInCellsY()];

  nh.param("pdist_weight", pdist_weight_, 4.0);
  nh.param("gdist_weight", gdist_weight_, 4.0);
  nh.param("occcost_weight", occcost_weight_, 5.0);
  nh.param("heading_lookahead", heading_lookahead_, 0.325);

  stuck_left_ = false;
  stuck_right_ = false;
  rotating_left_ = false;
  rotating_right_ = false;
  nh.param("oscillation_reset_dist", oscillation_reset_dist_, 0.05);

  escaping_ = false;
  nh.param("escape_reset_dist", escape_reset_dist_, 0.1);
  nh.param("escape_reset_theta", escape_reset_theta_, M_PI_2);

  temp_traj_pub_ = nh.advertise<nav_msgs::Path>("traj_candidates", 1);
}

DWAMotionPlanner::~DWAMotionPlanner()
{

}

void DWAMotionPlanner::setPath(const std::vector<geometry_msgs::PoseStamped> new_path)
{
  // 复制全局路径
  path_.resize(new_path.size());

  for (int i = 0; i < new_path.size(); i++)
  {
    path_[i] = new_path[i];
  }

  // 检查是否为空路径
  if (path_.size() > 0)
  {
    final_goal_x_ = path_[path_.size() - 1].pose.position.x;
    final_goal_y_ = path_[path_.size() - 1].pose.position.y;
    final_goal_position_valid_ = true;
  }
  // 若为空路径，则标记最终目标位置无效
  else
  {
    final_goal_position_valid_ = false;
  }

  // 构造目标地图
  constructGoalMap();

  // 构造路径地图
  constructPathMap();
}

Trajectory DWAMotionPlanner::getVelCmd(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
                                    tf::Stamped<tf::Pose>& vel_cmd)
{
  // 从参数获得当前位姿
  Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
  
  // 从参数获得当前速度
  Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));

  // 找到代价最低的轨迹，输入分别是目前车辆位置，速度以及加速度限制
  Trajectory best_traj = getBestTraj(pos[0], pos[1], pos[2], vel[0], vel[2], acc_lim_v_, acc_lim_vth_);

  // 判断轨迹代价
  if (best_traj.cost_ >= 0)
  { 
    // 设置速度指令
    tf::Vector3 v(best_traj.v_, 0, 0);
    vel_cmd.setOrigin(v);

    tf::Matrix3x3 vth;
    vth.setRotation(tf::createQuaternionFromYaw(best_traj.vth_));
    vel_cmd.setBasis(vth);
  }
  else
  {
    // 设置零速
    vel_cmd.setIdentity();
  }
  
  return best_traj;
}


void DWAMotionPlanner::constructPathMap()
{
  // 用最大值来初始化路径地图元素
  for (int i = 0; i < map_->getSizeInCellsX() * map_->getSizeInCellsY(); i++)
  {
    path_map_[i] = 1e9;
  }

  vector<unsigned int> candidates_;

  // 在路径地图上将全局路径点的值设置为0
  for (int i = 0; i < path_.size(); i++)
  {
    unsigned int mx, my;
    map_->world2Map(path_[i].pose.position.x, path_[i].pose.position.y, mx, my);
    unsigned int index = map_->cell2Index(mx, my);
    path_map_[index] = 0;
    candidates_.push_back(index);
  }

  mapSpread(path_map_, candidates_);
}

void DWAMotionPlanner::constructGoalMap()
{
  // 用最大值来初始化目标地图元素
  for (int i = 0; i < map_->getSizeInCellsX() * map_->getSizeInCellsY(); i++)
  {
    goal_map_[i] = 1e9;
  }

  vector<unsigned int> candidates_;

  // 在目标地图上将目标点的值设置为0
  unsigned int mx, my;
  map_->world2Map(final_goal_x_, final_goal_y_, mx, my);
  unsigned int index = map_->cell2Index(mx, my);
  goal_map_[index] = 0;
  candidates_.push_back(index);

  mapSpread(goal_map_, candidates_);
}

void DWAMotionPlanner::mapSpread(int* map_for_spread, std::vector<unsigned int> candidates)
{
  int count = 0;
  unsigned int current = candidates[0];
  unsigned char* map_data = map_->getData();

  while (1)
  {
    unsigned int mx, my;
    map_->index2Cell(current, mx, my);

    vector<unsigned int> around;
    // 获取当前节点四周的节点，注意检查是否到达边界
    if (mx > 0)
    {
      around.push_back(map_->cell2Index(mx - 1, my));
    }

    if (mx < map_->getSizeInCellsX())
    {
      around.push_back(map_->cell2Index(mx + 1, my));
    }

    if (my > 0)
    {
      around.push_back(map_->cell2Index(mx, my - 1));
    }

    if (my < map_->getSizeInCellsY())
    {
      around.push_back(map_->cell2Index(mx, my + 1));
    }

    for (int i = 0; i < around.size(); i++)
    {
      // 将四周的可行的节点加入候选节点
      if (map_data[around[i]] == FREE && map_for_spread[around[i]] > map_for_spread[current] + 1)
      {
        map_for_spread[around[i]] = map_for_spread[current] + 1;
        candidates.push_back(around[i]);
      }
    }

    // 传播已完成，退出
    if (count == candidates.size() - 1)
    {
      break;
    }

    // 将指针指向的下一个候选节点设置为当前节点，并且指针自增
    current = candidates[++count];
  }
}

Trajectory DWAMotionPlanner::getBestTraj(double x, double y, double theta, double v, double vth, double acc_v, double acc_vth)
{
  // 计算速度和角速度范围
  double max_v, min_v, max_vth, min_vth;

  if (final_goal_position_valid_)
  {
    double dist = hypot(final_goal_x_ - x, final_goal_y_ - y);
    max_v = min(max_v_, dist / sim_time_);
  }

  // DWA动态窗口，限制短时间内仿真的速度和角速度范围
  max_v = min(max_v_, v + acc_v * sim_period_);
  min_v = max(min_v_, v - acc_v * sim_period_);
  max_vth = min(max_vth_, vth + acc_vth * sim_period_);
  min_vth = max(min_vth_, vth - acc_vth * sim_period_);

  // 计算速度/角速度采样间隔
  double dv, dvth;

  dv = (max_v - min_v) / (v_samples_ - 1);
  dvth = (max_vth - min_vth) / (vth_samples_ - 1);

  Trajectory best_traj;
  best_traj.cost_ = -1.0;

  double v_current, vth_current;

  // 轨迹生成
  // 当未处于逃逸状态时，生成前向轨迹
  if (!escaping_)
  {
    v_current = min_v;
    bool include_straight = false;
    
    // 外层遍历速度
    for (int i = 0; i < v_samples_; i++)
    {
      vth_current = min_vth;

      // 内层遍历角速度
      for (int j = 0; j < vth_samples_; j++)
      {
        Trajectory current_traj;
        calcTraj(x, y, theta, v, vth, v_current, vth_current, acc_v, acc_vth, current_traj);
        
        if (current_traj.cost_ >= 0 && (current_traj.cost_ < best_traj.cost_ || best_traj.cost_ < 0))
        {
          best_traj = current_traj;
        }

        if (vth_current == 0.0)
        {
          include_straight = true;
        }

        vth_current += dvth;
      }

      // 如果范围内包括零角速度且由于离散化采样未被遍历到，将这种采样情况补充进来
      if (min_vth < 0 && max_vth > 0 && include_straight == false)
      {
        vth_current = 0.0;
        Trajectory current_traj;
        calcTraj(x, y, theta, v, vth, v_current, vth_current, acc_v, acc_vth, current_traj);
        
        if (current_traj.cost_ >= 0 && (current_traj.cost_ < best_traj.cost_ || best_traj.cost_ < 0))
        {
          best_traj = current_traj;
        }
      }

      v_current += dv;
    }
  }

  // // 生成零速轨迹，即原地自转轨迹
  // v_current = 0.0;
  // vth_current = min_vth;

  // double heading_dist = DBL_MAX;

  // for (int i = 0; i < vth_samples_; i++)
  // {
  //   // 避免采样到小于原地转速阈值的角速度
  //   if (abs(vth_current) < min_in_place_vth_)
  //   {
  //     vth_current = vth_current / abs(vth_current) * min_in_place_vth_;
  //   }

  //   Trajectory current_traj;
  //   calcTraj(x, y, theta, v, vth, v_current, vth_current, acc_v, acc_vth, current_traj);

  //   // 若新轨迹的代价更优
  //   if (current_traj.cost_ >= 0 && (current_traj.cost_ < best_traj.cost_ || best_traj.cost_ < 0))
  //   {
  //     // 获取新轨迹的末点
  //     double x_end = *(current_traj.x_.end() - 1);
  //     double y_end = *(current_traj.y_.end() - 1);
  //     double th_end = *(current_traj.th_.end() - 1);

  //     // 计算末点前方（heading lookahead）点坐标
  //     double x_ahead = x_end + heading_lookahead_ * cos(th_end);
  //     double y_ahead = y_end + heading_lookahead_ * sin(th_end);

  //     // 将其转换到栅格坐标系，并利用目标地图，获得其与目标的距离
  //     unsigned int cell_x, cell_y;
  //     if (map_->world2Map(x_ahead, y_ahead, cell_x, cell_y))
  //     {
  //       unsigned int index = map_->cell2Index(cell_x, cell_y);
  //       // double ahead_dist = (double)goal_map_[index];
  //       double ahead_dist = sqrt((x_ahead - final_goal_x_) * (x_ahead - final_goal_x_) + (y_ahead - final_goal_y_) * (y_ahead - final_goal_y_));

  //       // 若其与目标距离更短，认为轨迹更优，将其储存
  //       if (ahead_dist < heading_dist)
  //       {
  //         // 抑制震荡
  //         if (vth_current < 0 && !stuck_left_)
  //         {
  //           best_traj = current_traj;
  //           heading_dist = ahead_dist;
  //         }

  //         if (vth_current > 0 && !stuck_right_)
  //         {
  //           best_traj = current_traj;
  //           heading_dist = ahead_dist;
  //         }
  //       }
  //     }
  //   }

  //   vth_current += dvth;
  // } 

  // 轨迹生成工作结束

  // 检查最优轨迹的代价是否有效
  if (best_traj.cost_ >= 0)
  {
    // 震荡记录
    // 速度非正
    if (best_traj.v_ <= 0)
    {
      // 角速度为负
      if (best_traj.vth_ < 0)
      {
        if (rotating_right_)
        {
          stuck_right_ = true;
        }
        rotating_right_ = true;
      }
      // 角速度为正
      else if (best_traj.vth_ > 0)
      {
        if (rotating_left_)
        {
          stuck_left_ = true;
        }
        rotating_left_ = true;
      }

      // 本次记录震荡的位置
      prev_x_ = x;
      prev_y_ = y;
    }

    // 距离上一次记录震荡的位置走出了多远
    double dist = hypot(x - prev_x_, y - prev_y_);
    
    // 若已经离开足够远的距离，则重置震荡标志位
    if (dist > oscillation_reset_dist_)
    {
      rotating_left_ = false;
      rotating_right_ = false;
      stuck_left_ = false;
      stuck_right_ = false;
    }

    // 计算距离上一次进入逃逸状态的位置走出了多远
    dist = hypot(x - escape_x_, y - escape_y_);

    // 若已经离开足够远的距离，则重置逃逸状态标志位
    if(dist > escape_reset_dist_ ||
       abs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_)
    {
      escaping_ = false;
    }

    return best_traj;
  }

  // 若最优轨迹代价为负
  else
  {
    v_current = backup_v_;
    vth_current = 0.0;

    // 生成直线倒退轨迹
    Trajectory current_traj;
    calcTraj(x, y, theta, v, vth, v_current, vth_current, acc_v, acc_vth, current_traj);

    // 将倒退轨迹作为最优轨迹
    best_traj = current_traj;

    // 距离上一次记录震荡的位置走出了多远
    double dist = hypot(x - prev_x_, y - prev_y_);
    
    // 若已经离开足够远的距离，则重置震荡标志位
    if (dist > oscillation_reset_dist_)
    {
      rotating_left_ = false;
      rotating_right_ = false;
      stuck_left_ = false;
      stuck_right_ = false;
    }

    // 存在有效目标点时，进入逃逸模式
    if (!escaping_ && best_traj.cost_ > -2.0)
    {
      escape_x_ = x;
      escape_y_ = y;
      escape_theta_ = theta;
      escaping_ = true;
    }

    // 计算距离上一次进入逃逸状态的位置走出了多远
    dist = hypot(x - escape_x_, y - escape_y_);

    // 若已经离开足够远的距离，则重置逃逸状态标志位
    if(dist > escape_reset_dist_ ||
       abs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_)
    {
      escaping_ = false;
    }

    // 若后退轨迹遇障，还是继续后退，因为后退一点后会立刻进入原地自转模式
    if(best_traj.cost_ == -1.0)
    {
      best_traj.cost_ = 1.0;
    }

    return best_traj;
  }
}
  
void DWAMotionPlanner::calcTraj(double x, double y, double theta, double v, double vth, double v_samp, double vth_samp,
                             double acc_v, double acc_vth, Trajectory& traj)
{
  double x_current = x;             // 轨迹起点x坐标
  double y_current = y;             // 轨迹起点y坐标
  double th_current = theta;        // 轨迹起点朝向
  double v_current = v;             // 轨迹起点速度-当前速度
  double vth_current = vth;         // 轨迹起点角速度-当前角速度

  // 计算一条轨迹上的轨迹点数量
  int num_points = ceil(max(abs(v_samp) * sim_time_ / dist_gap_, abs(vth_samp) * sim_time_ / ang_gap_));
  
  // 至少一个轨迹点
  num_points = max(num_points, 1);

  // 计算轨迹点间的时间间隔
  double dt = sim_time_ / num_points;

  // 为航迹推演进行初始化
  traj.v_ = v_samp;
  traj.vth_ = vth_samp;
  traj.cost_ = -1.0;

  // 为各点迭代打分初始化
  double path_dist = 0.0;
  double goal_dist = 0.0;
  double occ_cost = 0.0;

  // 开始用采样速度和角速度进行轨迹模拟并打分
  for (int i = 0; i < num_points; i++)
  {
    // 如果轨迹跑出地图，则直接返回负代价
    unsigned int cell_x, cell_y;
    if (!map_->world2Map(x_current, y_current, cell_x, cell_y))
    {
      traj.cost_ = -1.0;
      return;
    }

    // 如果在该点处车辆足迹范围内遇障，则直接返回负代价
    // 找到当前足迹
    vector<MapCell> transformed_footprint;
    for (int i = 0; i < footprint_.size(); i++)
    { 
      double dis = hypot(abs(footprint_[i].x), abs(footprint_[i].y));
      double ang = atan2(footprint_[i].y, footprint_[i].x);
      
      double wx = x_current + footprint_[i].x + dis * (cos(ang + theta) - cos(ang));
      double wy = y_current + footprint_[i].y + dis * (sin(ang + theta) - sin(ang));
      
      unsigned int mx, my;
      if (!map_->world2Map(wx, wy, mx, my))
      {
        traj.cost_ = -3.0;
        return;
      }

      MapCell point;
      point.x = mx;
      point.y = my;

      transformed_footprint.push_back(point);
    }

    // 提取足迹及内部点
    const unsigned char* map_data = map_->getData();
    vector<MapCell> within_footprint;
    map_->getPolygon(transformed_footprint, within_footprint);

    // 遍历足迹及内部点，若遇障则返回负代价
    unsigned int footprint_cost = 0;
    unsigned int index;
    for (int i = 0; i < within_footprint.size(); i++)
    {
      index = map_->cell2Index(within_footprint[i].x, within_footprint[i].y);
      if (map_data[index] == NO_INFO ||
          map_data[index] == OBSTACLE ||
          map_data[index] == INFLATED_OBSTACLE)
      {
        if (map_data[index] == NO_INFO)
        {
          traj.cost_ = -2.0;
          return;
        }

        traj.cost_ = -1.0;
        return;
      }
      // 找到足迹范围内地图上值最大的一点，作为足迹的代价
      else if (map_data[index] > footprint_cost)
      {
        footprint_cost = map_data[index];
      }
    }

    // 用各轨迹点最大足迹代价更新障碍物代价
    occ_cost = max(occ_cost, (double)footprint_cost);

    // 用最后一个轨迹点来更新目标代价/路径代价
    path_dist = path_map_[map_->cell2Index(cell_x, cell_y)];
    goal_dist = goal_map_[map_->cell2Index(cell_x, cell_y)];
    //goal_dist = (x_current - final_goal_x_) * (x_current - final_goal_x_) + (y_current - final_goal_y_) * (y_current - final_goal_y_);

    // 如果目标距离或路径距离比地图所含的所有点数都大，那么认为目标点/路径无效
    if (path_dist >= double(map_->getSizeInCellsX() * map_->getSizeInCellsY()) ||
        goal_dist >= double(map_->getSizeInCellsX() * map_->getSizeInCellsY()))
    {
      traj.cost_ = -2.0;
      return;
    }

    // 若轨迹点在地图内且未遇障，认为其有效，加入轨迹
    traj.x_.push_back(x_current);
    traj.y_.push_back(y_current);
    traj.th_.push_back(th_current);

    // 推算下一轨迹点的速度/角速度
    v_current = computeNextVel(v_samp, v_current, acc_v, dt);
    vth_current = computeNextVel(vth_samp, vth_current, acc_vth, dt);
    
    // 推算下一轨迹点的位姿
    x_current += (v_current * cos(th_current)) * dt;
    y_current += (v_current * sin(th_current)) * dt;
    th_current += vth_current * dt;

    // 综合各打分项，得到该轨迹的代价
    double cost = pdist_weight_ * path_dist + gdist_weight_ * goal_dist + occcost_weight_ * occ_cost;
  
    // 为该轨迹代价赋值
    traj.cost_ = cost;

    // 记录实时轨迹模拟情况
    temp_traj_ = traj;
  }
}


double DWAMotionPlanner::computeNextVel(double v1, double v0, double acc, double dt)
{
  if((v1 - v0) >= 0) 
  {
    return min(v1, v0 + acc * dt);
  }
  
  return max(v1, v0 - acc * dt);
}

void DWAMotionPlanner::displayTraj()
{
  // 轨迹可视化
  if (temp_traj_.x_.empty() || temp_traj_.y_.empty())
  {
    return;
  }

  std::vector<geometry_msgs::PoseStamped> traj_current;
  ros::Time plan_time = ros::Time::now();

  for(int i = 0; i < temp_traj_.x_.size(); i++)
  {
    // 从栅格系转换到地图系
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = "map";
    pose.pose.position.x = temp_traj_.x_[i];
    pose.pose.position.y = temp_traj_.y_[i];
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    traj_current.push_back(pose);
  }

  nav_msgs::Path gui_traj;
  gui_traj.poses.resize(traj_current.size());

  if(!traj_current.empty())
  {
    gui_traj.header.frame_id = traj_current[0].header.frame_id;
    gui_traj.header.stamp = traj_current[0].header.stamp;
  }

  for(unsigned int i=0; i < traj_current.size(); i++)
  {
    gui_traj.poses[i] = traj_current[i];
  }

  temp_traj_pub_.publish(gui_traj);
}
