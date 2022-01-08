#include <robotcar_motion_planner/state_lattice/state_lattice_motion_planner.h>
#include <queue>
#include <ctime>
#include <math.h>
#include <string>
#include <algorithm>
#include <angles/angles.h>
#include <ros/console.h>
#include <eigen3/Eigen/Core>
#include <boost/algorithm/string.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#define PI 3.1415926

using namespace std;

StateLatticeMotionPlanner::StateLatticeMotionPlanner(bool use_param, Map* map, vector<geometry_msgs::Point> footprint)
{
  ros::NodeHandle nh("motion_planner");

  map_ = map;
  footprint_ = footprint;

  // 设置状态格层数及每层距离车体的距离
  layer_num_ = 2;
  layer_dis_.resize(layer_num_);
  layer_dis_[0] = 0;
  layer_dis_[1] = 10;

  // 设置状态格单层的格点数
  point_num_per_layer_ = 13;

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("trajectory", 1);
}

StateLatticeMotionPlanner::~StateLatticeMotionPlanner()
{

}

void StateLatticeMotionPlanner::findBestTraj(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> vel)
{ 
  global_pose_ = global_pose;
  vel_ = vel;

  // 存储自身位姿
  yaw_ = tf::getYaw(global_pose.getRotation());
  map_->world2Map(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), mx_self_, my_self_);
  wx_self_ = global_pose.getOrigin().getX();
  wy_self_ = global_pose.getOrigin().getY();

  // 构造状态格
  if (constructLattice() == false)
  {
    return;
  }

  // 最优轨迹搜索
  while (1)
  {
    int result = searchTrajectory();
    if (result == 1 || result == 3)
      break;
  }
  
  int row = vertex.size();
  for (int i = 0; i < row; i++)
  {
    delete[] edge[i];
    delete[] edge_info[i];
  }

  delete[] edge;
  delete[] edge_info;
}

bool StateLatticeMotionPlanner::constructLattice()
{
  // 线方程拟合
  bool res = lineCurveFit();

  if (res == false)
  {
    return false;
  }

  // 构造顶点集
  selectVertex();

  // 构造边集
  constructEdge();

  return true;
}

bool StateLatticeMotionPlanner::lineCurveFit()
{
  vertex.clear();
  vertex_f.clear();
 
  // 在一定范围内搜索车道线点，用于线方程拟合
  double r = 10.0;
  vector<MapCell> around_self(4);
  around_self[0].x = max(int(mx_self_) - int(r / map_->getResolution()), 0);
  around_self[0].y = max(int(my_self_) - int(r / map_->getResolution()), 0);
  around_self[1].x = around_self[0].x;
  around_self[1].y = min(int(my_self_) + int(r / map_->getResolution()), int(map_->getSizeInCellsX()) - 1);
  around_self[2].x = min(int(mx_self_) + int(r / map_->getResolution()), int(map_->getSizeInCellsX()) - 1);
  around_self[2].y = around_self[1].y;
  around_self[3].x = around_self[2].x;
  around_self[3].y = around_self[0].y;

  // 提取矩形范围内的地图点
  vector<MapCell> detect_space;
  map_->getPolygon(around_self, detect_space);
  
  vector<double> xvals, yvals;
  double min_dis_sq = 1e10;
  double wx_max = -1e10;
  double wx_min = 1e10;
  double wy_max = -1e10;
  double wy_min = 1e10;
  double wx_max_y, wx_min_y, wy_max_x, wy_min_x;

  // 在矩形范围内搜索车道线点
  for (int i = 0; i < detect_space.size(); i++)
  {
    unsigned int mx = detect_space[i].x;
    unsigned int my = detect_space[i].y;
    unsigned char val = map_->getValue(mx, my);
    if (val == 22)
    {
      double wx, wy;
      map_->map2World(mx, my, wx, wy);
      xvals.push_back(wx);
      yvals.push_back(wy);

      // 记录搜索框内x值、y值最大和最小的点
      if (wx > wx_max)
      {
        wx_max = wx;
        wx_max_y = wy;
      }
      if (wx < wx_min)
      {
        wx_min = wx;
        wx_min_y = wy;
      }
      if (wy > wy_max)
      {
        wy_max = wy;
        wy_max_x = wx;
      }
      if (wy < wy_min)
      {
        wy_min = wy;
        wy_min_x = wx;
      }

      // 找到距离车体最近的点作为车体在参考线上的投影点，即局部笛卡尔坐标系的原点
      double dis_sq = pow((wx - wx_self_), 2) + pow((wy - wy_self_), 2);
      if (dis_sq < min_dis_sq)
      {
        min_dis_sq = dis_sq;
        x_ref_ = wx;
        y_ref_ = wy;
      }
    }
  }

  // 按照目标来给，且这个目标是瞬时目标
  if (abs(wx_min - wx_max) > 5)
  {
    float ang = atan2(wx_max_y - wy_self_, wx_max - wx_self_);
    if (abs(ang - yaw_) <= PI / 2)
      axis_flag_ = 1;
    else
      axis_flag_ = 2;
  }
  else
  {
    float ang = atan2(wy_max - wy_self_, wy_max_x - wx_self_);
    if (abs(ang - yaw_) <= PI / 2)
      axis_flag_ = 3;
    else
      axis_flag_ = 4;
  }

  for (int i = 0; i < xvals.size(); i++)
  {
    double lx, ly;
    coordMap2Local(xvals[i], yvals[i], lx, ly);
    xvals[i] = lx;
    yvals[i] = ly;
  }
  
  // 计算局部笛卡尔坐标系下的坐标
  if (xvals.size() < 4)
  {
    return false;
  }
  
  // 拟合
  line_cof_ = cf_.polyFit(xvals, yvals, 3 );

  // 用拟合好的线方程生成Frenet坐标系
  FrameTransformer ftf(line_cof_, 12);
  ftf_ = ftf;

  // 线方程拟合可视化
  displayCurveFit();
  return true;
}

void StateLatticeMotionPlanner::selectVertex()
{
  // 将自身加入顶点集
  vector<double> single_vertex(3);
  single_vertex[0] = wx_self_;
  single_vertex[1] = wy_self_;
  single_vertex[2] = 0;
  vertex.push_back(single_vertex);

  vector<double> single_vertex_f(3);
  single_vertex_f[0] = 0;
  single_vertex_f[1] = 0;
  single_vertex_f[2] = 0;
  vertex_f.push_back(single_vertex_f);
  
  // 设置顶点并加入顶点集
  for (int i = 1; i < layer_num_; i++)
  {
    for (int j = 0; j < point_num_per_layer_; j++)
    {
      double s = layer_dis_[i];
      double d = -2.5 + j * 5.0 / (point_num_per_layer_ - 1);

      vector<double> point = ftf_.fre2Cart(s, d);
      coordLocal2Map(point[0], point[1], single_vertex[0], single_vertex[1]);
      single_vertex[2] = i;
      vertex.push_back(single_vertex);

      vector<double> single_vertex_f(3);
      single_vertex_f[0] = s;
      single_vertex_f[1] = d;
      single_vertex_f[2] = i;
      vertex_f.push_back(single_vertex_f);
    }
  }

  // 顶点可视化
  displayVertex();

  // 获取朝向、区率等约束
  setConstraints();
}

void StateLatticeMotionPlanner::setConstraints()
{
  // 设置朝向和曲率约束，朝向分别为各层投影到参考线上的朝向
  // 曲率分别为自身瞬时曲率和从第二层起的参考线曲率
  yaw_lane_.clear();
  k_constraint_.clear();
  
  // 线方程系数
  double a = line_cof_[0];
  double b = line_cof_[1];
  double c = line_cof_[2];
  double d = line_cof_[3];

  // 先加入自身投影处的朝向
  double theta0 = angleLocal2Map(atan(b));
  yaw_lane_.push_back(theta0);

  // 加入车体当前曲率
  double v, v_th, k0;
  v = vel_.getOrigin().getX();
  v_th = tf::getYaw(vel_.getRotation());
  abs(v) > 0.1 ? v = v : v = 0.0;
  abs(v_th) > 0.05 ? v_th = v_th : v_th = 0.0;
  v == 0.0 ? k0 = 0.0 : k0 = v_th / v;
  k_constraint_.push_back(k0);

  // 逐层计算朝向和曲率
  for (int i = 1; i < layer_num_; i++)
  {
    vector<double> r = ftf_.fre2Cart(layer_dis_[i], 0);
    double rx = r[0];
    double k = b + 2*c*rx + 3*d*rx*rx;
    double theta = angleLocal2Map(atan(k));
    double kappa = abs(2*c + 6*d*rx) / pow((1 + k*k), 1.5);

    yaw_lane_.push_back(theta);
    k_constraint_.push_back(kappa);
  }
}

void StateLatticeMotionPlanner::constructEdge()
{
  // 初始化边表
  int row = vertex.size(), col = vertex.size();
  edge = new double* [row];
  edge_info = new vector<vector<double>>* [row];
  for (int i = 0; i < row; i++)
  {
    edge[i] = new double [col];
    edge_info[i] = new vector<vector<double>> [col];
  }

  // 边表赋开销值，若不存在边则赋值为-1e5
  for (int i = 0; i < row; i++)
  {
    for (int j = 0; j < col; j++)
    {
      // 各节点只与高于自身所在层且相邻的层的节点间有边
      int dlayer = vertex[j][2] - vertex[i][2];
      if (vertex[i][2] >= vertex[j][2] || dlayer > 1)
      {
        edge[i][j] = -1e5;
        continue;
      }
      
      Path result = calTrajectory(i, j);

      // 轨迹打分
      edge[i][j] = evaluateEdge(vertex[j], vertex[j], result);
      edge_info[i][j] = result.data;
    }
  }

  // 边可视化
  displayEdge();
}

Path StateLatticeMotionPlanner::calTrajectory(int i, int j)
{ 
  double yaw_lane_s = yaw_lane_[vertex[i][2]];
  double yaw_lane_f = yaw_lane_[vertex[j][2]];
  double v0x = vertex[i][0];
  double v0y = vertex[i][1];
  double v1x = vertex[j][0];
  double v1y = vertex[j][1];
  int v0l = vertex[i][2];
  int v1l = vertex[j][2];

  // 从地图坐标系转换到“瞬时朝向”坐标系
  double x0, y0, theta0, k0, x1, y1, theta1, k1;
  x0 = (v0x - v0x) * cos(-1 * yaw_lane_s) - (v0y - v0y) * sin(-1 * yaw_lane_s);
  y0 = (v0x - v0x) * sin(-1 * yaw_lane_s) + (v0y - v0y) * cos(-1 * yaw_lane_s);
  x1 = (v1x - v0x) * cos(-1 * yaw_lane_s) - (v1y - v0y) * sin(-1 * yaw_lane_s);
  y1 = (v1x - v0x) * sin(-1 * yaw_lane_s) + (v1y - v0y) * cos(-1 * yaw_lane_s);

  // 从车体出发的边的初始朝向为当前姿态，末朝向为瞬时车道方向；其它边的其实朝向和末朝向均为瞬时车道方向
  if (v0l == 0)
  {
    theta0 = angleNormalization(yaw_ - yaw_lane_s);
    k0 = 0;
  }
  else
  {
    theta0 = 0;
    k0 = k_constraint_[v0l];
  }
  theta1 = angleNormalization(yaw_lane_f - yaw_lane_s);
  k1 = k_constraint_[v1l];

  // 设置边界条件
  bvp_.setBoundaryValue(x0, y0, theta0, k0, x1, y1, theta1, k1);
  
  // BVP计算
  bvp_.calParams();

  // 先在车体坐标系下通过旋转和平移将轨迹解除归一化，再从车体系转换回地图系
  vector<vector<double>> traj_points;
  int traj_num = 80;
  double x_last = -10;
  double k_sum = 0;
  double a, b, c, d, s;
  bvp_.getParams(a, b, c, d, s);
  for (int i = 0; i <= traj_num; i++)
  {
    double s_cur = s / traj_num * i;

    // 解除归一化，先旋转后平移
    double x, y, k;
    x = bvp_.calCn(a, b, c, d, s_cur, 0) * cos(theta0) - bvp_.calSn(a, b, c, d, s_cur, 0) * sin(theta0) + x0;
    
    // Frenet坐标系x轴方向每隔0.3m输出一个轨迹点
    if (x - x_last < 0.3)
      continue;

    y = bvp_.calCn(a, b, c, d, s_cur, 0) * sin(theta0) + bvp_.calSn(a, b, c, d, s_cur, 0) * cos(theta0) + y0;
    k = a + b * s_cur * s_cur + c * s_cur * s_cur * s_cur + d * s_cur * s_cur * s_cur * s_cur;
    k_sum += abs(k);

    // 从瞬时Frenet系到地图系
    vector<double> point(2);
    point[0] = x * cos(yaw_lane_s) - y * sin(yaw_lane_s) + v0x;
    point[1] = x * sin(yaw_lane_s) + y * cos(yaw_lane_s) + v0y;
    traj_points.push_back(point);
    x_last = x;
  }

  Path result;
  result.data = traj_points;
  result.k_sum = k_sum;

  return result;
}

float StateLatticeMotionPlanner::evaluateEdge(vector<double> v0, vector<double> v1, Path result)
{ 
  float cost;        // 总开销
  float cost_dev;    // 与参考线间的偏离
  float cost_len;    // 参考线方向长度
  float cost_k;      // 总曲率
  float cost_con;    // 一致性

  // 各项开销权重
  float wd = 0.8;
  float wl = 0; // 应该为负？
  float wk = 0.2;
  float wc = 0.3;

  int l0 = v0[2];
  int l1 = v1[2];
  float s = layer_dis_[l1] - layer_dis_[l0];

  // 偏离车道中心线检测
  cost_dev = detectDeviation(result.data);

  // 长度检测
  cost_len = s;

  // 曲率检测
  cost_k = result.k_sum;

  // 一致性检测
  // TODO：改进一致性检测
  cost_con = 0;//detectContinuousity(result.data);

  // 计算总开销
  cost = wd * cost_dev + wl * cost_len + wk * cost_k + wc * cost_con;
  return cost;
}

float StateLatticeMotionPlanner::detectCollision(vector<double> edge_point)
{
  // 将机器人足迹转换到当前足迹点上
  vector<MapCell> transformed_footprint;
  double x_current = edge_point[0];
  double y_current = edge_point[1];
  for (int i = 0; i < footprint_.size(); i++)
  { 
    double dis = hypot(abs(footprint_[i].x), abs(footprint_[i].y));
    double ang = atan2(footprint_[i].y, footprint_[i].x);
    
    double wx = x_current + footprint_[i].x + dis * (cos(ang + yaw_) - cos(ang));
    double wy = y_current + footprint_[i].y + dis * (sin(ang + yaw_) - sin(ang));
    
    // 若足迹不在地图上，返回-3
    unsigned int mx, my;
    if (!map_->world2Map(wx, wy, mx, my))
    {
      return -3.0;
    }

    MapCell point;
    point.x = mx;
    point.y = my;
    transformed_footprint.push_back(point);
  }

  // 提取足迹及内部点
  vector<MapCell> within_footprint;
  map_->getPolygon(transformed_footprint, within_footprint);

  // 遍历足迹及内部点，若遇障碍/膨胀障碍则返回负代价
  double footprint_cost = 0;
  for (int i = 0; i < within_footprint.size(); i++)
  { 
    // 障碍物碰撞检测 + 与外侧车道线碰撞检测
    unsigned char val = map_->getValue(within_footprint[i].x, within_footprint[i].y);
    if (val >= INFLATED_OBSTACLE - INFLATED_RANGE/* == OBSTACLE */ || val == 20 || val == 24)
    {
      return -1.0;
    }
    // 找到足迹范围内地图上值最大的一点，作为足迹的代价
    else if (val > footprint_cost)
    {
      footprint_cost = val;
    }
  }

  return footprint_cost;
}

float StateLatticeMotionPlanner::detectDeviation(vector<vector<double>> edge)
{
  // 累加整条轨迹上的轨迹点与参考线的绝对距离
  float dis = 0.0;
  for (int i = 0; i < edge.size(); i++)
  {
    double d, lx, ly;
    coordMap2Local(edge[i][0], edge[i][1], lx, ly);
    vector<double> fp = ftf_.cart2Fre(lx, ly);
    
    switch (axis_flag_)
    {
      case 1:
        dis += abs(fp[1] + 1.5);
        break;
      case 2:
        dis += abs(fp[1] - 1.5);
        break;
      case 3:
        dis += abs(fp[1] - 1.5);
        break;
      case 4:
        dis += abs(fp[1] + 1.5);
        break;
      default:
        break;
    }
  }

  return dis;
}

float StateLatticeMotionPlanner::detectContinuousity(vector<vector<double>> edge)
{
  if (last_traj_.points.empty() == true)
  {
    return 0;
  }

  // 将上一历元的规划结果转换到当前历元的Frenet坐标系下
  if (sd_cof_l_.empty() == true)
  {
    for (int i = 0; i < last_traj_.points.size(); i++)
    {
      double xl, yl;
      coordMap2Local(last_traj_.points[i].positions[0], last_traj_.points[i].positions[1], xl, yl);
      
      // 从重合部分开始
      if (xl < 0)
        continue;

      // 重合部分的点带入当前历元的Frenet坐标系下得到坐标
      yl = last_traj_.points[i].positions[1] - y_ref_;
      vector<double> p = ftf_.cart2Fre(xl, yl);
      
      sl_list_.push_back(p[0]);
      dl_list_.push_back(p[1]);
    }

    sd_cof_l_ = cf_.polyFit(sl_list_, dl_list_, 3);
  }

  float d_sum = 0;

  for (int i = 0; i < edge.size(); i += 4)
  {
    double lx, ly;
    coordMap2Local(edge[i][0], edge[i][1], lx, ly);
    vector<double> p = ftf_.cart2Fre(lx, ly);

    // 计算重合部分
    float sc = p[0];
    if (sc > sl_list_.back())
      break;

    float dc = p[1];
    float dl = cf_.polyVal(sd_cof_l_, sc);
    d_sum += abs(dl - dc);
  }

  return d_sum;
}

int StateLatticeMotionPlanner::searchTrajectory()
{
  // 构造优先级队列，首元素为该点开销，末元素顶点号
  priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> q;
  q.push(make_pair(0.0, 0));

  // 设置访问LUT，初始化为所有顶点未访问；LUT三种状态分别为0:Unvisited/1:Visited/2:Closed
  int* visited;
  visited = new int[vertex.size()];
  memset(visited, 0, vertex.size() * sizeof(int));

  // 设置起点为已访问
  visited[0] = 1;

  // 初始化顶点开销表
  double* vertex_cost;
  vertex_cost = new double[vertex.size()];
  fill(vertex_cost, vertex_cost + vertex.size(), 1e5);

  // 初始化后向指针表
  int* back_point;
  back_point = new int[vertex.size()];
  fill(back_point, back_point + vertex.size(), 1e5);

  // 遍历状态格所有顶点
  while (q.size() != 0)
  {
    // 弹出最小堆的堆顶元素，作为当前扩展的节点，弹出时标记节点状态为Closed
    int current = q.top().second;
    vertex_cost[current] = q.top().first;
    q.pop();
    visited[current] = 2;
    
    // 遍历当前扩展节点的邻点
    for (int i = 0; i < vertex.size(); i++)
    {
      // 若存在边
      if (edge[current][i] != -1e5)
      {
        // 若邻点未被访问，加入
        if (visited[i] == 0)
        {
          q.push(make_pair(vertex_cost[current] + edge[current][i], i));
          visited[i] = 1;
          back_point[i] = current;
          vertex_cost[i] = vertex_cost[current] + edge[current][i];
        }
        // 若邻点已访问过，且开销可以降低，则更新其开销
        else if (visited[i] == 1 && vertex_cost[i] > vertex_cost[current] + edge[current][i])
        {
          // 此处考虑优化
          vector<pair<double, int>> temp;
          while (q.top().second != i)
          {
            temp.push_back(q.top());
            q.pop();
          }
          q.pop();
          q.push(make_pair(vertex_cost[current] + edge[current][i], i));
          for (int i = 0; i < temp.size(); i++)
          {
            q.push(temp[i]);
          }
          back_point[i] = current;
          vertex_cost[i] = vertex_cost[current] + edge[current][i];
        }
      }
    }
  }

  // 从最外层节点开始，选择代价最小的节点作为目标，若外层没有目标，则向里搜索
  double cost = 1e5;
  int record;
  int layer = layer_num_;
  bool find_target = false;
  while (find_target == false)
  {
    for (int i = 1; i < vertex.size(); i++)
    {
      if (vertex[i][2] == layer)
      {
        if (vertex_cost[i] < cost)
        {
          record = i;
          cost = vertex_cost[i];
          find_target = true;
        }
      }
    }
    
    // 图中没有符合条件的轨迹
    if (layer-- == 0)
    {
      // 发布空轨迹，提醒规划失败
      trajectory_msgs::JointTrajectory traj;
      traj.header.frame_id = "map";
      // traj_pub.publish(traj);

      return 3;
    }
  }

  // 从目标回溯轨迹至起点并存储，作为最优轨迹上的点，首元素为目标，末元素为起点
  vector<int> v_on_traj;
  while (1)
  {
    v_on_traj.push_back(record);
    if (record == 0)
      break;
    record = back_point[record];
  }

  // 碰撞检测，若最优轨迹未通过，则在边表取消碰撞边，重新搜索，直到搜索到可行解
  for (int i = 0; i < v_on_traj.size() - 1; i++)
  {
    vector<vector<double>> traj = edge_info[v_on_traj[i + 1]][v_on_traj[i]];
    for (int j = 0; j < traj.size(); j += 5)
    {
      double cost = detectCollision(traj[j]);
      if (cost < 0)
      {
        edge[v_on_traj[i + 1]][v_on_traj[i]] = -1e5;
        
        // 轨迹未通过碰撞检测，继续下一轮搜索
        return 2;
      }
    }
  }

  // 可视化最优轨迹
  displayBestEdge(v_on_traj);

  // 发布最优轨迹
  publishTrajectory(v_on_traj);

  // 释放
  delete[] visited;
  delete[] vertex_cost;
  delete[] back_point;

  // 成功找到一条无碰撞轨迹
  return 1;
}

void StateLatticeMotionPlanner::publishTrajectory(vector<int> v_on_traj)
{
  // 轨迹发布
  trajectory_msgs::JointTrajectory traj;
  traj.header.frame_id = "map";
  for (int i = v_on_traj.size() - 1; i > 0; i--)
  {
    for (int j = 0; j < edge_info[v_on_traj[i]][v_on_traj[i - 1]].size(); j++)
    {
      trajectory_msgs::JointTrajectoryPoint traj_points;
      double x = edge_info[v_on_traj[i]][v_on_traj[i - 1]][j][0];
      double y = edge_info[v_on_traj[i]][v_on_traj[i - 1]][j][1];
      traj_points.positions.push_back(x);
      traj_points.positions.push_back(y);
      traj.points.push_back(traj_points);
    }
  }

  traj_pub.publish(traj);
  last_traj_ = traj;
  sd_cof_l_.clear();
  sl_list_.clear();
  dl_list_.clear();
}

void StateLatticeMotionPlanner::displayVertex()
{
  // 顶点可视化
  visualization_msgs::Marker points;
  points.id = 1;
  points.header.frame_id = "map";
  points.header.stamp = ros::Time::now();
  points.ns = "planner";
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.4;
  points.scale.y = 0.4;
  points.color.b = 1.0f;
  points.color.a = 0.7;

  for (int i = 0; i < vertex.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = vertex[i][0];
    p.y = vertex[i][1];
    points.points.push_back(p);
  }

  marker_pub.publish(points);
}

void StateLatticeMotionPlanner::displayEdge()
{
  // 边可视化
  visualization_msgs::Marker line_list;
  line_list.id = 2;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "planner";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.05;
  line_list.scale.y = 0.2;
  line_list.color.b = 1.0;
  line_list.color.a = 0.8;
  line_list.points.clear();
  geometry_msgs::Point p;
  for (int i = 0; i < vertex.size(); i++)
  {
    for (int j = 0; j < vertex.size(); j++)
    {
      if (edge[i][j] != -1e5)
      {
        for (int k = 0; k < edge_info[i][j].size(); k++)
        {
          p.x = edge_info[i][j][k][0];
          p.y = edge_info[i][j][k][1];
          
          // 由于LINE_LIST的显示特点，做如下处理
          if (k == 0 || k == edge_info[i][j].size() - 1)
            line_list.points.push_back(p);
          else
          {
            line_list.points.push_back(p);
            line_list.points.push_back(p);
          }
        }
      }
    }
  }
  marker_pub.publish(line_list);
}

void StateLatticeMotionPlanner::displayBestEdge(std::vector<int> v_on_traj)
{
  // 最优边可视化
  visualization_msgs::Marker line_list;
  line_list.id = 3;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "planner";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.2;
  line_list.scale.y = 0.2;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  line_list.points.clear();
  geometry_msgs::Point p;
  for (int i = v_on_traj.size() - 1; i > 0; i--)
  {
    for (int j = 0; j < edge_info[v_on_traj[i]][v_on_traj[i - 1]].size(); j++)
    {
      p.x = edge_info[v_on_traj[i]][v_on_traj[i - 1]][j][0];
      p.y = edge_info[v_on_traj[i]][v_on_traj[i - 1]][j][1];
      if (j == 0 || j == edge_info[v_on_traj[i]][v_on_traj[i - 1]].size() - 1)
        line_list.points.push_back(p);
      else
      {
        line_list.points.push_back(p);
        line_list.points.push_back(p);
      }
    }
  }
  marker_pub.publish(line_list);
}

void StateLatticeMotionPlanner::displayCurveFit()
{
  // 线方程拟合可视化
  visualization_msgs::Marker line;
  line.id = 4;
  line.header.frame_id = "map";
  line.header.stamp = ros::Time::now();
  line.ns = "planner";
  line.action = visualization_msgs::Marker::ADD;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = 0.4;
  line.scale.y = 0.4;
  line.color.r = 0.5f;
  line.color.b = 0.5f;
  line.color.a = 0.7;

  for (double xi = -10; xi <= 10; xi += 0.05)
  {
    geometry_msgs::Point p;
    coordLocal2Map(xi, cf_.polyVal(line_cof_, xi), p.x, p.y);
    line.points.push_back(p);
  }

  marker_pub.publish(line);
}

void StateLatticeMotionPlanner::coordMap2Local(double mx, double my, double& lx, double& ly)
{
  switch (axis_flag_)
  {
    case 1:
    {
      lx = mx - x_ref_;
      ly = my - y_ref_;
    }
    break;
    
    case 2:
    {
      lx = x_ref_ - mx;
      ly = my - y_ref_;
    }
    break;

    case 3:
    {
      lx = my - y_ref_;
      ly = mx - x_ref_;
    }
    break;

    case 4:
    {
      lx = y_ref_ - my;
      ly = mx - x_ref_;
    }
    break;

    default:
    break;
  }
}

void StateLatticeMotionPlanner::coordLocal2Map(double lx, double ly, double& mx, double& my)
{
  switch (axis_flag_)
  {
    case 1:
    {
      mx = lx + x_ref_;
      my = ly + y_ref_;
    }
    break;
    
    case 2:
    {
      mx = x_ref_ - lx;
      my = ly + y_ref_;
    }
    break;
    
    case 3:
    {
      mx = ly + x_ref_;
      my = lx + y_ref_;
    }
    break;

    case 4:
    {
      mx = ly + x_ref_;
      my = y_ref_ - lx;
    }
    break;

    default:
    break;
  }
}

double StateLatticeMotionPlanner::angleLocal2Map(double theta)
{
  switch (axis_flag_)
  {
    case 1:
      return theta;
    
    case 2:
    { 
      if (theta >= 0)
        return PI - theta;
      else
        return -1 * PI - theta;
    }
    
    case 3:
    {
      if (theta >= -0.5 * PI && theta <= 1.5 * PI)
        return PI / 2 - theta;
      else if (theta < -0.5 * PI)
        return -1.5 * PI - theta;
      else
        return 2.5 * PI - theta;
    }

    case 4:
    {
      if (theta >= -0.5 * PI && theta <= 1.5 * PI)
        return theta - PI / 2;
      else if (theta < -0.5 * PI)
        return 1.5 * PI + theta;
      else
        return theta - 2.5 * PI;
    }

    default:
    break;
  }
}

double StateLatticeMotionPlanner::angleNormalization(double theta)
{
  if (theta > PI)
    return theta - 2 * PI;
  else if (theta < -1 * PI)
    return theta + 2 * PI;
  else
    return theta;
}