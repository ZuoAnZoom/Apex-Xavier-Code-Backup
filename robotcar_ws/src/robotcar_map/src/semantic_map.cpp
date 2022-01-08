#include <robotcar_map/semantic_map.h>
#include <robotcar_map/map_manager.h>

#include <thread>
#include <iostream>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace tf;

SemanticMap::SemanticMap(std::string name)
{
  name_ = name;
  obs_empty_ = true;
  data_ = NULL;
  first_time_ = true;
}

SemanticMap::~SemanticMap()
{
}

void SemanticMap::onInitialize()
{
  default_value_ = NO_INFO;

  // 同步主地图尺寸
  match();

  initMap(size_x_, size_y_);

  // 获取主地图坐标系
  main_map_frame_ = map_manager_ptr_->getMainMapFrame();
  
  ros::NodeHandle nh(name_), g_nh;

  string lidar_topic;
  nh.param("lidar_enable", lidar_enable_, 1);
  nh.param("lane_perception_enable", lane_perception_enable_, 1);
  nh.param("obstacle_perception_enable", obstacle_perception_enable_, 1);
  nh.param("lidar_topic", lidar_topic, string("/scan"));
  nh.param("lidar_frame", lidar_frame_, std::string("laser"));
  nh.param("min_obstacle_height", min_obstacle_height_, 0.0);
  nh.param("max_obstacle_height", max_obstacle_height_, 5.0);
  nh.param("max_obstacle_distance", max_obstacle_distance_, 50.0);
  nh.param("fliter_length", fliter_length_, 2.0);
  nh.param("fliter_width", fliter_width_, 1.4);

  // 话题订阅，队列长度设置为1，实时处理数据
  if (lidar_enable_ == true)
  {
    lidar_sub_ = g_nh.subscribe(lidar_topic, 1, &SemanticMap::inputLaserData, this);
  }

  if (lane_perception_enable_ == true)
  {
    lane_sub_ = g_nh.subscribe("/perception/lanes", 1, &SemanticMap::inputLanePerception, this);
  }

  if (obstacle_perception_enable_ == true)
  {
    obstacle_sub_ = g_nh.subscribe("/perception/obstacles", 1, &SemanticMap::inputObstaclePerception, this);
  }

  // 启动消息处理线程
  std::thread spin_task(spinTopic);
  spin_task.detach();
}

void SemanticMap::inputLaserData(const sensor_msgs::LaserScanConstPtr& msg)
{
  // 从LaserScan消息转换为PCL2格式
  sensor_msgs::PointCloud2 cloud;
  cloud.header = msg->header;
  laser_geometry::LaserProjection projector_;
  projector_.transformLaserScanToPointCloud(msg->header.frame_id, *msg, cloud, *tf_);

  // 将PCL2格式点云转换为PointCloud<pcl::PointXYZ> 格式
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);
  pcl::PointCloud < pcl::PointXYZ > cloud_lidar;
  pcl::fromPCLPointCloud2(pcl_pc2, cloud_lidar);

  // 过滤掉点云中车体的部分
  for (int i = 0; i < cloud_lidar.size(); i++)
  {
    if (cloud_lidar[i].x > -1.8 && cloud_lidar[i].x < 0.2 &&
        cloud_lidar[i].y > -0.8 && cloud_lidar[i].y < 0.8)
    {
      cloud_lidar.erase(cloud_lidar.begin() + i--);
    }
  }

  // 将雷达坐标原点从雷达坐标系转换到主地图坐标系，并进行存储
  Stamped <tf::Vector3> lidar_origin_main;
  Stamped <tf::Vector3> lidar_origin_lidar(tf::Vector3(0, 0, 0),
                        pcl_conversions::fromPCL(cloud_lidar.header).stamp, lidar_frame_);

  tfl.waitForTransform(main_map_frame_, lidar_origin_lidar.frame_id_, lidar_origin_lidar.stamp_, ros::Duration(0.5));
  
  try
  {
    tfl.transformPoint(main_map_frame_, lidar_origin_lidar, lidar_origin_main);
  }
  catch (tf::TransformException &ex) 
  {
    ros::Duration(1.0).sleep();
    return;
  }
  
  LidarObs observation;
  observation.origin_.x = lidar_origin_main.getX();
  observation.origin_.y = lidar_origin_main.getY();
  observation.origin_.z = lidar_origin_main.getZ();

  // 将雷达观测到的点云从雷达坐标系转换到主地图坐标系
  pcl::PointCloud <pcl::PointXYZ> cloud_main;

  try
  {
    pcl_ros::transformPointCloud(main_map_frame_, cloud_lidar, cloud_main, *tf_);
  }
  catch (tf::TransformException &ex) 
  {
    ros::Duration(1.0).sleep();
    return;
  }

  cloud_main.header.stamp = cloud_lidar.header.stamp;

  // 剔除点云中高度超过阈值的点，将剩余点存放进观测值结构体中
  int cloud_size = cloud_main.points.size();
  observation.cloud_.points.resize(cloud_size);
  int count = 0;

  for (int i = 0; i < cloud_size; i++)
  { 
    if (cloud_main.points[i].z <= max_obstacle_height_
        && cloud_main.points[i].z >= min_obstacle_height_)
    {
      observation.cloud_.points[count++] = cloud_main.points[i];
    }
  }

  observation.cloud_.points.resize(count);
  observation.cloud_.header.stamp = cloud_lidar.header.stamp;
  observation.cloud_.header.frame_id = cloud_main.header.frame_id;
  
  lock_.lock();
  observations_.push_back(observation);
  lock_.unlock();
}

void SemanticMap::inputLanePerception(const robotcar_map::lanes& msg)
{
  // 存储车道线点
  int lane_num = msg.num;

  l_lock_.lock();

  lanes_x.clear();
  lanes_y.clear();
  
  lanes_stamp_ = msg.stamp;

  if (lane_num > 0)
  {
    lanes_x.push_back(msg.x_0);
    lanes_y.push_back(msg.y_0);
  }

  if (lane_num > 1)
  {
    lanes_x.push_back(msg.x_1);
    lanes_y.push_back(msg.y_1);
  }

  if (lane_num > 2)
  {
    lanes_x.push_back(msg.x_2);
    lanes_y.push_back(msg.y_2);
  }

  if (lane_num > 3)
  {
    lanes_x.push_back(msg.x_3);
    lanes_y.push_back(msg.y_3);
  }

  l_lock_.unlock();
}

void SemanticMap::inputObstaclePerception(const robotcar_map::obstacles& msg)
{
  o_lock_.lock();

  obstacles_stamp_ = msg.stamp;
  obstacle_x_min_ = msg.x_min;
  obstacle_x_max_ = msg.x_max;
  obstacle_y_min_ = msg.y_min;
  obstacle_y_max_ = msg.y_max;

  o_lock_.unlock();
}

void SemanticMap::markLaserObstacles(const LidarObs& observation)
{
  for (int i = 0; i < observation.cloud_.size(); i++)
  {
    double distance_sq = (observation.cloud_.points[i].x - observation.origin_.x) * (observation.cloud_.points[i].x - observation.origin_.x) +
                         (observation.cloud_.points[i].y - observation.origin_.y) * (observation.cloud_.points[i].y - observation.origin_.y);
    
    if (distance_sq > max_obstacle_distance_ * max_obstacle_distance_)
      continue;
    
    unsigned int mx, my;

    // TODO: 考虑地物优先级
    if (!world2Map(observation.cloud_.points[i].x, observation.cloud_.points[i].y, mx, my)
        /* || getValue(mx, my) == LANE */)
      continue;
    
    setValue(mx, my, OBSTACLE);
  }
}

void SemanticMap::markLaserFreespace(const LidarObs& observation, double* min_x, double* min_y, double* max_x, double* max_y)
{
  // 获取雷达在主地图上的坐标
  double ox = observation.origin_.x;
  double oy = observation.origin_.y;

  // 确保雷达坐标被包含在bound范围内
  includeBound(ox, oy, min_x, min_y, max_x, max_y);

  // 将坐标转换为cell单位
  unsigned int x0, y0;

  if (!world2Map(ox, oy, x0, y0))
  {
    ROS_WARN("当前雷达位置不在地图范围内，无法更新障碍地图...\n");
    return;
  }

  // 获取地图尺寸
  const double origin_x = origin_x_, origin_y = origin_y_;
  const double end_x = origin_x + size_x_ * resolution_;
  const double end_y = origin_y + size_y_ * resolution_;

  // 对当前观测值中的点云点进行迭代，并对主地图范围外的点进行约束
  for (int i = 0; i < observation.cloud_.size(); i++)
  {
    double wx = observation.cloud_.points[i].x;
    double wy = observation.cloud_.points[i].y;

    double distance_sq = (wx - ox) * (wx - ox) + (wy - oy) * (wy - oy);
    
    if (distance_sq > (max_obstacle_distance_ / 2.0) * (max_obstacle_distance_ / 2.0))
      continue;

    if (wx < origin_x)
    {
      wy = oy + (wy - oy) * (ox - origin_x) / (ox - wx);
      wx = origin_x;
    }

    if (wy < origin_y)
    {
      wx = ox + (wx - ox) * (oy - origin_y) / (oy - wy);
      wy = origin_y;
    }

    if (wx > end_x)
    {
      wy = oy + (wy - oy) * (end_x - ox) / (wx - ox);
      wx = end_x;
    }

    if (wy > end_y)
    {
      wx = ox + (wx - ox) * (end_y - oy) / (wy - oy);
      wy = end_y;
    }

    // 将约束后的点转换为cell单位
    unsigned int mx, my;
    if (!world2Map(wx, wy, mx, my))
      continue;

    // 提取雷达和障碍物连线，将连线上的cell的值设置为FREE
    // 并确保连线上的每一个cell都被包含进bound范围
    std::vector<MapCell> line;
    getLine(x0, y0, mx, my, line);

    for (int i = 0; i < line.size(); i++)
    {
      setValue(line[i].x, line[i].y, FREE);
      double x_trans, y_trans;
      map2World(line[i].x, line[i].y, x_trans, y_trans);
      includeBound(x_trans, y_trans, min_x, min_y, max_x, max_y);
    }
  }
}

void SemanticMap::markVisualLanes(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  // 存储车道线观测值
  l_lock_.lock();
  vector<vector<double>> lanes_obs_x = lanes_x;
  vector<vector<double>> lanes_obs_y = lanes_y;
  ros::Time lanes_stamp = lanes_stamp_;
  l_lock_.unlock();

  // 获取自身位姿
  unsigned int mx_self, my_self;
  float wx_self = robotcar_x;
  float wy_self = robotcar_y;
  world2Map(wx_self, wy_self, mx_self, my_self);

  // 对车道线按横向顺序排序，由右至左
  bool swap = false;
  for (int i = lanes_obs_x.size() - 2; i >= 0; i--)
  {
    for (int j = 0; j <= i; j++)
    {
      if (lanes_obs_y[j][0] > lanes_obs_y[j+1][0])
      {
        lanes_obs_y[j].swap(lanes_obs_y[j+1]);
        lanes_obs_x[j].swap(lanes_obs_x[j+1]);
        swap = true;
      }
    }
    if (swap == false)
    {
      break;
    }
  }

  // 观测值转换到地图坐标系下
  try
  {
    tfl.waitForTransform("left_camera", "map", lanes_stamp, ros::Duration(10));
    for (int i = 0; i < lanes_obs_x.size(); i++)
    {
      for (int j = 0; j < lanes_obs_x[i].size(); j++)
      {
        double wx, wy;
        camera2Map(lanes_obs_x[i][j], lanes_obs_y[i][j], wx, wy, lanes_stamp_);
        lanes_obs_x[i][j] = wx;
        lanes_obs_y[i][j] = wy;
      }
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // 判断误检测并将其剔除
  vector<vector<double>> obs_cof;
  for (int i = 0; i < lanes_obs_x.size(); i++)
  {
    obs_cof.push_back(cf_.polyFit(lanes_obs_x[i], lanes_obs_y[i], 1));
  }

  if (first_time_ == false)
  {
    for (int i = 0; i < obs_cof.size(); i++)
    {
      if (abs(obs_cof[i][1] - slope_last_) > 2)
      {
        printConfig();
        ROS_INFO("出现误检测，已剔除\n");
        lanes_obs_x.erase(lanes_obs_x.begin() + i);
        lanes_obs_y.erase(lanes_obs_y.begin() + i);
      }
    }
  }

  // 提取搜索空间
  double r = 10.0;
  vector<MapCell> detect_space;
  vector<MapCell> around_self(4);
  around_self[0].x = max(int(mx_self) - int(r / resolution_), 0);
  around_self[0].y = max(int(my_self) - int(r / resolution_), 0);
  around_self[1].x = around_self[0].x;
  around_self[1].y = min(int(my_self) + int(r / resolution_), int(size_x_) - 1);
  around_self[2].x = min(int(mx_self) + int(r / resolution_), int(size_x_) - 1);
  around_self[2].y = around_self[1].y;
  around_self[3].x = around_self[2].x;
  around_self[3].y = around_self[0].y;
  getPolygon(around_self, detect_space);

  // 搜索地图上的车道线点及其CODE
  vector<unsigned int> lanes_code;
  vector<vector<geometry_msgs::Point>> lanes_on_map;
  for (int i = 0; i < detect_space.size(); i++)
  {
    unsigned char val = map_manager_ptr_->getMap()->getValue(detect_space[i].x, detect_space[i].y);
    if (val >= LANE && val <= LANE + LANE_RANGE)
    {
      // 更新该区域车道线点前，先将原先车道线点存储并清除
      geometry_msgs::Point point;
      map2World(detect_space[i].x, detect_space[i].y, point.x, point.y);
      setValue(detect_space[i].x, detect_space[i].y, FREE);

      // 在存储过程中，将提取到的车道线点对应存放
      bool find = false;
      for (int j = 0; j < lanes_code.size(); j++)
      {
        if (lanes_code[j] == val)
        {
          find = true;
          lanes_on_map[j].push_back(point);
        }
      }
      
      // 若提取到新的车道线点，则新建分类并存放
      if (find == false)
      {
        vector<geometry_msgs::Point> lane;
        lanes_on_map.push_back(lane);
        lanes_on_map[lanes_on_map.size() - 1].push_back(point);
        lanes_code.push_back(val);
      }
    }
  }

  // 找到最大CODE，若观测到新的车道线，补充到地图上时在此基础上设置新的CODE
  unsigned char max_code = 1;
  for (int i = 0; i < lanes_code.size(); i++)
  {
    if (lanes_code[i] > max_code)
    {
      max_code = lanes_code[i];
    }
  }

  // 观测值与地图进行匹配
  vector<int> matching_res;
  int ref_map_index = -1;
  int ref_obs_index = -1;
  if (lanes_on_map.empty() == false)
  {
    // 每条观测到的车道线与地图上的车道线进行匹配
    for (int i = 0; i < lanes_obs_x.size(); i++)
    {
      vector<double> bias(lanes_on_map.size());
      for (int j = 0; j < lanes_on_map.size(); j++)
      {
        bias[j] = 0;
        for (int k = 0; k < lanes_obs_x[i].size(); k++)
        {
          float d_sq_min = 1e10;
          for (int l = 0; l < lanes_on_map[j].size(); l++)
          {
            float d_sq = pow((lanes_on_map[j][l].x - lanes_obs_x[i][k]), 2) + pow((lanes_on_map[j][l].y - lanes_obs_y[i][k]), 2);
            if (d_sq < d_sq_min)
            {
              d_sq_min = d_sq;
            }
          }
          bias[j] += sqrt(d_sq_min);
        }
      }

      // 将地图上与观测到的车道线偏差最小的车道线作为其匹配结果并记录
      double bias_min = 1e10;
      int bias_min_i;
      for (int m = 0; m < bias.size(); m++)
      {
        if (bias[m] < bias_min)
        {
          bias_min = bias[m];
          bias_min_i = m;
        }
      }

      // 在偏差阈值范围内，认为匹配成功，否则认为需要向地图新增车道线
      if (bias_min < 1e5)
      {
        matching_res.push_back(bias_min_i);
        ref_obs_index = i;
        ref_map_index = bias_min_i;
      }
      else
      {
        max_code += 2;
        lanes_code.push_back(max_code);
        matching_res.push_back(lanes_code.size() - 1);
      }
    }
  }

  // 恢复地图上存在而未检测到的车道线
  for (int i = 0; i < lanes_on_map.size(); i++)
  {
    // 找到地图上未被匹配的车道线
    bool matching_success = false;
    for (int j = 0; j < matching_res.size(); j++)
    {
      if (matching_res[j] == i)
      {
        matching_success = true;
        break;
      }
    }

    if (matching_success == true)
    {
      continue;
    }

    // 用该次观测检测到的其他车道线来恢复未被匹配的车道线
    vector<double> obs_x_cal, obs_y_cal;
    if (ref_map_index != -1)
    {
      // 找到地图上未被匹配的车道线的中心点
      int mid = lanes_on_map[i].size() / 2;
      int closest_of_ref_i;
      double dmin = 1e10;

      // 找到地图上用于恢复的车道线距离其最近的点
      for (int k = 0; k < lanes_on_map[ref_map_index].size(); k++)
      {
        double d = sqrt(pow((lanes_on_map[ref_map_index][k].x - lanes_on_map[i][mid].x), 2) +
                  pow((lanes_on_map[ref_map_index][k].y - lanes_on_map[i][mid].y), 2));
        if (d < dmin)
        {
          dmin = d;
          closest_of_ref_i = k;
        }
      }

      // 获取平移关系
      double dx = lanes_on_map[i][mid].x - lanes_on_map[ref_map_index][closest_of_ref_i].x;
      double dy = lanes_on_map[i][mid].y - lanes_on_map[ref_map_index][closest_of_ref_i].y;
      
      // 平移车道线，恢复出未被匹配的车道线的观测值
      for (int k = 0; k < lanes_obs_x[ref_obs_index].size(); k++)
      {
        obs_x_cal.push_back(lanes_obs_x[ref_obs_index][k] + dx);
        obs_y_cal.push_back(lanes_obs_y[ref_obs_index][k] + dy);
      }
    }

    lanes_obs_x.push_back(obs_x_cal);
    lanes_obs_y.push_back(obs_y_cal);
    matching_res.push_back(i);
  }

  // 将观测值和地图上的车道线对应拟合
  double slope_sum = 0;
  for (int i = 0; i < lanes_obs_x.size(); i++)
  {
    int id;
    vector<double> x = lanes_obs_x[i], y = lanes_obs_y[i];

    // 若非新增车道线，则加入地图上对应的车道线点一起拟合
    if (lanes_on_map.empty() == false)
    {
      id = matching_res[i];
      if (id < lanes_on_map.size())
      {
        for (int j = 0; j < lanes_on_map[id].size(); j++)
        {
          x.push_back(lanes_on_map[id][j].x);
          y.push_back(lanes_on_map[id][j].y);
        }
      }
    }

    // 确定拟合的x坐标范围
    double x_min = 1e10, x_max = -1e10;
    for (int k = 0; k < x.size(); k++)
    {
      x[k] -= wx_self;
      y[k] -= wy_self;
      x[k] < x_min ? x_min = x[k]: x_min;
      x[k] > x_max ? x_max = x[k]: x_max;
      includeBound(x[k], y[k], min_x, min_y, max_x, max_y);
    }

    // 拟合
    vector<double> cof = cf_.polyFit(x, y, 3);

    double x_min_extend = x_min - 0 * (x_max - x_min);
    double x_max_extend = x_max + 0 * (x_max - x_min);

    for (double xi = x_min_extend; xi < x_max_extend; xi += (x_max - x_min) / 100.0)
    {
      double xc = xi + wx_self;
      double yc = cf_.polyVal(cof, xi) + wy_self;
      unsigned int xm, ym;
      if (!world2Map(xc, yc, xm, ym))
      {
        continue;
      }
      if (lanes_on_map.empty() == false)
      {
        setValue(xm, ym, lanes_code[id]);
      }
      else
      {
        setValue(xm, ym, i * 2 + LANE);
      }
    }

    // 记录本次车道线观测的平均斜率，用于后续观测的误检
    double x_mid = 0.5 * (x_min + x_max);
    double slope = cof[1] + 2 * x_mid * cof[2] + 3 * x_mid * x_mid * cof[3];
    slope_sum += slope;
  }

  slope_last_ = slope_sum / lanes_obs_x.size();
  first_time_ = false;
}

void SemanticMap::markVisualObstacles(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  o_lock_.lock();
  ros::Time obstacles_stamp = obstacles_stamp_;
  vector<double> obstacle_x_min = obstacle_x_min_;
  vector<double> obstacle_x_max = obstacle_x_max_;
  vector<double> obstacle_y_min = obstacle_y_max_;
  vector<double> obstacle_y_max = obstacle_y_min_;
  o_lock_.unlock();

  for (int i = 0; i < obstacle_y_min.size(); i++)
  {
    obstacle_y_min[i] = -obstacle_y_min[i];
    obstacle_y_max[i] = -obstacle_y_max[i];
  }

  unsigned int mx_self, my_self;
  float wx_self = robotcar_x;
  float wy_self = robotcar_y;
  world2Map(wx_self, wy_self, mx_self, my_self);

  // 提取搜索空间
  double r = 20.0;
  vector<MapCell> detect_space;
  vector<MapCell> around_self(4);
  around_self[0].x = max(int(mx_self) - int(r / resolution_), 0);
  around_self[0].y = max(int(my_self) - int(r / resolution_), 0);
  around_self[1].x = around_self[0].x;
  around_self[1].y = min(int(my_self) + int(r / resolution_), int(size_x_) - 1);
  around_self[2].x = min(int(mx_self) + int(r / resolution_), int(size_x_) - 1);
  around_self[2].y = around_self[1].y;
  around_self[3].x = around_self[2].x;
  around_self[3].y = around_self[0].y;
  getPolygon(around_self, detect_space);

  for (int i = 0; i < detect_space.size(); i++)
  {
    unsigned int val = getValue(detect_space[i].x, detect_space[i].y);
    if (val < LANE || val > LANE + LANE_RANGE)
    {
      setValue(detect_space[i].x, detect_space[i].y, FREE);
    }
  }

  int obstacles_num = obstacle_x_max.size();

  try
  {
    tfl.waitForTransform("map", "left_camera", obstacles_stamp, ros::Duration(10));

    for (int i = 0; i < obstacles_num; i++)
    {
      if (obstacle_x_min[i] > 20.0)
      {
        continue;
      }

      vector<MapCell> bounding_box(4);
      camera2Cell(obstacle_x_min[i], obstacle_y_min[i], bounding_box[0], obstacles_stamp);
      camera2Cell(obstacle_x_min[i], obstacle_y_max[i], bounding_box[1], obstacles_stamp);
      camera2Cell(obstacle_x_max[i], obstacle_y_max[i], bounding_box[2], obstacles_stamp);
      camera2Cell(obstacle_x_max[i], obstacle_y_min[i], bounding_box[3], obstacles_stamp);

      vector<MapCell> obstacle;
      getPolygon(bounding_box, obstacle);
      for (int j = 0; j < obstacle.size(); j++)
      {
        unsigned int val = getValue(obstacle[j].x, obstacle[j].y);
        if (val < LANE || val > LANE + LANE_RANGE)
        {
          setValue(obstacle[j].x, obstacle[j].y, OBSTACLE);
        }
      }

      for (int j = 0; j < bounding_box.size(); j++)
      {
        includeBound(bounding_box[j].x, bounding_box[j].y, min_x, min_y, max_x, max_y);
      }
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
}

void SemanticMap::updateBounds(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y,
                  double* max_x, double* max_y)
{
  if (lidar_enable_ == true)
  {
    if (observations_.empty())
    { 
      obs_empty_ = true;
      return;
    }

    obs_empty_ = false;
    
    // 读取雷达观测值buffer，并将其清空
    lock_.lock();
    list<LidarObs> observations_copy = observations_;
    observations_.clear();
    lock_.unlock();
    
    list<LidarObs>::iterator iter;
    for (iter = observations_copy.begin(); iter != observations_copy.end(); iter++)
    {
      markLaserFreespace(*iter, min_x, min_y, max_x, max_y);
      markLaserObstacles(*iter);
    }
  }

  if (lane_perception_enable_ == true)
  {
    markVisualLanes(robotcar_x, robotcar_y, robotcar_yaw, min_x, min_y, max_x, max_y);
  }

  if (obstacle_perception_enable_ == true)
  {
    markVisualObstacles(robotcar_x, robotcar_y, robotcar_yaw, min_x, min_y, max_x, max_y);
  }

  // 确保当前机器人足迹包含进更新范围内
  transformFootprint(robotcar_x, robotcar_y, robotcar_yaw);

  for (int i = 0; i < transformed_footprint_.size(); i++)
  {
    includeBound(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void SemanticMap::updateData(Map& main_map, int min_i, int min_j, int max_i, int max_j)
{
  updateWithKnownData(main_map, min_i, min_j, max_i, max_j);
}

void SemanticMap::includeBound(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y)
{
  *min_x = min(x, *min_x);
  *min_y = min(y, *min_y);
  *max_x = max(x, *max_x);
  *max_y = max(y, *max_y);
}

void SemanticMap::transformFootprint(double robotcar_x, double robotcar_y, double robotcar_yaw)
{
  transformed_footprint_.clear();

  for (int i = 0; i < footprint_.size(); i++)
  { 
    double dis = hypot(abs(footprint_[i].x), abs(footprint_[i].y));
    double ang = atan2(footprint_[i].y, footprint_[i].x);
    
    geometry_msgs::Point point;
    point.x = robotcar_x + footprint_[i].x + dis * (cos(ang + robotcar_yaw) - cos(ang));
    point.y = robotcar_y + footprint_[i].y + dis * (sin(ang + robotcar_yaw) - sin(ang));
    transformed_footprint_.push_back(point);
  }
}

void SemanticMap::camera2Map(double camera_x, double camera_y, double& map_x, double& map_y, ros::Time stamp)
{
  Stamped <tf::Vector3> map_point;
  Stamped <tf::Vector3> camera_point(tf::Vector3(camera_x, camera_y, 0), stamp, "left_camera");

  tfl.transformPoint("map", camera_point, map_point);
  map_x = map_point.getX();
  map_y = map_point.getY();
}

void SemanticMap::camera2Cell(double camera_x, double camera_y, MapCell& map_cell, ros::Time stamp)
{
  int mx, my;
  double wx, wy;

  camera2Map(camera_x, camera_y, wx, wy, stamp);
  world2MapWithBound(wx, wy, mx, my);

  map_cell.x = mx;
  map_cell.y = my;
}

void SemanticMap::spinTopic()
{
  ros::spin();
}