#include <robotcar_map/static_map.h>
#include <robotcar_map/map_manager.h>

#include <iostream>

using namespace std;

StaticMap::StaticMap(string name)
{
  name_ = name;
  use_static_map_bound_ = false;
  use_static_map_data_ = false;
}

StaticMap::~StaticMap()
{

}

void StaticMap::onInitialize()
{
  main_map_frame_ = map_manager_ptr_->getMainMapFrame();

  // 在地图名的空间下提取参数
  ros::NodeHandle nh(name_), g_nh; 
  ros::Rate r(10);
  int map_src;
  string map_topic;
  string map_dir;
  string ws_dir;

  // 工作空间目录
  g_nh.param("workspace_abs_dir", ws_dir, string("/home/h/robotcar_ws/src/"));
  // 地图源：1-话题/2-文件/3-空地图
  nh.param("map_src", map_src, 1);
  // 若地图源为话题，输入地图话题名
  nh.param("map_topic", map_topic, string("static_map"));
  // 若地图源为文件，输入文件目录
  nh.param("map_dir", map_dir, string("maps/png_maps/roundrect_with_obstacles.png"));

  map_dir = ws_dir + string("robotcar_map/") + map_dir;

  // 地图输入源为话题
  if (map_src == 1)
  {
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticMap::topicCallback, this);

    // 当接收到一张地图后，关闭静态地图节点
    while (!map_received_ && g_nh.ok()) 
    {
      ros::spinOnce();
      r.sleep();
    }
  }
  // 地图输入源为文件
  else if (map_src == 2)
  {
    fileCallback(map_dir, &nh);
  }
  // 地图输入源为空地图
  else if (map_src == 3)
  {
    setEmptyMap(&nh);
  }
}

void StaticMap::topicCallback(const nav_msgs::OccupancyGridConstPtr& input_map)
{
  unsigned int size_x = input_map->info.width;
  unsigned int size_y = input_map->info.height;
  double resolution = input_map->info.resolution;
  double origin_x = input_map->info.origin.position.x;
  double origin_y = input_map->info.origin.position.y;
    
  Map* main_map = map_manager_ptr_->getMap();

  if (main_map->getSizeInCellsX() != size_x ||
      main_map->getSizeInCellsY() != size_y ||
      main_map->getResolution()   != resolution ||
      main_map->getOriginX()      != origin_x ||
      main_map->getOriginY()      != origin_y)
  {
    main_map->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  // 设置本层地图的尺寸
  resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  
  int index = 0;

  sub_map_frame_ = input_map->header.frame_id;

  // 设置本层地图的数据
  for (int i = 0; i < size_y; ++i)
  {
    for (int j = 0; j < size_x; ++j)
    {
      unsigned char value = input_map->data[index];
      data_[index] = interpretTopicValue(value);
      ++index;
    }
  }
  
  // 设置标志位
  map_received_ = true;

  // 当接收到了一张静态地图后，关闭话题订阅句柄
  map_sub_.shutdown();
}

void StaticMap::fileCallback(string file, ros::NodeHandle* nh_ptr)
{
  // 读地图
  cv::Mat image;
  image = cv::imread(file);

  if (image.data== nullptr)
  {
    ROS_ERROR("地图文件不存在！\n");
    return;
  }

  unsigned int size_x = image.cols;
  unsigned int size_y = image.rows;

  // 当从文件读取地图时，需要从参数服务器读取以下地图参数
  double resolution, origin_x, origin_y;
  nh_ptr->getParam("resolution", resolution);
  nh_ptr->getParam("origin_x", origin_x);
  nh_ptr->getParam("origin_y", origin_y);

  Map* main_map = map_manager_ptr_->getMap();

  if (main_map->getSizeInCellsX() != size_x ||
      main_map->getSizeInCellsY() != size_y ||
      main_map->getResolution()   != resolution ||
      main_map->getOriginX()      != origin_x ||
      main_map->getOriginY()      != origin_y)
  {
    main_map->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  
  int index = 0;

  sub_map_frame_ = "static_map";

  // 设置本层地图的数据
  for (int i = size_y - 1; i > 0; --i)
  {
    for (int j = 0; j < size_x; ++j)
    {
      int b = image.at<cv::Vec3b>(i,j)[0];
      int g = image.at<cv::Vec3b>(i,j)[1];
      int r = image.at<cv::Vec3b>(i,j)[2];
      
      data_[index] = interpretRGBValue(r, g, b);
      ++index;
    }
  }
  
  map_received_ = true;
}

void StaticMap::setEmptyMap(ros::NodeHandle* nh_ptr)
{
  // 当从文件读取地图时，需要从参数服务器读取以下地图参数
  int size_x, size_y;
  double resolution, origin_x, origin_y;
  nh_ptr->getParam("size_x_empty", size_x);
  nh_ptr->getParam("size_y_empty", size_y);
  nh_ptr->getParam("resolution_empty", resolution);
  nh_ptr->getParam("origin_x_empty", origin_x);
  nh_ptr->getParam("origin_y_empty", origin_y);

  Map* main_map = map_manager_ptr_->getMap();

  if (main_map->getSizeInCellsX() != size_x ||
      main_map->getSizeInCellsY() != size_y ||
      main_map->getResolution()   != resolution ||
      main_map->getOriginX()      != origin_x ||
      main_map->getOriginY()      != origin_y)
  {
    main_map->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  // 设置本层地图的尺寸
  resizeMap(size_x, size_y, resolution, origin_x, origin_y);

  sub_map_frame_ = "static_map";
  
  // 设置标志位
  map_received_ = true;

  // 当接收到了一张静态地图后，关闭话题订阅句柄
  map_sub_.shutdown();
}

unsigned char StaticMap::interpretTopicValue(unsigned char value)
{
  // 构造三元地图（Trinary Map）：即只有空余、障碍、未知三种状态，没有介于其间的值
  if (value == NO_INFO_TOPIC)
  { 
    return NO_INFO;
  }
  else if (value >= OBSTACLE_TOPIC)
  {
    return OBSTACLE;
    // return LANE;
  }
  else
  {
    return FREE;
  }
}

unsigned char StaticMap::interpretRGBValue(int r, int g, int b)
{
  int lane_num = 0;
  vector<int> lane_g;

  if (r == FREE_R && g == FREE_G && b == FREE_B)
  {
    return FREE;
  }
  else if (r == OBSTACLE_R && g == OBSTACLE_G && b == OBSTACLE_B)
  {
    return OBSTACLE;
  }
  else if(r == LANE_R && g >= LANE_G && g <= LANE_G + LANE_G_RANGE && b == LANE_B)
  {    
    return LANE + g - LANE_G;
  }
  else
  {
    return FREE;
  }
}

void StaticMap::updateBounds(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y,
                  double* max_x, double* max_y)
{
  // 只在静态地图传入后用其更新一次bound，之后都不再用其更新
  if (use_static_map_bound_ == true)
  {
    return;
  }
  
  // 确保传入的范围不小于整张静态地图的范围
  double wx, wy;

  map2World(0, 0, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  map2World(size_x_, size_y_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  use_static_map_bound_ = true;
}

void StaticMap::updateData(Map& main_map, int min_i, int min_j, int max_i, int max_j)
{
  if (use_static_map_data_ == true)
  {
    return;
  }

  if (map_received_)
    updateWithAllData(main_map, min_i, min_j, max_i, max_j);

  use_static_map_data_ = true;
}