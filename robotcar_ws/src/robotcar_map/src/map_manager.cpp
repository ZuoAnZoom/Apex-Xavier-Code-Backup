#include <robotcar_map/map_manager.h>

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>

using namespace std;

MapManager::MapManager(string main_map_frame):
    main_map_(),
    main_map_frame_(main_map_frame),
    map_seq_(0)
{
  // 主地图的默认值设为FREE
  main_map_.setDefaultValue(FREE);
}

MapManager::~MapManager()
{
  while (sub_maps_ptr_.size() > 0)
  {
    sub_maps_ptr_.pop_back();
  }
}

void MapManager::addSubMap(boost::shared_ptr<Map> sub_map_ptr)
{
  sub_maps_ptr_.push_back(sub_map_ptr);
}

void MapManager::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                          double origin_y)
{
  boost::unique_lock<boost::recursive_mutex> lock(*(main_map_.getMutex()));

  // 重设大小，设置默认值
  main_map_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  
  // 逐层同步子地图的尺寸
  for (vector< boost::shared_ptr<Map> >::iterator iter = sub_maps_ptr_.begin(); iter != sub_maps_ptr_.end();
       iter++)
  {
    (*iter)->match();
  }
}

void MapManager::updateMap(double robotcar_x, double robotcar_y, double robotcar_yaw)
{
  if (sub_maps_ptr_.size() == 0)
  {
    printConfig();
    ROS_INFO("没有任何子地图可供使用，无法更新地图...\n");
    return;
  }

  // 确保每层子地图对主地图进行更改时，其他层子地图不会干扰
  boost::unique_lock<boost::recursive_mutex> lock(*(main_map_.getMutex()));
  
  // bound初始化为最小值
  double min_x = 1e30, min_y = 1e30, max_x = -1e30, max_y = -1e30;

  for (vector< boost::shared_ptr<Map> >::iterator iter = sub_maps_ptr_.begin(); iter != sub_maps_ptr_.end();
       iter++)
  {
    (*iter)->updateBounds(robotcar_x, robotcar_y, robotcar_yaw, &min_x, &min_y, &max_x, &max_y);
  }

  int min_x_c, min_y_c, max_x_c, max_y_c;
  main_map_.world2MapWithBound(min_x, min_y, min_x_c, min_y_c);
  main_map_.world2MapWithBound(max_x, max_y, max_x_c, max_y_c);

  if (min_x_c > max_x_c || min_y_c > max_y_c)
  {
    // cout << "初始化Bound失败..." << endl;
    return;
  }

  // 在更新后的bound范围内进行清理
  // for (int i = min_x_c; i <= max_x_c; i++)
  // {
  //   for (int j = min_y_c; j <= max_y_c; j++)
  //   {
  //     main_map_.setValue(i, j, main_map_.getDefaultValue());
  //   }
  // }

  for (vector< boost::shared_ptr<Map> >::iterator iter = sub_maps_ptr_.begin(); iter != sub_maps_ptr_.end();
       iter++)
  {
    (*iter)->updateData(main_map_, min_x_c, min_y_c, max_x_c, max_y_c);
  }

}

void MapManager::setFootprint(const std::vector<geometry_msgs::Point>& footprint)
{
  footprint_ = footprint;

  for (vector< boost::shared_ptr<Map> >::iterator iter = sub_maps_ptr_.begin(); iter != sub_maps_ptr_.end();
       iter++)
  {
    (*iter)->setFootprint(footprint);
  }
}

void MapManager::publishMap()
{
  ros::NodeHandle nh;
  map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("main_map", 1);
  
  ros::Time stamp = ros::Time::now();

  // 创建地图消息
  nav_msgs::OccupancyGrid map;

  // 设置时间戳
  map.header.seq = map_seq_++;
  map.header.stamp.sec = stamp.sec;
  map.header.stamp.nsec = stamp.nsec;
  map.header.frame_id = main_map_frame_;

  ros::Time load = ros::Time::now();

  map.info.map_load_time.sec = load.sec;
  map.info.map_load_time.nsec = load.nsec;
  
  // 设置地图大小、分辨率等参数
  map.info.resolution = main_map_.getResolution();
  map.info.width = main_map_.getSizeInCellsX();
  map.info.height = main_map_.getSizeInCellsY();
  map.info.origin.position.x = main_map_.getOriginX();
  map.info.origin.position.y = main_map_.getOriginY();
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;

  unsigned int num = main_map_.getSizeInCellsX() * main_map_.getSizeInCellsY();
  unsigned char* map_ptr = main_map_.getData();

  // 设置地图信息
  for (unsigned int i = 0; i < num; i++)
  {
    unsigned char map_value = map_ptr[i];
    if (map_value == FREE)
    {
      map.data.push_back(WHITE);
    }

    else if (map_value == OBSTACLE)
    {
      map.data.push_back(BLACK);
    }

    else if (map_value == NO_INFO)
    {
      map.data.push_back(LIGHT_GREEN);
    }

    else if (map_value >= LANE && map_value <= LANE + LANE_RANGE)
    {
      map.data.push_back(GREEN);
    }

    else
    {
      double value = map_value * 100.0 / OBSTACLE;
      map.data.push_back((int)value);
    }
  }
 
  // 发布地图
  map_pub_.publish(map);
}
