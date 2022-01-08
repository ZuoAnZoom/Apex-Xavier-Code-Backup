#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <string>
#include <robotcar_map/map.h>

class MapManager
{
public:
  MapManager(std::string main_map_frame);

  ~MapManager();

  // 更新地图
  void updateMap(double robotcar_x, double robotcar_y, double robotcar_yaw);

  // 设置主地图尺寸并用其对各层子地图进行设置，当无静态地图时可以使用
  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y);
  
  // 添加子地图
  void addSubMap(boost::shared_ptr<Map> sub_map_ptr);
  
  // 设置足迹
  void setFootprint(const std::vector<geometry_msgs::Point>& footprint);
  
  // 发布更新后的地图
  void publishMap();
  
  // 返回主地图指针
  Map* getMap()
  {
    return &main_map_;
  }

  // 获取主地图所在坐标系
  std::string getMainMapFrame()
  {
    return main_map_frame_;
  }

private:
  Map main_map_;                                         // 主地图1（仅含障碍物）
  std::string main_map_frame_;                           // 主地图坐标系
  std::vector<boost::shared_ptr<Map>> sub_maps_ptr_;     // 指向各层子地图的指针
  std::vector<geometry_msgs::Point> footprint_;          // 存储车辆足迹

  ros::Publisher map_pub_;                               // 地图发布
  int map_seq_;                                          // 发布序列
};

#endif /* MAP_MANAGER_H */