#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>
#include <algorithm>
#include <boost/thread.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <robotcar_map/map_code.h>
#include <robotcar_general/print_helper.h>

struct MapCell
{
  MapCell()
  {
    x = 0;
    y = 0;
  }
  
  MapCell(unsigned int mx, unsigned int my)
  {
    x = mx;
    y = my;
  }

  unsigned int x, y;
};

class MapManager;

class Map
{
public:
  // =================== 地图数据 ===================

  // 默认构造函数
  Map();
  
  // 构造一张空地图
  Map(std::string name, unsigned int size_x, unsigned int size_y, double resolution, 
      double origin_x, double origin_y, unsigned char default_value = 0);

  // 复制地图
  Map(const Map& map);

  // 复制地图
  Map& operator=(const Map& map);

  // 析构函数
  ~Map();

  // 重设地图大小，并恢复地图数据为默认
  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                 double origin_y);

  // 根据主地图设置各层子地图尺寸等信息
  void match();

  // 获取地图某点值
  unsigned char getValue(unsigned int mx, unsigned int my) const;

  // 设置地图某点值
  void setValue(unsigned int mx, unsigned int my, unsigned char value);

  // 栅格/地图坐标转换
  void map2World(unsigned int mx, unsigned int my, double& wx, double& wy) const;

  bool world2Map(double wx, double wy, unsigned int& mx, unsigned int& my) const;

  void world2MapWithoutBound(double wx, double wy, int& mx, int& my) const;

  void world2MapWithBound(double wx, double wy, int& mx, int& my) const;

  // 索引/栅格坐标转换
  inline unsigned int cell2Index(unsigned int mx, unsigned int my) const { return my * size_x_ + mx; }

  inline void index2Cell(unsigned int index, unsigned int& mx, unsigned int& my) const 
              { my = index / size_x_; mx = index - (my * size_x_);}

  // 获取地图信息
  unsigned char* getData() const;

  unsigned int getSizeInCellsX() const;

  unsigned int getSizeInCellsY() const;

  double getSizeInMetersX() const;

  double getSizeInMetersY() const;

  double getOriginX() const;

  double getOriginY() const;

  double getResolution() const;

  boost::recursive_mutex* getMutex() const;
  
  // 获取/设置地图默认值
  inline unsigned char getDefaultValue() { return default_value_; }

  inline void setDefaultValue(unsigned char default_value) { default_value_ = default_value; }

  // 提取两点连线上的cell，地图边界以外的部分舍去
  bool getLine(int mx_1, int my_1, int mx_2, int my_2, std::vector<MapCell>& line);
  
  // 提取凸多边形内部cell
  void getPolygon(const std::vector<MapCell> vertex, std::vector<MapCell>& cells);
  
  // 设置凸多边形内部cell的值
  bool setPolygonValue(const std::vector<geometry_msgs::Point>& vertex, unsigned char value);

  // 设置凸多边形内部cell的值
  bool setPolygonValue(const std::vector<MapCell>& vertex, unsigned char value);

  // 设置足迹
  void setFootprint(const std::vector<geometry_msgs::Point>& footprint);
  
  
  // =================== 地图操作 ===================

  // 启动本层地图更新
  void initialize(MapManager* map_manager_ptr, tf::TransformListener *tf);

  // 各层地图真正的初始化函数
  virtual void onInitialize() {}

  // 获得地图更新范围，基于地图系
  virtual void updateBounds(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y) {}

  // 更新地图数据，基于栅格系
  virtual void updateData(Map& main_map, int min_i, int min_j, int max_i, int max_j) {}

  // 用本层地图已知数据更新主地图
  void updateWithKnownData(Map& main_map, int min_i, int min_j, int max_i, int max_j);
  
  // 用本层地图全部数据更新主地图
  void updateWithAllData(Map& main_map, int min_i, int min_j, int max_i, int max_j);

  // 用本层地图较主地图更大的数据更新主地图
  void updateWithMaxKnownData(Map& main_map, int min_i, int min_j, int max_i, int max_j);

protected:
  // 初始化地图参数，并将地图设置为默认值
  void initMap(unsigned int size_x, unsigned int size_y);

  // 将地图数据恢复默认值
  void resetMap();
  
  // 删除地图
  void deleteMap();

  // 存取地图时上锁
  inline void lockMap() { boost::unique_lock<boost::recursive_mutex> lock(*mutex_); }

protected:
  MapManager* map_manager_ptr_;                   // 指向主地图的指针
  std::vector<geometry_msgs::Point> footprint_;   // 由主地图传来的机器人足迹
  tf::TransformListener* tf_;                     // 坐标转换
  boost::recursive_mutex* mutex_;                 // 地图锁
  std::string name_;                              // 地图名称
  unsigned int size_x_;                           // 地图x向尺寸
  unsigned int size_y_;                           // 地图y向尺寸
  double resolution_;                             // 地图分辨率
  double origin_x_;                               // 地图原点x坐标
  double origin_y_;                               // 地图原点y坐标
  unsigned char* data_;                           // 地图数据，范围0-255
  unsigned char default_value_;                   // 地图数据默认值
};

#endif /* MAP_H */
