#ifndef INFLATION_MAP_H
#define INFLATION_MAP_H

#include <robotcar_map/map.h>

class InflationMap : public Map
{
public:
  InflationMap(std::string name);

  ~InflationMap();
  
  // 获取参数及主地图数据
  void onInitialize();

  // 获取地图更新范围，确保膨胀操作不会超出范围
  void updateBounds(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y,
                    double* max_x, double* max_y);

  // 更新主地图
  void updateData(Map& main_map, int min_i, int min_j, int max_i, int max_j);
  
  // 设置膨胀参考表
  void setUpReferenceTable();

private:
  double inflation_radius_;           // 膨胀半径
  double inflation_bound_;            // 膨胀边界
  double inflation_decay_;            // 衰减参数
  unsigned char** reference_table_;   // 膨胀参考表
};

#endif /* INFLATION_MAP_H */