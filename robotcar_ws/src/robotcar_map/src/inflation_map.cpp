#include <robotcar_map/inflation_map.h>
#include <robotcar_map/map_manager.h>

#include <math.h>

using namespace std;

InflationMap::InflationMap(string name)
{
  name_ = name;
}

InflationMap::~InflationMap()
{
  delete reference_table_;
}

void InflationMap::onInitialize()
{
  ros::NodeHandle nh(name_);

  nh.param("inflation_radius", inflation_radius_, 1.5);
  nh.param("inflation_bound", inflation_bound_, 20.0);
  nh.param("inflation_decay", inflation_decay_, 0.5);

  // 同步主地图尺寸
  match();

  // 设置膨胀参考表
  setUpReferenceTable();
}

void InflationMap::setUpReferenceTable()
{
  double radius = inflation_radius_ * resolution_;
  int bound = int(inflation_bound_ * resolution_) + 1;
  
  // 膨胀参考表记录原点周围1/4圆范围内的地图数据
  reference_table_ = new unsigned char*[bound + 1];

  for (int i = 0; i < bound + 1; i++)
  {
    reference_table_[i] = new unsigned char[bound + 1];
    
    for (int j = 0; j < bound + 1; j++)
    {
      if (i == 0 && j == 0)
      {
        reference_table_[i][j] = OBSTACLE;
      }

      else if (hypot(i, j) <= radius)
      {
        reference_table_[i][j] = INFLATED_OBSTACLE;
      }

      else if (hypot(i, j) <= bound)
      {
        double value = max(INFLATED_OBSTACLE * exp((-1.0) * inflation_decay_ * (hypot(i, j) - radius)), double(INFLATED_OBSTACLE - INFLATED_RANGE));
        reference_table_[i][j] = (unsigned char)value;
      }

      else
      {
        reference_table_[i][j] = 0;
      }
    }
  }
}

void InflationMap::updateBounds(double robotcar_x, double robotcar_y, double robotcar_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
  // 将范围也进行“膨胀”，并确保膨胀后的范围不会超出地图大小
  *min_x = max((*min_x - inflation_bound_), origin_x_);
  *min_y = max((*min_y - inflation_bound_), origin_y_);
  *max_x = min((*max_x + inflation_bound_), origin_x_ + size_x_ * resolution_);
  *max_y = min((*max_y + inflation_bound_), origin_y_ + size_y_ * resolution_);
}

void InflationMap::updateData(Map& main_map, int min_i, int min_j, int max_i, int max_j)
{
  data_ = main_map.getData();

  // 提取当前地图上的所有障碍点
  vector<MapCell> obstacle_cells;

  for (unsigned int i = min_i; i <= max_i; i++)
  {
    for (unsigned int j = min_j; j <= max_j; j++)
    {
      if (getValue(i, j) == OBSTACLE)
      {
        if (i != 0 && i != size_x_ && j != 0 && j != size_y_)
        {
          if (data_[cell2Index(i - 1, j)] == OBSTACLE &&
              data_[cell2Index(i + 1, j)] == OBSTACLE &&
              data_[cell2Index(i, j - 1)] == OBSTACLE &&
              data_[cell2Index(i, j + 1)] == OBSTACLE)
          {
            continue;
          }
        }

        MapCell point{i, j};
        obstacle_cells.push_back(point);
      }
    }
  }
  
  int bound = int(inflation_bound_ * resolution_) + 1;

  // 借助膨胀参考表，对每个障碍点进行膨胀操作
  for (int n = 0; n < obstacle_cells.size(); n++)
  {
    for (int i = (-1)*bound; i <= bound; i++)
    {
      for (int j = (-1)*bound; j <= bound; j++)
      {
        if (obstacle_cells[n].x + i < 0 || obstacle_cells[n].x + i > size_x_ -1 ||
            obstacle_cells[n].y + j < 0 || obstacle_cells[n].y + j > size_y_ -1)
        {
          continue;
        }
        
        unsigned int index = cell2Index(obstacle_cells[n].x + i, obstacle_cells[n].y + j);

        if (data_[index] < reference_table_[abs(i)][abs(j)] && (data_[index] < LANE || data_[index] > LANE + LANE_RANGE))
        {
          data_[index] = reference_table_[abs(i)][abs(j)];
        } 
      }
    }
  }

}