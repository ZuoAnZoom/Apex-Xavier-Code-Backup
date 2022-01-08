#include <robotcar_map/map.h>
#include <robotcar_map/map_manager.h>

#include <cstdio>
#include <iostream>

using namespace std;

Map::Map():data_(NULL), default_value_(FREE), tf_(NULL)
{
  mutex_ = new boost::recursive_mutex();
}

Map::Map(string name, unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y, unsigned char default_value):
  name_(name), size_x_(size_x), size_y_(size_y), resolution_(resolution), origin_x_(origin_x), 
  origin_y_(origin_y), data_(NULL), default_value_(default_value), tf_(NULL)
{
  // 构造空地图，数据为默认值
  mutex_ = new boost::recursive_mutex();

  initMap(size_x_, size_y_);
  resetMap();
}

Map& Map::operator=(const Map& map)
{
  lockMap();
  deleteMap();

  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  initMap(size_x_, size_y_);
  memcpy(data_, map.data_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

Map::Map(const Map& map)
{
  lockMap();
  *this = map;
}

Map::~Map()
{
  deleteMap();
  delete mutex_;
}

void Map::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y)
{
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;        

  initMap(size_x, size_y);
  resetMap();
}

void Map::match()
{
  Map* master = map_manager_ptr_->getMap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

unsigned char Map::getValue(unsigned int mx, unsigned int my) const
{
  return data_[cell2Index(mx, my)];
}

void Map::setValue(unsigned int mx, unsigned int my, unsigned char value)
{
  lockMap();
  data_[cell2Index(mx, my)] = value;
}

void Map::map2World(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}


bool Map::world2Map(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_x_ || wy < origin_y_ ||
      wx > origin_x_+getSizeInMetersX()|| wy > origin_y_+getSizeInMetersY())
    return false;
  
  mx = int((wx - origin_x_) / resolution_);
  my = int((wy - origin_y_) / resolution_);
  return true;
}

void Map::world2MapWithoutBound(double wx, double wy, int& mx, int& my) const
{
  mx = int((wx - origin_x_) / resolution_);
  my = int((wy - origin_y_) / resolution_);
}

void Map::world2MapWithBound(double wx, double wy, int& mx, int& my) const
{
  if (wx < origin_x_)
    mx = 0;
  else if (wx > origin_x_ + getSizeInMetersX())
    mx = size_x_ - 1;
  else
    mx = int((wx - origin_x_) / resolution_);

  if (wy < origin_y_)
    my = 0;
  else if (wy > origin_y_ + getSizeInMetersY())
    my = size_y_ - 1;
  else
    my = int((wy - origin_y_) / resolution_);
}

unsigned char* Map::getData() const
{
  return data_;
}

unsigned int Map::getSizeInCellsX() const
{
  return size_x_;
}

unsigned int Map::getSizeInCellsY() const
{
  return size_y_;
}

double Map::getSizeInMetersX() const
{
  return (size_x_ - 0.5) * resolution_;
}

double Map::getSizeInMetersY() const
{
  return (size_y_ - 0.5) * resolution_;
}

double Map::getOriginX() const
{
  return origin_x_;
}

double Map::getOriginY() const
{
  return origin_y_;
}

double Map::getResolution() const
{
  return resolution_;
}

boost::recursive_mutex* Map::getMutex() const
{
  return mutex_;
}

void Map::initMap(unsigned int size_x, unsigned int size_y)
{
  lockMap();
  delete[] data_;
  data_ = new unsigned char[size_x * size_y];
}

void Map::resetMap()
{
  lockMap();
  memset(data_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void Map::deleteMap()
{
  lockMap();
  delete[] data_;
  data_ = NULL;
}

bool Map::getLine(int mx_1, int my_1, int mx_2, int my_2, vector<MapCell>& line)
{
  // Bresenham算法
  int x1_copy = mx_1;
  int y1_copy = my_1;
  int x2_copy = mx_2;
  int y2_copy = my_2;

  if (mx_1 > mx_2)
  {
    swap(mx_1, mx_2);
    swap(my_1, my_2);
  }

  bool k_negative = false;
  bool k_bigger_than_1 = false;

  if (my_1 > my_2)
  {
    k_negative = true;
    my_2 = my_1 + (my_1 - my_2);
  }

  if ((my_2 - my_1) > (mx_2 - mx_1))
  {
    k_bigger_than_1 = true;
    unsigned int mx_2_ = mx_2;
    mx_2 = mx_1 + (my_2 - my_1);
    my_2 = my_1 + (mx_2_ - mx_1);
  }
  
  double k = double(my_2-my_1+1)/(mx_2-mx_1+1);
  double b = my_2 - k*mx_2;

  int temp_x = mx_1, temp_y = my_1;

  MapCell first;
  for (int i = 1; i<=(mx_2-mx_1+1); i++)
  {
    double d = 2*(k*temp_x+b) - (temp_y+1) - temp_y;

    if (d > 0)
      temp_y++;

    MapCell temp_process(temp_x, temp_y);

    if (k_bigger_than_1)
    {
      unsigned int temp_ = temp_process.y;
      temp_process.y = my_1 + (temp_process.x - mx_1);
      temp_process.x = mx_1 + (temp_ - my_1);
    }

    if (k_negative)
    {
      temp_process.y = my_1 - (temp_process.y - my_1);
    }

    if (temp_process.x >= 0 && temp_process.x < size_x_ && temp_process.y >= 0 && temp_process.y < size_y_)
    {
      line.push_back(temp_process);
    }

    if (i == 1)
      first = temp_process;

    temp_x++;
  }

  if (first.x != x1_copy || first.y != y1_copy)
  {
    vector<MapCell> line_reverse;
    for (int i = line.size() - 1; i >= 0; i--)
    {
      line_reverse.push_back(line[i]);
    }
    line = line_reverse;
  }
  
  return true;
} 

// 注意按顺序给出顶点
void Map::getPolygon(const std::vector<MapCell> vertex, std::vector<MapCell>& cells)
{
  vector<MapCell> edge, edges;

  for (int i = 0; i < vertex.size(); i++)
  {
    if (i != (vertex.size()-1))
      getLine(vertex[i].x, vertex[i].y, vertex[i+1].x, vertex[i+1].y, edge);
    else
      getLine(vertex[i].x, vertex[i].y, vertex[0].x, vertex[0].y, edge);
    
    for (int j = 0; j < edge.size(); j++)
      edges.push_back(edge[j]);

    edge.clear();
  }

  unsigned int i = 0;
  while (i < edges.size() - 1)
  {
    if (edges[i].x > edges[i + 1].x)
    {
      swap(edges[i], edges[i + 1]);
      if (i > 0)
        --i;
    }
    else
      ++i;
  }
  
  for (unsigned int i = edges[0].x, j = 0; i <= edges[edges.size()-1].x; i++)
  {
    unsigned int y_max = edges[j].y;
    unsigned int y_min = edges[j].y;

    while (edges[j].x == i)
    {
      y_max = max(edges[j].y, y_max);
      y_min = min(edges[j].y, y_min);
      j++;
    }

    for (int k = y_min; k <= y_max; k++)
    {
      MapCell temp;
      temp.x = i;
      temp.y = k;
      cells.push_back(temp);
    }
  }
}

bool Map::setPolygonValue(const std::vector<geometry_msgs::Point>& vertex, unsigned char value)
{
  // 将车辆轮廓点从世界坐标系转换到地图坐标系
  vector<MapCell> vertex_map;
  for (int i = 0; i < vertex.size(); i++ )
  {
    MapCell p;

    if (!world2Map(vertex[i].x, vertex[i].y, p.x, p.y))
    {
      return false;
    }

    vertex_map.push_back(p);
  }
  
  if (setPolygonValue(vertex_map, value) == false)
  {
    return false;
  }

  return true;
}

bool Map::setPolygonValue(const std::vector<MapCell>& vertex, unsigned char value)
{ 
  for (int i = 0; i < vertex.size(); i++)
  {
    if (vertex[i].x >= size_x_ || vertex[i].y >= size_y_)
    {
      return false;
    }
  }

  vector<MapCell> polygon_cells;
  getPolygon(vertex, polygon_cells);

  for (int i = 0; i < polygon_cells.size(); i++)
  {
    int index = cell2Index(polygon_cells[i].x, polygon_cells[i].y);
    data_[index] = value;    
  }

  return true;
}

void Map::setFootprint(const std::vector<geometry_msgs::Point>& footprint)
{
  footprint_ = footprint;
}

void Map::initialize(MapManager* map_manager_ptr, tf::TransformListener *tf)
{
  tf_ = tf;
  map_manager_ptr_ = map_manager_ptr;
  onInitialize();
}

void Map::updateWithKnownData(Map& main_map, int min_i, int min_j, int max_i, int max_j)
{
  unsigned char* main_map_data = main_map.getData();
  unsigned int span = main_map.getSizeInCellsX(); // 一行

  for (int j = min_j; j <= max_j; j++)
  {
    unsigned int x = span * j + min_i; 
    for (int i = min_i; i <= max_i; i++)
    {
      if (data_[x] != NO_INFO)
      {
        main_map_data[x] = data_[x];
      }
      x++;
    }
  }
}

void Map::updateWithAllData(Map& main_map, int min_i, int min_j, int max_i, int max_j)
{
  unsigned char* main_map_data = main_map.getData();
  unsigned int span = main_map.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int x = span * j + min_i; 
    for (int i = min_i; i < max_i; i++)
    {
      main_map_data[x] = data_[x];
      x++;
    }
  }
}

void Map::updateWithMaxKnownData(Map& main_map, int min_i, int min_j, int max_i, int max_j)
{
  unsigned char* main_map_data = main_map.getData();
  unsigned int span = main_map.getSizeInCellsX();

  for (int j = min_j; j <= max_j; j++)
  {
    unsigned int x = span * j + min_i; 
    for (int i = min_i; i <= max_i; i++)
    {
      if (data_[x] != NO_INFO && (main_map_data[x] == NO_INFO || main_map_data[x] < data_[x]))
      {
        main_map_data[x] = data_[x];
      }
      x++;
    }
  }
}

// int main()
// {
//   // Map test(100, 100, 1, 0, 0, 0);

//   // vector<MapCell> vertex;
//   // MapCell a,b,c,d,e;
//   // a.x = 90;
//   // a.y = 90;
//   // b.x = 80;
//   // b.y = 20;
//   // c.x = 10;
//   // c.y = 30;
//   // d.x = 30;
//   // d.y = 75;
//   // vertex.push_back(a);
//   // vertex.push_back(b);
//   // vertex.push_back(c);
//   // vertex.push_back(d);

//   // vector<MapCell> cells;

//   // test.getPolygon(vertex,cells);

//   // test.getLine(0,20,10,0,cells);
// } 