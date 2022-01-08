#ifndef GLOBAL_MAP_H
#define GLOBAL_MAP_H

#include <math.h>
#include <string>
#include <vector>
#include <pqxx/pqxx>

using namespace pqxx;
using namespace std;

class Point2d
{
public:
    double x;
    double y;

    Point2d(double x1 = 0.0, double y1 = 0.0):x(x1), y(y1) {}
    ~Point2d(){}
};

class GlobalMap
{
public:
  GlobalMap(string StrConnInfo);

  ~GlobalMap();

  //连接数据
  bool Connectiondb();

  //断开数据库
  void DisDBConnection();

  // 初始化
  void init();

  // 初始化指针：-1
  void initPtr();

  // 初始化状态：0
  void initState();

  // 初始化开销：1e8
  void initCost();

  // 通过点号查询某顶点的后向指针
  int getPtr(int id);

  // 通过点号修改某顶点的后向指针
  void updatePtr(int id, int value);

  // 通过点号查询某顶点的状态
  int getState(int id);

  // 通过点号修改某顶点的状态
  void updateState(int id, int value);

  // 通过点号查询某顶点的开销
  float getCost(int id);

  // 通过点号修改某顶点的开销
  void updateCost(int id, float value);

  // 通过点号查询某顶点的G开销
  float getCostG(int id);

  // 通过点号修改某顶点的G开销
  void updateCostG(int id, float value);

  // 通过点号查询某顶点的H开销
  float getCostH(int id);

  // 通过点号修改某顶点的H开销
  void updateCostH(int id, float value);

  // 获取开销最小的顶点的点号
  int getMinCostVertex();

  // 找到距离起点最近的顶点
  vector<int> findStartVertex(double x,double y);
  
  // 找到距离目标点最近的顶点
  vector<int> findGoalVertex(double x, double y);

  // 通过点号查询邻居点
  vector<int> findNeighbor(int id);
  
  // 通过点号查询某两点之间的边长
  double getLength(int id_s, int id_t);

  // 通过点号计算某两点之间的距离
  double getDistance(int id_s, int id_t);

  //根据顶点id查询顶点坐标
  Point2d getCoord(int id);

public:
  connection *m_pConn;
  
  string m_StrConnInfo;
};

#endif /* GLOBAL_MAP_H */