#include <iostream>
#include <robotcar_map/global_map.h>

using namespace pqxx;
using namespace std;

GlobalMap::GlobalMap(string StrConnInfo)
{
  m_StrConnInfo = StrConnInfo;
}

GlobalMap::~GlobalMap() {}

bool GlobalMap::Connectiondb()
{
  m_pConn = new connection(m_StrConnInfo);
  bool bret;
  if (m_pConn == NULL)
  {
    cout << "db connection ptr is null " << endl;
  }

  if (m_pConn->is_open())
  {
    cout << "Opened database successfully: " << m_pConn->dbname() << endl;
    bret = true;
  }
  else
  {
    cout << "Can't open database" << endl;
    bret = false;
  }
  return bret;
}

void GlobalMap::DisDBConnection()
{
  if (m_pConn == NULL)
  {
    cout << "db connection ptr is null" << endl;
    return;
  }
  m_pConn->disconnect();
  if (m_pConn != NULL)
  {
    delete m_pConn;
    m_pConn = NULL;
  }
}

void GlobalMap::init()
{
  initPtr();
  initState();
  initCost();
}

void GlobalMap::initPtr()
{
  work worker(*m_pConn);
  string sql_ptr;
  sql_ptr = "update vertices_to_mercator set back_ptr = -1 ;";
  worker.exec(sql_ptr);
  worker.commit();
}

void GlobalMap::initState()
{
  work worker(*m_pConn);
  string sql_state;
  sql_state = "update vertices_to_mercator set state = 0 ;";
  worker.exec(sql_state);
  worker.commit();
}

void GlobalMap::initCost()
{
  work worker(*m_pConn);
  string sql_cost;
  sql_cost = "update vertices_to_mercator set cost = 1e8";
  worker.exec(sql_cost);
  sql_cost = "update vertices_to_mercator set cost_g = 1e8";
  worker.exec(sql_cost);
  sql_cost = "update vertices_to_mercator set cost_h = 1e8";
  worker.exec(sql_cost);
  worker.commit();
}

int GlobalMap::getPtr(int id)
{
  nontransaction nts(*m_pConn);
  string sql;
  sql = "select back_ptr from vertices_to_mercator where id = " + to_string(id) + ";";
  result res = nts.exec(sql);
  int ptr = res[0][0].as<int>();
  return ptr;
}

void GlobalMap::updatePtr(int id, int value)
{
  work worker(*m_pConn);
  string sql;
  sql = "update vertices_to_mercator set back_ptr = " + to_string(value) + "where id = " + to_string(id) + ";";
  worker.exec(sql);
  worker.commit();
}

int GlobalMap::getState(int id)
{
  nontransaction nts(*m_pConn);
  string sql;
  sql = "select state from vertices_to_mercator where id = " + to_string(id) + ";";
  result res = nts.exec(sql);
  int state = res[0][0].as<int>();
  return state;
}

void GlobalMap::updateState(int id, int value)
{
  work worker(*m_pConn);
  string sql;
  sql = "update vertices_to_mercator set state = " + to_string(value) + "where id = " + to_string(id) + ";";
  worker.exec(sql);
  worker.commit();
}

float GlobalMap::getCost(int id)
{
  nontransaction nts(*m_pConn);
  string sql;
  sql = "select cost from vertices_to_mercator where id = " + to_string(id) + ";";
  result res = nts.exec(sql);
  float cost = res[0][0].as<float>();
  return cost;
}

void GlobalMap::updateCost(int id, float value)
{
  work worker(*m_pConn);
  string sql;
  sql = "update vertices_to_mercator set cost = " + to_string(value) + "where id = " + to_string(id) + ";";
  worker.exec(sql);
  worker.commit();
}

float GlobalMap::getCostG(int id)
{
  nontransaction nts(*m_pConn);
  string sql;
  sql = "select cost_g from vertices_to_mercator where id = " + to_string(id) + ";";
  result res = nts.exec(sql);
  float cost = res[0][0].as<float>();
  return cost;
}

void GlobalMap::updateCostG(int id, float value)
{
  work worker(*m_pConn);
  string sql;
  sql = "update vertices_to_mercator set cost_g = " + to_string(value) + "where id = " + to_string(id) + ";";
  worker.exec(sql);
  worker.commit();
}

float GlobalMap::getCostH(int id)
{
  nontransaction nts(*m_pConn);
  string sql;
  sql = "select cost_h from vertices_to_mercator where id = " + to_string(id) + ";";
  result res = nts.exec(sql);
  float cost = res[0][0].as<float>();
  return cost;
}

void GlobalMap::updateCostH(int id, float value)
{
  work worker(*m_pConn);
  string sql;
  sql = "update vertices_to_mercator set cost_h = " + to_string(value) + "where id = " + to_string(id) + ";";
  worker.exec(sql);
  worker.commit();
}

int GlobalMap::getMinCostVertex()
{
  nontransaction nts(*m_pConn);
  string sql;
  sql = "select id from vertices_to_mercator where state = 1 order by cost asc limit 1;";
  
  result res = nts.exec(sql);
  if (res.empty() == true)
  {
    return -1;
  }

  return res[0][0].as<int>();
}

double GlobalMap::getLength(int id_s, int id_t)
{
  nontransaction nts(*m_pConn);
  string sql;
  sql = "with ctep1 as (select geom as p1 from vertices_to_mercator where id =" + to_string(id_s) + "), ctep2 as (select geom as p2 from vertices_to_mercator where id = " + to_string(id_t) + ")select st_distance(p1,p2) from ctep1 inner join ctep2 on 1=1;";

  result res = nts.exec(sql);
  double distance = res[0][0].as<double>();
  return distance;
}

double GlobalMap::getDistance(int id_s, int id_t)
{
  Point2d point_s = getCoord(id_s);
  Point2d point_t = getCoord(id_t);

  return sqrt(pow((point_s.x - point_t.x), 2) + pow((point_s.y - point_t.y), 2));
}

Point2d GlobalMap::getCoord(int id)
{
  nontransaction nts(*m_pConn);
  string sql;
  Point2d coord;
  sql = "select x,y from vertices_to_mercator where id = " + to_string(id) + ";";
  result res = nts.exec(sql);
  coord.x = res[0][0].as<double>();
  coord.y = res[0][1].as<double>();
  return coord;
}

vector<int> GlobalMap::findStartVertex(double x, double y)
{
  vector<int> nid;
  nontransaction nts(*m_pConn);
  string str1, str2, str3, sql;
  str1 = "with ctepoint as (select 'SRID=900913;POINT( ";
  str2 = to_string(x) + " " + to_string(y);
  str3 = ")'::geometry as point "
       "),ctebuff as ( "
       "select st_buffer((select point from ctepoint),50.000000) as buff "
       "), cteroads as ( "
       "select ST_Distance((select point from ctepoint),geom) as dist, "
       "gid,source, target,one_way from ways_to_mercator "
       "where ST_Intersects(geom,(select buff from ctebuff)) "
       "), ctecand as ( "
       "select one_way,source,target from cteroads order by dist limit 1 "
       ")select * from ctecand;";
  sql = str1 + str2 + str3;
  result res = nts.exec(sql);
  int nrecord = res.size();
  int oneway, source, target;
  oneway = res[0][0].as<int>();
  source = res[0][1].as<int>();
  target = res[0][2].as<int>();
  nid.push_back(target);
  if (oneway != 1)
  {
    nid.push_back(source);
  }
  return nid;
}

vector<int> GlobalMap::findGoalVertex(double x, double y)
{
  vector<int> nid;
  nontransaction nts(*m_pConn);
  string str1, str2, str3, sql;
  str1 = "with ctepoint as (select 'SRID=900913;POINT( ";
  str2 = to_string(x) + " " + to_string(y);
  str3 = ")'::geometry as point "
       "),ctebuff as ( "
       "select st_buffer((select point from ctepoint),10.000000) as buff "
       "), cteroads as ( "
       "select ST_Distance((select point from ctepoint),geom) as dist, "
       "gid,source, target,one_way from ways_to_mercator "
       "where ST_Intersects(geom,(select buff from ctebuff)) "
       "), ctecand as ( "
       "select one_way,source,target from cteroads order by dist limit 1 "
       ")select * from ctecand;";
  sql = str1 + str2 + str3;
  result res = nts.exec(sql);
  int oneway, source, target;
  oneway = res[0][0].as<int>();
  source = res[0][1].as<int>();
  target = res[0][2].as<int>();
  nid.push_back(source);
  if (oneway != 1)
  {
    nid.push_back(target);
  }
  return nid;
}

vector<int> GlobalMap::findNeighbor(int id)
{
  vector<int> nid;
  nontransaction nts(*m_pConn);
  string str1, str2, str3, sql;
  str1 = "with cteid as ( select ";
  str2 = to_string(id);
  str3 = "as id), ctevertice as ( "
       "select one_way,source,target from ways_to_mercator wm "
       "where (select id from cteid) in (wm.source, wm.target) "
       ")select * from ctevertice;";
  sql = str1 + str2 + str3;
  result res = nts.exec(sql);
  int nrecord = res.size();
  int oneway, source, target;
  for (int row = 0; row < nrecord; row++)
  {
    oneway = res[row][0].as<int>();
    source = res[row][1].as<int>();
    target = res[row][2].as<int>();
    if (oneway == 1)
    {
      if( target != id )
      {
        nid.push_back(target);
      }
    }
    else 
    {
      if (target == id)
      {
        nid.push_back(source);
      }
      if (source == id)
      {
        nid.push_back(target);
      }
    }
  }
  return nid;
}