#include <robotcar_map/map.h>
#include <robotcar_map/static_map.h>
#include <robotcar_map/semantic_map.h>
#include <robotcar_map/inflation_map.h>
#include <robotcar_map/map_manager.h>

#include <tf/transform_datatypes.h>

using namespace std;

void getPose(tf::Stamped<tf::Pose>& global_pose)
{
  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = "base_link";
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();

  tf::TransformListener tf_(ros::Duration(10));
  
  while (1)
  {
    try
    {
      tf_.transformPose("map", robot_pose, global_pose);
    }
    catch (tf::LookupException& ex)
    {
      continue;
    }
    catch (tf::ConnectivityException& ex)
    {
      continue;
    }
    catch (tf::ExtrapolationException& ex)
    {
      continue;
    }
    
    break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_manager");

  MapManager manager("map");

  vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point a, b, c, d;
  a.x = 1; a.y = 0.7;
  b.x = -1; b.y = 0.7;
  c.x = -1; c.y = -0.7;
  d.x = 1; d.y = -0.7;
  footprint.push_back(a);
  footprint.push_back(b);
  footprint.push_back(c);
  footprint.push_back(d);

  boost::shared_ptr<Map> static_map_ptr(new StaticMap("static_map"));
  boost::shared_ptr<Map> semantic_map_ptr(new SemanticMap("semantic_map"));
  // boost::shared_ptr<Map> inflation_map_ptr(new InflationMap("inflation_map"));

  manager.addSubMap(static_map_ptr);
  manager.addSubMap(semantic_map_ptr);
  // manager.addSubMap(inflation_map_ptr);

  manager.setFootprint(footprint);

  tf::TransformListener tf(ros::Duration(10));

  static_map_ptr->initialize(&manager, &tf);
  semantic_map_ptr->initialize(&manager, &tf);
  // inflation_map_ptr->initialize(&manager, &tf);

  static_map_ptr->onInitialize();
  semantic_map_ptr->onInitialize();
  // inflation_map_ptr->onInitialize();
  
  while(1)
  {
    tf::Stamped<tf::Pose> global_pose;
    getPose(global_pose);
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    double yaw = tf::getYaw(global_pose.getRotation());
    manager.updateMap(global_pose.getOrigin().x(),
                        global_pose.getOrigin().y(),
                        tf::getYaw(global_pose.getRotation()));
    manager.publishMap();
  }
}