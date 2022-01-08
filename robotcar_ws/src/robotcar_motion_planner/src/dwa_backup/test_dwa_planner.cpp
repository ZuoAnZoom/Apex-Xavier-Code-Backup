#include <robotcar_motion_planner/dwa/dwa_motion_planner.h>
#include <robotcar_map/map.h>
#include <robotcar_map/static_map.h>
#include <robotcar_map/semantic_map.h>
#include <robotcar_map/inflation_map.h>
#include <robotcar_map/map_manager.h>
#include <robotcar_path_planner/dijkstra_path_planner.h>
#include <robotcar_path_planner/astar_path_planner.h>

#include <thread>
#include <functional>
#include <nav_msgs/Odometry.h>

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

void Candidates(PathPlanner* planner)
{
  while(1)
  {
    planner->publishCandidates();
  }
}

nav_msgs::Odometry base_odom_;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  base_odom_.child_frame_id = msg->child_frame_id;
}

void spin()
{
  while (1) 
    ros::spinOnce();
}

void publishPath(PathPlanner* planner)
{
  while (1)
    planner->publishPath();
}

void dispTraj(DWAMotionPlanner* planner)
{
  while(1)
    planner->displayTraj();
}


void updateMap(MapManager* manager, double& x, double& y, double& yaw, tf::Stamped<tf::Pose>& global_pose)
{
  while (1)
  {
    getPose(global_pose);
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();
    yaw = tf::getYaw(global_pose.getRotation());
    manager->updateMap(global_pose.getOrigin().x(),
                        global_pose.getOrigin().y(),
                        tf::getYaw(global_pose.getRotation()));
    manager->publishMap();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_path_planner");

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
  boost::shared_ptr<Map> obstacle_map_ptr(new SemanticMap("obstacle_map"));
  boost::shared_ptr<Map> inflation_map_ptr(new InflationMap("inflation_map"));

  manager.addSubMap(static_map_ptr);
  manager.addSubMap(obstacle_map_ptr);
  manager.addSubMap(inflation_map_ptr);

  manager.setFootprint(footprint);

  tf::TransformListener tf(ros::Duration(10));

  static_map_ptr->initialize(&manager, &tf);
  obstacle_map_ptr->initialize(&manager, &tf);
  inflation_map_ptr->initialize(&manager, &tf);

  static_map_ptr->onInitialize();
  obstacle_map_ptr->onInitialize();
  inflation_map_ptr->onInitialize();
  

  tf::Stamped<tf::Pose> global_pose;
  double x, y, yaw;
  thread updateMapTask(bind(updateMap, &manager, x, y, yaw, global_pose));
  updateMapTask.detach();


  ros::NodeHandle nh("path_planner");

  // 从参数服务器提取目标
  int goal_x_, goal_y_;
  nh.param("goal_x", goal_x_, 350);
  nh.param("goal_y", goal_y_, 350);

  // 从参数服务器提取方法
  int method_;
  nh.param("method", method_, 1);

  PathPlanner* planner;

  switch (method_)
  {
    case 1:
      planner = new DijkstraPathPlanner();
      break;

    case 2:
      planner = new AstarPathPlanner();
      break;

    // case 3:
    //   planner = new RRTPathPlanner();
    //   break;

    // case 4:
    //   planner = new PFMPathPlanner();
    //   break;
    
    default:
      break;
  }

  ros::Rate r(0.5);
  r.sleep();
  // planner->setMap(*(manager.getMap()));
  planner->setMap(*static_map_ptr);


  getPose(global_pose);
  x = global_pose.getOrigin().x();
  y = global_pose.getOrigin().y();
  unsigned int s[2], g[2];
  planner->world2Map(x,y,s[0],s[1]);
  g[0] = goal_x_;
  g[1] = goal_y_;
  planner->setStart(s);
  planner->setGoal(g);

  planner->calcPath();

  std::thread pathTask(bind(publishPath, planner));
  pathTask.detach();



  ros::NodeHandle g_nh;
  ros::Subscriber odom_sub_ = g_nh.subscribe("odom", 1, odomCallback);
  ros::Publisher vel_pub_ = g_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  std::thread spinTask(spin);
  spinTask.detach();

  DWAMotionPlanner mplanner(1, manager.getMap(), footprint);
  mplanner.setPath(planner->getPath());  

  tf::Stamped<tf::Pose> vel, vel_cmd;
  while (1)
  {
    geometry_msgs::Twist global_vel;
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;
    vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
    vel.frame_id_ = base_odom_.child_frame_id;
    vel.stamp_ = ros::Time();

    getPose(global_pose);
    mplanner.getVelCmd(global_pose, vel, vel_cmd);

    geometry_msgs::Twist v;
    v.linear.x = vel_cmd.getOrigin().getX();
    v.angular.z = tf::getYaw(vel_cmd.getRotation());

    vel_pub_.publish(v);

  }

  return 1;
}