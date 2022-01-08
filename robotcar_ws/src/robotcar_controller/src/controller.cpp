#include <robotcar_controller/controller.h>
using namespace std;

Controller::Controller()
{
  ros::NodeHandle nh;
  v_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  traj_sub_ = nh.subscribe("/motion_planner/trajectory", 1, &Controller::trajCallback, this);

  ros::NodeHandle controller_nh("controller");
  controller_nh.param("lookahead_dis_", lookahead_dis_, 1.0);

  // 开始监听轨迹话题
  thread spin_task(spinTopic);
  spin_task.detach();

  receive_traj_ = false;

  // 是否记录，以及时间间隔
  record_ = true;
  rec_dt_ = 0.05;

  // 记录前轮转角
  if (record_ == true)
  {
    // 获取当前时间
    struct tm *t;
    time_t tt;
    time(&tt);
    t = localtime(&tt);

    // 提取年月日
    int year, mon, day, h, m, s;
    year = t->tm_year + 1900;
    mon = t->tm_mon + 1;
    day = t->tm_mday;
    h = t->tm_hour;
    m = t->tm_min;
    s = t->tm_sec;

    // 格式对齐
    string year_s, mon_s, day_s, h_s, m_s, s_s;
    year_s = to_string(year);
    mon < 10 ? mon_s = "0" + to_string(mon) : mon_s = to_string(mon);
    day < 10 ? day_s = "0" + to_string(day) : day_s = to_string(day);
    h < 10 ? h_s = "0" + to_string(h) : h_s = to_string(h);
    m < 10 ? m_s = "0" + to_string(m) : m_s = to_string(m);
    s < 10 ? s_s = "0" + to_string(s) : s_s = to_string(s);

    string file_name = year_s + mon_s + day_s + "_" + h_s + m_s + s_s + ".txt";

    of_.open("~/robotcar_ws/src/robotcar_controller/record/" + file_name, fstream::out);

    last_t_ = clock();
  }
}

Controller::~Controller()
{
  of_.close();
}

void Controller::trajCallback(const trajectory_msgs::JointTrajectory &msg)
{
  mu_.lock();
  traj_w_ = msg;
  receive_traj_ = true;
  mu_.unlock();
}

void Controller::spinTopic()
{
  ros::spin();
}

void Controller::ctrlRobotCar()
{
  // 若还未收到轨迹，则直接返回
  if (receive_traj_ == false)
    return;

  // 轨迹更新和使用时加锁
  mu_.lock();

  // 将轨迹转换到车体坐标系，得到车体系下轨迹
  vector<geometry_msgs::Point> traj_b;
  if (traj_w_.header.frame_id != "base_link")
  {
    tf_.waitForTransform("base_link", traj_w_.header.frame_id, ros::Time(0), ros::Duration(1));
    for (int i = 0; i < traj_w_.points.size(); i++)
    {
      tf::Stamped<tf::Vector3> point(tf::Vector3(traj_w_.points[i].positions[0], traj_w_.points[i].positions[1], 0), ros::Time(0), traj_w_.header.frame_id);
      tf::Stamped<tf::Vector3> point_b;
      tf_.transformPoint("base_link", point, point_b);
      geometry_msgs::Point traj_point;
      traj_point.x = point_b.getX();
      traj_point.y = point_b.getY();
      traj_b.push_back(traj_point);
    }
  }

  mu_.unlock();

  // 设置前视距离
  double lookahead_dis_ = 6.0;

  // 计算瞬时目标点
  double x, y, dis;
  bool get_target_ = false;
  for (int i = 0; i < traj_b.size(); i++)
  {
    if (traj_b[i].x < 0)
      continue;

    dis = sqrt(traj_b[i].x * traj_b[i].x + traj_b[i].y * traj_b[i].y);

    if (dis > lookahead_dis_)
    {
      x = traj_b[i].x;
      y = traj_b[i].y;
      get_target_ = true;
      break;
    }
  }

  double kappa;

  // 若轨迹上找到可行目标点
  if (get_target_ == true)
  {
    // 用瞬时目标点生成目标曲率
    double l_sq = x * x + y * y;
    kappa = 3.0 * y / l_sq;

    // 设置速度指令
    v_.linear.x = 0.7;
    v_.angular.z = atan(kappa * 1.02) / 3.1415926 * 180; //v_.linear.x * kappa;
    // v_.angular.z = min(max(v_.angular.z, -10.0), 10.0);
  }
  // 若轨迹上无可行目标点，或规划失败发布了空轨迹，制停
  else
  {
    v_.linear.x = 0;
    v_.angular.z = 0;
  }

  // 下发速度指令
  v_pub_.publish(v_);

  // 打印车辆前轮转角
  printConfig();
  ROS_INFO("车辆前轮转角为：%f\n", atan(1.02 * kappa) / 3.14 * 180);

  if (record_ == true && double(clock() - last_t_) / CLOCKS_PER_SEC >= rec_dt_)
  {
    last_t_ = clock();
    of_ << atan(1.02 * kappa) / 3.14 * 180 << endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  Controller ctrler;

  while (1)
  {
    ctrler.ctrlRobotCar();
    usleep(5e4);
  }

  return 1;
}