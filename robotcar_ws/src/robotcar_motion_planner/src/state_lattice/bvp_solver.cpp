#include <robotcar_motion_planner/state_lattice/bvp_solver.h>
#include <iostream>

using namespace Eigen;
using namespace std;

BVPSolver::BVPSolver()
{
  // 离线设置LUT
  Eigen::Matrix<float, 10, 5> lut1, lut2, lut3, lut4, lut5, lut6, lut7, lut8, lut9, lut10, lut11, lut12, lut13, lut14;

  lut1 <<   0.0000,   0.0000,   0.0000,   0.0000,   0.9500,   // y = 0.0
            0.0000,  19.8263, -38.6620,  16.7539,   1.5384,   // y = 1.0
            0.0000,   8.3970,  -8.9973,   2.1423,   2.7998,   // y = 2.0
            0.0000,   4.0212,  -2.8115,   0.4368,   4.2908,   // y = 3.0
            0.0000,   2.2739,  -1.1616,   0.1319,   5.8729,   // y = 4.0
            0.0000,   1.4428,  -0.5772,   0.0513,   7.4992,   // y = 5.0
            0.0000,   0.9910,  -0.3249,   0.0237,   9.1498,   // y = 6.0
            0.0000,   0.7205,  -0.1998,   0.0123,  10.8153,   // y = 7.0
            0.0000,   0.5465,  -0.1313,   0.0070,  12.4905,   // y = 8.0
            0.0000,   0.4283,  -0.0907,   0.0043,  14.1723;   // y = 9.0

  lut2 <<   0.0000,   0.0000,   0.0000,   0.0000,   1.9000,
            0.0000,   5.6099,  -7.5630,   2.2658,   2.2253,
            0.0000,   4.9566,  -4.8328,   1.0471,   3.0769,
            0.0000,   3.2453,  -2.2920,   0.3597,   4.2478,
            0.0000,   2.0992,  -1.1247,   0.1339,   5.5997,
            0.0000,   1.4183,  -0.6029,   0.0569,   7.0576,
            0.0000,   1.0053,  -0.3514,   0.0273,   8.5816,
            0.0000,   0.7430,  -0.2196,   0.0144,  10.1488,
            0.0000,   0.5685,  -0.1452,   0.0082,  11.7459,
            0.0000,   0.4475,  -0.1005,   0.0050,  13.3642;

  lut3 <<   0.0000,	  0.0000,	  0.0000,	  0.0000,	  2.8500,
            0.0000,	  2.0445,	 -1.9967,	  0.4333,	  3.0718,
            0.0000,	  2.5817,	 -2.0975,	  0.3787,	  3.6924,
            0.0000,	  2.2029,	 -1.4319,	  0.2068,	  4.6153,
            0.0000,   1.6735,	 -0.8734,	  0.1013,	  5.7483,
            0.0000,   1.2432,  -0.5309,   0.0504,   7.0245,
            0.0000,   0.9330,  -0.3332,   0.0264,   8.3995,
            0.0000,   0.7144,  -0.2177,   0.0147,   9.8442,
            0.0000,   0.5592,  -0.1479,   0.0087,  11.3395,
            0.0000,   0.4468,  -0.1041,   0.0054,  12.8723;

  lut4 <<   0.0000,   0.0000,   0.0000,   0.0000,   3.8000,
            0.0000,   0.9333,  -0.7056,   0.1186,   3.9678,
            0.0000,   1.4025,  -0.9454,   0.1416,   4.4505,
            0.0000,   1.4248,  -0.8223,   0.1055,   5.1984,
            0.0000,   1.2391,  -0.6041,   0.0654,   6.1537,
            0.0000,   1.0126,  -0.4181,   0.0384,   7.2660,
            0.0000,   0.8113,  -0.2865,   0.0225,   8.4957,
            0.0000,   0.6499,  -0.1987,   0.0135,   9.8138,
            0.0000,   0.5248,  -0.1406,   0.0084,  11.1993,
            0.0000,   0.4287,  -0.1018,   0.0054,  12.6369;

  lut5 <<   0.0000,   0.0000,   0.0000,   0.0000,   4.7500,
            0.0000,   0.4962,  -0.3048,   0.0416,   4.8847,
            0.0000,   0.8188,  -0.4654,   0.0588,   5.2780,
            0.0000,   0.9292,  -0.4724,   0.0534,   5.9014,
            0.0000,   0.8942,  -0.3993,   0.0396,   6.7183,
            0.0000,   0.7931,  -0.3093,   0.0268,   7.6922,
            0.0000,   0.6765,  -0.2308,   0.0175,   8.7913,
            0.0000,   0.5678,  -0.1705,   0.0114,   9.9897,
            0.0000,   0.4749,  -0.1264,   0.0075,  11.2671,
            0.0000,   0.3982,  -0.0947,   0.0050,  12.6076;

  lut6  <<  0.0000,   0.0000,   0.0000,   0.0000,   5.7000,
            0.0000,   0.2932,  -0.1513,   0.0174,   5.8125,
            0.0000,   0.5111,  -0.2496,   0.0271,   6.1436,
            0.0000,   0.6233,  -0.2801,   0.0280,   6.6758,
            0.0000,   0.6454,  -0.2622,   0.0237,   7.3849,
            0.0000,   0.6112,  -0.2224,   0.0180,   8.2447,
            0.0000,   0.5507,  -0.1790,   0.0129,   9.2306,
            0.0000,   0.4831,  -0.1404,   0.0091,  10.3207,
            0.0000,   0.4184,  -0.1092,   0.0063,  11.4967,
            0.0000,   0.3606,  -0.0849,   0.0044,  12.7435;

  lut7 <<   0.0000,   0.0000,   0.0000,   0.0000,   6.6500,
            0.0000,   0.1870,  -0.0832,   0.0082,   6.7466,
            0.0000,   0.3375,  -0.1440,   0.0136,   7.0322,
            0.0000,   0.4320,  -0.1729,   0.0154,   7.4953,
            0.0000,   0.4716,  -0.1743,   0.0143,   8.1195,
            0.0000,   0.4701,  -0.1587,   0.0119,   8.8857,
            0.0000,   0.4435,  -0.1361,   0.0093,   9.7748,
            0.0000,   0.4046,  -0.1127,   0.0070,  10.7690,
            0.0000,   0.3620,  -0.0916,   0.0052,  11.8525,
            0.0000,   0.3205,  -0.0739,   0.0038,  13.0116;

  lut8  <<  0.0000,   0.0000,   0.0000,   0.0000,   7.6000,
            0.0000,   0.1263,  -0.0493,   0.0043,   7.6846,
            0.0000,   0.2333,  -0.0882,   0.0074,   7.9355,
            0.0000,   0.3089,  -0.1110,   0.0089,   8.3449,
            0.0000,   0.3506,  -0.1182,   0.0089,   8.9011,
            0.0000,   0.3637,  -0.1138,   0.0079,   9.5899,
            0.0000,   0.3562,  -0.1028,   0.0066,  10.3968,
            0.0000,   0.3362,  -0.0892,   0.0053,  11.3071,
            0.0000,   0.3098,  -0.0755,   0.0041,  12.3075,
            0.0000,   0.2813,  -0.0630,   0.0031,  13.3859;

  lut9 <<   0.0000,   0.0000,   0.0000,   0.0000,   8.5500,
            0.0000,   0.0892,  -0.0310,   0.0024,   8.6252,
            0.0000,   0.1675,  -0.0568,   0.0043,   8.8489,
            0.0000,   0.2272,  -0.0740,   0.0053,   9.2155,
            0.0000,   0.2655,  -0.0820,   0.0056,   9.7162,
            0.0000,   0.2841,  -0.0824,   0.0053,  10.3407,
            0.0000,   0.2869,  -0.0777,   0.0047,  11.0773,
            0.0000,   0.2786,  -0.0701,   0.0039,  11.9144,
            0.0000,   0.2635,  -0.0616,   0.0032,  12.8408,
            0.0000,   0.2448,  -0.0530,   0.0026,  13.8459;

  lut10 <<  0.0000,	  0.0000,	  0.0000,	  0.0000,	  9.5014,
            0.0000,	  0.0653,	 -0.0205,	  0.0014,	  9.5677,
            0.0000,	  0.1241,	 -0.0381,	  0.0026,	  9.7695,
            0.0000,	  0.1713,	 -0.0509,	  0.0034,	 10.1011,
            0.0000,	  0.2047,	 -0.0582,	  0.0037,	 10.5561,
            0.0000,   0.2244,  -0.0605,   0.0036,  11.1263,
            0.0000,   0.2323,  -0.0590,   0.0033,  11.8028,
            0.0000,   0.2311,  -0.0551,   0.0029,  12.5760,
            0.0000,   0.2236,  -0.0499,   0.0025,  13.4365,
            0.0000,   0.2120,  -0.0442,   0.0021,  14.3754;

  lut11 <<  0.0000,   0.0000,   0.0000,   0.0000,  10.4500,
            0.0000,   0.0492,  -0.0140,   0.0009,  10.5116,
            0.0000,   0.0943,  -0.0265,   0.0016,  10.6953,
            0.0000,   0.1320,  -0.0360,   0.0022,  10.9979,
            0.0000,   0.1605,  -0.0422,   0.0025,  11.4145,
            0.0000,   0.1794,  -0.0451,   0.0025,  11.9387,
            0.0000,   0.1895,  -0.0453,   0.0024,  12.5634,
            0.0000,   0.1924,  -0.0435,   0.0022,  13.2806,
            0.0000,   0.1897,  -0.0404,   0.0019,  14.0827,
            0.0000,   0.1832,  -0.0367,   0.0016,  14.9618;

  lut12 <<  0.0000,   0.0000,   0.0000,   0.0000,  11.4000,
            0.0000,   0.0380,  -0.0100,   0.0006,  11.4565,
            0.0000,   0.0733,  -0.0189,   0.0011,  11.6251,
            0.0000,   0.1037,  -0.0261,   0.0015,  11.9033,
            0.0000,   0.1278,  -0.0312,   0.0017,  12.2873,
            0.0000,   0.1451,  -0.0341,   0.0018,  12.7720,
            0.0000,   0.1558,  -0.0350,   0.0017,  13.3516,
            0.0000,   0.1609,  -0.0344,   0.0016,  14.0197,
            0.0000,   0.1614,  -0.0328,   0.0015,  14.7698,
            0.0000,   0.1583,  -0.0305,   0.0013,  15.5951;

  lut13 <<  0.0000,   0.0000,   0.0000,   0.0000,  12.3500,
            0.0000,   0.0299,  -0.0072,   0.0004,  12.4021,
            0.0000,   0.0581,  -0.0139,   0.0007,  12.5579,
            0.0000,   0.0828,  -0.0194,   0.0010,  12.8153,
            0.0000,   0.1032,  -0.0235,   0.0012,  13.1713,
            0.0000,   0.1186,  -0.0261,   0.0013,  13.6218,
            0.0000,   0.1292,  -0.0274,   0.0013,  14.1621,
            0.0000,   0.1353,  -0.0275,   0.0012,  14.7868,
            0.0000,   0.1377,  -0.0267,   0.0011,  15.4904,
            0.0000,   0.1370,  -0.0253,   0.0010,  16.2673;

  lut14 <<  0.0000,   0.0000,   0.0000,   0.0000,  13.3000,
            0.0000,   0.0240,  -0.0054,   0.0003,  13.3484,
            0.0000,   0.0468,  -0.0104,   0.0005,  13.4932,
            0.0000,   0.0671,  -0.0147,   0.0007,  13.7326,
            0.0000,   0.0844,  -0.0180,   0.0009,  14.0644,
            0.0000,   0.0980,  -0.0203,   0.0009,  14.4850,
            0.0000,   0.1080,  -0.0216,   0.0010,  14.9907,
            0.0000,   0.1145,  -0.0220,   0.0009,  15.5769,
            0.0000,   0.1179,  -0.0218,   0.0009,  16.2390,
            0.0000,   0.1187,  -0.0210,   0.0008,  16.9721;
  
  lut_.push_back(lut1);
  lut_.push_back(lut2);
  lut_.push_back(lut3);
  lut_.push_back(lut4);
  lut_.push_back(lut5);
  lut_.push_back(lut6);
  lut_.push_back(lut7);
  lut_.push_back(lut8);
  lut_.push_back(lut9);
  lut_.push_back(lut10);
  lut_.push_back(lut11);
  lut_.push_back(lut12);
  lut_.push_back(lut13);
  lut_.push_back(lut14);
};

BVPSolver::BVPSolver(double x0, double y0, double theta0, double k0, double x1, double y1, double theta1, double k1)
{
  x0_ = x0;
  y0_ = y0;
  theta0_ = theta0;
  k0_ = k0;
  x1_ = x1;
  y1_ = y1;
  theta1_ = theta1;
  k1_ = k1;
}

void BVPSolver::setBoundaryValue(double x0, double y0, double theta0, double k0, double x1, double y1, double theta1, double k1)
{
  x0_ = x0;
  y0_ = y0;
  theta0_ = theta0;
  k0_ = k0;
  x1_ = x1;
  y1_ = y1;
  theta1_ = theta1;
  k1_ = k1;
}

double BVPSolver::calCn(double a, double b, double c, double d, double s, double n)
{
  // 分段近似的端点数量
  int num = 20;

  double solution = 0;

  for (int i = 0; i <= num; i++)
  {
    double s_cur = s / num * i;

    double k;
    if (i == 0 || i == num)
      k = 1;
    else if (i % 2 == 0)
      k = 2;
    else if (i % 2 == 1)
      k = 4;

    solution += k * pow(s_cur, n) * cos(a*s_cur + 1.0/2.0*b*pow(s_cur,2) + 1.0/3.0*c*pow(s_cur,3) + 1.0/4.0*d*pow(s_cur,4));
  }

  solution = solution / 3.0 * s / (num - 1.0);
}

double BVPSolver::calSn(double a, double b, double c, double d, double s, double n)
{
  // 分段近似的端点数量
  int num = 20;

  double solution = 0;

  for (int i = 0; i <= num; i++)
  {
    double s_cur = s / num * i;

    double k;
    if (i == 0 || i == num)
      k = 1;
    else if (i % 2 == 0)
      k = 2;
    else if (i % 2 == 1)
      k = 4;

    solution += k * pow(s_cur, n) * sin(a*s_cur + 1.0/2.0*b*pow(s_cur,2) + 1.0/3.0*c*pow(s_cur,3) + 1.0/4.0*d*pow(s_cur,4));
  }

  solution = solution / 3 * s / (num - 1.0);
}

void BVPSolver::calParams()
{
  // 约束归一化，先将在车体系下将轨迹平移到原点，再将轨迹旋转-theta0，使其在车体系下起始朝向为0
  double x = (x1_ - x0_) * cos(-1 * theta0_) - (y1_ - y0_) * sin(-1 * theta0_);
  double y = (x1_ - x0_) * sin(-1 * theta0_) + (y1_ - y0_) * cos(-1 * theta0_);
  double theta = theta1_ - theta0_;
  double k0 = k0_;
  double k1 = k1_;
  // cout << x << " " << y << " " << theta << endl;

  int lut_index, param_index, negative_flag;

  // 根据x选择LUT
  lut_index = max(min(int(x - 0.5), 13), 0);

  // 根据y选择某LUT中的参数行
  param_index = max(min(int((abs(y) + 0.5) / 1.0), 9), 0);

  // 符号处理
  int sign;
  y >= 0 ? sign = 1 : sign = -1;

  // 根据LUT赋初值
  double a = lut_[lut_index](param_index, 0) * sign;
  double b = lut_[lut_index](param_index, 1) * sign;
  double c = lut_[lut_index](param_index, 2) * sign;
  double d = lut_[lut_index](param_index, 3) * sign;
  double s = lut_[lut_index](param_index, 4);

  // 牛顿迭代法求参数
  if (theta == 0)
    iterateParams(a, b, c, d, s, x, y, theta, k0, k1);
  else
  { 
    if (theta > 0)
    {
      for (double theta_t = 0.0; theta_t < theta; theta_t += 0.1)
        iterateParams(a, b, c, d, s, x, y, theta_t, k0, k1);
      iterateParams(a, b, c, d, s, x, y, theta, k0, k1);
    }
    else
    {
      for (double theta_t = 0.0; theta_t > theta; theta_t -= 0.1)
        iterateParams(a, b, c, d, s, x, y, theta_t, k0, k1);
      iterateParams(a, b, c, d, s, x, y, theta, k0, k1);
    }
  }

  // 存储计算结果
  a_ = a;
  b_ = b;
  c_ = c;
  d_ = d;
  s_ = s;
}

void BVPSolver::iterateParams(double& a, double& b, double& c, double& d, double& s, double x, double y, double theta, double k0, double k1)
{
  // 参数矩阵
  Matrix<float, 5, 1> q;
  q << a,
       b,
       c,
       d,
       s;

  // 约束方程
  Matrix<float, 5, 1> g;

  // 约束方程对参数的雅阁比矩阵
  Matrix<float, 5, 5> dg;

  while (1)
  {
    // 预计算状态
    double s2 = s*s;
    double s3 = s*s*s;
    double s4 = s*s*s*s;
    double theta_f = a*s + 1.0/2.0*b*s2 + 1.0/3.0*c*s3 + 1.0/4.0*d*s4;
    double x_f = calCn(a,b,c,d,s,0);
    double y_f = calSn(a,b,c,d,s,0);
    double k_s = a;
    double k_f = a + b*s + c*s2 + d*s3;

    // 构造约束方程
    g << theta_f - theta,
         x_f - x,
         y_f - y,
         k_s - k0,
         k_f - k1;

    // 构造约束方程对参数的雅阁比矩阵
    dg << s,                     1.0/2.0*s2,                  1.0/3.0*s3,                  1.0/4.0*s4,                  k_f,
          -1*calSn(a,b,c,d,s,1), -1.0/2.0*calSn(a,b,c,d,s,2), -1.0/3.0*calSn(a,b,c,d,s,3), -1.0/4.0*calSn(a,b,c,d,s,4), cos(theta_f),
          calCn(a,b,c,d,s,1),    1.0/2.0*calCn(a,b,c,d,s,2),  1.0/3.0*calCn(a,b,c,d,s,3),  1.0/4.0*calCn(a,b,c,d,s,4),  sin(theta_f),
          1,                     0,                           0,                           0,                           0,
          1,                     s,                           s2,                          s3,                          b+2*c*s+3*d*s2;

    // 牛顿迭代法求解
    Matrix<float, 5, 1> dq;
    dq = dg.inverse() * g;
    q = q - dq;

    // 参数更新
    a = q(0, 0);
    b = q(1, 0);
    c = q(2, 0);
    d = q(3, 0);
    s = q(4, 0);

    // 迭代收敛后退出循环
    bool terminate = true;
    for (int i = 0; i < 5; i++)
    {
      if (abs(g(i, 0)) > 0.001)
      {
        terminate  = false;
        break;
      }
    }

    // 退出
    if (terminate == true)
      break;
  }

  return;
}

void BVPSolver::getParams(double& a, double& b, double& c, double& d, double& s)
{
  a = a_;
  b = b_;
  c = c_;
  d = d_;
  s = s_;
}