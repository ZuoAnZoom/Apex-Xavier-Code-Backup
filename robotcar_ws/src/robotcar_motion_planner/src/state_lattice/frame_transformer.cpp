#include <robotcar_motion_planner/state_lattice/frame_transformer.h>
#include <ctime>

using namespace std;
using namespace Eigen;

FrameTransformer::FrameTransformer()
{
  
}

FrameTransformer::FrameTransformer(vector<double> line_cof, float len)
{
  // 拷贝线方程系数
  line_cof_.resize(line_cof.size());
  for (int i = 0; i < line_cof.size(); i++)
  {
    line_cof_[i] = line_cof[i];
  }

  // 线方程离散化，寻找x、y、s间函数关系，用于坐标转换和参考点投影

  // 先判断斜率，若斜率绝对值过大，则直接认为是平行于y轴的特殊情况
  if (abs(line_cof[1]) > 100 || isnan(line_cof[1]) == true)
  {
    double xi = 0;
    double yi = 0;
    double si = 0;
    for (int i = 0;; i++)
    {
      yi = 0.05 * i;

      if (i != 0)
        si = s_fit[i - 1] + calDis(xi, yi, x_fit[i - 1], y_fit[i - 1]);

      if (si > len)
        break;

      x_fit.push_back(xi);
      y_fit.push_back(yi);
      s_fit.push_back(si);
    }
  }
  // 若非平行于y轴的特殊情况
  else
  {
    // 寻找x方向恰当的离散距离
    double dx = 0.05;
    // while (cf_.polyVal(line_cof_, dx) > 0.05)
    // {
    //   dx *= 0.5;
    // }

    // 按照计算的离散距离进行离散化
    double si = 0;
    int i = 0;
    for (int i = 0;; i++)
    {
      double xi = 0 + i * dx;
      double yi = cf_.polyVal(line_cof_, xi);

      if (i != 0)
        si = s_fit[i - 1] + calDis(xi, yi, x_fit[i - 1], y_fit[i - 1]);

      if (si > len)
        break;

      x_fit.push_back(xi);
      y_fit.push_back(yi);
      s_fit.push_back(si);
    }
  }

  // 回归得到系数并存储
  xs_cof_ = cf_.polyFit(s_fit, x_fit, 5);
  ys_cof_ = cf_.polyFit(s_fit, y_fit, 5);
}

FrameTransformer::~FrameTransformer()
{

}

vector<double> FrameTransformer::fre2Cart(double s, double d)
{
  double x_ref = cf_.polyVal(xs_cof_, s);
  double y_ref = cf_.polyVal(ys_cof_, s);
  double theta_ref = atan(line_cof_[1] +  2 * line_cof_[2] * x_ref + 3 * line_cof_[3] * x_ref * x_ref);
  
  vector<double> cart_point(2);
  cart_point[0] = x_ref - sin(theta_ref) * d;
  cart_point[1] = y_ref + cos(theta_ref) * d;
  
  return cart_point;
}

vector<double> FrameTransformer::cart2Fre(double x, double y)
{
  // 投影到线方程上得到参考点
  vector<double> nearest_p = project2Line(x, y);
  double px = nearest_p[0];
  double py = nearest_p[1];
  double ps = nearest_p[2];
  double dis = nearest_p[3];
  double theta = atan(line_cof_[1] +  2 * line_cof_[2] * px + 3 * line_cof_[3] * px * px);
  double cross_rd_nd = cos(theta) * (y - py) - sin(theta) * (x - px);

  vector<double> fre_point(2);
  fre_point[0] = ps;
  fre_point[1] = copysign(dis, cross_rd_nd);
  
  return fre_point;
}

std::vector<double> FrameTransformer::project2Line(double x, double y)
{
  double xn, yn, sn;
  double min_dis_sq = 1e10;

  for (int i = 0; i < x_fit.size(); i++)
  {
    double dis_sq = pow((x_fit[i] - x), 2) + pow((y_fit[i] - y), 2);
    if (dis_sq < min_dis_sq)
    {
      min_dis_sq = dis_sq;
      xn = x_fit[i];
      yn = y_fit[i];
      sn = s_fit[i];
    }
  }

  vector<double> near_p(4);
  near_p[0] = xn;
  near_p[1] = yn;
  near_p[2] = sn;
  near_p[3] = sqrt(min_dis_sq);
  return near_p;
}

double FrameTransformer::calDis(double x0, double y0, double x1, double y1)
{
  return sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2));
}