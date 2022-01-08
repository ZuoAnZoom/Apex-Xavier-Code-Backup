#ifndef FRAME_TRANSFORMER
#define FRAME_TRANSFORMER

#include <math.h>
#include <vector>
#include <robotcar_general/curve_fitter.h>

// 功能：Frenet坐标系与Cartesian坐标系间的相互转换，以及给出一个多项式回归的接口
class FrameTransformer
{
public:
  FrameTransformer();

  // 初始化，输入线方程的系数，x坐标始末点
  FrameTransformer(std::vector<double> line_cof, float len);

  ~FrameTransformer();

  // 从Frenet坐标系转换到Cartesian坐标系
  std::vector<double> fre2Cart(double s, double d);

  // 从Cartesian坐标系转换到Frenet坐标系
  std::vector<double> cart2Fre(double x, double y);

private:
  // 寻找Cartesian坐标系下一点到车道线的投影
  std::vector<double> project2Line(double x, double y);

  // 返回两点间欧式距离
  double calDis(double x0, double y0, double x1, double y1);

  CurveFitter cf_;

  std::vector<double> line_cof_;         // 线方程系数
  std::vector<double> xs_cof_;           // x(s)系数
  std::vector<double> ys_cof_;           // y(s)系数
  
  // 线方程离散化
  std::vector<double> x_fit;
  std::vector<double> y_fit;
  std::vector<double> s_fit;
};

#endif /* FRAME_TRANSFORMER */
