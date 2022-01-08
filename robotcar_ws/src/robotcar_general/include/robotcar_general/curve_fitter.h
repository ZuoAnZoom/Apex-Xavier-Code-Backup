#ifndef CURVE_FIT
#define CURVE_FIT

#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>

class CurveFitter
{
public:
  // 曲线拟合
  std::vector<double> polyFit(std::vector<double> xvals, std::vector<double> yvals, int order);

  // 获取曲线上一点的函数值：输入系数与变量，输出函数值
  double polyVal(std::vector<double> cof, double x);
};

#endif /* CURVE_FIT */
