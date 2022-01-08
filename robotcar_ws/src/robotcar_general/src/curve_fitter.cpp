#include <robotcar_general/curve_fitter.h>

using namespace std;
using namespace Eigen;

// 获取拟合的参数曲线在x时的y值
double CurveFitter::polyVal(vector<double> cof, double x) 
{
  double result = 0.0;
  for (int i = 0; i < cof.size(); i++) 
  {
      result += cof[i] * pow(x, i);
  }
  return result;
}

// 多项式拟合的一个函数,返回拟合的参数曲线系数
vector<double> CurveFitter::polyFit(vector<double> x, vector<double> y, int order)
{
  Eigen::VectorXd xvals(x.size()), yvals(y.size());
  for (int i = 0; i < x.size(); i++)
  {
    xvals[i] = x[i];
    yvals[i] = y[i];
  }

  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
      A(i, 0) = 1.0;

  for (int j = 0; j < xvals.size(); j++) 
  {
      for (int i = 0; i < order; i++) 
          A(j, i + 1) = A(j, i) * xvals(j);
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  vector<double> cof(result.size());
  for (int i = 0; i < result.size(); i++)
  {
    cof[i] = result[i];
  }

  return cof;
}