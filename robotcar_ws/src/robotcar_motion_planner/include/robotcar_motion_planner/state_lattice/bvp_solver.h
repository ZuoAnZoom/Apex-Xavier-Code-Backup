#ifndef BVP_SOLVER
#define BVP_SOLVER

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

class BVPSolver
{
public:
  BVPSolver();

  BVPSolver(double x0, double y0, double theta0, double k0, double x1, double y1, double theta1, double k1);

  // 设置边界约束
  void setBoundaryValue(double x0, double y0, double theta0, double k0, double x1, double y1, double theta1, double k1);

  // 计算参数
  void calParams();

  // 辛普森公式近似求解s^n*cos(theta(s))的积分
  double calCn(double a, double b, double c, double d, double s, double n);

  // 辛普森公式近似求解s^n*sin(theta(s))的积分
  double calSn(double a, double b, double c, double d, double s, double n);

  // 获取系数
  void getParams(double& a, double& b, double& c, double& d, double& s);

private:
  void iterateParams(double& a, double& b, double& c, double& d, double& s, double x, double y, double theta, double k0, double k1);

  // 边界约束
  double x0_, y0_, theta0_, k0_;
  double x1_, y1_, theta1_, k1_;

  // 参数值
  double a_, b_, c_, d_, s_;

  // LUT，列为5参数，行从0以步长1递增至9
  std::vector<Eigen::Matrix<float, 10, 5>> lut_;
};

#endif /*BVP_SOLVER*/
