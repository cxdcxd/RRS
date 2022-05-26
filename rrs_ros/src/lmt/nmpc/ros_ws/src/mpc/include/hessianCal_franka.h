#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,7,7> Matrix7d;

using namespace  std;
class HessianCal
{
public:
  static void getAnalyticalJacobian(const vector<double>& qs, Matrix7d& jacobian);
  static void getAnalyticalJacobianOmega(const vector<double> &qs, Eigen::Matrix<double,6,7> &jacobian);
  static void getEEFTransform(const vector<double> &qs, Eigen::Matrix3d &rotm, Eigen::Vector3d &translation);
  static void rotm2quat(const Eigen::Matrix3d &rotm, Eigen::Vector4d &quat);
  static void getPosition(const vector<double>& qs, Vector7d& eef_position);
  static void getAnalyJaco4Ref(const int link_index, const vector<double>& qs,  Eigen::Matrix<double,3,7>& jacobian, const Eigen::Vector3d& ref);

};
