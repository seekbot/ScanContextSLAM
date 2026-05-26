#include "utilities.hpp"

int main(int argc, char** argv) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) =
    (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX())).toRotationMatrix();
  T.col(3).head<3>() = Eigen::Vector3d(1.2, -0.4, 0.8);

  Eigen::Matrix4d T2 = gtsamPoseToPoseEig(poseEigToGtsamPose(T));
  double err = (T - T2).norm();
  std::cout << "round-trip error = " << err << "\n";     // expect < 1e-12

  return 0;
}