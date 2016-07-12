#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

int main() {
  Eigen::Affine3d r = create_rotation_matrix(1.5,0,0);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));

  Eigen::Matrix4d m = (t * r).matrix(); // Option 1
  cout <<"Matrix4d: \n" <<  m<<"\n";
  cout<<"Rotation \n" << m.block(0,0,3,3)<<"\n";
  cout<<"Vector: \n" << m.block<4,1>(0,3);

  return 0;
}