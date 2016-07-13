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
  Eigen::Affine3d r = create_rotation_matrix(1.5,2,2);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));


  
  Eigen::Matrix4d m = (t * r).matrix(); // Option 1

  Eigen::Affine3d a = (r*t); // Option 1

  cout<< a.operator()(0,0)<<"\n";
  cout<< a.operator()(0,1)<<"\n";
  cout<< a.operator()(0,2)<<"\n";
  cout<< a.operator()(0,3)<<"\n";
  cout<< a.operator()(1,0)<<"\n";
  cout<< a.operator()(1,1)<<"\n";
  cout<< a.operator()(1,2)<<"\n";
  cout<< a.operator()(1,3)<<"\n";
  cout<< a.operator()(2,0)<<"\n";
  cout<< a.operator()(2,1)<<"\n";
  cout<< a.operator()(2,2)<<"\n";
  cout<< a.operator()(2,3)<<"\n";
  cout<< a.operator()(3,0)<<"\n";
  cout<< a.operator()(3,1)<<"\n";
  cout<< a.operator()(3,2)<<"\n";
  cout<< a.operator()(3,3)<<"\n";

  
  //cout<< a.data()[3];

 /*
  cout <<"Matrix4d: \n" <<  m<<"\n";
  cout<<"Rotation \n" << m.block(0,0,3,3)<<"\n";
  cout<<"Vector: \n" << m.block<4,1>(0,3);
*/
  

  return 0;
}