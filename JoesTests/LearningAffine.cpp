//TO DO: use quarternions to make affine. Then integrate into demo.cpp
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;

int main() {

  //make rotation R from quarternion
  Eigen::Quaterniond q;  
  q = Eigen::Quaterniond(0.1,1,1,1);
  q.normalize();
  Eigen::Affine3d aq = Eigen::Affine3d(q);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));
  Eigen::Affine3d a = (t*aq); // important to keep t*r
 
  cout<<"Affine3d rotation matrix\n";
  cout << a.linear()<<"\n";
  cout << "\n";   //print out 3x3 rotation matrix
  cout << a.linear().block(0,0,1,1)<<"\n";
  cout << a.linear().block(0,1,1,1)<<"\n";
  cout << a.linear().block(0,2,1,1)<<"\n";
  cout << a.linear().block(1,0,1,1)<<"\n";
  cout << a.linear().block(1,1,1,1)<<"\n";
  cout << a.linear().block(1,2,1,1)<<"\n";
  cout << a.linear().block(2,0,1,1)<<"\n";
  cout << a.linear().block(2,1,1,1)<<"\n";
  cout << a.linear().block(2,2,1,1)<<"\n";
  cout << a(1,1)+10;   //print out 3x3 rotation matrix
  cout << "\n";
  



  

  return 0;
}