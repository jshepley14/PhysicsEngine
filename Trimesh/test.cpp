#include "sceneValidator.h"
#include <map>
#include <cassert>              
#include <string.h>
#include <fstream>  
#include <cmath>   
#include <chrono>                 //used for timing code
#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;


int main (int argc, char **argv)
{
  

 // Here's all the input***********************************************************************************************************
  vector<string> filenames = {"/home/joeshepley/Projects/PhysicsEngine/Trimesh/object_files/teacup.obj",
                            "/home/joeshepley/Projects/PhysicsEngine/Trimesh/object_files/milk_carton.obj",
                             "/home/joeshepley/Projects/PhysicsEngine/Trimesh/object_files/red_mug.obj"};
  
  vector<string> modelnames = {"teacup", "milk_carton", "mug"};
  
  //make affine3d's
  Eigen::Quaterniond q;  
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);
  q.normalize();
  Eigen::Affine3d aq = Eigen::Affine3d(q);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(-2.5,0, 1.59)));
  Eigen::Affine3d a = (t*aq); 
    
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);
  q.normalize();
  aq = Eigen::Affine3d(q);
  t = (Eigen::Translation3d(Eigen::Vector3d(2,0,1.1)));
  Eigen::Affine3d b = (t*aq); 
    
  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);
  q.normalize();
  aq = Eigen::Affine3d(q);
  t =  (Eigen::Translation3d(Eigen::Vector3d(-2,0,0.66)));
  Eigen::Affine3d c = (t*aq); 

  vector<Eigen::Affine3d> model_poses = {a,b,c};  //unbalanced teacup on mug

  q = Eigen::Quaterniond(0.5, 0.5, 0, 0);
  q.normalize();
  aq = Eigen::Affine3d(q);
  t =  (Eigen::Translation3d(Eigen::Vector3d(-2,0, 1.59)));
  Eigen::Affine3d a2 = (t*aq); 
  
  vector<Eigen::Affine3d> model_poses2 = {a2,b,c};  //teacup falling from the sky
// Here's all the input^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^









  

  SceneValidator scene;                            //construct the object
  scene.setParams("DRAW", true);                   //visualize what's actually happening 
  scene.setParams("PRINT_CHECKER_RESULT", true);   //print out the results
  
  
  // API functions
  scene.setModels(modelnames, filenames);        //set all the models and get their data  

  scene.isValidScene(modelnames, model_poses);   //check scene 1
  scene.isValidScene(modelnames, model_poses2);  //check scene 2

  
 


  return 0;
}