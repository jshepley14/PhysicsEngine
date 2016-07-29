/****************************************************/
//     Author:  Joe Shepley
//    Contact:  jshepley14@gmail.com
//   Location:  Carnegie Mellon University Robotics Institute
//       Date:  July 2016
//Description:  This code provides functions which can be used to check if an set of objects are in static
//              equilibrium or not.  The method to do this relies on in Open Dynamics Engine, a physics engine.
//    Gitthub:  https://github.com/jshepley14/PhysicsEngine
/****************************************************/

#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ode/ode.h>
#ifndef SCENEVALIDATOR_H
#define SCENEVALIDATOR_H



class SceneValidator{
    private:
     dThreadingThreadPoolID pool;           //used for ODE's threating functions
     dThreadingImplementationID threading;  //used for ODE's threating functions
        
    public:
	SceneValidator(double GRAVITYx, double GRAVITYy, double GRAVITYz, double PLANEa, double PLANEb, double PLANEc, double PLANEd, double DEFAULT_SCALE);  //custom constructor
        SceneValidator();   //default constructor
        ~SceneValidator();  //destructor

        /*  Allows the user to set certain parameters. All parameters able to be set are in explained in detail in sceneValidator.cpp */
        bool setParams(std::string param_name, double param_value);

        /* Allows user to set scale of specific object. thisObject is the nth model in the modelnames vector. To scale the modelnames[0], thisObject should equal 0 */
        bool setScale(int thisObject, double scaleFactor);
         
        /* Allows user to set camera viewpoint when rendering a scene */
	bool setCamera(float x, float y, float z, float h, float p, float r);

        /*Provides model names and path to all models that will be tested by isValidScene.
        / Input a list of model names and a list of corresponding file locations. 
        / Files should be in .obj format.  setModels is called once.
        */ 
        void setModels(std::vector<std::string> modelnames, std::vector<std::string> filepath);


        /*Checks if a scene is physically valid given a configuration of objects in physics simulator.
        / model_poses sets the 6 DoF pose for each object.          
        */ 
        bool isValidScene(std::vector<std::string> modelnames, std::vector<Eigen::Affine3d> model_poses);
       
};

#endif


