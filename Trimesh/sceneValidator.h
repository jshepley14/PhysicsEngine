#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ode/ode.h>
#ifndef SCENEVALIDATOR_H
#define SCENEVALIDATOR_H



class SceneValidator{
    private:
     dThreadingThreadPoolID pool;
     dThreadingImplementationID threading;
     
    public:
	SceneValidator(double GRAVITYx, double GRAVITYy, double GRAVITYz, double PLANEa, double PLANEb, double PLANEc, double PLANEd);  //custom constructor
        SceneValidator();   //default constructor
        ~SceneValidator();  //destructor

        /*  Allows the user to set certain parameters */
        bool setParams(std::string param_name, double param_value);

        /* Allows user to set scale of specific object */
        bool setScale(int thisObject, double scaleFactor);

        /*Provides model names and path to all models that will be tested by isValidScene.
        / Input a list of model names and a list of corresponding file locations. 
        / Files should be in .obj format.  setModels is called once.
        */ 
        void setModels(std::vector<std::string> modelnames, std::vector<std::string> filepath);


        /*Checks for static equilibrium given a configuration of objects in physics simulator.
        / model_IDs is a subset of setModels's model list. 
        / model_poses sets the 6 DoF pose for each object.          
        */ 
        bool isValidScene(std::vector<std::string> modelnames, std::vector<Eigen::Affine3d> model_poses);
         
};

#endif


