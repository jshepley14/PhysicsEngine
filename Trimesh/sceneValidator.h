#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ode/ode.h>

/* Do I need these?
#ifndef sceneValidator.h
#define sceneValidator.h
*/

class SceneValidator{
    private:
     dThreadingThreadPoolID pool;
     dThreadingImplementationID threading;

    public:
        SceneValidator();
        ~SceneValidator();

        /*  Allows the user to set certain parameters such as the THRESHOLD, Gravity,
        /   number of checks, number of checks per step, DRAW, printing out certain things etc?
        */
        bool setParams(std::string param_name, double param_value);

         

        /*Provides model names and path to all models that will be tested by isStable.
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


