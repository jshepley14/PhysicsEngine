class SceneValidator{


    public:

        /*Provides model names and path to all models that will be tested by isStable.
        / Input a list of model names and a list of corresponding file locations. 
        / Files should be in .obj format.  setModels is called once.
        */ 
        void setModels(std:vector<string> modelnames, std:vector<string> filepath);


        /*Checks for static equilibrium given a configuration of objects in physics simulator.
        / model_IDs is a subset of setModels's model list. 
        / model_poses sets the 6 DoF pose for each object.          
        */ 
        bool isValidScene(std:vector<string> model_IDs, std:vector<Eigen:: Affine3d> model-poses) 
}


//isStable with plane definition.  More complicated. Might use, might not use.
/*
        /Checks for static equilibrium given a configuration of objects in physics simulator.
        / planeNormal and planePoint define the "ground".
        / model_IDs is a subset of setModels's model list. 
        / model_poses sets the 6 DoF pose for each object.          
        bool isStable(Vector3d planeNormal, Point3d planePoint, std:vector<string> model_IDs, std:vector<Eigen:: Affine3d> model-poses)
*/
