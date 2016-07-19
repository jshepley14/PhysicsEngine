//Joe Shepley CMU 2016
//Static Equilibrium tester in ODE physics engine
//
// ***^^^PRESS N FOR A "RESET", PRESS J FOR A FALLING OBJECT^^^***
//
/****************************************************/



#include "sceneValidator.h"
#include <map>
#include <cassert>              
#include <ode/ode.h>              //main physics engine library
#include <drawstuff/drawstuff.h>  //this is the graphics library
#include <string.h>
#include <fstream>  
#include <cmath>   
#include <chrono>                 //used for timing code
#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "objLoader.h"           //used for parsing .obj file

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions, really only dsDrawTriangleD is used in this program 
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif

// some constants
#define NUM 200			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 64		// maximum number of contact points per body
using namespace std;



//Variables that can be set in setParams()
static int STEP1=6;
static int STEP2=14;
static int STEP3=20;
static int STEP4=110;
static double GRAVITYx = 0;
static double GRAVITYy = 0;
static double GRAVITYz = -0.5;     // yes, this is not -9.8, but this was what the default trimesh demo had 
static double PLANEa = 0;          // Equation of a plane: a*x+b*y+c*z = d The normal vector must have legnth 1
static double PLANEb = 0;
static double PLANEc = 1;           
static double PLANEd = 0;          
static double THRESHHOLD = 0.08;   //amount objects allowed to move while still being marked as in static equilibrium
static double TIMESTEP = 0.1;      //controls how far each step is taken
static double FRICTON_mu =  1.0;   //if you set this to 0 it will be very slippery
static double FRICTON_mu2 =  0.0;  //changing this doesn't seem to do much
static double BOUNCE = 0.0;        //change the bounciness
static double BOUNCE_vel = 0.0;    //change the bounciness speed
static double SOFT_CFM = 0.01;     //makes "system more numerically robust" according to ODE manual. Not 100% sure what it does... The current number is from a default demo.
static bool DRAW = false;          //used to switch on or off the drawing of the scene
static bool PRINT_START_POS = false;  //print an object's intial x,y,z center
static bool PRINT_END_POS   = false;  //print an object's final x,y,z center
static bool PRINT_DELTA_POS = false;  //print an object's delta x,y,z for its center 
static bool PRINT_CHECKER_RESULT = false; //print the 



//timer variables part of c++11 
static chrono::steady_clock::time_point startTime, endTime;  //variables used for timing purposes

//variables used when DRAW = true
static int counter = 0;         
static int dsSTEP=100;
static int HEIGHT =500;
static int WIDTH = 1000;
 
    

// dynamics and collision objects

struct MyObject {
  dBodyID body;			 // the body
  dGeomID geom[GPB]; // geometries representing this body

  // Trimesh only - double buffered matrices for 'last transform' setup
  dReal matrix_dblbuff[ 16 * 2 ];
  int last_matrix_index;

  int IDnumber;      //the number of the object        
  dReal center[3];    //the center x,y,z coordinates
  int indCount;      //number of triangles (indices)
  int vertCount;     //number of vertices
  vector< vector<int> > indexDrawVec;
  vector<int> indexGeomVec;
  vector<float> vertexDrawVec;
  vector<float> vertexGeomVec;
  vector<float> centerOfMass;
};



static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int selected = -1;	// selected object
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?
static std::map<std::string, MyObject> m;
typedef dReal dVector3R[3];



// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = FRICTON_mu; 
    contact[i].surface.mu2 = FRICTON_mu2;
    contact[i].surface.bounce = BOUNCE; 
    contact[i].surface.bounce_vel = BOUNCE_vel; 
    contact[i].surface.soft_cfm = SOFT_CFM;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
      if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}


// start simulation - set viewpoint
static void start()
{
  //static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  //static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  static float xyz[3]={-0.0559,-8.2456,6.0500};
  static float hpr[3]= { 89.0000,-25.0000,0.0000};
  dsSetViewpoint (xyz,hpr);
}


static bool inStaticEquilibrium(MyObject &object){
    double startX = object.center[0];
    double startY = object.center[1];
    double startZ = object.center[2];
    double endX = dBodyGetPosition(object.body)[0];
    double endY = dBodyGetPosition(object.body)[1];
    double endZ = dBodyGetPosition(object.body)[2];
    double deltaX = std::abs(startX - endX);
    double deltaY = std::abs(startY - endY); 
    double deltaZ = std::abs(startZ - endZ);

    if (PRINT_START_POS){
      cout<<"Start: "<<startX<<", "<<startY<<", "<<startZ<<endl;
    }
    if (PRINT_END_POS){
      cout<<"  End: "<<endX<<", "<<endY<<", "<<endZ<<endl;
    }
    if (PRINT_DELTA_POS){
      cout<<"Delta: "<<deltaX<<", "<<deltaY<<", "<<deltaZ<<endl;
    }
    
    if ( deltaX > THRESHHOLD || deltaY > THRESHHOLD || deltaZ > THRESHHOLD){
        return false;
    } else{ 
        return true;
    }
}
 



static bool isValid(std::vector<string> modelnames){
    bool stable = true;
    for (int i=0; i<num; i++){ 
       auto mappedObject= m.find(modelnames[i]);
       if (!inStaticEquilibrium(mappedObject->second) ){
          //cout << "FALSE: Not in static equilibrium" << endl;
          stable = false;
          break;        
       } 
    } 
    if (stable == true){
      if(PRINT_CHECKER_RESULT){
        cout << "TRUE"<<endl;
      }
      return true;
    } else{
      if(PRINT_CHECKER_RESULT){
        cout << "FALSE"<<endl;
      }
      
      return false;
    }
}




//To Do
void setObject (MyObject &object, int number, char* filename){
  int SCALE =100;
  
  //Load the file
  objLoader *objData = new objLoader();
	objData->load(filename);

  object.IDnumber = number;
  object.indCount = objData->faceCount;
  object.vertCount = objData->vertexCount;


  //Make 2D vector of indices for drawing 
  int indexCount = object.indCount; 
	for(int i=0; i<indexCount; i++)
	{	vector<int> temp_vec;
		temp_vec.push_back((objData->faceList[i])->vertex_index[0]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[1]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[2]);
		object.indexDrawVec.push_back(temp_vec);
	}

  //Make 1D vector of indices for geometry
  for(int i=0; i< indexCount; i++)
  {	
    object.indexGeomVec.push_back((objData->faceList[i])->vertex_index[0]);
    object.indexGeomVec.push_back((objData->faceList[i])->vertex_index[1]);
    object.indexGeomVec.push_back((objData->faceList[i])->vertex_index[2]);   
  }

  //Make 1D vector of vertices for drawing
  int vertCount =  object.vertCount;
	for(int i=0; i< vertCount ; i++){
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[0]/SCALE );
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[1]/SCALE );
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[2]/SCALE );	
	}
  
  //Make 1D vector of vertices for geometry
  for(int i=0; i< vertCount ; i++){
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[0]/SCALE );
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[1]/SCALE );
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[2]/SCALE );	
	}

} 

  

void makeObject (MyObject &object){
    int i,j,k;
    dMass m;

  //build Trimesh geom
  dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(new_tmdata, object.vertexGeomVec.data(), 3 * sizeof(float), 
	     object.vertCount, (int*)object.indexGeomVec.data(), object.indCount*3, 3 * sizeof(int));
  object.geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);
  dGeomSetData(object.geom[0], new_tmdata);        
  dMassSetTrimesh( &m, DENSITY, object.geom[0] );

  //gets the absolute bounding box, this code isn't current used
  dReal aabb[6];
  dGeomGetAABB (object.geom[0], aabb);
  dReal maxZ = aabb[5];  //<- maxZ would be the highest Z position in the AABB


  object.body = dBodyCreate (world); 
  dBodySetData (object.body,(void*)(size_t)i);
  //set body loop
  for (k=0; k < GPB; k++){
      if (object.geom[k]){
          dGeomSetBody(object.geom[k],object.body);
      }
  } 
  dGeomSetOffsetPosition(object.geom[0], -m.c[0],-m.c[1], -m.c[2]);
  dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]); 
  dBodySetMass(object.body,&m);

}




void translateObject(MyObject &object, const dReal* center, const dMatrix3 R ){
      object.center[0] = center[0];
      object.center[1] = center[1];
      object.center[2] = center[2];
      dBodySetPosition (object.body, center[0], center[1], center[2]);
      dBodySetRotation (object.body, R);
      dBodySetLinearVel(object.body, 0, 0, 0);
      dBodySetAngularVel(object.body, 0, 0, 0);
}



// set previous transformation matrix for trimesh
void setCurrentTransform(dGeomID geom){
   const dReal* Pos = dGeomGetPosition(geom);
   const dReal* Rot = dGeomGetRotation(geom);

   const dReal Transform[16] = 
   {
     Rot[0], Rot[4], Rot[8],  0,
     Rot[1], Rot[5], Rot[9],  0,
     Rot[2], Rot[6], Rot[10], 0,
     Pos[0], Pos[1], Pos[2],  1
   };
   dGeomTriMeshSetLastTransform( geom, *(dMatrix4*)(&Transform) );
}




// simulation loop
static void simLoop (int pause)
{
  //if DRAW = true, this is used to terminate the simloop from dsSimulationLoop()
  if (counter == dsSTEP){
      dsStop();             
  }
  counter ++;


  //define the space and collide function
  dSpaceCollide (space,0,&nearCallback);

#if 1
  if (!pause) 
  {
    for (int i=0; i<num; i++)
      for (int j=0; j < GPB; j++)
        if (obj[i].geom[j])
          if (dGeomGetClass(obj[i].geom[j]) == dTriMeshClass)
            setCurrentTransform(obj[i].geom[j]);
  }
#endif

  if (!pause) dWorldQuickStep (world,TIMESTEP); //originally 0.05

  for (int j = 0; j < dSpaceGetNumGeoms(space); j++){
	  dSpaceGetGeom(space, j);
  }
  // remove all contact joints
  dJointGroupEmpty (contactgroup);

 
  if(DRAW){
    dsSetColor (1,1,0);
    dsSetTexture (DS_WOOD);
  } 
  

  for (int i=0; i<num; i++) {
    for (int j=0; j < GPB; j++) {
      if (obj[i].geom[j]) {  
        if (dGeomGetClass(obj[i].geom[j]) == dTriMeshClass) {
          const dReal* Pos = dGeomGetPosition(obj[i].geom[j]);
          const dReal* Rot = dGeomGetRotation(obj[i].geom[j]);
  
        if (DRAW) {
            for (int ii = 0; ii < obj[i].indCount; ii++) {
                const dReal v[9] = { // explicit conversion from float to dReal
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][0] * 3 + 0],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][0] * 3 + 1],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][0] * 3 + 2],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][1] * 3 + 0],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][1] * 3 + 1],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][1] * 3 + 2],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][2] * 3 + 0],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][2] * 3 + 1],
                  obj[i].vertexDrawVec[obj[i].indexDrawVec[ii][2] * 3 + 2]
                };
                dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
              }
        }
         

      // tell the tri-tri collider the current transform of the trimesh --
          // this is fairly important for good results.     
		  // Fill in the (4x4) matrix.
		  dReal* p_matrix = obj[i].matrix_dblbuff + ( obj[i].last_matrix_index * 16 );

		  p_matrix[ 0 ] = Rot[ 0 ];	p_matrix[ 1 ] = Rot[ 1 ];	p_matrix[ 2 ] = Rot[ 2 ];	p_matrix[ 3 ] = 0;
		  p_matrix[ 4 ] = Rot[ 4 ];	p_matrix[ 5 ] = Rot[ 5 ];	p_matrix[ 6 ] = Rot[ 6 ];	p_matrix[ 7 ] = 0;
		  p_matrix[ 8 ] = Rot[ 8 ];	p_matrix[ 9 ] = Rot[ 9 ];	p_matrix[10 ] = Rot[10 ];	p_matrix[11 ] = 0;
		  p_matrix[12 ] = Pos[ 0 ];	p_matrix[13 ] = Pos[ 1 ];	p_matrix[14 ] = Pos[ 2 ];	p_matrix[15 ] = 1;

		  // Flip to other matrix.
		  obj[i].last_matrix_index = !obj[i].last_matrix_index;

		  dGeomTriMeshSetLastTransform( obj[i].geom[j], 
			  *(dMatrix4*)( obj[i].matrix_dblbuff + obj[i].last_matrix_index * 16 ) );
  
        } 
      }
    }
  }
}





void drawstuffsimLoop(){
  int argc=NULL;
  char **argv=NULL;
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = NULL;
  fn.stop = NULL;
  fn.path_to_textures = "/home/joeshepley/ode-0.13.1/drawstuff/textures";
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
}


void SceneValidator::setModels(std::vector<string> modelnames, std::vector<string> filenames){

    //set the data in an obj array
   if( modelnames.size() != filenames.size()){
          std::cout<<"***ERROR*** in setModels(std::vector<string> modelnames, std::vector<string> filenames). The problem is that modelnames is not the same size as filenames"<<endl;
   } else{
      num = filenames.size();
      for (int i =0; i < num; i++){
          char *charfilenames = new char[filenames[i].length() + 1];
          std::strcpy(charfilenames, filenames[i].c_str());
          setObject(obj[i], i, charfilenames );
          makeObject(obj[i]);
      }
   }
   //make hashmap between modelnames and their data
   for (int i =0; i < num; i++){
      m[modelnames[i]]=obj[i];
   }
}



static bool isStableStill(std::vector<string> modelnames, int step){ 
    if (DRAW){
      counter=0;
      dsSTEP=step; //200;
      drawstuffsimLoop();
    } else {
      for(int i = 0; i <= step; i++) {
        simLoop(0);
       }
    }    
    return isValid(modelnames);
}






bool  SceneValidator::setParams(std::string param_name, double param_value){   
      if( param_name.compare("STEP1") == 0 ){     
        STEP1 = param_value;
        return true;
      } else if( param_name.compare("STEP2") == 0 ){    
        STEP2 = param_value;
        return true;
      } else if( param_name.compare("STEP3") == 0 ){       
        STEP3 = param_value;
        return true;
      } else if( param_name.compare("STEP4") == 0 ){    
        STEP4 = param_value;
        return true;
      } else if( param_name.compare("GRAVITYx") == 0 ){    
        GRAVITYx = param_value;
        return true;
      } else if( param_name.compare("GRAVITYy") == 0 ){   
        GRAVITYy = param_value;
        return true;
      } else if( param_name.compare("GRAVITYz") == 0 ){  
        GRAVITYz = param_value;
        return true;
      } else if( param_name.compare("PLANEa") == 0 ){   
        PLANEa = param_value;
        return true;
      } else if( param_name.compare("PLANEb") == 0 ){
        PLANEb = param_value;
        return true;
      } else if( param_name.compare("PLANEc") == 0 ){ 
        PLANEc = param_value;
        return true;
      } else if( param_name.compare("PLANEd") == 0 ){    
        PLANEd = param_value;
        return true;
      } else if( param_name.compare("THRESHHOLD") == 0 ){   
        THRESHHOLD = param_value;
        return true;
      } else if( param_name.compare("TIMESTEP") == 0 ){   
        TIMESTEP = param_value;
        return true;
      } else if( param_name.compare("FRICTON_mu") == 0 ){      
        FRICTON_mu = param_value;
        return true;
      } else if( param_name.compare("FRICTON_mu2") == 0 ){   
        FRICTON_mu2 = param_value;
        return true;
      } else if( param_name.compare("BOUNCE") == 0 ){     
        BOUNCE = param_value;
        return true;
      } else if( param_name.compare("BOUNCE_vel") == 0 ){    
        BOUNCE_vel = param_value;
        return true;
      } else if( param_name.compare("SOFT_CFM") == 0 ){    
        SOFT_CFM = param_value;
        return true;
      } else if( param_name.compare("DRAW") == 0 ){    
        DRAW = param_value;
        return true;
      } else if( param_name.compare("PRINT_START_POS") == 0 ){    
        PRINT_START_POS = param_value;
        return true;
      } else if( param_name.compare("PRINT_END_POS") == 0 ){    
        PRINT_END_POS = param_value;
        return true;
      } else if( param_name.compare("PRINT_DELTA_POS") == 0 ){    
        PRINT_DELTA_POS = param_value;
        return true;
      } else if( param_name.compare("PRINT_CHECKER_RESULT") == 0 ){    
        PRINT_CHECKER_RESULT = param_value;
        return true;
      } else {
        cout<<"Invalid parameter name: "<<param_name;
        return false;
      }     
}



bool SceneValidator::isValidScene(std::vector<string> modelnames, std::vector<Eigen::Affine3d> model_poses){
    //set all the Objects's positions
    num = modelnames.size();
    for (int i =0; i < num; i++){
       auto mappedObject= m.find(modelnames[i]);
       
       Eigen::Affine3d a = model_poses[i];
       const dMatrix3 R = { 
         a(0,0), a(0,1), a(0,2),  
         a(1,0), a(1,1), a(1,2),  
         a(2,0), a(2,1), a(2,2)    }; 
       const dReal center[3] = {a.translation()[0],a.translation()[1], a.translation()[2]};
       translateObject(mappedObject->second, center, R);      
    }
    
    //complete series of checks to see if scene is still stable or not
    if (!isStableStill(modelnames, STEP1)){
         return false;
    } else
    if (!isStableStill(modelnames, STEP2)){
         return false;
    } else
    if (!isStableStill(modelnames, STEP3)){
         return false;
    } else
    if (!isStableStill(modelnames, STEP4)){
         return false;
    }     
    else{
         return true;
    }
}

//constuctor
SceneValidator::SceneValidator(){
  // create threading world
  dInitODE2(0);
  world = dWorldCreate();
  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,GRAVITYx,GRAVITYy,GRAVITYz);
  dWorldSetCFM (world,1e-5);
  dCreatePlane (space,PLANEa,PLANEb,PLANEc,PLANEd); 
  dAllocateODEDataForThread(dAllocateMaskAll);
  threading = dThreadingAllocateMultiThreadedImplementation();
  pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);
}

//destructor
SceneValidator::~SceneValidator(){ 
  // destroy threading and world
  dThreadingImplementationShutdownProcessing(threading);
  dThreadingFreeThreadPool(pool);
  dWorldSetStepThreadingImplementation(world, NULL, NULL);
  dThreadingFreeImplementation(threading);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
}






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







/* The following is what would go in main() */

  SceneValidator scene;
  scene.setParams("DRAW", true);
  scene.setParams("PRINT_CHECKER_RESULT", true);
 
 
  // API functions
  scene.setModels(modelnames, filenames);

  scene.isValidScene(modelnames, model_poses);
  scene.isValidScene(modelnames, model_poses2);

  
 


  return 0;
}







