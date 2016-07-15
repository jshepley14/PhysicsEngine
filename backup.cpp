//Joe Shepley CMY 2016
//Static Equilibrium tester in ODE physics engine
//
// ***^^^PRESS N FOR A "RESET", PRESS J FOR A FALLING OBJECT^^^***
//
/****************************************************
//                   To Do
//make a setScene() function
//how to detect if two trimeshes are stuck inside eachother?
// Again, test how long it will take to check 1000 scenes
//
//Venkat: what the grid layout will be? should i only test discrete numbers?
//Venkat: figure out affine3d
//
//try to print out final rotation matrix
//need a metric of what is actually stable, ground truth test data
//see why pikachu didn't work? could not find file. maybe instead of segfault catch that error? ask venkat
//create vector of strings, create a scene
// object.rotation?  
//remember the importance of num++ or num in general, creating and deleting
//find the right balance for friction
/**********************************************/
#include <map>
#include <cassert>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>  //used for timing code
#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "objLoader.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
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

//#define DRAW  //used to switch on or off the drawing of the scene
static int HEIGHT =500;
static int WIDTH = 1000;




//Some COUNT, TIMESTEP pairs: 6, 0.1  and 20, 0.05
//static equilibrium constants
static int GlobalCounter = 0;
static int counter = 0;          //iterator which will reach COUNT
static int dsSTEP=100;
static int STEP = 20; //20?            //how long until we want to wait to check satic equlibrium?
static double THRESHHOLD = 0.08;  //how much movement is allowed
//static double GRAVITY = -19.8;    
static double TIMESTEP = 0.1;

//Time step originally was at 0.01, but we may
//be able to increase this making calculations faster
//most efficient would prob be 0.01 ≤ TIMESTEP ≤ 0.03 


//timer variables
//need to add -std=c++11 right after g++ to compile
static chrono::steady_clock::time_point startTime, endTime;


//***********SOME POSITION CONSTANTS**************
const dReal center00[3] = {-2.3,0,1.82}; // 
const dReal center01[3] = {-0.1,-0.1,3.1};
const dReal center02[3] = {-2,0,1.84};
const dReal center03[3] = {1.4,0,0.65};
const dReal center04[3] = {-2,0,2.82};
const dReal center05[3] = {1.32,0.3,0.7};


const dReal center10[3] = {2,0,0.44}; // 
const dReal center11[3] = {3,0,0.44}; //
const dReal center12[3] = {-2,0,2.76}; //


const dReal center20[3] = {-2,0,0.66}; //
const dReal center21[3] = {-1.93,0,0.66}; //


const dReal center4[3] = {0,0,4}; //
const dReal center5[3] = {0,0,0.63}; //
const dReal center6[3] = {0,-4,0.7}; //


const dReal center1_2[3] = {1.0,1.0,3.0}; 
const dReal center2_2[3] = {2.0,2.0,3.0}; 
const dReal center3_2[3] = {3.0,3.0,3.0}; 
const dReal center4_2[3] = {4.0,4.0,3.0}; 

const dReal center30[3] = {0,0, 1.1}; 
const dReal center31[3] = {0,0, 2.1};


const dReal center50[3] = {0,1, 0.44};
const dReal center51[3] = {0,2, 0.44};
const dReal center52[3] = {0,3, 0.44};
const dReal center53[3] = {0,4, 0.44};
const dReal center54[3] = {0,5, 0.44};
const dReal center55[3] = {0,6, 0.44};
const dReal center56[3] = {0,7, 0.44};
const dReal center57[3] = {0,8, 0.44};
const dReal center58[3] = {0,9, 0.44};
const dReal center59[3] = {0,10, 0.44};
const dReal center60[3] = {2,0, 0.44};
const dReal center61[3] = {2,1, 0.44};
const dReal center62[3] = {2,2, 0.44};
const dReal center63[3] = {2,3, 0.44};
const dReal center64[3] = {2,4, 0.44};
const dReal center65[3] = {2,5, 0.44};
const dReal center66[3] = {2,6, 0.44};
const dReal center67[3] = {2,7, 0.44};
const dReal center68[3] = {2,8, 0.44};
const dReal center69[3] = {2,9, 0.44};

const dReal center50x[3] = {3,1, 0.44};
const dReal center51x[3] = {3,2, 0.44};
const dReal center52x[3] = {3,3, 0.44};
const dReal center53x[3] = {3,4, 0.44};
const dReal center54x[3] = {3,5, 0.44};
const dReal center55x[3] = {3,6, 0.44};
const dReal center56x[3] = {3,7, 0.44};
const dReal center57x[3] = {3,8, 0.44};
const dReal center58x[3] = {3,9, 0.44};
const dReal center59x[3] = {3,10, 0.44};
const dReal center60x[3] = {4,0, 0.44};
const dReal center61x[3] = {4,1, 0.44};
const dReal center62x[3] = {4,2, 0.44};
const dReal center63x[3] = {4,3, 0.44};
const dReal center64x[3] = {4,4, 0.44};
const dReal center65x[3] = {4,5, 0.44};
const dReal center66x[3] = {4,6, 0.44};
const dReal center67x[3] = {4,7, 0.44};
const dReal center68x[3] = {4,8, 0.44};
const dReal center69x[3] = {4,9, 0.44};





const dMatrix3 matrixStandard = { 1, 0, 0,
                                  0, 0, -1,
                                  0, 0, 0  };

const dMatrix3 matrixLean30 = { 1,         0,         0,
                            0,  0.500347, -0.865825,
                            0,  0.865825,  0.500347  };

const dMatrix3 matrixLean70 = { 1,         0,         0,
                            0,  0.29967, -0.98545,
                            0,  0.98545, 0.29967  };                            

const dMatrix3 matrixSideways = { 0, 0, 1,
                                  0, -1, 0,
                                  1, 0, 0  };

 const dMatrix3 matrixLean80 = { 1,         0,         0,
                            0,  00.0707372, -0.997495,
                            0,  0.997495, 0.0707372  };

 const dMatrix3 matrixMilk_CartonSideways = {0.992128,-0.119376,-0.0378241,
                         0,0.121999,0.989542,
                         0.0769628,0,0.028241};                                                            

                                  
                                          



//***********************************************


// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom[GPB];		// geometries representing this body

  // Trimesh only - double buffered matrices for 'last transform' setup
  dReal matrix_dblbuff[ 16 * 2 ];
  int last_matrix_index;

  int IDnumber;      //the number of the object
  //ADD fileName; ?         
  dReal center[3];    //the center x,y,z coordinates
  //ADD rotation
  //ADD color?
  int indCount;      //number of triangles (indices)
  int vertCount;     //number of vertices
  vector< vector<int> > indexDrawVec;
  vector<int> indexGeomVec;
  vector<float> vertexDrawVec;
  vector<float> vertexGeomVec;
  vector<float> centerOfMass;
};

class data // This if for center of mass calculations
{
public:
    float x1,y1,z1;
    float x2,y2,z2;
    float x3,y3,z3;
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
    contact[i].surface.mu = 1; //find the right balance //dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.0; //0.1;
    contact[i].surface.bounce_vel = 0.0; // 0.1;
    contact[i].surface.soft_cfm = 0.01;
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
  dAllocateODEDataForThread(dAllocateMaskAll);
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
    //cout << "counter = " << COUNT  << " \n" << endl;
    //cout << "THRESHHOLD : " << THRESHHOLD << " \n" << endl;
    //Check if object moved since initialization
    //cout<<"          X            Y            Z"  <<endl;
    //cout<<"Start: "<<startX<<", "<<startY<<", "<<startZ<<endl;
    //cout<<"  End: "<<endX<<", "<<endY<<", "<<endZ<<endl;
    //cout<<"Delta: "<<deltaX<<", "<<deltaY<<", "<<deltaZ<<endl;
    if ( deltaX > THRESHHOLD || deltaY > THRESHHOLD || deltaZ > THRESHHOLD){
        return false;
        //cout << "FALSE: Not in static equilibrium" << endl;
    } else{ 
        return true;
        //cout << "TRUE: Is in static equilibrium" << endl;
    }
}
 


//eventually it should take a list of nums/IDs use hashmap?
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
      //cout << "TRUE"<<endl;
      return true;
    } else{
      //cout << "FALSE"<<endl;
      return false;
    }
}

//eventually it should take a list of nums/IDs use hashmap?
static void destroyObjects(int num){
    //destroy the scene
    int k;
    for (int i=0; i<num; i++){     
        dBodyDestroy (obj[i].body);
        for (k=0; k < GPB; k++) {
            if (obj[i].geom[k]) dGeomDestroy (obj[i].geom[k]);
        }
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

  
  //cout<<object.center[0]<<","<< object.center[1]<<","<< object.center[2]<<endl;

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

  //gets the absolute bounding box
  dReal aabb[6];
  dGeomGetAABB (object.geom[0], aabb);
  //cout<<aabb[5]/2<<"\n";
  dReal maxZ = aabb[5]/2;

  object.body = dBodyCreate (world);
  
  //set positions
  //dBodySetPosition (object.body, object.center[0], object.center[1], object.center[2]);
  //dBodySetPosition (object.body, center[0], center[1], center[2]);
  //dBodySetPosition (object.body, object.center[0], object.center[1], object.center[2]);
  //dBodySetRotation (object.body,R);
  //~~~dBodySetPosition (object.body, center[0], center[1], maxZ/2);
  //~~~dRFromAxisAndAngle (R,0,0,0,0); //0,0,1, dRandReal()*10.0-5.0);
  
  dBodySetData (object.body,(void*)(size_t)i);


  //set body loop
  for (k=0; k < GPB; k++){
      if (object.geom[k]){
          dGeomSetBody(object.geom[k],object.body);
      }
  }
   
  //dGeomSetPosition(object.geom[0], m.c[0], m.c[1], m.c[2]);  <-- could be used for later? idk
  dGeomSetOffsetPosition(object.geom[0], -m.c[0],-m.c[1], -m.c[2]);
  dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]); 
  dBodySetMass(object.body,&m);

}






//To Do
void createObject (MyObject &object, int number, const dReal* center, const dMatrix3 R, char* filename){
  int SCALE =100;
  
  //Load the file
  objLoader *objData = new objLoader();
	objData->load(filename);

  //Get index and vertex count
  //string s(filename);
  //obj.fileName=filename;
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


  int i,j,k;
  dMass m;
  object.IDnumber = number; 

  
  //cout<<object.center[0]<<","<< object.center[1]<<","<< object.center[2]<<endl;




    //build Trimesh geom
  dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(new_tmdata, object.vertexGeomVec.data(), 3 * sizeof(float), 
	     object.vertCount, (int*)object.indexGeomVec.data(), object.indCount*3, 3 * sizeof(int));
  object.geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);
  dGeomSetData(object.geom[0], new_tmdata);        
  dMassSetTrimesh( &m, DENSITY, object.geom[0] );

  //gets the absolute bounding box
  dReal aabb[6];
  dGeomGetAABB (object.geom[0], aabb);
  //cout<<aabb[5]/2<<"\n";
  dReal maxZ = aabb[5]/2;

  object.body = dBodyCreate (world);
  //set positions
  //dBodySetPosition (object.body, object.center[0], object.center[1], object.center[2]);
  //dBodySetPosition (object.body, center[0], center[1], center[2]);

  object.center[0] = center[0];
  object.center[1] = center[1];
  object.center[2] = center[2];
  dBodySetPosition (object.body, object.center[0], object.center[1], object.center[2]);
  //dBodySetPosition (object.body, center[0], center[1], maxZ/2);
  //dRFromAxisAndAngle (R,0,0,0,0); //0,0,1, dRandReal()*10.0-5.0);
  dBodySetRotation (object.body,R);
  dBodySetData (object.body,(void*)(size_t)i);


  //set body loop
  for (k=0; k < GPB; k++){
      if (object.geom[k]){
          dGeomSetBody(object.geom[k],object.body);
      }
  }
  
  
  //dGeomSetPosition(object.geom[0], m.c[0], m.c[1], m.c[2]);  <-- could be used for later? idk
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


// called when a key pressed
static void command (int cmd)
{
    int i;
    if (cmd == 'j'){
        //add a new object
        i = num;
        num++;
        //createObject(obj[i], i, center4, matrixStandard, "red_mug.obj");
    }


    if (cmd == 'n') {
      //testing this
      //translateObject(obj[0], center1, matrixStandard);
    }


}




//destroys an object/ objects?
void destroyObject(){
      /* this may be useful code for later
        if (num < NUM) {
            i = num;
            num++;
        }
        else {
            i = nextobj;
            nextobj++;
            if (nextobj >= num) nextobj = 0;

            // destroy the body and geoms for slot i
            dBodyDestroy (obj[i].body);
            for (k=0; k < GPB; k++) {
                if (obj[i].geom[k]) dGeomDestroy (obj[i].geom[k]);
            }
            memset (&obj[i],0,sizeof(obj[i]));
        }
        */ 
}  



//*************DONT NEED***********************************************
// draw a geom
void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
  if (!g) return;
  if (!pos) pos = dGeomGetPosition (g);
  if (!R) R = dGeomGetRotation (g);

  int type = dGeomGetClass (g);

  
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths (g,sides);
    dsDrawBox (pos,R,sides);
  }
  else if (type == dSphereClass) {
    dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
  }
  else if (type == dCapsuleClass) {
    dReal radius,length;
    dGeomCapsuleGetParams (g,&radius,&length);
    dsDrawCapsule (pos,R,length,radius);
  }
  else if (type == dCylinderClass) {
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }

  if (show_aabb) {
    // draw the bounding box for this geom
    dReal aabb[6];
    dGeomGetAABB (g,aabb);
    dVector3 bbpos;
    for (int i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
    dVector3 bbsides;
    for (int j=0; j<3; j++) bbsides[j] = aabb[j*2+1] - aabb[j*2];
    dMatrix3 RI;
    dRSetIdentity (RI);
    dsSetColorAlpha (1,0,0,0.5);
    dsDrawBox (bbpos,RI,bbsides);
  }
}
//^^^^^^^^^^^^^^^^^DONT NEED^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^





// set previous transformation matrix for trimesh
void setCurrentTransform(dGeomID geom)
{
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




  if (counter == dsSTEP){
      dsStop();             
  }

  counter ++;


  //print out the final rotation matrix
  /*
  if (counter == 400){
    const dReal* rotation = dGeomGetRotation(obj[4].geom[0]);
    cout<<"const dMatrix3 matrixMilk_CartonSideways = {"<<rotation[0]<<","<<rotation[1]<<","<<rotation[2]<<","<<endl;
    cout<<"                         "<<rotation[3]<<","<<rotation[4]<<","<<rotation[5]<<","<<endl;
    cout<<"                         "<<rotation[6]<<","<<rotation[7]<<","<<rotation[8]<<"};"<<endl;
    
  }
  */

  


  //dsSetColor (0,0,2);
  dSpaceCollide (space,0,&nearCallback);


#if 1
  // What is this for??? - Bram
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



  #ifdef DRAW
  dsSetColor (1,1,0);
  dsSetTexture (DS_WOOD);
  #else
  #endif

  for (int i=0; i<num; i++) {
    for (int j=0; j < GPB; j++) {
      if (obj[i].geom[j]) {
        if (i==selected) {
          //dsSetColor (0,0.7,1);
        }
        else if (! dBodyIsEnabled (obj[i].body)) {
          //dsSetColor (1,0,0);
        }
        else {
          //dsSetColor (1,1,0);
        }
      
        if (dGeomGetClass(obj[i].geom[j]) == dTriMeshClass) {
          const dReal* Pos = dGeomGetPosition(obj[i].geom[j]);
          const dReal* Rot = dGeomGetRotation(obj[i].geom[j]);
  
        #ifdef DRAW
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
          #else
          #endif

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
  
        } else {
          //drawGeom (obj[i].geom[j],0,0,show_aabb);
        }
      }
    }
  }


}


Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
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


void setModels(std::vector<string> modelnames, std::vector<string> filenames){

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

    #ifdef DRAW
    counter=0;
    dsSTEP=step; //200;
    drawstuffsimLoop();
    #else
    for(int i = 0; i <= step; i++) {
      simLoop(0);
    }
    #endif
    return isValid(modelnames);
}


//To Do:
// acess everything from the hashmap..or think of something else
bool isValidScene(std::vector<string> modelnames, std::vector<Eigen::Affine3d> model_poses){
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
    
  
    if (! isStableStill(modelnames, 6)){
         return false;
    } else
    if (! isStableStill(modelnames, 14)){
         return false;
    } else
    if (! isStableStill(modelnames, 20)){
         return false;
    } else
    if (! isStableStill(modelnames, 110)){
         return false;
    } 
    
    else{
         return true;
    }
}



//To Do:
//do some more testing?
//try to make git work for you remember there is a pushing problem
//tell venkat the API is made, talk to him about what main() looks like
//   and if the create ODE is gonna be a problem
//make a header file and test it in another test_sceneValidator.cpp
//make a  cmake?




/*
void setUpODE(){
  
  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  dWorldSetCFM (world,1e-5);
  dCreatePlane (space,0,0,1,0);
  //memset (obj,0,sizeof(obj));
  dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
  dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);
}
*/


/*
void closeDownODE(){
  dThreadingImplementationShutdownProcessing(threading);
  dThreadingFreeThreadPool(pool);
  dWorldSetStepThreadingImplementation(world, NULL, NULL);
  dThreadingFreeImplementation(threading);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
}*/



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
  t =  (Eigen::Translation3d(Eigen::Vector3d(-2,0, 3.59)));
  Eigen::Affine3d a2 = (t*aq); 
  
  vector<Eigen::Affine3d> model_poses2 = {a2,b,c};  //teacup falling from the sky
// Here's all the input^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


  /* The following is what would go in main() */

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  dWorldSetCFM (world,1e-5);
  dCreatePlane (space,0,0,1,0); 
  dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
  dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);


  // Our two functions
  setModels(modelnames, filenames);

  int NUMBERofSCENES=1000;
  int SCENESperLOOP=2;
  int NUMBERofLOOPS = NUMBERofSCENES/SCENESperLOOP;
  startTime = chrono::steady_clock::now();
  for (int i =0; i <= NUMBERofLOOPS; i++) {

     isValidScene(modelnames, model_poses);
     isValidScene(modelnames, model_poses2);
     
  }
  endTime = chrono::steady_clock::now();
  auto diff = endTime - startTime;
  cout <<"Time: "<< chrono::duration <double, milli> (diff).count() << " ms" << endl;
  cout<< "Scenes: "<<NUMBERofSCENES<<endl;
  cout<<" Time per Scene: "<<(chrono::duration <double, milli> (diff).count())/NUMBERofSCENES<<"ms"<<endl;

  // destroy world
  dThreadingImplementationShutdownProcessing(threading);
  dThreadingFreeThreadPool(pool);
  dWorldSetStepThreadingImplementation(world, NULL, NULL);
  dThreadingFreeImplementation(threading);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();



  return 0;
}

