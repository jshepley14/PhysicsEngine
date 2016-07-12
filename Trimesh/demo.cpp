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

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>  //used for timing code
#include <stdio.h>
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

#define DRAW  //used to switch on or off the drawing of the scene
static int HEIGHT =500;
static int WIDTH = 1000;




//Some COUNT, TIMESTEP pairs: 6, 0.1  and 20, 0.05
//static equilibrium constants
static int GlobalCounter = 0;
static int counter = 0;          //iterator which will reach COUNT
static int dsSTEP=100;
static int STEP = 20; //20?            //how long until we want to wait to check satic equlibrium?
static double THRESHHOLD = 0.8;  //how much movement is allowed
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
const dReal center01[3] = {0,0,3.1};
const dReal center02[3] = {-2,0,1.84};
const dReal center03[3] = {1.4,0,0.65};
const dReal center04[3] = {-2,0,2.82};
const dReal center05[3] = {1.32,0.3,0.7};

const dReal center10[3] = {2,0,0.44}; // 
const dReal center11[3] = {3,0,0.44}; //
const dReal center12[3] = {-2,0,2.73}; //


const dReal center20[3] = {-2,0,0.66}; //


const dReal center4[3] = {0,0,4}; //
const dReal center5[3] = {0,0,0.63}; //
const dReal center6[3] = {0,-4,0.7}; //


const dReal center1_2[3] = {1.0,1.0,3.0}; 
const dReal center2_2[3] = {2.0,2.0,3.0}; 
const dReal center3_2[3] = {3.0,3.0,3.0}; 
const dReal center4_2[3] = {4.0,4.0,3.0}; 

const dReal center30[3] = {0,0, 1.1}; 
const dReal center31[3] = {0,0, 2.1};


const dMatrix3 matrixStandard = { 1, 0, 0,
                                  0, 0, 0,
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
static bool isValidScene(int num){
    bool stable = true;
    for (int i=0; i<num; i++){ 
       if (!inStaticEquilibrium(obj[i]) ){
          //cout << "FALSE: Not in static equilibrium" << endl;
          stable = false;
          break;        
       } 
    } 
    if (stable == true){
      //cout << "TRUE: Is in static equilibrium......................." << endl;
      cout << "TRUE"<<endl;
    } else{
      //cout << "FALSE: Not in static equilibrium......................" << endl;
      cout << "FALSE"<<endl;
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


int main (int argc, char **argv)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = NULL;
  fn.path_to_textures = "/home/joeshepley/ode-0.13.1/drawstuff/textures";

  // create world
  dInitODE2(0);
  world = dWorldCreate();
 
  space = dSimpleSpaceCreate(0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  dWorldSetCFM (world,1e-5);
  dCreatePlane (space,0,0,1,0);
  memset (obj,0,sizeof(obj));
  
  dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
  dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
  dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
  // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
  dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);

  
  /*
  / void setModels(std:vector<string> modelnames, std:vector<string> filepath);  
  */
 
  num = 4;   //number of elements.  must specify!!!!!
  int i = 0;
  setObject(obj[i], i,  "red_mug.obj");
  setObject(obj[1], 10,  "teacup.obj");
  setObject(obj[2], 10,  "red_mug.obj");
  setObject(obj[3], 10,  "milk_carton.obj");
  //set the new scene     
  makeObject(obj[0]);     
  makeObject(obj[1]);    
  makeObject(obj[2]);    
  makeObject(obj[3]);

  
  

 
  /*
  /   bool isStable(std:vector<string> model_IDs, std:vector<Eigen:: Affine3d> model-poses)
  */

  //set the scene
  translateObject(obj[0], center02, matrixStandard);
  translateObject(obj[1], center12, matrixStandard);
  translateObject(obj[2], center20, matrixStandard);
  translateObject(obj[3], center30, matrixMilk_CartonSideways);
  //run simulation
  #ifdef DRAW
  counter=0;
  dsSTEP=200;
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
  #else
  STEP = 20;
  for(int i = 0; i <= STEP; i++) {
    simLoop(0);
  }
  #endif
  //check if valid
  cout<<"Scene 1: ";
  isValidScene(num);

  //set the scene
  translateObject(obj[0], center02, matrixStandard);
  translateObject(obj[1], center10, matrixStandard);
  translateObject(obj[2], center20, matrixStandard);
  translateObject(obj[3], center30, matrixMilk_CartonSideways);
  //run simulation
  #ifdef DRAW
  counter=0;
  dsSTEP=200;
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
  #else
  STEP = 20;
  for(int i = 0; i <= STEP; i++) {
    simLoop(0);
  }
  #endif
  //check if valid
  cout<<"Scene 2: ";
  isValidScene(num);
  

  //set the scene
  translateObject(obj[0], center05, matrixLean70);
  translateObject(obj[1], center11, matrixStandard);
  translateObject(obj[2], center20, matrixStandard);
  translateObject(obj[3], center30, matrixMilk_CartonSideways);
  //run simulation
  #ifdef DRAW
  counter=0;
  dsSTEP=200;
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
  #else
  STEP = 12;
  for(int i = 0; i <= STEP; i++) {
    simLoop(0);
  }
  #endif
  //check if valid
  cout<<"Scene 3: ";
  isValidScene(num);

  //set the scene
  translateObject(obj[0], center03, matrixLean30);
  translateObject(obj[1], center11, matrixStandard);
  translateObject(obj[2], center20, matrixStandard);
  translateObject(obj[3], center30, matrixStandard);
  //run simulation
  #ifdef DRAW
  counter=0;
  dsSTEP=300;
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
  #else
  STEP = 12;
  for(int i = 0; i <= STEP; i++) {
    simLoop(0);
  }
  #endif
  //check if valid
  cout<<"Scene 4: ";
  isValidScene(num);

  //set the scene
  translateObject(obj[0], center01, matrixStandard);
  translateObject(obj[1], center10, matrixStandard);
  translateObject(obj[2], center20, matrixStandard);
  translateObject(obj[3], center30, matrixStandard);
  //run simulation
  #ifdef DRAW
  counter=0;
  dsSTEP=200;
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
  #else
  STEP = 12;
  for(int i = 0; i <= STEP; i++) {
    simLoop(0);
  }
  #endif
  //check if valid
  cout<<"Scene 5: ";
  isValidScene(num);

 
   //set the scene
  translateObject(obj[0], center00, matrixStandard);
  translateObject(obj[1], center10, matrixStandard);
  translateObject(obj[2], center20, matrixStandard);
  translateObject(obj[3], center30, matrixStandard);
  //run simulation
  #ifdef DRAW
  counter=0;
  dsSTEP=200;
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
  #else
  STEP = 200;
  for(int i = 0; i <= STEP; i++) {
    simLoop(0);
  }
  #endif
  //check if valid
  cout<<"Scene 6: ";
  isValidScene(num);

 

  //set the scene
  translateObject(obj[0], center00, matrixStandard);
  translateObject(obj[1], center10, matrixStandard);
  translateObject(obj[2], center20, matrixStandard);
  translateObject(obj[3], center31, matrixStandard);
  //run simulation
  #ifdef DRAW
  counter=0;
  dsSTEP=200;
  dsSimulationLoop (argc,argv,WIDTH,HEIGHT,&fn);
  #else
  STEP = 12;
  for(int i = 0; i <= STEP; i++) {
    simLoop(0);
  }
  #endif
  //check if valid
  cout<<"Scene 7: ";
  isValidScene(num);

   
  



  

  







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

//Venkat's pseudo-code
/*
main() {
SetScene(...)
for (500 counts) {
CheckSceneValid(scene)
}
SetScene(...)
for (500 counts) {
CheckSceneValid(...)
}
}

CheckSceneValid(scene) {
  bool EQUAl = ForwardSimulateScene(scene, NUM_TIMESTEPS, forward_simulated_scene);
  return equAL;
}
*/