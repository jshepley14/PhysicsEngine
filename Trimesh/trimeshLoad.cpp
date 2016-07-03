// bunny.cpp 
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
//#include "cube.h"


//used for loading file
#include <stdio.h>
#include "objLoader.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;



#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawLine dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif

#define DENSITY (50.0)		// density of all objects
#define MAX_CONTACTS 4// 32 //change to 64?  // maximum number of contact points per body

typedef struct MyObject {
  dBodyID body;	       
  dGeomID geom;	
  dReal   m;         //mass
  dReal   lx,ly,lz;  //lengths
  int indCount;      //number of triangles (indices)
  int vertCount;     //number of vertices
  vector< vector<int> > indexDrawVec;
  vector<int> indexGeomVec;
  vector<float> vertexDrawVec;
  vector<float> vertexGeomVec;
  vector<float> centerOfMass;
} MyObject;



static dsFunctions fn;
static dWorldID world;
static dSpaceID space;
static MyObject bunny, plate;
static dGeomID  ground;
static dJointID fixedplate;
static dJointGroupID contactgroup;
static dTriMeshDataID TriData;  

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];  
  for (int i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu         = dInfinity; //1.0?
    contact[i].surface.mu2         = 0;
    contact[i].surface.bounce     = 0;
    contact[i].surface.bounce_vel = 0;
    contact[i].surface.soft_cfm   = 0.01;
    //contact[i].surface.soft_erp   = 0.95;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity(RI);
    for (int i=0; i<numc; i++) {
      dJointID c = dJointCreateContact(world,contactgroup,contact+i);
      dJointAttach(c,b1,b2);
    }
  }
}

static void start()
{
  static float xyz[3] = {  3.1640f,  -2.3079f, 1.7600f};
  static float hpr[3] = {125.5000f, -17.0000f, 0.0000f};
  dsSetViewpoint (xyz,hpr);
}

void drawPlate() 
{
  dsSetColor(0.0, 0.0, 1.1);
  dVector3 sides2;
  dGeomBoxGetLengths(plate.geom,sides2);
  dsDrawBoxD(dGeomGetPosition(plate.geom),
	     dGeomGetRotation(plate.geom),sides2);  
}

void drawBunny()
{
  dsSetColor(1.1,1.1,0.0);
  for (int i = 0; i < bunny.indCount ; i++){
    const dReal v[9] = { 
      bunny.vertexDrawVec[bunny.indexDrawVec[i][0] * 3 + 0],
      bunny.vertexDrawVec[bunny.indexDrawVec[i][0] * 3 + 1],
      bunny.vertexDrawVec[bunny.indexDrawVec[i][0] * 3 + 2],
      bunny.vertexDrawVec[bunny.indexDrawVec[i][1] * 3 + 0],
      bunny.vertexDrawVec[bunny.indexDrawVec[i][1] * 3 + 1],
      bunny.vertexDrawVec[bunny.indexDrawVec[i][1] * 3 + 2],
      bunny.vertexDrawVec[bunny.indexDrawVec[i][2] * 3 + 0],
      bunny.vertexDrawVec[bunny.indexDrawVec[i][2] * 3 + 1],
      bunny.vertexDrawVec[bunny.indexDrawVec[i][2] * 3 + 2]
     };   
     dsDrawTriangle(dGeomGetPosition(bunny.geom),
	     dGeomGetRotation(bunny.geom), &v[0], &v[3], &v[6], 1);
  }
}



static void simLoop(int pause)
{
  dSpaceCollide(space,0,&nearCallback);

  if (!pause) dWorldStep(world,0.005);
  dJointGroupEmpty (contactgroup);
 
  drawPlate();
  drawBunny(); 
}

static void makePlate() {
  dMass mass;
  dReal x0 = -5.0, y0 = 0.0, z0 = 0.5;

  plate.lx = 2.0;  plate.ly   =  2.0;
  plate.lz = 0.01; plate.m    = 10.0;
  plate.body  = dBodyCreate(world); 
  
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, plate.m, plate.lx, plate.ly, plate.lz);
  dBodySetMass(plate.body,&mass);            
  dBodySetPosition(plate.body, x0, y0, z0 + plate.lz/2); 

  plate.geom = dCreateBox(space,plate.lx,plate.ly,plate.lz);
  dGeomSetBody(plate.geom,plate.body);     

  fixedplate = dJointCreateFixed(world,0);
 	dJointAttach(fixedplate,plate.body,0); 
  dJointSetFixed(fixedplate);
} 


void makeBunny() 
{


  //Load the file
  objLoader *objData = new objLoader();
	objData->load("milk_carton.obj");

  //Get index and vertex count
  bunny.indCount = objData->faceCount;
  bunny.vertCount = objData->vertexCount;

  //get the centerOfMass
  bunny.centerOfMass = {0,0,1};

  //Make 2D vector of indices for drawing 
  int indexCount = bunny.indCount; 
	for(int i=0; i<indexCount; i++)
	{	vector<int> temp_vec;
		temp_vec.push_back((objData->faceList[i])->vertex_index[0]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[1]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[2]);
		bunny.indexDrawVec.push_back(temp_vec);
	}

  //Make 1D vector of indices for geometry
  for(int i=0; i< indexCount; i++)
  {	
    bunny.indexGeomVec.push_back((objData->faceList[i])->vertex_index[0]);
    bunny.indexGeomVec.push_back((objData->faceList[i])->vertex_index[1]);
    bunny.indexGeomVec.push_back((objData->faceList[i])->vertex_index[2]);   
  }

  //Make 1D vector of vertices for drawing
  int SCALE =100;
  int vertCount =  bunny.vertCount;
	for(int i=0; i< vertCount ; i++){
		bunny.vertexDrawVec.push_back( objData->vertexList[i]->e[0]/SCALE);
		bunny.vertexDrawVec.push_back( objData->vertexList[i]->e[1]/SCALE);
		bunny.vertexDrawVec.push_back( objData->vertexList[i]->e[2]/SCALE);	
	}
  
  //Make 1D vector of vertices for geometry
  for(int i=0; i< vertCount ; i++){
		bunny.vertexGeomVec.push_back( objData->vertexList[i]->e[0]/SCALE -bunny.centerOfMass[0]);
		bunny.vertexGeomVec.push_back( objData->vertexList[i]->e[1]/SCALE -bunny.centerOfMass[1]);
		bunny.vertexGeomVec.push_back( objData->vertexList[i]->e[2]/SCALE -bunny.centerOfMass[2]);	
	}

  //Build trimesh
  TriData = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(TriData, bunny.vertexGeomVec.data(), 3 * sizeof(float), 
	     bunny.vertCount, (int*)bunny.indexGeomVec.data(), indexCount*3, 3 * sizeof(int));
 


  //experimenting
  dReal x = 0, y = 0, z = 0;
  dReal mx = 0, my = 0, mz = 1;
  bunny.body = dBodyCreate(world);
  dBodySetPosition(bunny.body, mx, my, mz);
  dMatrix3 Rotation;
  dRFromAxisAndAngle(Rotation, 1, 1, 0, M_PI / 2);
  dBodySetRotation(bunny.body, Rotation);
  dMass mass;
  bunny.geom = dCreateTriMesh(space, TriData, 0, 0, 0);
  dGeomSetBody(bunny.geom, bunny.body);
  dMassSetTrimesh( &mass, DENSITY, bunny.geom );
  //dBodySetMass(bunny.body, &mass);
  dGeomSetPosition(bunny.geom, mx, my, mz);
  dMassTranslate(&mass, mx, my, mz);


 /*
  //Length, Position, Mass, Rotation
  dReal x0 = 0, y0 = 5, z0 = 1;
	bunny.m  = .10;
	bunny.lx = 0; bunny.ly = 5; bunny.lz = 1.0;
  bunny.body = dBodyCreate(world);
  bunny.geom = dCreateTriMesh(space, TriData, 0, 0, 0);
  dGeomSetBody(bunny.geom, bunny.body);
  dMass mass;
  dMassSetBoxTotal(&mass, bunny.m, bunny.lx, bunny.ly, bunny.lz);
  dBodySetMass(bunny.body, &mass);
  dBodySetPosition(bunny.body, x0, y0, z0);
  dMatrix3 Rotation;
  dRFromAxisAndAngle(Rotation, 1, 1, 0, M_PI / 2);
  dBodySetRotation(bunny.body, Rotation);
 */


}


void setDrawStuff()
{
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.path_to_textures = "/home/joeshepley/ode-0.13.1/drawstuff/textures";
}

int main(int argc, char **argv) 
{

  dInitODE();
  setDrawStuff();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground = dCreatePlane(space,0,0,1,0);
  dWorldSetGravity(world,0,0,-9.8); 
  dWorldSetERP(world, 1.0);
  dWorldSetCFM(world, 1e-4);
  
  makePlate();
  makeBunny();
  
  dsSimulationLoop(argc,argv,640,480,&fn);

  dJointGroupDestroy(contactgroup);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  return 0;
}
