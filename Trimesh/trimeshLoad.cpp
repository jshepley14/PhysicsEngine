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

#define MAX_CONTACTS 32 

typedef struct MyObject {
  dBodyID body;	       
  dGeomID geom;	
  dReal   m; 
  dReal   lx,ly,lz;
  int indCount;
  int vertCount;
  vector< vector<int> > indexVec;
  vector<float> dvertVec;
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
    contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP;
    contact[i].surface.mu         = 1.0; 
    contact[i].surface.bounce     = 0.1;
    contact[i].surface.soft_cfm   = 0.0001;
    contact[i].surface.soft_erp   = 0.95;
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
      bunny.dvertVec[bunny.indexVec[i][0] * 3 + 0],
      bunny.dvertVec[bunny.indexVec[i][0] * 3 + 1],
      bunny.dvertVec[bunny.indexVec[i][0] * 3 + 2],
      bunny.dvertVec[bunny.indexVec[i][1] * 3 + 0],
      bunny.dvertVec[bunny.indexVec[i][1] * 3 + 1],
      bunny.dvertVec[bunny.indexVec[i][1] * 3 + 2],
      bunny.dvertVec[bunny.indexVec[i][2] * 3 + 0],
      bunny.dvertVec[bunny.indexVec[i][2] * 3 + 1],
      bunny.dvertVec[bunny.indexVec[i][2] * 3 + 2]
     };   
     dsDrawTriangle(dGeomGetPosition(bunny.geom),
	     dGeomGetRotation(bunny.geom), &v[0], &v[3], &v[6], 1);
  }
}

/*
void drawBunny()
{
  dsSetColor(1.1,1.1,0.0);

  for (int i = 0; i < IndexCount / 3; i++){
    const dReal v[9] = { 
      Vertices[Indices[i][0] * 3 + 0],
      Vertices[Indices[i][0] * 3 + 1],
      Vertices[Indices[i][0] * 3 + 2],
      Vertices[Indices[i][1] * 3 + 0],
      Vertices[Indices[i][1] * 3 + 1],
      Vertices[Indices[i][1] * 3 + 2],
      Vertices[Indices[i][2] * 3 + 0],
      Vertices[Indices[i][2] * 3 + 1],
      Vertices[Indices[i][2] * 3 + 2]
     };   
     dsDrawTriangle(dGeomGetPosition(bunny.geom),
	     dGeomGetRotation(bunny.geom), &v[0], &v[3], &v[6], 1);
  }
}
*/


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
  dReal x0 = 0.0, y0 = 0.0, z0 = 0.5;

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
  //load the file
  objLoader *objData = new objLoader();
	objData->load("cube.obj");
  bunny.indCount = objData->faceCount;
  bunny.vertCount = objData->vertexCount;

  //make 2D vector of indices for drawing 
  int indexCount = bunny.indCount; 
	for(int i=0; i<indexCount; i++)
	{	vector<int> temp_vec;
		temp_vec.push_back((objData->faceList[i])->vertex_index[0]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[1]);
		temp_vec.push_back((objData->faceList[i])->vertex_index[2]);
		bunny.indexVec.push_back(temp_vec);
	}

  //make 1D vector of indices for geometry
  static vector<int> dataVec;
  for(int i=0; i< indexCount; i++)
  {	
    dataVec.push_back((objData->faceList[i])->vertex_index[0]);
    dataVec.push_back((objData->faceList[i])->vertex_index[1]);
    dataVec.push_back((objData->faceList[i])->vertex_index[2]);   
  }

  //make 1D vector of vertices for drawing
  int SCALE =100;
  int vertCount =  bunny.vertCount;
	for(int i=0; i< vertCount ; i++){
		bunny.dvertVec.push_back( objData->vertexList[i]->e[0]/SCALE);
		bunny.dvertVec.push_back( objData->vertexList[i]->e[1]/SCALE);
		bunny.dvertVec.push_back( objData->vertexList[i]->e[2]/SCALE);	
	}

  //make 1D vector of vertices for geometry
	static vector<float> vertVec;
	for(int i=0; i< vertCount ; i++){
		vertVec.push_back( objData->vertexList[i]->e[0]/SCALE);
		vertVec.push_back( objData->vertexList[i]->e[1]/SCALE);
		vertVec.push_back( objData->vertexList[i]->e[2]/SCALE);	
	}

//change to double??

  dReal x0 = 0, y0 = 0, z0 = 2.5;
	bunny.m  = 10.0;
	bunny.lx = 2.0; bunny.ly = 0.5; bunny.lz = 1.0;
  
  //trimesh stuff
  TriData = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(TriData, vertVec.data(), 3 * sizeof(float), 
	     bunny.vertCount, (int*)dataVec.data(), indexCount*3, 3 * sizeof(int));
 
 //(int*)Indices


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
