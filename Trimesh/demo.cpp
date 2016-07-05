//Joe Shepley CMY 2016
//Static Equilibrium tester in ODE physics engine
/****************************************************
//                   To Do
//
//comment out/ delete unneccesarly code?
//try to understand more what's happening with num and i
//think critically about using an array.whatevr instead of bunny.whatver
//instead of pressing 'j', just call the createObject function
//work on loading in the .obj file data
/**********************************************/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
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


//***********SOME POSITION CONSTANTS**************
const dReal center1[3] = {0,0,6}; // length of edges

//***********************************************


// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom[GPB];		// geometries representing this body

  // Trimesh only - double buffered matrices for 'last transform' setup
  dReal matrix_dblbuff[ 16 * 2 ];
  int last_matrix_index;

  int IDnumber;             //the number of the object
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
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
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
  printf ("To drop another object, press j:\n");
}



 



//To Do
void createObject (MyObject &object, int number, const dReal* center){
  int SCALE =100;

  //Load the file
  objLoader *objData = new objLoader();
	objData->load("milk_carton.obj");

  //Get index and vertex count
  object.indCount = objData->faceCount;
  object.vertCount = objData->vertexCount;

  //get the center of mass
  int numTriangles = objData->faceCount; 
	data triangles[numTriangles];
	// fill the triangles array with the data in the STL file
	for (int i =0; i < numTriangles; i ++){
		obj_face *o = objData->faceList[i];
		//cout<<o->vertex_index[0]<<"," <<o->vertex_index[1]<<","<< o->vertex_index[2] <<endl;
		triangles[i].x1=objData->vertexList[ o->vertex_index[0] ]->e[0] ;  //(int)(objData->vertexList[ o->vertex_index[0] ]->e[0]/ROUNDSCALE) * ROUNDSCALE ; used for scaling
		triangles[i].y1=objData->vertexList[ o->vertex_index[0] ]->e[1] ;
		triangles[i].z1=objData->vertexList[ o->vertex_index[0] ]->e[2] ;
		triangles[i].x2=objData->vertexList[ o->vertex_index[1] ]->e[0] ;
		triangles[i].y2=objData->vertexList[ o->vertex_index[1] ]->e[1] ;
		triangles[i].z2=objData->vertexList[ o->vertex_index[1] ]->e[2] ;
		triangles[i].x3=objData->vertexList[ o->vertex_index[2] ]->e[0] ;
		triangles[i].y3=objData->vertexList[ o->vertex_index[2] ]->e[1] ;
		triangles[i].z3=objData->vertexList[ o->vertex_index[2] ]->e[2] ;
	}
    
  double totalVolume = 0, currentVolume;
  double xCenter = 0, yCenter = 0, zCenter = 0;

  for (int i = 0; i < numTriangles; i++)
  {
      totalVolume += currentVolume = (triangles[i].x1*triangles[i].y2*triangles[i].z3 - triangles[i].x1*triangles[i].y3*triangles[i].z2 - triangles[i].x2*triangles[i].y1*triangles[i].z3 + triangles[i].x2*triangles[i].y3*triangles[i].z1 + triangles[i].x3*triangles[i].y1*triangles[i].z2 - triangles[i].x3*triangles[i].y2*triangles[i].z1) / 6;
      xCenter += ((triangles[i].x1 + triangles[i].x2 + triangles[i].x3) / 4) * currentVolume;
      yCenter += ((triangles[i].y1 + triangles[i].y2 + triangles[i].y3) / 4) * currentVolume;
      zCenter += ((triangles[i].z1 + triangles[i].z2 + triangles[i].z3) / 4) * currentVolume;
  }

  

  float COMX = (xCenter/totalVolume)/SCALE;
  float COMY = (yCenter/totalVolume)/SCALE;
  float COMZ = (zCenter/totalVolume)/SCALE;
  printf("My COM:    %.4f, %.4f, %.4f\n", COMX, COMY, COMZ);
  //object.centerOfMass = {COMX,COMY,COMZ};
  object.centerOfMass = {0,0,1};




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
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[0]/SCALE - object.centerOfMass[0]);
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[1]/SCALE - object.centerOfMass[1]);
		object.vertexDrawVec.push_back( objData->vertexList[i]->e[2]/SCALE - object.centerOfMass[2]);	
	}
  
  //Make 1D vector of vertices for geometry
  for(int i=0; i< vertCount ; i++){
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[0]/SCALE - object.centerOfMass[0]);
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[1]/SCALE - object.centerOfMass[1]);
		object.vertexGeomVec.push_back( objData->vertexList[i]->e[2]/SCALE - object.centerOfMass[2]);	
	}









  int i,j,k;
  dMass m;
  object.IDnumber = number; 
  object.center[0] = center[0];
  object.center[1] = center[1];
  object.center[2] = center[2];
  
  object.body = dBodyCreate (world);
  dMatrix3 R;
    //set your own positions
  dBodySetPosition (object.body, center1[0], center1[1], center1[2]);
  dRFromAxisAndAngle (R,0,0,0,7); //0,0,1, dRandReal()*10.0-5.0);
  dBodySetRotation (object.body,R);
  dBodySetData (object.body,(void*)(size_t)i);

  //build Trimesh
  dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(new_tmdata, object.vertexGeomVec.data(), 3 * sizeof(float), 
	     object.vertCount, (int*)object.indexGeomVec.data(), indexCount*3, 3 * sizeof(int));

//  dGeomTriMeshDataBuildSingle(new_tmdata, &Vertices[0], 3 * sizeof(float), VertexCount, 
//                              (dTriIndex*)&Indices[0], IndexCount, 3 * sizeof(dTriIndex));
  object.geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);
  // remember the mesh's dTriMeshDataID on its userdata for convenience.
  dGeomSetData(object.geom[0], new_tmdata);        
  dMassSetTrimesh( &m, DENSITY, object.geom[0] );
  printf("ODE's COM: %.4f, %.4f, %.4f\n", m.c[0], m.c[1], m.c[2]);
  dGeomSetPosition(object.geom[0], m.c[0], m.c[1], m.c[2]);
  dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]); 
  //dGeomSetPosition(object.geom[0], center1[0], center1[1], center1[2]);
  //dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);

  //set body loop
  for (k=0; k < GPB; k++){
      if (object.geom[k]){
          dGeomSetBody(object.geom[k],object.body);
      }
  }


  dBodySetMass(object.body,&m);

}



// called when a key pressed
static void command (int cmd)
{
    int i;
    if (cmd == 'j'){
        //add a new object
        i = num;
        num++;
        createObject(obj[i], i, center1);
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
  dsSetColor (0,0,2);
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

  if (!pause) dWorldQuickStep (world,0.05);

  for (int j = 0; j < dSpaceGetNumGeoms(space); j++){
	  dSpaceGetGeom(space, j);
  }

  // remove all contact joints
  dJointGroupEmpty (contactgroup);

  dsSetColor (1,1,0);
  dsSetTexture (DS_WOOD);
  for (int i=0; i<num; i++) {
    for (int j=0; j < GPB; j++) {
      if (obj[i].geom[j]) {
        if (i==selected) {
          dsSetColor (0,0.7,1);
        }
        else if (! dBodyIsEnabled (obj[i].body)) {
          dsSetColor (1,0,0);
        }
        else {
          dsSetColor (1,1,0);
        }
      
        if (dGeomGetClass(obj[i].geom[j]) == dTriMeshClass) {
          const dReal* Pos = dGeomGetPosition(obj[i].geom[j]);
          const dReal* Rot = dGeomGetRotation(obj[i].geom[j]);
  
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

/*
          for (int ii = 0; ii < obj[i].indCount; ii++) {
            const dReal v[9] = { // explicit conversion from float to dReal
              Vertices[obj[i].indexDrawVec[ii][0] * 3 + 0],
              Vertices[obj[i].indexDrawVec[ii][0] * 3 + 1],
              Vertices[obj[i].indexDrawVec[ii][0] * 3 + 2],
              Vertices[obj[i].indexDrawVec[ii][1] * 3 + 0],
              Vertices[obj[i].indexDrawVec[ii][1] * 3 + 1],
              Vertices[obj[i].indexDrawVec[ii][1] * 3 + 2],
              Vertices[obj[i].indexDrawVec[ii][2] * 3 + 0],
              Vertices[obj[i].indexDrawVec[ii][2] * 3 + 1],
              Vertices[obj[i].indexDrawVec[ii][2] * 3 + 2]
            };
            dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
          }
  */ 

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
          drawGeom (obj[i].geom[j],0,0,show_aabb);
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
  fn.stop = 0;
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

  //create an object
  num++;
  int i = 0;
  createObject(obj[i], i, center1);
  
  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

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
