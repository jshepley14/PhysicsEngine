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
#include "texturepath.h"
//#include "bunny_geom.h"
#include "milk_carton2.h"


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
  //rotation
  //color
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
  dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
  dBodySetRotation (object.body,R);
  dBodySetData (object.body,(void*)(size_t)i);

  dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(new_tmdata, &Vertices[0], 3 * sizeof(float), VertexCount, 
                              (dTriIndex*)&Indices[0], IndexCount, 3 * sizeof(dTriIndex));
  object.geom[0] = dCreateTriMesh(space, new_tmdata, 0, 0, 0);
  // remember the mesh's dTriMeshDataID on its userdata for convenience.
  dGeomSetData(object.geom[0], new_tmdata);        
  dMassSetTrimesh( &m, DENSITY, object.geom[0] );
  printf("mass at %f %f %f\n", m.c[0], m.c[1], m.c[2]);
  dGeomSetPosition(object.geom[0], m.c[0], m.c[1], m.c[2]);
  dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);

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
          dTriIndex* Indices = (dTriIndex*)::Indices;

          const dReal* Pos = dGeomGetPosition(obj[i].geom[j]);
          const dReal* Rot = dGeomGetRotation(obj[i].geom[j]);
        
          for (int ii = 0; ii < IndexCount / 3; ii++) {
            const dReal v[9] = { // explicit conversion from float to dReal
              Vertices[Indices[ii * 3 + 0] * 3 + 0],
              Vertices[Indices[ii * 3 + 0] * 3 + 1],
              Vertices[Indices[ii * 3 + 0] * 3 + 2],
              Vertices[Indices[ii * 3 + 1] * 3 + 0],
              Vertices[Indices[ii * 3 + 1] * 3 + 1],
              Vertices[Indices[ii * 3 + 1] * 3 + 2],
              Vertices[Indices[ii * 3 + 2] * 3 + 0],
              Vertices[Indices[ii * 3 + 2] * 3 + 1],
              Vertices[Indices[ii * 3 + 2] * 3 + 2]
            };
            dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
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
