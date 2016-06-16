// sample6.cpp  by Kosei Demura 2005-2011
// My web site is http://demura.net
// This program uses the Open Dynamics Engine (ODE) by Russell Smith.
// The ODE web site is http://ode.org/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox  dsDrawBoxD
#endif

static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
static int flag = 0; //<-don't need this?
dsFunctions fn;


const dReal   radius = 0.2;
const dReal   mass   = 1.0;
const dReal sides1[3] = {0.5,0.5,1.0}; // length of edges
const dReal sides2[3] = {0.5,0.5,1.0}; // length of edges
const dReal sides3[3] = {0.5,0.5,1.0}; // length of edges
const dReal sides4[3] = {0.5,0.5,1.0}; // length of edges

typedef struct {
  dBodyID body;
  dGeomID geom;
} MyObject;
MyObject ball, box1, box2, box3, box4 ;

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 10;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

  
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode = dContactBounce;
      contact[i].surface.mu   = dInfinity;
      contact[i].surface.bounce     = 0.0; // (0.0~1.0) restitution parameter
      contact[i].surface.bounce_vel = 0.0; // minimum incoming velocity for bounce
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
    }
  
}


static void simLoop (int pause)
{
  const dReal *pos,*R;
  const dReal *Box1pos, *Box1R;
  const dReal *Box2pos, *Box2R;
  const dReal *Box3pos, *Box3R;
  const dReal *Box4pos, *Box4R;
 
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,0.01);
  dJointGroupEmpty(contactgroup);

  // what is this flag stuff?
  if (flag == 0) dsSetColor(1.0, 0.0, 0.0);
  else           dsSetColor(0.0, 0.0, 1.0);
  //


  //draw sphere
  pos = dBodyGetPosition(ball.body);
  R   = dBodyGetRotation(ball.body);
  dsDrawSphere(pos,R,radius);

  //draw Box1
  Box1pos = dBodyGetPosition(box1.body);
  Box1R   = dBodyGetRotation(box1.body);
  dsDrawBox(Box1pos,Box1R,sides1);

   //draw Box2
  Box2pos = dBodyGetPosition(box2.body);
  Box2R   = dBodyGetRotation(box2.body);
  dsDrawBox(Box2pos,Box2R,sides2);

   //draw Box3
  Box3pos = dBodyGetPosition(box3.body);
  Box3R   = dBodyGetRotation(box3.body);
  dsDrawBox(Box3pos,Box3R,sides3);

   //draw Box4
  Box4pos = dBodyGetPosition(box4.body);
  Box4R   = dBodyGetRotation(box4.body);
  dsDrawBox(Box4pos,Box4R,sides4);


}

//set camera viewpoint
void start()
{
  static float xyz[3] = {0.0,-3.0,1.0};
  static float hpr[3] = {90.0,0.0,0.0};
  dsSetViewpoint (xyz,hpr);
}

void  prepDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.stop    = NULL;
  fn.path_to_textures = "/home/joeshepley/ode-0.13.1/drawstuff/textures";
}

int main (int argc, char *argv[])
{
  dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;
  dMass m1;

  prepDrawStuff();

  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world,0,0,-9.8);

  // Create a ground
  ground = dCreatePlane(space,0,0,1,0);

  // Create a ball
  ball.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetSphereTotal(&m1,mass,radius);
  dBodySetMass(ball.body,&m1);
  dBodySetPosition(ball.body, x0, y0, z0);
  ball.geom = dCreateSphere(space,radius);
  dGeomSetBody(ball.geom,ball.body);

 // Create a box1
  box1.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetBoxTotal(&m1,mass,sides1[0], sides1[1], sides1[2]);
  dBodySetMass(box1.body,&m1);
  dBodySetPosition(box1.body, x0, y0+0.1, z0+2);
  box1.geom = dCreateBox(space,sides1[0], sides1[1], sides1[2]);
  dGeomSetBody(box1.geom,box1.body);

  // Create a box2
  box2.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetBoxTotal(&m1,mass,sides2[0], sides2[1], sides2[2]);
  dBodySetMass(box2.body,&m1);
  dBodySetPosition(box2.body, x0, y0+1, z0+2);
  box2.geom = dCreateBox(space,sides2[0], sides2[1], sides2[2]);
  dGeomSetBody(box2.geom,box2.body);

  // Create a box3
  box3.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetBoxTotal(&m1,mass,sides3[0], sides3[1], sides3[2]);
  dBodySetMass(box3.body,&m1);
  dBodySetPosition(box3.body, x0-1, y0+0.1, z0+2);
  box3.geom = dCreateBox(space,sides3[0], sides3[1], sides3[2]);
  dGeomSetBody(box3.geom,box3.body);

  // Create a box4
  box4.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetBoxTotal(&m1,mass,sides4[0], sides4[1], sides4[2]);
  dBodySetMass(box4.body,&m1);
  dBodySetPosition(box4.body, x0+1, y0+0.1, z0+2);
  box4.geom = dCreateBox(space,sides4[0], sides4[1], sides4[2]);
  dGeomSetBody(box4.geom,box4.body);




  dsSimulationLoop (argc,argv,352,288,&fn);

  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
