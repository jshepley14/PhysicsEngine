// sample14.cpp  by Kosei Demura 2005-2011
// My web site is http://demura.net
// This program uses the Open Dynamics Engine (ODE) by Russell Smith.
// The ODE web site is http://ode.org/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#endif

// #define DRAW 　先頭のコメントを外すと描画します。

static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
dsFunctions fn;


const dReal   radius = 0.2;
const dReal   mass   = 1.0;

typedef struct {
  dBodyID body;
  dGeomID geom;
} MyObject;
MyObject ball;

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 10;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

  if (isGround)  {
		for (int i = 0; i < n; i++) {
      contact[i].surface.mode = dContactBounce;
      contact[i].surface.mu   = dInfinity;
      contact[i].surface.bounce     = 1.0; // (0.0~1.0) restitution parameter
      contact[i].surface.bounce_vel = 0.0; // minimum incoming velocity for bounce
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
    }
  }
}


static void simLoop (int pause)
{
  static long step = 0;
  const dReal *pos,*R;

  dSpaceCollide(space,0,&nearCallback);

  dWorldStep(world,0.01);

  dJointGroupEmpty(contactgroup);


  pos = dBodyGetPosition(ball.body);
  R   = dBodyGetRotation(ball.body);

#ifdef DRAW
  dsSetColor(1.0, 0.0, 0.0);
  dsDrawSphere(pos,R,radius);
  printf("%5d steps x=%.3f y=%.3f z=%.3f \n",step++,pos[0],pos[1],pos[2]);
#else
  printf("%5d steps x=%.3f y=%.3f z=%.3f \n",step++,pos[0],pos[1],pos[2]);
#endif
}

void start()
{
  static float xyz[3] = {0.0,-3.5,1.0};
  static float hpr[3] = {90.0,0.0,0.0};
  dsSetViewpoint (xyz,hpr);
  dsSetSphereQuality(3);
}

void  prepDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.stop    = NULL;
  fn.path_to_textures = "../../drawstuff/textures";
}

int main (int argc, char *argv[])
{
  dReal x0 = 0.0, y0 = 0.0, z0 = 3.0;
  dMass m1;

  prepDrawStuff();

  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world,0,0,-0.5);

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

#ifdef DRAW
  dsSimulationLoop (argc,argv,600,480,&fn);
#else
  while (1) {
    simLoop(0);
  }
#endif

  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
