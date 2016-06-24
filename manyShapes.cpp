// sample6.cpp  by Kosei Demura 2005-2011
// My web site is http://demura.net
// This program uses the Open Dynamics Engine (ODE) by Russell Smith.
// The ODE web site is http://ode.org/

#include <ode/ode.h>   //physics library
#include <drawstuff/drawstuff.h>  //graphics library
#include <string.h>  
#include <iostream>  //cout
#include <cmath>   //math functions
#include <time.h>  //used for timing code
#include <chrono>  //used for timing code

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox  dsDrawBoxD
#endif

#define DRAW  //used to switch on or off the drawing of the scene

using namespace std;

//function declarations
static void inStaticEquilibrium(double startZ, double endZ );
static void printEndpositions(const dReal *Box1pos);


//static equilibrium constants
static int counter = 0;          //iterator which will reach COUNT
static int COUNT = 30;           //how long until we want to wait to check satic equlibrium?
static double THRESHHOLD = 0.1;  //how much movement is allowed
static double GRAVITY = -19.8;   
static double TIMESTEP = 0.02;    //Time step originally was at 0.01, but we may
                                 //be able to increase this making calculations faster
                                 //most efficient would prob be 0.01 ≤ TIMESTEP ≤ 0.03
//timer variables
//need to add -std=c++11 right after g++ to compile
static chrono::steady_clock::time_point startTime, endTime;

//ID declarations
static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
static int flag = 0; //<-don't need this?
dsFunctions fn;

//object construction variables
const dReal   radius = 0.2; //1.0; //sphere
const dReal   mass   = 1.0;



//Box side length declarations
double B1Lengthx, B1Lengthy, B1Lengthz; //Box1
double B2Lengthx, B2Lengthy, B2Lengthz; //Box2 
double B3Lengthx, B3Lengthy, B3Lengthz; //Box3
double B4Lengthx, B4Lengthy, B4Lengthz; //Box4
const dReal sides1[3] = {B1Lengthx=0.5,B1Lengthy=0.5,B1Lengthz=1.0}; // length of edges
const dReal sides2[3] = {B2Lengthx=0.5,B2Lengthy=0.5,B2Lengthz=1.0}; // length of edges
const dReal sides3[3] = {B3Lengthx=0.5,B3Lengthy=0.5,B3Lengthz=1.0}; // length of edges
const dReal sides4[3] = {B4Lengthx=0.5,B4Lengthy=0.5,B4Lengthz=1.0}; // length of edges

//Position variables
dReal S1x, S1y, S1z;  //Sphere1
dReal B1x, B1y, B1z; //Box1
dReal B2x, B2y, B2z; //Box2 
dReal B3x, B3y, B3z; //Box3
dReal B4x, B4y, B4z; //Box4



//....................................................................................................
//TITLE: leaning block falling.  Example of configuration that requires COUNT to be large OR small Threshold 

//Position declarations
const dReal centerSphr[3] = {S1x=-2.0,S1y=0,S1z=0.2}; // length of edges
const dReal center1[3] = {B1x=0.0,B1y=0,B1z=0.55}; // length of edges
const dReal center2[3] = {B2x=0,B2y=1,B2z=0.5}; // length of edges
const dReal center3[3] = {B3x=0,B3y=2,B3z=0.5}; // length of edges
const dReal center4[3] = {B4x=0,B4y=3,B4z=0.5}; // length of edges
//Rotation declarations
const dMatrix3 B1matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0.5  }; 
const dMatrix3 B2matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B3matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B4matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 




//********************************************************************************************8*******************
//object declarations
typedef struct {
  dBodyID body;
  dGeomID geom;
} MyObject;
MyObject ball, box1, box2, box3, box4 ;


static void createBox(MyObject &box, const dReal* center, const dReal* sides, const dMatrix3 MatrixR)
{
    dMass m1;
    dReal mass = 1.0; //[kg]

    box.body = dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetBoxTotal(&m1,mass,sides[0], sides[1], sides[2]);
    dBodySetMass(box.body,&m1);
    dBodySetPosition(box.body, center[0], center[1], center[2]);
    dBodySetRotation (box.body, MatrixR);
    box.geom = dCreateBox(space,sides[0], sides[1], sides[2]);
    dGeomSetBody(box.geom,box.body);    
}

static void drawBox( MyObject &box, const dReal* sides){
    const dReal *pos1,*R1;
    pos1 = dBodyGetPosition(box.body);
    R1   = dBodyGetRotation(box.body);

    #ifdef DRAW
    dsDrawBox(pos1,R1,sides);
    #else
    #endif
}



//This function checks for static equilibrium
/* Later we could turn this into a bool return. Additionally, one
*  could feed in dReal types instead of doubles. One would also
*  want to be able to feed in an arbitrary number of object data.
*
*/
static bool inStaticEquilibrium(const dReal* center, MyObject &object){

    double startX = center[0];
    double startY = center[1];
    double startZ = center[2];
    double endX = dBodyGetPosition(object.body)[0];
    double endY = dBodyGetPosition(object.body)[1];
    double endZ = dBodyGetPosition(object.body)[2];
    double deltaX = std::abs(startX - endX);
    double deltaY = std::abs(startY - endY);
    double deltaZ = std::abs(startZ - endZ);
    
    //Check if object moved since initialization
    if ( deltaX > THRESHHOLD || deltaY > THRESHHOLD || deltaZ > THRESHHOLD){
        cout << "NOT in static equilibrium" << endl;
        return false;
    } else{ 
        cout << "YES: Is in static equilibrium" << endl;
        return true;
    }
    cout<<"          X            Y            Z"  <<endl;
    cout<<"Start: "<<startX<<", "<<startY<<", "<<startZ<<endl;
    cout<<"  End: "<<endX<<", "<<endY<<", "<<endZ<<endl;
    cout<<"Delta: "<<deltaX<<", "<<deltaY<<", "<<deltaZ<<endl;
}



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


  //startTime = chrono::steady_clock::now();

  //keep working on way to confirm that timer is working
  if (counter == 0){
     startTime = chrono::steady_clock::now();
  }

  const dReal *pos,*R;
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,TIMESTEP);
  dJointGroupEmpty(contactgroup);
  pos = dBodyGetPosition(ball.body);
  R   = dBodyGetRotation(ball.body);
  

 //draw stuff
#ifdef DRAW
  dsDrawSphere(pos,R,radius);
  dsSetColor(1,0,0);
  drawBox(box1, sides1);
  dsSetColor(0,1,0);
  drawBox(box2, sides2);
  dsSetColor(0,0,1);
  drawBox(box3, sides3);
  dsSetColor(0,12,200);
  drawBox(box4, sides4);   
#else
  drawBox(box1, sides1); 
  drawBox(box2, sides2);
  drawBox(box3, sides3);
  drawBox(box4, sides4);
  //print the positions and time step or whatever when your not drawing
  //printf("%5d steps x=%.3f y=%.3f z=%.3f \n",(int)step++,pos[0],pos[1],pos[2]);
#endif

  
  /*
   *  Static Equilibrium Detection
   *  
  */ 
  
  if (counter == COUNT){
    if(
      !inStaticEquilibrium(center1, box1) || //feed it in the Zinital and Zfinal coordinates of Box4
      !inStaticEquilibrium(center2, box2) ||
      !inStaticEquilibrium(center3, box3) ||
      !inStaticEquilibrium(center4, box4) ||
      !inStaticEquilibrium(centerSphr, ball) ){
        cout<<"NOT in static equilibrium"<<endl;
      }else{
        cout<<"YES in static equilibrium"<<endl;
      }

      //end chrono timer
      endTime = chrono::steady_clock::now();
      auto diff = endTime - startTime;
      cout << chrono::duration <double, milli> (diff).count() << " ms" << endl;
      //seconds
      //chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(endTime - startTime);
      //cout << "It took me " << time_span.count() << " seconds.";


    }

    //Dont forget about the counter!
    counter++;   

}






static void printEndpositions(const dReal *Box1pos){

}


//set camera viewpoint
void start()
{

  // front viewpoint
  static float xyz[3] = {0.0,-3.0,1.0};
  static float hpr[3] = {90.0,0.0,0.0};
  
  
  //side viewpoint (4.2704,0.8378,0.6800,-180.0000,-1.5000,0.0000)
  //static float xyz[3] = {4.2704,0.8378,0.6800};
  //static float hpr[3] = {-180.0000,-1.5000,0.0000};

  //top viewpoint
  //static float xyz[3] = {0.8934,0.2358,5.7000};
  //static float hpr[3] = {89.5000,-93.0000,0.0000};
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

  //My addition
  //dReal B3x=-1.0, B3y=0.1, B3z=4.0;

  dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;
  dMass m1;

  prepDrawStuff();

  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world,0,0,GRAVITY);

  // Create a ground
  ground = dCreatePlane(space,0,0,1,0);

  // Create a ball
  ball.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetSphereTotal(&m1,mass,radius);
  dBodySetMass(ball.body,&m1);
  dBodySetPosition(ball.body, centerSphr[0], centerSphr[1], centerSphr[2]);
  //dBodySetPosition(ball.body, x0, y0, z0);
  ball.geom = dCreateSphere(space,radius);
  dGeomSetBody(ball.geom,ball.body);

  //create sphere
  // ????
  //create box1
  createBox(box1, center1, sides1, B1matrix);
  //create box3
  createBox(box2, center2, sides2, B2matrix);
  //create box3
  createBox(box3, center3, sides3, B3matrix);
  //create box3
  createBox(box4, center4, sides4, B4matrix);


  
  
  #ifdef DRAW
  dsSimulationLoop (argc,argv,600,480,&fn);
  #else
  while (1) {
    simLoop(0);
  }
  #endif

  //dsSimulationLoop (argc,argv,352,288,&fn);

  


  dWorldDestroy (world);
  dCloseODE();

  return 0;
}