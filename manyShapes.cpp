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
static int COUNT = 6;           //how long until we want to wait to check satic equlibrium?
static double THRESHHOLD = 0.1;  //how much movement is allowed
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



//TITLE: Box is balanced on sphere. Four boxes and a sphere in static equilibrium, 
//ID: 3
//Position declarations
const dReal centerSphr[3] = {S1x=0.222523,S1y=-0.0803006,S1z=0.2};
const dReal center1[3] = {B1x=0.315066,B1y=0.120175,B1z=0.590846};
const dReal center2[3] = {B2x=0.305423,B2y=0.600279,B2z=0.25};
const dReal center3[3] = {B3x=-1.08545,B3y=-0.410903,B3z=0.25};
const dReal center4[3] = {B4x=1.13042,B4y=0.0812032,B4z=0.5};

//Rotation declarations
const dMatrix3 B1matrix = {-0.0343974,-1.49881e-11, 0.997988,
                                0,          -0.30751, 0.942834,
                             -0.0085887,        0,   -0.930998  };
const dMatrix3 B2matrix = { -3.35709e-12,-1.27513e-11,     1,
                             0,            -6.47146e-13,     1,
                             1.27513e-11,           0,      -1  };
const dMatrix3 B3matrix = {-4.82185e-16, -0.0168371,  0.999858,
                                0,        -1.73472e-18, 0.999858,
                             0.0168371,          0,          -1  };
const dMatrix3 B4matrix = {1,           -3.61907e-18, 6.88306e-20,
                             0,            3.61907e-18,          1,
                             -6.39488e-19,         0,      -6.88306e-20  };


/*
//Position declarations
const dReal centerSphr[3] = {S1x=0.0,S1y=0,S1z=3.5}; // length of edges
const dReal center1[3] = {B1x=0.0,B1y=0.8,B1z=0.5}; // length of edges
const dReal center2[3] = {B2x=0.0,B2y=-0.8,B2z=0.5}; // length of edges
const dReal center3[3] = {B3x=-0.8,B3y=0.0,B3z=0.5}; // length of edges
const dReal center4[3] = {B4x=0.8,B4y=0.0,B4z=0.5}; // length of edges
//Rotation declarations
const dMatrix3 B1matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B2matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B3matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B4matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 

*/

/*
//Position declarations
const dReal centerSphr[3] = {S1x=-2.0,S1y=0,S1z=1}; // length of edges
const dReal center1[3] = {B1x=0,B1y=0,B1z=1.26}; // length of edges
const dReal center2[3] = {B2x=-0.5,B2y=0,B2z=0.5}; // length of edges
const dReal center3[3] = {B3x=0.5,B3y=0,B3z=0.5}; // length of edges
const dReal center4[3] = {B4x=0,B4y=0,B4z=1.8}; // length of edges
//Rotation declarations
const dMatrix3 B1matrix = { 0, 0, 1,
                            0, -1, 0,
                            1, 0, 0  }; 
const dMatrix3 B2matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B3matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B4matrix = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 

*/








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

  /*
  const dReal *Box1pos, *Box1R;
  const dReal *Box2pos, *Box2R;
  const dReal *Box3pos, *Box3R;
  */
 
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,TIMESTEP);
  dJointGroupEmpty(contactgroup);

  // what is this flag stuff?
  //if (flag == 0) dsSetColor(1.0, 0.0, 0.0);
  //else           dsSetColor(0.0, 0.0, 1.0);
  //


 
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
      double zpos = dBodyGetPosition(box1.body)[2];
      inStaticEquilibrium(B1z, zpos); //feed it in the Zinital and Zfinal coordinates of Box4

  //end chrono timer
  endTime = chrono::steady_clock::now();
  auto diff = endTime - startTime;
  cout << chrono::duration <double, milli> (diff).count() << " ms" << endl;

  //seconds
  //chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(endTime - startTime);
  //cout << "It took me " << time_span.count() << " seconds.";


    }

   
  /*
   *  This next block of code prints out all the final positions and rotations
   *  so that one could paste the results back into the code and get static equilibrium
   *  instead of doing Box1pos to access the coordinates maybe I should do 
   *  dBodyGetPosition(box1.body) and dBodyGetRotation(box1.body) so that its generalized.
  */ 

  /*
  if (counter == 200) { //at 200 assume objects are at rest
    cout << "const dReal centerSphr[3] = {S1x="<<pos[0]<<",S1y="<<pos[1]<<",S1z="<<pos[2]<<"};\n" <<endl; // length of edges
    cout << "const dReal center1[3] = {B1x="<<Box1pos[0]<<",B1y="<<Box1pos[1]<<",B1z="<<Box1pos[2]<<"};\n" <<endl; // length of edges
    cout << "const dReal center2[3] = {B2x="<<Box2pos[0]<<",B2y="<<Box2pos[1]<<",B2z="<<Box2pos[2]<<"};\n" <<endl; // length of edges
    cout << "const dReal center3[3] = {B3x="<<Box3pos[0]<<",B3y="<<Box3pos[1]<<",B3z="<<Box3pos[2]<<"};\n" <<endl; // length of edges
    cout << "const dReal center4[3] = {B4x="<<Box4pos[0]<<",B4y="<<Box4pos[1]<<",B4z="<<Box4pos[2]<<"};\n" <<endl; // length of edges
    cout<<"const dMatrix3 B1matrix[3][3] = {  { "<<Box1R[0]<<","<<Box1R[1]<<","<<Box1R[2]<<"},"<<endl;
    cout<<"                            { "<<Box1R[3]<<","<<Box1R[4]<<","<<Box1R[5]<<"},"<<endl;
    cout<<"                            { "<<Box1R[6]<<","<<Box1R[7]<<","<<Box1R[8]<<"}  };"<<endl;
    cout<<"const dMatrix3 B2matrix[3][3] = {  { "<<Box2R[0]<<","<<Box2R[1]<<","<<Box2R[2]<<"},"<<endl;
    cout<<"                            { "<<Box2R[3]<<","<<Box2R[4]<<","<<Box2R[5]<<"},"<<endl;
    cout<<"                            { "<<Box2R[6]<<","<<Box2R[7]<<","<<Box2R[8]<<"}  };"<<endl;
    cout<<"const dMatrix3 B3matrix[3][3] = {  { "<<Box3R[0]<<","<<Box3R[1]<<","<<Box3R[2]<<"},"<<endl;
    cout<<"                            { "<<Box3R[3]<<","<<Box3R[4]<<","<<Box3R[5]<<"},"<<endl;
    cout<<"                            { "<<Box3R[6]<<","<<Box3R[7]<<","<<Box3R[8]<<"}  };"<<endl;
   
    cout<<"const dMatrix3 B4matrix[3][3] = {  { "<<Box4R[0]<<","<<Box4R[1]<<","<<Box4R[2]<<"},"<<endl;
    cout<<"                            { "<<Box4R[3]<<","<<Box4R[4]<<","<<Box4R[5]<<"},"<<endl;
    cout<<"                            { "<<Box4R[6]<<","<<Box4R[7]<<","<<Box4R[8]<<"}  };"<<endl;
   
  }
 */


    //Dont forget about the counter!
    counter++;   

}



//This function checks for static equilibrium
/* Later we could turn this into a bool return. Additionally, one
*  could feed in dReal types instead of doubles. One would also
*  want to be able to feed in an arbitrary number of object data.
*
*/
static void inStaticEquilibrium(double startZ, double endZ ){

    double deltaZ = std::abs(startZ - endZ);
    cout << "StartPosition: " << startZ << " \n" << endl;
    cout << "EndPosition: " << endZ << " \n" << endl;
    cout << "delta Z: " << deltaZ << " \n" << endl;
    cout << "counter = " << COUNT  << " \n" << endl;
    cout << "THRESHHOLD : " << THRESHHOLD << " \n" << endl;
    //Check if object moved since initialization
    if ( (deltaZ) > THRESHHOLD){
        cout << "FALSE: Not in static equilibrium\n" << endl;
    } else 
        cout << "True: Is in static equilibrium\n" << endl;
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

  dWorldSetGravity(world,0,0,-9.8);

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