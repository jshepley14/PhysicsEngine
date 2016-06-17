// sample6.cpp  by Kosei Demura 2005-2011
// My web site is http://demura.net
// This program uses the Open Dynamics Engine (ODE) by Russell Smith.
// The ODE web site is http://ode.org/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <string.h>
#include <iostream>
#include <cmath>
#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox  dsDrawBoxD
#endif
using namespace std;

//function declarations
static void inStaticEquilibrium(double startZ, double endZ );
static void printEndpositions(const dReal *Box1pos);

//static equilibrium constants
static int counter = 0;          //iterator which will reach COUNT
static int COUNT = 10;           //how long until we want to wait to check satic equlibrium?
static double THRESHHOLD = 0.1;  //how much movement is allowed
static double TIMESTEP = 0.02;    //Time step originally was at 0.01, but we may
                                 //be able to increase this making calculations faster
                                 //most efficient would prob be 0.01 ≤ TIMESTEP ≤ 0.03
//ID declarations
static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
static int flag = 0; //<-don't need this?
dsFunctions fn;

//object construction variables
const dReal   radius = 0.2; //sphere
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


/*
//Position declarations
const dReal centerSphr[3] = {S1x=0.0,S1y=0.0,S1z=2.0}; // length of edges
const dReal center1[3] = {B1x=0.0,B1y=0.1,B1z=4.0}; // length of edges
const dReal center2[3] = {B2x=0.0,B2y=1.0,B2z=4.0}; // length of edges
const dReal center3[3] = {B3x=-1.0,B3y=0.1,B3z=4.0}; // length of edges
const dReal center4[3] = {B4x=1.0,B4y=0.1,B4z=4.0}; // length of edges

//Rotation declarations
const dMatrix3 B1matrix[3][3] = {  { 0, 1, 1},
                                { 0, 1, 1},
                                { 0, 1, 1}  }; 
const dMatrix3 B2matrix[3][3] = {  { 0, 1, 1},
                                { 0, 0.5, 1},
                                { 0, 0.7, 1}  }; 
const dMatrix3 B3matrix[3][3] = {  { 0, 1, 0},
                                { 0.8, 1, 1},
                                { 0, 0.3, 1}  }; 
const dMatrix3 B4matrix[3][3] = {  { 0, 1, 1},
                                { 0, 1, 1},
                                { 0, 1, 1}  }; 

*/

//Use this for static equilibrium example
//trying to implement new example, make the boxes stacked and what not

const dReal centerSphr[3] = {S1x=4,S1y=4,S1z=4};
const dReal center1[3] = {B1x=0.5,B1y=0,B1z=0.5};
const dReal center2[3] = {B1x=-0.5,B1y=0,B1z=0.5};
const dReal center3[3] = {B1x=0.5,B1y=0,B1z=1.5};
const dReal center4[3] = {B1x=-0.5,B1y=0,B1z=1.5};

const dMatrix3 B1matrix[3][3] = {  { 0, 1, 1},
                                { 0, 1, 1},
                                { 0, 1, 1}  }; 
const dMatrix3 B2matrix[3][3] = {  { 0, 1, 1},
                                { 0, 1, 1},
                                { 0, 1, 1}  }; 
const dMatrix3 B3matrix[3][3] = {  { 0, 1, 1},
                                { 0, 1, 1},
                                { 0, 1, 1}  }; 
const dMatrix3 B4matrix[3][3] = {  { 0, 1, 1},
                                { 0, 1, 1},
                                { 0, 1, 1}  }; 







//********************************************************************************************8*******************
//object declarations
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
  dWorldStep(world,TIMESTEP);
  dJointGroupEmpty(contactgroup);

  // what is this flag stuff?
  if (flag == 0) dsSetColor(1.0, 0.0, 0.0);
  else           dsSetColor(0.0, 0.0, 1.0);
  //


  //draw sphere
  pos = dBodyGetPosition(ball.body);
  R   = dBodyGetRotation(ball.body);
  dsDrawSphere(pos,R,radius);

  dsSetColor(1,0,0);
  //draw Box1
  Box1pos = dBodyGetPosition(box1.body);
  Box1R   = dBodyGetRotation(box1.body);
  dsDrawBox(Box1pos,Box1R,sides1);

  dsSetColor(0,1,0);
   //draw Box2
  Box2pos = dBodyGetPosition(box2.body);
  Box2R   = dBodyGetRotation(box2.body);
  dsDrawBox(Box2pos,Box2R,sides2);

dsSetColor(0,0,1);
   //draw Box3
  Box3pos = dBodyGetPosition(box3.body);
  Box3R   = dBodyGetRotation(box3.body);
  dsDrawBox(Box3pos,Box3R,sides3);

dsSetColor(0,12,200);
   //draw Box4
  Box4pos = dBodyGetPosition(box4.body);
  Box4R   = dBodyGetRotation(box4.body);
  dsDrawBox(Box4pos,Box4R,sides4);


  /*
   *  Static Equilibrium Detection
   *  
  */ 
  if (counter == COUNT){
        inStaticEquilibrium(B4z, Box4pos[2]); //feed it in the Zinital and Zfinal coordinates of Box4
    }

   
  /*
   *  This next block of code prints out all the final positions and rotations
   *  so that one could paste the results back into the code and get static equilibrium
   *  instead of doing Box1pos to access the coordinates maybe I should do 
   *  dBodyGetPosition(box1.body) and dBodyGetRotation(box1.body) so that its generalized.
  */ 
  if (counter == COUNT) { //previously used 400 instead of COUNT
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
  static float xyz[3] = {0.0,-3.0,1.0};
  static float hpr[3] = {90.0,0.0,0.0};
  
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

  // Create a box1
  box1.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dBodySetPosition(box1.body, B1x-1+1, B1y, B1z);
  dMassSetBoxTotal(&m1,mass,sides1[0], sides1[1], sides1[2]);
  dBodySetMass(box1.body,&m1);
  dBodySetPosition(box1.body,center1[0], center1[1], center1[2]);
  cout<<"ignore->"<<y0+-1.1<<" "<<z0+2<< endl;
  //dBodySetPosition(box1.body, x0, y0+0.1, z0+2);
  dBodySetRotation (box1.body, B1matrix[3][3]);      //Can comment this out if you dont want weird rotation
  box1.geom = dCreateBox(space,sides1[0], sides1[1], sides1[2]);
  dGeomSetBody(box1.geom,box1.body);

  // Create a box2
  box2.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetBoxTotal(&m1,mass,sides2[0], sides2[1], sides2[2]);
  dBodySetMass(box2.body,&m1);
  dBodySetPosition(box2.body, center2[0], center2[1], center2[2]);
  dBodySetRotation (box2.body, B2matrix[3][3]);
  box2.geom = dCreateBox(space,sides2[0], sides2[1], sides2[2]);
  dGeomSetBody(box2.geom,box2.body);

  // Create a box3
  box3.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetBoxTotal(&m1,mass,sides3[0], sides3[1], sides3[2]);
  dBodySetMass(box3.body,&m1);
  dBodySetPosition(box3.body, center3[0], center3[1], center3[2]);
  dBodySetRotation (box3.body, B3matrix[3][3]);
  box3.geom = dCreateBox(space,sides3[0], sides3[1], sides3[2]);
  dGeomSetBody(box3.geom,box3.body);

  // Create a box4
  box4.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetBoxTotal(&m1,mass,sides4[0], sides4[1], sides4[2]);
  dBodySetMass(box4.body,&m1);
  dBodySetPosition(box4.body, center4[0], center4[1], center4[2]);
  dBodySetRotation (box4.body, B4matrix[3][3]);
  box4.geom = dCreateBox(space,sides4[0], sides4[1], sides4[2]);
  dGeomSetBody(box4.geom,box4.body);




  dsSimulationLoop (argc,argv,352,288,&fn);

  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
