// StaticEqChecker.cpp  by Joe Shepley
// compile the code by typing in the following on the terminal 
// 
//
//
// This program uses the Open Dynamics Engine (ODE) by Russell Smith.
// The ODE web site is http://ode.org/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <string.h>
#include <iostream>
#include <cmath>
#include <chrono>  //used for timing code

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox  dsDrawBoxD
#endif

//#define DRAW  //used to switch on or off the drawing of the scene

using namespace std;

//function declarations
//static void inStaticEquilibrium(double startZ, MyObject &object );
static void printEndpositions(const dReal *Box1pos);


//static equilibrium constants
static int counter = 0;          //iterator which will reach COUNT
static int COUNT = 30; //10;           //how long until we want to wait to check satic equlibrium?
static double THRESHHOLD = 0.1;  //how much movement is allowed
static double GRAVITY = -19.8;    
static double TIMESTEP = 0.02;    /*Time step originally was at 0.01, but we may
                                   *be able to increase this making calculations faster
                                   *most efficient would prob be 0.01 ≤ TIMESTEP ≤ 0.03 */
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


//TITLE: Balanced ball on four blocks
//ID: 1
//Position declarations
const dReal centerSphr[3] = {S1x=0.0,S1y=0,S1z=1.9}; 
const dReal center1[3] = {B1x=0.0,B1y=0.8,B1z=0.5}; 
const dReal center2[3] = {B2x=0.0,B2y=-0.8,B2z=0.5}; 
const dReal center3[3] = {B3x=-0.8,B3y=0.0,B3z=0.5}; 
const dReal center4[3] = {B4x=0.8,B4y=0.0,B4z=0.5}; 
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



//....................................................................................................
//TITLE: Exploding configuration
//ID: 2
const dReal NewCenter1[3] = {B3x=0,B3y=0,B3z=2};
const dReal NewCenter2[3] = {B3x=-1,B3y=0,B3z=2};
const dReal NewCenter3[3] = {B3x=1,B3y=4,B3z=2};
const dReal NewCenter4[3] = {B3x=0,B3y=0,B3z=5};



//....................................................................................................
//TITLE: Dropping four boxes and a sphere from the air
//ID: 3
//Position declarations
const dReal centerSphr_2[3] = {S1x=0.0,S1y=0.0,S1z=2.0}; // length of edges
const dReal center1_2[3] = {B1x=0.0,B1y=0.1,B1z=4.0}; // length of edges
const dReal center2_2[3] = {B2x=0.0,B2y=1.0,B2z=4.0}; // length of edges
const dReal center3_2[3] = {B3x=-1.0,B3y=0.1,B3z=4.0}; // length of edges
const dReal center4_2[3] = {B4x=1.0,B4y=0.1,B4z=4.0}; // length of edges

//Rotation declarations
const dMatrix3 B1matrix_2 = { 0, 1, 1,
                              0, 1, 1,
                              0, 1, 1  }; 
const dMatrix3 B2matrix_2 = { 0,  1,   1,
                              0, 0.5, 1,
                              0, 0.7, 1  }; 
const dMatrix3 B3matrix_2 = { 0,   1, 0,
                              0.8, 1, 1,
                              0, 0.3, 1  }; 
const dMatrix3 B4matrix_2 = { 0, 1, 1,
                              0, 1, 1,
                              0, 1, 1  }; 

//....................................................................................................
//TITLE: Box is balanced on sphere. Four boxes and a sphere in static equilibrium, 
//ID: 4
//Position declarations
const dReal centerSphr_3[3] = {S1x=0.222523,S1y=-0.0803006,S1z=0.2};
const dReal center1_3[3] = {B1x=0.315066,B1y=0.120175,B1z=0.590846};
const dReal center2_3[3] = {B2x=0.305423,B2y=0.600279,B2z=0.25};
const dReal center3_3[3] = {B3x=-1.08545,B3y=-0.410903,B3z=0.25};
const dReal center4_3[3] = {B4x=1.13042,B4y=0.0812032,B4z=0.5};

//Rotation declarations
const dMatrix3 B1matrix_3 = {-0.0343974,-1.49881e-11, 0.997988,
                                0,          -0.30751, 0.942834,
                             -0.0085887,        0,   -0.930998  };
const dMatrix3 B2matrix_3 = { -3.35709e-12,-1.27513e-11,     1,
                             0,            -6.47146e-13,     1,
                             1.27513e-11,           0,      -1  };
const dMatrix3 B3matrix_3 = {-4.82185e-16, -0.0168371,  0.999858,
                                0,        -1.73472e-18, 0.999858,
                             0.0168371,          0,          -1  };
const dMatrix3 B4matrix_3 = {1,           -3.61907e-18, 6.88306e-20,
                             0,            3.61907e-18,          1,
                             -6.39488e-19,         0,      -6.88306e-20  };

//....................................................................................................
//TITLE: box tower
//ID: 5
//Position declarations
const dReal centerSphr_5[3] = {S1x=-2.0,S1y=0,S1z=0.2}; // length of edges
const dReal center1_5[3] = {B1x=0,B1y=0,B1z=1.26}; // length of edges
const dReal center2_5[3] = {B2x=-0.5,B2y=0,B2z=0.5}; // length of edges
const dReal center3_5[3] = {B3x=0.5,B3y=0,B3z=0.5}; // length of edges
const dReal center4_5[3] = {B4x=0,B4y=0,B4z=1.8}; // length of edges
//Rotation declarations
const dMatrix3 B1matrix_5 = { 0, 0, 1,
                            0, -1, 0,
                            1, 0, 0  }; 
const dMatrix3 B2matrix_5 = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B3matrix_5 = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B4matrix_5 = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 

//....................................................................................................
//TITLE: leaning block falling.  Example of configuration that requires COUNT to be large OR small Threshold 

//Position declarations
const dReal centerSphr_6[3] = {S1x=-2.0,S1y=0,S1z=0.2}; // length of edges
const dReal center1_6[3] = {B1x=0.0,B1y=0,B1z=0.55}; // length of edges
const dReal center2_6[3] = {B2x=0,B2y=1,B2z=0.5}; // length of edges
const dReal center3_6[3] = {B3x=0,B3y=2,B3z=0.5}; // length of edges
const dReal center4_6[3] = {B4x=0,B4y=3,B4z=0.5}; // length of edges
//Rotation declarations
const dMatrix3 B1matrix_6 = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0.5  }; 
const dMatrix3 B2matrix_6 = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B3matrix_6 = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 
const dMatrix3 B4matrix_6 = { 1, 0, 0,
                            0, 0, 0,
                            0, 0, 0  }; 

//********************************************************************************************8*******************
//object declarations
typedef struct {
  dBodyID body;
  dGeomID geom;
} MyObject;
MyObject ball, box1, box2, box3, box4 ;


//Functions

//**BOXES**
//creates a box 
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


//draws a box
static void drawBox( MyObject &box, const dReal* sides){
    const dReal *pos1,*R1;
    pos1 = dBodyGetPosition(box.body);
    R1   = dBodyGetRotation(box.body);
    #ifdef DRAW
    dsDrawBox(pos1,R1,sides);
    #else
    #endif
}

//destroys a box
static void destroyBox(MyObject box)
{
    dBodyDestroy(box.body);
    dGeomDestroy(box.geom);
}


//**BALLS**
// Create a ball
void createBall(MyObject &ball, const dReal* center, const dReal radius )
{
    dMass m1;
    //dReal x0 = 0.0, y0 = 0.5, z0 = 1.0, radius = 0.1; //[m]
    dReal mass = 1.0; //[kg]

    ball.body = dBodyCreate(world);   // changed
    dMassSetZero(&m1);
    dMassSetSphereTotal(&m1,mass,radius);
    dBodySetMass(ball.body,&m1);             // changed
    dBodySetPosition(ball.body, center[0], center[1], center[2]); // changed

    ball.geom = dCreateSphere(space,radius); // add
    dGeomSetBody(ball.geom,ball.body);       // add
}


//draws a ball
static void drawBall( MyObject &ball, const dReal radius){
    const dReal *pos1,*R1;
    pos1 = dBodyGetPosition(ball.body);
    R1   = dBodyGetRotation(ball.body);
    dsDrawSphere(pos1,R1,radius);
}

//destroys a box
static void destroyBall(MyObject ball)
{
    dBodyDestroy(ball.body);
    dGeomDestroy(ball.geom);
}


void newScene(MyObject &box1, const dReal* center1, const dReal* sides1, const dMatrix3 B1matrix,
              MyObject &box2, const dReal* center2, const dReal* sides2, const dMatrix3 B2matrix,
              MyObject &box3, const dReal* center3, const dReal* sides3, const dMatrix3 B3matrix,
              MyObject &box4, const dReal* center4, const dReal* sides4, const dMatrix3 B4matrix,
              MyObject &ball, const dReal* centerSphr, const dReal radius )
{
    // destroy
    //dJointGroupDestroy(contactgroup); <- do we need this?
    destroyBox(box1);
    destroyBox(box2);
    destroyBox(box3);
    destroyBox(box4);
    destroyBall(ball);

    // create
    //contactgroup = dJointGroupCreate(0);  <- do we need this?
    //boxes
    createBox(box1, center1, sides1, B1matrix);
    createBox(box2, center2, sides2, B2matrix);
    createBox(box3, center3, sides3, B3matrix);
    createBox(box4, center4, sides4, B4matrix);
    //ball
    createBall(ball, centerSphr, radius);
}





//Collisions Detection
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







//This function checks for static equilibrium
/* Later we could turn this into a bool return. Additionally, one
*  could feed in dReal types instead of doubles. One would also
*  want to be able to feed in an arbitrary number of object data.
*
*/
static void inStaticEquilibrium(double startZ, MyObject &object){

    double endZ = dBodyGetPosition(object.body)[2];
    double deltaZ = std::abs(startZ - endZ);
    //cout << "counter = " << COUNT  << " \n" << endl;
    //cout << "THRESHHOLD : " << THRESHHOLD << " \n" << endl;
    //Check if object moved since initialization
    if ( (deltaZ) > THRESHHOLD){
        //cout << "FALSE: Not in static equilibrium" << endl;
    } else{ 
      //  cout << "TRUE: Is in static equilibrium" << endl;
    }
   // cout << "StartPosition: " << startZ << " " << endl;
  //  cout << "EndPosition: " << endZ << " " << endl;
   // cout << "delta Z: " << deltaZ << " " << endl;
}






static void simLoop (int pause)
{
  
  //start timer
    if (counter == 0){
     startTime = chrono::steady_clock::now();
    }

 
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,TIMESTEP);
  dJointGroupEmpty(contactgroup);

  // what is this flag stuff?
  //if (flag == 0) dsSetColor(1.0, 0.0, 0.0);
  //else           dsSetColor(0.0, 0.0, 1.0);
  //


   //draw stuff
   #ifdef DRAW
     dsSetColor(0.8,.78,.10);
     drawBall(ball, radius);
     dsSetColor(1,0,0);
     drawBox(box1, sides1);
     dsSetColor(0,1,0);
     drawBox(box2, sides2);
     dsSetColor(0,0,1);
     drawBox(box3, sides3);
     dsSetColor(0,.12,.2); //0,12,200 is a bright turquoise
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

  //  To Do:  Eventually go back to card demo and ask Shvivam to help you make a class/constructor
  //   Fix the static equ checker below, fix the inputs etc.
  
   if (counter == COUNT){ //now we want to check
  //  cout<<"FALSE: Scene 1"<<endl;   
    inStaticEquilibrium(centerSphr[2], ball); //feed it in the Zinital and Zfinal coordinates of Box4
    //end chrono timer
    endTime = chrono::steady_clock::now();
    auto diff = endTime - startTime;
   // cout << chrono::duration <double, milli> (diff).count() << " ms\n" << endl;
    }

  if (counter == COUNT){ //100
    //  cout<<"FALSE: Scene 2"<<endl;
     newScene(box1, NewCenter1, sides1, B1matrix,
              box2, NewCenter2, sides1, B2matrix,
              box3, NewCenter3, sides1, B3matrix,
              box4, NewCenter4, sides1, B4matrix,
              ball, centerSphr, radius);
      
  }

  if (counter == COUNT*2){ // COUNT + 100now we want to check   
       inStaticEquilibrium(centerSphr[2], ball); //feed it in the Zinital and Zfinal coordinates of Box4
       
       //end chrono timer
    endTime = chrono::steady_clock::now();
    auto diff = endTime - startTime;
  //  cout << chrono::duration <double, milli> (diff).count() << " ms\n" << endl;
    
    
    }

///What's up with this scene???  also need to adjust variables
  if (counter == COUNT*2){ //200
   //   cout<<"FALSE: Scene 3"<<endl;
     newScene(box1, center1_2, sides1, B1matrix_2,
              box2, center2_2, sides1, B2matrix_2,
              box3, center3_2, sides1, B3matrix_2,
              box4, center4_2, sides1, B4matrix_2,
              ball, centerSphr_2, radius);
      
  }

  if (counter == COUNT*3){ //now we want to check   
       inStaticEquilibrium(centerSphr_2[2], ball); //feed it in the Zinital and Zfinal coordinates of Box4
       
    }

  if (counter == COUNT*3){ //300
   //   cout<<"TRUE: Scene 4"<<endl;
     newScene(box1, center1_3, sides1, B1matrix_3,
              box2, center2_3, sides1, B2matrix_3,
              box3, center3_3, sides1, B3matrix_3,
              box4, center4_3, sides1, B4matrix_3,
              ball, centerSphr_3, radius);
      
  }

  if (counter == COUNT*4){ //now we want to check   
       inStaticEquilibrium(centerSphr_3[2], ball); //feed it in the Zinital and Zfinal coordinates of Box4
       
    }

  if (counter == COUNT*4){ //400
   //   cout<<"TRUE: Scene 5"<<endl;
     newScene(box1, center1_5, sides1, B1matrix_5,
              box4, center2_5, sides1, B2matrix_5,
              box3, center3_5, sides1, B3matrix_5,
              box2, center4_5, sides1, B4matrix_5,
              ball, centerSphr_5, radius);
      
  }

  if (counter == COUNT*5){ //now we want to check   
       inStaticEquilibrium(centerSphr_5[2], ball); //feed it in the Zinital and Zfinal coordinates of Box4

        

    }




    if (counter == COUNT*5){ //500
    //  cout<<"FALSE: Scene 6"<<endl;
     newScene(box1, center1_6, sides1, B1matrix_6,
              box4, center2_6, sides1, B2matrix_6,
              box3, center3_6, sides1, B3matrix_6,
              box2, center4_6, sides1, B4matrix_6,
              ball, centerSphr_6, radius);
      
  }

  if (counter == COUNT*6){ //now we want to check   
       inStaticEquilibrium(center1_6[2], box1); //feed it in the Zinital and Zfinal coordinates of Box4

        //end chrono timer
    endTime = chrono::steady_clock::now();
    auto diff = endTime - startTime;
    cout << chrono::duration <double, milli> (diff).count() << " ms\n" << endl;
    dsStop	();  //break out of the simLoop    
        //counter = 0;  //loop through all the scenes
        
    }


    counter++;   

}



//was supposed to be a function that printed final positions
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

  //some constant prob unused though?
  dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;
  dMass m1;

  //initialize the space
  prepDrawStuff();
  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world,0,0,GRAVITY);   //set gravity z position -9.8 m/s^2

  // Create ground
  ground = dCreatePlane(space,0,0,1,0);

  // Create a ball
  createBall(ball, centerSphr, radius);

  //create boxes
  createBox(box1, center1, sides1, B1matrix);
  createBox(box2, center2, sides2, B2matrix);
  createBox(box3, center3, sides3, B3matrix);
  createBox(box4, center4, sides4, B4matrix);



  
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