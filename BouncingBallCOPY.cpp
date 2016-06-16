#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <string.h>
#include <iostream>
#include <cmath>
#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#endif
using namespace std;


//Joe's code
static int counter = 0;    //iterator which will reach COUNT
static int COUNT = 10;    //how long do we want to wait to check movement
static double Z_COORDINATE = 0.45; //The object's center's initial height
static double THRESHHOLD = 0.1;  //how much movement is allowed


// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dBodyID body;	
static dGeomID geom;	
static dMass m;
static dJointGroupID contactgroup;
//When the collision system detects that two objects are colliding, it calls this routine which determines the points of contact and creates temporary joints. The surface parameters of the joint (friction, bounce velocity, CFM, etc) are also set here.

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact;  
    contact.surface.mode = dContactBounce | dContactSoftCFM;
    // friction parameter
    contact.surface.mu = dInfinity;
    // bounce is the amount of "bouncyness".
    contact.surface.bounce = 0.0;  //0.9;
    // bounce_vel is the minimum incoming velocity to cause a bounce
    contact.surface.bounce_vel = 0.0; //0.1;
    // constraint force mixing parameter
    contact.surface.soft_cfm = 0.001;  
    if (int numc = dCollide (o1,o2,1,&contact.geom,sizeof(dContact))) {
        dJointID c = dJointCreateContact (world,contactgroup,&contact);
        dJointAttach (c,b1,b2);
    }
}
//This function is called at the start of the simulation to set up the point of view of the camera.

// start simulation - set viewpoint
static void start()
{
    static float xyz[3] = {2.0,-2.0,1.7600};
    static float hpr[3] = {140.000,-17.0000,0.0000};
    dsSetViewpoint (xyz,hpr);
}
//This is the main simulation loop that calls the collision detection function, steps the simulation, resets the temporary contact joint group, and redraws the objects at their new position.

// simulation loop
static void simLoop (int pause)
{
    const dReal *pos;
    const dReal *R;
    // find collisions and add contact joints
    dSpaceCollide (space,0,&nearCallback);
    // step the simulation
    dWorldQuickStep (world,0.01);  
    // remove all contact joints
    dJointGroupEmpty (contactgroup);
    // redraw sphere at new location
    pos = dGeomGetPosition (geom);
    R = dGeomGetRotation (geom);
    dsDrawSphere (pos,R,dGeomSphereGetRadius (geom));

    //Joe's code
    
    if (counter == COUNT){
        double deltaZ = std::abs(Z_COORDINATE - pos[2]);
        cout << "StartPosition: " << Z_COORDINATE << " \n" << endl;
        cout << "EndPosition: " << pos[2] << " \n" << endl;
        cout << "delta Z: " << deltaZ << " \n" << endl;
        cout << "counter = " << COUNT  << " \n" << endl;
        cout << "THRESHHOLD : " << THRESHHOLD << " \n" << endl;
        
        //Check if object moved since initialization
            if ( (deltaZ) > THRESHHOLD){
                cout << "FALSE: Not in static equilibrium" << endl;
            } else 
                cout << "True: Is in static equilibrium" << endl;

    }
    counter++;

}
//When the program starts, the callbacks are set up, everything is initialized, and then the simulation is started.

int main (int argc, char **argv)
{
   
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.stop = 0;
    fn.command = 0;
    fn.path_to_textures = "/home/joeshepley/ode-0.13.1/drawstuff/textures";
 
    dInitODE ();
    // create world
    world = dWorldCreate ();
    space = dHashSpaceCreate (0);
    dWorldSetGravity (world,0,0,-9.8);
    dWorldSetCFM (world,1e-5);
    dCreatePlane (space,0,0,1,0);
    contactgroup = dJointGroupCreate (0);
    // create object
    body = dBodyCreate (world);
    geom = dCreateSphere (space,0.5);
    dMassSetSphere (&m,1,0.5);
    dBodySetMass (body,&m);
    dGeomSetBody (geom,body);
    // set initial position
    dBodySetPosition (body,0,0,Z_COORDINATE);
    // run simulation
    dsSimulationLoop (argc,argv,960,480,&fn); 
    // clean up
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();
    return 0;
}
