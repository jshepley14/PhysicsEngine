/*************************************************************************
 *    Author: Joe Shepley                                                *
 *    Description: This program will be used for checking static         *
 *    equilibrium among a set of objects.                                *                                     
 *                                                                       *
 *************************************************************************/

#include <vector>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <string.h>
#include <iostream>
#include <cmath>
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#endif
using namespace std;

static int levels = 5;
static int ncards = 0;

static dSpaceID space;
static dWorldID world;
static dJointGroupID contactgroup;

struct Card {
    dBodyID body;
    dGeomID geom;
    static const dReal sides[3]; //side lengths
    //static const dReal center[3]; //center position
    //const dReal* sides
    

    Card()
    {
        
        body = dBodyCreate(world);
        geom = dCreateBox(space, sides[0], sides[1], sides[2]);
        dGeomSetBody(geom, body);
        dGeomSetData(geom, this);
        dMass mass;
        mass.setBox(1, sides[0], sides[1], sides[2]);
        dBodySetMass(body, &mass);
    }

    ~Card()
    {
        dBodyDestroy(body);
        dGeomDestroy(geom);
    }
    
    void draw() const
    {
        dsDrawBox(dBodyGetPosition(body),
                  dBodyGetRotation(body), sides);
    }
};
//set up with dimensions
static const dReal cwidth=.5, cthikness=.02, clength=1;
const dReal Card::sides[3] = { cwidth, cthikness, clength };


struct Box {
    dBodyID body;
    dGeomID geom;
    static const dReal sides[3]; //side lengths
    //static const dReal center[3]; //center position
    int ID;

    Box()
    {
        
        body = dBodyCreate(world);
        geom = dCreateBox(space, sides[0], sides[1], sides[2]);
        dGeomSetBody(geom, body);
        dGeomSetData(geom, this);
        dMass mass;
        mass.setBox(1, sides[0], sides[1], sides[2]);
        dBodySetMass(body, &mass);
    }

    ~Box()
    {
        dBodyDestroy(body);
        dGeomDestroy(geom);
    }
    
    void draw() const
    {
        dsDrawBox(dBodyGetPosition(body),
                  dBodyGetRotation(body), sides);
    }
    void createObject(int ID){
        this->ID = ID;
    }
    
    int getID() {
        cout<<"ID"<<ID<<endl;
    }
    

};

/*
class Object                   // begin declaration of the class
{
    dBodyID body;
    dGeomID geom;
    static const dReal sides[3]; //side lengths
    public:
    Object();
*/







//set up with dimensions
static const dReal Boxwidth=.5, Boxthikness=.02, Boxlength=1;
const dReal Box::sides[3] = { Boxwidth, Boxthikness, Boxlength };




//set up the positions
//static const dReal x=.5, y=.02, z=1;
//const dReal Card::center[3] = { x, y, z };

std::vector<Card*> cards;

int getncards(int levels)
{
    return (3*levels*levels + levels) / 2;
}


void place_cards()
{
    ncards = getncards(levels);
    // destroy removed cards (if any)
    int oldcards = cards.size();
    for (int i=ncards; i<oldcards; ++i)
        delete cards[i];
    cards.resize(ncards);
    
    // construct new cards (if any)    
    for (int i=oldcards; i<ncards; ++i)
        cards[i] = new Card;
    
    // for each level
    int c = 0;
    dMatrix3 right, left, hrot;
    dReal angle = 20*M_PI/180.;
    dRFromAxisAndAngle(right, 1, 0, 0, -angle);
    dRFromAxisAndAngle(left, 1, 0, 0, angle);

    dRFromAxisAndAngle(hrot, 1, 0, 0, 91*M_PI/180.);
    
    dReal eps = 0.05;
    dReal vstep = cos(angle)*clength + eps;
    dReal hstep = sin(angle)*clength + eps;
    
    for (int lvl=0; lvl<levels; ++lvl) {
        // there are 3*(levels-lvl)-1 cards in each level, except last
        int n = (levels-lvl);
        dReal height = (lvl)*vstep + vstep/2;
        // inclined cards
        for (int i=0; i<2*n; ++i, ++c) {
            dBodySetPosition(cards[c]->body, 
                    0,
                    -n*hstep + hstep*i,
                    height
                    );
            if (i%2)
                dBodySetRotation(cards[c]->body, left);
            else
                dBodySetRotation(cards[c]->body, right);
        }
        
        if (n==1) // top of the house
            break;
        
        // horizontal cards
        for (int i=0; i<n-1; ++i, ++c) {
            dBodySetPosition(cards[c]->body,
                    0,
                    -(n-1 - (clength-hstep)/2)*hstep + 2*hstep*i,
                    height + vstep/2);
            dBodySetRotation(cards[c]->body, hrot);
        }
    }
    
}


std::vector<Card*> mycards;
int numberOfObjects =1;
void newScene() {
    
    //destroy cards
    for (int i=0; i<mycards.size(); ++i)
        delete mycards[i];

    mycards.resize(1);
    //create cards
    mycards[0] = new Card;
    dBodySetPosition(mycards[0]->body, 1, 0, 2);
    //dBodySetRotation(mycards[0]->body, );
    puts("test");
}


std::vector<Box*> boxes;
int numberOfBoxObjects =1;
void newBoxScene() {
    
    //destroy cards
    for (int i=0; i<boxes.size(); ++i)
        delete boxes[i];

    boxes.resize(1);
    //create cards
    boxes[0] = new Box;
    boxes[0]->createObject(2);
    boxes[0]->getID();
    dBodySetPosition(boxes[0]->body, 1, 1, 2);    
    //dBodySetRotation(mycards[0]->body, );
    puts("test");
}



void start()
{
    puts("Controls:");
    puts("   SPACE - reposition cards");
    puts("   -     - one less level");
    puts("   =     - one more level");
}

static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    const int MAX_CONTACTS = 8;
    dContact contact[MAX_CONTACTS];
    
    int numc = dCollide (o1, o2, MAX_CONTACTS,
                        &contact[0].geom,
                        sizeof(dContact));
    
    for (int i=0; i<numc; i++) {
        contact[i].surface.mode = dContactApprox1;
        contact[i].surface.mu = 5;
        dJointID c = dJointCreateContact (world, contactgroup, contact+i);
        dJointAttach (c, b1, b2);
    }
}


void simLoop(int pause)
{
    if (!pause) {
        dSpaceCollide (space, 0, &nearCallback);
        dWorldQuickStep(world, 0.01);
        dJointGroupEmpty(contactgroup);
    }
    
    dsSetColor (1,1,0);
    for (int i=0; i<ncards; ++i) {
        dsSetColor (1, dReal(i)/ncards, 0);
        cards[i]->draw();
    }

    mycards[0]->draw();
    boxes[0]->draw();
    
}

void command(int c)
{
    switch (c) {
        case '=':
            levels++;
            place_cards();
            break;
        case '-':
            levels--;
            if (levels <= 0)
                levels++;
            place_cards();
            break;
        case ' ':
            place_cards();
            break;
        case 'n':
            newScene();
            break;
        case 'b':
            newBoxScene();
            break;
    }
}

int main(int argc, char **argv)
{
    dInitODE();
    
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = "/home/joeshepley/ode-0.13.1/drawstuff/textures";
    
    
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.8);
    dWorldSetQuickStepNumIterations(world, 50); // <-- increase for more stability
    
    space = dSimpleSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dGeomID ground = dCreatePlane(space, 0, 0, 1, 0);
    
    
    
    place_cards();
    newScene();
    newBoxScene();

    // run simulation
    dsSimulationLoop (argc, argv, 640, 480, &fn);
    
    levels = 0;
    place_cards();
    newScene(); //added this
    newBoxScene();

    dJointGroupDestroy(contactgroup);
    dWorldDestroy(world);
    dGeomDestroy(ground);
    dSpaceDestroy(space);
    
    dCloseODE();
}
