# PhysicsEngine

High level view for this project:

In robotics, often times a robot's intended purpose is to manipulate objects
whether it be picking items from a shelf and sorting them (Amazon Picking Challenge) 
or even grabbing ingredients from the table to cook. In these tasks it's critical
for the robot to understand what it is picking up and exactly where it is in the enviornment.
Perceiving many objects' positions and orientations in cluttered environments
is a challenging task and certain algorithms have been developed to solve this 
by virtually examining every object configuration that could possibly be generated and 
comparing it to the original image the robot sees.  PhysicsEngine contains a program that
takes in an object scene and tests if the objects are in a valid configuration or 
not.  This prevents the algorithm from spending time comparing 1000s of scenes when in reality
it never needed to because those scenes would not exist in real life due to the laws of physics.
The result of PhysicsEngine is that the computation time for the robot's perception algorithm is 
drastically reduced.



Details and lower level goal:

Coded in c++, uses Open Dynamics Engine, Drawstuff, and Eigen libraries.
Given some number of objects along with their exact positions and orientations, the 
goal is to check if this set of objects called a "scene" is in static equilibrium or 
not.  Because reducing computation time is the ultimate goal, I try to accomplish the 
scene validation as fast as possible while still giving good results.  This is why Open
Dynamics Engine was chose as the physics library.  Drawstuff is a primitive graphics 
library which is not used during the actual application of this program. Eigen is briefly 
used to manipulate some of the transform matrices.
