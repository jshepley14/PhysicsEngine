# PhysicsEngine

High level view for this project:

Perceiving many objects' positions and orientations in cluttered environments
is a challenging task and certain algorithms have been developed to solve this 
by virtually examining every object configuration that could possibly exists and 
comparing it to the original image the robot sees.  PhysicsEngine contains code that
takes in an object scene and tests if the objects are in a valid configuration or 
not before comparing them to the original image the robot sees, and the result is
that the computation time is drastically reduced.



Details and lower level goal:

Coded in c++, uses Open Dynamics Engine, Drawstuff, and Eigen libraries.
Given some number of objects along with their exact positions and orientations, the 
goal is to check if this set of objects called a "scene" is in static equilibrium or 
not.  Because reducing computation time is the ultimate goal, I try to accomplish the 
scene validation as fast as possible while still giving good results.  This is why Open
Dynamics Engine was chose as the physics library.  Drawstuff is a primitive graphics 
library which is not used during the actual application of this program. Eigen is briefly 
used to manipulate some of the transform matrices.
