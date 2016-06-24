//This is a file that contains x,y,z positions as well as rotation matrices for various shapes 
//in order to initialize the program will various configurations
//This is not supposed to be a compiled file but viewing it in a c++ edtor such as visual studios is ideal

//....................................................................................................
//TITLE: Dropping four boxes and a sphere from the air
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

//....................................................................................................
//TITLE: Box is balanced on sphere. Four boxes and a sphere in static equilibrium, radius of sphere is 0.2
//ID: 3
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
//TITLE: Boxes stacked, ball in between.  Four boxes and a sphere in static equilibrium.

//Position declarations
const dReal centerSphr[3] = {S1x=0.0,S1y=0,S1z=1.9}; // length of edges
const dReal center1[3] = {B1x=0.0,B1y=0.8,B1z=0.5}; // length of edges
const dReal center2[3] = {B2x=0.0,B2y=-0.8,B2z=0.5}; // length of edges
const dReal center3[3] = {B3x=-0.8,B3y=0.0,B3z=0.5}; // length of edges
const dReal center4[3] = {B4x=0.8,B4y=0.0,B4z=0.5}; // length of edges
//Rotation declarations
const dMatrix3 B1matrix[3][3] = {{ 1.0, 0, 0},
                                 { 0, 1, 0},
                                 { 0, 1, 1}  }; 
const dMatrix3 B2matrix[3][3] = {{ 1, 0, 0},
                                 { 0, 1, 0},
                                 { 0, 0, 1}  }; 
const dMatrix3 B3matrix[3][3] = {{ 1, 0, 0},
                                 { 0, 1, 0},
                                 { 0, 0, 1}  }; 
const dMatrix3 B4matrix[3][3] = {{ 1, 0, 0},
                                 { 0, 1, 0},
                                 { 0, 0, 1}  }; 
//....................................................................................................
//TITLE: leaning block falling.  Example of configuration that requires COUNT to be large OR small Threshold 

//Position declarations
const dReal centerSphr[3] = {S1x=-2.0,S1y=0,S1z=1}; // length of edges
const dReal center1[3] = {B1x=0.0,B1y=0,B1z=0.5}; // length of edges
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


//TITLE: Stacked blocks in high tower. In static equilibrium
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