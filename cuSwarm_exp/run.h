#ifndef RUN_H
#define RUN_H

/********************
***** INCLUDES ******
********************/

// System includes
#include <sstream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <queue>
#include <iterator>
#include <set>

// Project includes
#include "kernels.cuh"
#include "data_ops.h"
#include "utils.h"

#include "cell.h"



/*********************
***** VARIABLES ******
*********************/

// 1. Simulation variables
// Half the length of the world edge
float ws_2;               
// Step counter
ulong step_num = 0;           
// Indicates if initial state has passed
bool initial_passed = false;      
// Simulation paused state
bool paused;              
// Parameters structure
Parameters p;             
// Goal heading / speed (flocking only)
float3 goal_vector;           
// The goal direction in a discrete way, e.g. up/down/left/right
int goal_vector_discrete;
// Goal heading and error goal heading
float goal_heading, goal_heading_err; 
// Goal point (for rendezvous to point)
float2 goal_point;            
// Grid covering the whole environment, showing how explored each cell is
int* explored_grid;           
                    
// 2. Robot positions (x, y, z, color)
float4* positions;            
/// Array on host: the positions of the obstacles detected 
/// at this time stamp from cuda device
float4* positions_obs_from_cuda;
/// Array on local: the positions of all the obstacles 
/// which have been detected so far from positions_obs_from_cuda
float4* positions_obs;
// Robot velocities (x, y, z)
float3* velocities;           
// Robot modes
int* modes;               
// List of current swarm leaders
int* leaders;             
// Nearest leader array
int* nearest_leaders;         
// Leader countdown array
uint* leader_countdowns;        
// Laplacian matrix of robot connectivity (range, range_r, range_f, range_l)
int4* laplacian;            
// Articulation pts (min vertex cut set)
bool* ap;               
// List of obstacles in the environment (top_left_x, top_left_y, w, h)
float4* obstacles;            
// List of targets in the environment (x, y, unseen(0) seen(1) or found(2))
int3* targets;              
// Simulation step points for start/end of failures
uint2* failures;              
// Number of simulation minutes
uint minutes;             
// Occupancy grid for the environment
bool* occupancy;            
// Data object (see data_ops.cpp for data calculations)
Data data;                

// 3. Variables for trust tracking
// Current command to change goal (1) or due to lack of
// trust (2); 0 indicates no current command
int command_trust = 0;          
// If user has verified their trust level
bool trust_verified = true;       

// 4. Datas for drawing information graphs
// Targets found during each second
int* targets_by_second;         
// Heading variance at each second
float* heading_var_by_second;     
// Swarm area covered at each second
float* area_by_second;          

// 5. GUI variables
// Mouse variables
float mouse_start_x, mouse_start_y, mouse_last_x, mouse_last_y;
int mb = -1;              
// Variables for camera view transform
float rotate_x0, rotate_y0;       
float translate_x0, translate_y0, translate_z0;
// Frames counter for FPS calculation
uint frames = 0;            
uint last_frames = 0;
// Vertex buffer object and resource
GLuint vbo_name;           
struct cudaGraphicsResource* cuda_vbo_resource;

// 6. Log file
// Main log files
FILE* output_f;             
// Name of the log file
std::stringstream output_fname;     

// 7. Contact-based Coverage Rectilinear (CCR) algorithm
std::vector<Cell> cells;
std::vector<std::tuple<float2,float2> > CCR_placeholders;
float prev_max_x;
float prev_min_x;
float prev_max_y;
float prev_min_y;
int CCR_status;




/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

// User interface functions
void drawInterface(float world_size, float window_width, float window_height);
void drawEllipse(float cx, float cy, float w, float h, bool fill);
void drawText(float x, float y, char *string, GLfloat r, GLfloat g, GLfloat b);
void keyboard(unsigned char key, int x, int y);
void keyboardSpecial(int key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void moveUp();
void moveLeft();
void moveRight();
void moveDown();
void glResetModelAndProjection();

// OpenGL utility functions
void initGL(int argc, char **argv);
//void createVBO(GLuint* vbo);
void createVBO(GLuint* vbo, struct cudaGraphicsResource **vbo_res,
    unsigned int vbo_res_flags, uint size);
void deleteVBO(GLuint* vbo, struct cudaGraphicsResource ** vbo_res);
static void display(void);
void screenToWorld(float3 screen, float3 *world);
void worldToScreen(float3 world, float2 *screen);
void resetCamera();

// Helper functions
void calculateFPS(int value);
void loadParametersFromFile(std::string filename);
void processParam(std::vector<std::string> tokens);
void generateWorld();
void calculateOccupancyGrid();
bool checkCollision(float x, float y);
void promptTrust(int a);
void updateExplored();
void exitSimulation();
void printDataHeader();
bool EqualFloat3(const float3 src1, const float3 src2);

// Function to execute each step
static void step(int value);

#endif
