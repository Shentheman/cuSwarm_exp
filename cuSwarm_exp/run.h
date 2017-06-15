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

// Project includes
#include "kernels.cuh"
#include "data_ops.h"
#include "utils.h"

// enum for the GLUT 'button' variable
#define GLUT_NULL_BUTTON -1
//#define GLUT_LEFT_BUTTON 0 // Already defined in glutMouseFunc
//#define GLUT_MIDDLE_BUTTON 1 // Already defined in glutMouseFunc
//#define GLUT_RIGHT_BUTTON 2 // Already defined in glutMouseFunc
#define GLUT_WHEEL_UP_PRESSED 3
#define GLUT_WHEEL_DOWN_PRESSED 4
#define GLUT_WHEEL_UP_RELEASED 5
#define GLUT_WHEEL_DOWN_RELEASED 6
#define BEHAVIOR_RENDEZVOUS 0
#define BEHAVIOR_FLOCKING 1
#define BEHAVIOR_DISPERSION 2
#define BEHAVIOR_RENDEZVOUS_TO_POINT 3




/*********************
***** VARIABLES ******
*********************/

// Simulation variables
float ws_2;               // Half the length of the world edge
ulong step_num = 0;           // Step counter
bool initial_passed = false;      // Indicates if initial state has passed
bool paused;              // Simulation paused state
Parameters p;             // Parameters structure
float3 goal_vector;           // Goal heading / speed (flocking only)
float goal_heading, goal_heading_err; // Goal heading and error goal heading
float2 goal_point;            // Goal point (for rendezvous to point)
int* explored_grid;           // Grid covering the whole environment, 
                    // showing how explored each cell is
float4* positions;            // Robot positions (x, y, z, color)
float3* velocities;           // Robot velocities (x, y, z)
int* modes;               // Robot modes
int* leaders;             // List of current swarm leaders
int* nearest_leaders;         // Nearest leader array
uint* leader_countdowns;        // Leader countdown array
int4* laplacian;            // Laplacian matrix of robot connectivity
                    // (range, range_r, range_f, range_l)
bool* ap;               // Articulation pts (min vertex cut set)
float4* obstacles;            // List of obstacles in the environment
                    // (top_left_x, top_left_y, w, h)
int3* targets;              // List of targets in the environment
                    // (x, y, unseen(0) seen(1) or found(2))
uint2* failures;              // Simulation step points for start/end of failures
uint minutes;             // Number of simulation minutes
bool* occupancy;            // Occupancy grid for the environment
Data data;                // Data object (see data_ops.cpp for data calculations)

// Variables for trust tracking
int command_trust = 0;          // Current command to change goal (1) or due to lack of
                    // trust (2); 0 indicates no current command
bool trust_verified = true;       // If user has verified their trust level

// Datas for drawing information graphs
int* targets_by_second;         // Targets found during each second
float* heading_var_by_second;     // Heading variance at each second
float* area_by_second;          // Swarm area covered at each second

// GUI variables
float mouse_start_x, mouse_start_y, mouse_last_x, mouse_last_y;
int mb = -1;              // Mouse variables

float rotate_x0, rotate_y0;       // Variables for camera view transform
float translate_x0, translate_y0, translate_z0;

uint frames = 0;            // Frames counter for FPS calculation
uint last_frames = 0;

GLuint vbo_swarm;           // Vertex buffer object and resource
struct cudaGraphicsResource* cuda_vbo_resource;

// Log file
FILE* output_f;             // Main log files
std::stringstream output_fname;     // Name of the log file

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
void createVBO(GLuint* vbo);
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

// Function to execute each step
static void step(int value);

#endif
