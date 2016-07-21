#ifndef RUN_H
#define RUN_H

/********************
***** INCLUDES ******
********************/

// System includes
#include <sstream>
#include <math.h>
#include <thread>
#include <fstream>

// Project includes
#include "kernels.cuh"
#include "data_ops.h"

/*********************
***** VARIABLES ******
*********************/

// Simulation variables
uint num_robots;						// Number of robots
uint ws_uint;							// Length of a side of the square world
float ws_2;								// Half the length of the world edge
ulong step_num = 0;						// Step counter
bool paused;							// Simulation paused state
Parameters p;							// Parameters structure
float3 goal_vector;						// Goal heading / speed (flocking only)
float goal_heading;						// goal heading only
int** explored_grid;					// Grid covering the whole environment, 
										// showing how explored each cell is

// Data variables
float4* positions;						// Robot positions
float3* velocities;						// Robot velocities
int* modes;								// Robot modes
int* leaders;							// List of current swarm leaders
int* nearest_leaders;					// Nearest leader array
uint* leader_countdowns;				// Leader countdown array
float4* obstacles;						// List of obstacles in the environment
bool* occupancy;						// Occupancy grid for the environment
float* data;							// Data array (see data_ops.cpp)
vector<Point> robot_ch;					// Convex hull of the robots

// Log file
FILE* output;							// Main log file
std::stringstream filename;				// Name of the log file
bool log_created = false;				// Indicates if log file has been created

#ifdef GUI
float mouse_start_x, mouse_start_y, mouse_last_x, mouse_last_y;
int mb = -1;							// Mouse variables

int screen_width, screen_height;		// Variables for screen dimensions

float rotate_x0, rotate_y0;				// Variables for camera view transform
float translate_x0, translate_y0, translate_z0;

uint frames = 0;						// Frames counter for FPS calculation
uint last_frames = 0;

GLuint vbo_swarm;						// Vertex buffer object and resource
struct cudaGraphicsResource* cuda_vbo_resource;
#endif

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

#ifdef GUI
// User interface functions
void drawInterface(float world_size, float window_width, float window_height);
void drawEllipse(float cx, float cy, float w, float h);
void keyboard(unsigned char key, int x, int y);
void keyboardSpecial(int key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void moveUp();
void moveLeft();
void moveRight();
void moveDown();

// OpenGL utility functions
void initGL(int argc, char **argv);
void createVBO(GLuint* vbo);
void deleteVBO(GLuint* vbo, struct cudaGraphicsResource ** vbo_res);
static void display(void);

// Helper functions
void calculateFPS(int value);
void screenToWorld(float3 screen, float3 *world);
void worldToScreen(float3 world, float2 *screen);
void drawText(float x, float y, float x_scale, float y_scale, const char *string, 
	GLfloat r, GLfloat g, GLfloat b);
void resetCamera();
void clearUserPositionEstimate();
#endif
float eucl2(float x1, float y1, float x2, float y2);
void loadParameters(std::string filename);
void generateObstacles();
void calculateOccupancyGrid();
bool checkCollision(float x, float y);
void updateExplored();
void exitSimulation();
#ifdef LOGGING
void printDataHeader();
#endif
#if defined(GUI) && defined(LOGGING)
void logUserHeadingCommand();
#endif

// Function to execute each step
static void step(int value);

#endif
