#ifndef RUN_H
#define RUN_H

/********************
***** INCLUDES ******
********************/

// System includes
#include <sstream>
#include <thread>
#include <fstream>
#include <ctime>
#include <iomanip>

// Project includes
#include "kernels.cuh"
#include "data_ops.h"
#include "planning.h"
#include "utils.h"

/*********************
***** VARIABLES ******
*********************/

// Simulation variables
float ws_2;								// Half the length of the world edge
ulong step_num = 0;						// Step counter
bool initial_passed = false;			// Indicates if initial state has passed
bool paused;							// Simulation paused state
Parameters p;							// Parameters structure
float3 goal_vector;						// Goal heading / speed (flocking only)
float goal_heading, goal_heading_err;	// Goal heading and error goal heading
int* explored_grid;						// Grid covering the whole environment, 
										// showing how explored each cell is
float4* positions;						// Robot positions (x, y, z, color)
float3* velocities;						// Robot velocities (x, y, z)
int* modes;								// Robot modes
int* leaders;							// List of current swarm leaders
int* nearest_leaders;					// Nearest leader array
uint* leader_countdowns;				// Leader countdown array
int4* laplacian;						// Laplacian matrix of robot connectivity
										// (max_a, max_b, max_c, max_d)
bool* ap;								// Articulation pts (min vertex cut set)
float4* obstacles;						// List of obstacles in the environment
										// (top_left_x, top_left_y, w, h)
int3* targets;							// List of targets in the environment
										// (x, y, found[0 or 1])
bool* occupancy;						// Occupancy grid for the environment
Data data;								// Data object (see data_ops.cpp for data 
										// calculations)
vector<Decision> sequence;				// Current behavior sequence

float4 goal_region;						// Goal region
bool failure;

// Log file
FILE* output_f, *world_f;				// Main log and world files
std::stringstream output_fname;			// Name of the log file

// GUI variables
float mouse_start_x, mouse_start_y, mouse_last_x, mouse_last_y;
float mouse_x, mouse_y;
int mb = -1;							// Mouse variables
int command_trust = 0;					// Current command to change goal (1) or 
										// due to lack of trust (2);
										// 0 indicates no current command

float rotate_x0, rotate_y0;				// Variables for camera view transform
float translate_x0, translate_y0, translate_z0;

uint frames = 0;						// Frames counter for FPS calculation
uint last_frames = 0;

GLuint vbo_swarm;						// Vertex buffer object and resource
struct cudaGraphicsResource* cuda_vbo_resource;

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

// User interface functions
void drawInterface(float world_size, float window_width, float window_height);
void drawEllipse(float cx, float cy, float w, float h, bool fill);
void drawText(float x, float y, const unsigned char *string, GLfloat r, 
	GLfloat g, GLfloat b);
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
void clearUserPositionEstimate();
void loadParametersFromFile(std::string filename);
void processParam(std::vector<std::string> tokens);
void generateWorld(char* filepath);
void loadSavedMap(char* filepath);
void calculateOccupancyGrid();
bool checkCollision(float x, float y);
bool checkGoalReached();
void generateGoal();
void injectFailure();
void clearFailure();
void updateExplored();
void exitSimulation();
void printDataHeader();

// Automation functions
float getBestHeading();
void printDecisionSequence(vector<Decision> b_seq);
void automateExplore();

// Function to execute each step
static void step(int value);

#endif
