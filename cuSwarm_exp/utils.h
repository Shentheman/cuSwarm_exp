#ifndef UTILS_H
#define UTILS_H

#include <math.h>

/*******************
***** DEFINES ******
*******************/

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#ifndef DEG2RAD
#define DEG2RAD 0.01745329251f
#endif
#ifndef BLOCK_SIZE
#define BLOCK_SIZE 256
#endif
#ifndef NORTH
#define NORTH 1.57079632679f
#endif
#ifndef EAST
#define EAST 0.0f
#endif
#ifndef SOUTH
#define SOUTH -1.57079632679f
#endif
#ifndef WEST
#define WEST 3.14159265358f
#endif

// enum for the GLUT 'button' variable
#ifndef GLUT_NULL_BUTTON
#define GLUT_NULL_BUTTON -1
#endif
//#define GLUT_LEFT_BUTTON 0 // Already defined in glutMouseFunc
//#define GLUT_MIDDLE_BUTTON 1 // Already defined in glutMouseFunc
//#define GLUT_RIGHT_BUTTON 2 // Already defined in glutMouseFunc
#ifndef GLUT_WHEEL_UP_PRESSED
#define GLUT_WHEEL_UP_PRESSED 3
#endif
#ifndef GLUT_WHEEL_DOWN_PRESSED
#define GLUT_WHEEL_DOWN_PRESSED 4
#endif
#ifndef GLUT_WHEEL_UP_RELEASED
#define GLUT_WHEEL_UP_RELEASED 5
#endif
#ifndef GLUT_WHEEL_DOWN_RELEASED
#define GLUT_WHEEL_DOWN_RELEASED 6
#endif

#ifndef BEHAVIOR_RENDEZVOUS
#define BEHAVIOR_RENDEZVOUS 0
#endif
#ifndef BEHAVIOR_FLOCKING
#define BEHAVIOR_FLOCKING 1
#endif
#ifndef BEHAVIOR_DISPERSION
#define BEHAVIOR_DISPERSION 2
#endif
#ifndef BEHAVIOR_RENDEZVOUS_TO_POINT
#define BEHAVIOR_RENDEZVOUS_TO_POINT 3
#endif

#ifndef MODE_NOISE
#define MODE_NOISE -1
#endif
#ifndef MODE_LEADER
#define MODE_LEADER 0
#endif
#ifndef MODE_NON_LEADER_MAX
#define MODE_NON_LEADER_MAX 99
#endif

#ifndef LAPLACIAN_CONNECTED
#define LAPLACIAN_CONNECTED -1
#endif
#ifndef LAPLACIAN_DISCONNECTED
#define LAPLACIAN_DISCONNECTED 0
#endif

#ifndef LEADER_NON_EXIST
#define LEADER_NON_EXIST -1
#endif

//This grid has been explored before and it is an obstacle
//We found this grid to be obstacle in the past
#ifndef GRID_EXPLORED_OBS
#define GRID_EXPLORED_OBS 1
#endif
//This grid is being explored at this moment and it is an obstacle
//We found this grid to be obstacle in this iteration step
#ifndef GRID_EXPLORING_OBS
#define GRID_EXPLORING_OBS 2
#endif
#ifndef GRID_EXPLORING_FREE
#define GRID_EXPLORING_FREE 3
#endif


#ifndef GRID_UNEXPLORED
#define GRID_UNEXPLORED -1
#endif

//navigation direction used in run.cpp, similar to goal_vector
#ifndef GOAL_VECTOR_UP
#define GOAL_VECTOR_UP 1
#endif
#ifndef GOAL_VECTOR_DOWN
#define GOAL_VECTOR_DOWN 2
#endif
#ifndef GOAL_VECTOR_LEFT
#define GOAL_VECTOR_LEFT 3
#endif
#ifndef GOAL_VECTOR_RIGHT
#define GOAL_VECTOR_RIGHT 4
#endif
#ifndef GOAL_VECTOR_NULL
#define GOAL_VECTOR_NULL -1
#endif

#ifndef INF_COORDINATE
#define INF_COORDINATE p.world_size*100
#endif

// Contact-based Coverage Rectilinear (CCR) algorithm
// Go right on floor
#ifndef CCR_STATUS_NULL
#define CCR_STATUS_NULL 0
#endif
// Go right on floor
#ifndef CCR_STATUS_A_ALPHA
#define CCR_STATUS_A_ALPHA 1
#endif
// Go up
#ifndef CCR_STATUS_A_BETA
#define CCR_STATUS_A_BETA 2
#endif
// Go down
#ifndef CCR_STATUS_A_GAMMA
#define CCR_STATUS_A_GAMMA 3
#endif
// Go right on ceiling
#ifndef CCR_STATUS_A_DELTA
#define CCR_STATUS_A_DELTA 4
#endif
// Go right on floor but encounter obstacle
#ifndef CCR_STATUS_F
#define CCR_STATUS_F 5
#endif
// Go right but loss contact with floor
#ifndef CCR_STATUS_D
#define CCR_STATUS_D 6
#endif
// Go up but enconter obstacle
#ifndef CCR_STATUS_E
#define CCR_STATUS_E 7
#endif
// Go up but pass ceiling
#ifndef CCR_STATUS_B
#define CCR_STATUS_B 8
#endif
// Go right but loss contact with ceiling
#ifndef CCR_STATUS_C
#define CCR_STATUS_C 9
#endif










/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

typedef unsigned int uint;
typedef unsigned long ulong;

struct Parameters
{
  int debug;
  float align_weight;
  float ang_bound;
  uint behavior;
  float cohere_weight;
  bool confirm_quit;
  float current;
  uint hops;
  uint leader_selection;
  bool log_data;
  float range_r;
  float range_f;
  float range_d;
  float range_o;
  float range_l;
  float range;
  uint max_explore;
  uint max_obstacle_size;
  float noise;
  uint num_obstacles;
  uint num_robots;
  uint point_size;
  float repel_weight;
  bool add_failures;
  bool show_ap;
  bool highlight_leaders;
  bool highlight_pioneers;
  bool show_connections;
  bool show_convex_hull;
  bool show_explored;
  bool show_info_graph;
  bool show_leaders;
  bool show_range;
  bool show_non_leaders;
  bool query_trust;
  float start_size;
  uint step_limit;
  bool training;
  uint targets;
  float vel_bound;
  uint window_height;
  uint window_width;
  uint world_size;
};

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

float eucl2(float x1, float y1, float x2, float y2);

#endif
