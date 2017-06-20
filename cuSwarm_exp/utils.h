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

/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

typedef unsigned int uint;
typedef unsigned long ulong;

struct Parameters
{
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
