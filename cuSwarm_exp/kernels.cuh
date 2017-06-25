#ifndef KERNELS_H
#define KERNELS_H

/********************
***** INCLUDES ******
********************/

// System includes
#include <algorithm>
#include <iostream>
#include <float.h>
#include <cstring>
#include <stdio.h>
#include <assert.h>


// Cuda includes
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <vector_types.h>
//#include "device_launch_parameters.h"

// OpenGL includes
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cuda_gl_interop.h>

// Project includes
#include "utils.h"


/*******************
***** DEFINES ******
*******************/
#ifndef RAY_TRACE_INTERVAL
#define RAY_TRACE_INTERVAL ((float)(PI/45.0f))
#endif

#ifndef NUM_ANGLE_RAY_TRACE
#define NUM_ANGLE_RAY_TRACE  ((int)(ceilf((2.0f*PI-0)/RAY_TRACE_INTERVAL)))
#endif

/// When the last element == -1.0f, the one pos_obs is NULL
#ifndef NULL_POSITION_OBSTACLE
#define NULL_POSITION_OBSTACLE (make_float4(0.0f,0.0f,0.0f,-1.0f))
#endif

/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

/// In kernels.cu, we set each components as (R,G,B,Alpha) in [0,255]
/// Then due to the property of union, the float c is located at the same address
/// as the uchar4. So the float c has 4 bytes. Then the graphic driver of opengl
/// will transform each byte in float c to a uchar and display the color accordingly.
/// For example, red = uchar4 (255,0,0,255) 
/// = float c with 4 bytes (ffffffff,0,0,ffffffff)
union Color
{
  /// 1 float
  float c;
  /// 4 uchars
  uchar4 components;
  /// union is the type where multiple data types are saved in one location
  /// this Color union takes 4 bytes
  /// It transforms 4 chars into 1 float
};

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

// CUDA memory functions
void cudaAllocate(Parameters p);
void cuFree();

// Kernel launches
void launchInitKernel(Parameters p, 
  struct cudaGraphicsResource **vbo_resource);
void launchMainKernel(float3 gh, float2 gp, uint sn, int* leaders, 
  bool* ap, Parameters p, struct cudaGraphicsResource **vbo_resource);
void launchInitKernel(Parameters p);
void launchMainKernel(float3 gh, float2 gp, uint sn, int* leaders, 
  bool* ap, Parameters p);

// CUDA host<->device copy functions
void getData(uint n, uint n_grid, float4* positions, 
  float3* velocities, int* modes,
  float4* positions_obs);
void getData(uint n, uint n_grid, float4* positions, 
  float3* velocities, int* modes, 
  int* nearest_leader, uint* leader_countdown, float4* positions_obs);
void getLaplacian(uint n, int4* laplacian);
void setData(uint n, float4* positions, float3* velocities, int* modes);
void setData(uint n, float4* positions, float3* velocities, int* modes, 
  int* nearest_leader, uint* leader_countdown);
void setOccupancy(Parameters p, bool* occupancy);

/*********************************************
***** FORWARD DECLARED DEVICE FUNCTIONS ******
*********************************************/

__global__ void init_kernel(float4* pos, float3* vel, int* mode, 
  curandState* rand_state, ulong seed, float2* flow_pos, float2* flow_dir, 
  int* nearest_leader, uint* leader_countdown, Parameters p,
  float4* pos_obs, int* counteraaa);

__global__ void side_kernel(float4* pos, int* mode, int* leaders, 
  curandState* rand_state, Parameters p, int* nearest_leader, 
  uint* leader_countdown, int4* laplacian, uint sn);

__global__ void main_kernel(float4* pos, float3* vel, int* mode, 
  float3 goal_heading, float2 goal_point, curandState* rand_state, 
  bool* ap, float2* flow_pos, float2* flor_dir, bool* occupancy, 
  Parameters p, uint sn, 
  float4* pos_obs, int* counteraaa);

__device__ void rendezvous(float3 dist3, float2* min_bounds, 
  float2* max_bounds, float2* repel, bool is_ap, Parameters p);

__device__ void flock(int myMode, float3 nVel, int nMode, float3 dist3, 
  float2* repel, float2* align, float2* cohere, bool is_ap, Parameters p);

__device__ void disperse(float3 dist3, float2* repel, float2* cohere, 
  bool is_ap, Parameters p);

__device__ void rendezvousToPoint(float3 dist3, float2* repel, Parameters p);

__device__ void obstacleAvoidance(float4 myPos, float2* avoid, 
  float* dist_to_obstacle, bool* occupancy, Parameters p, 
  float4* pos_obs, uint robot_index, bool &obs_encountered);

__device__ int occupancySub2Ind(float x, float y, Parameters p);

__device__ void setColorSwarm(uchar4* color, int mode, bool is_ap, uint i, 
  Parameters p, bool obs_encountered);

__device__ void setColorGrid(uchar4* color, int grid);

__device__ float euclidean(float2 vector);

__device__ void rescale(float2* vel, float value, bool is_value_limit);

__device__ void normalizeAngle(float* angle);

__device__ void capAngularVelocity(float2 old, float2* goal, float max);

#endif
