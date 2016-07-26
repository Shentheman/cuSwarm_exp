#ifndef GPU_SWARM_H
#define GPU_SWARM_H

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

/********************
***** INCLUDES ******
********************/

// System includes
#include <algorithm>
#include <iostream>

// Cuda includes
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <vector_types.h>
#include "device_launch_parameters.h"

// OpenGL includes
#include <gl/glew.h>
#include <gl/freeglut.h>
#include <cuda_gl_interop.h>

/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

typedef unsigned int uint;
typedef unsigned long ulong;

union Color
{
	float c;
	uchar4 components;
};

struct Parameters
{
	float align_weight;
	float automated;
	float ang_bound;
	float behavior;
	float cohere_weight;
	float current;
	float explore_cell_size;
	float hops;
	float information_mode;
	float max_a;
	float max_b;
	float max_c;
	float max_d;
	float max_explore;
	float max_obstacle_size;
	float noise;
	float num_obstacles;
	float num_robots;
	float point_size;
	float repel_weight;
	float show_gui;
	float update_period;
	float vel_bound;
	float window_height;
	float window_width;
	float world_size;
};

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

void cudaAllocate(Parameters p, bool* occupancy);
void cuFree();

void launchInitKernel(Parameters p, struct cudaGraphicsResource **vbo_resource);
void launchMainKernel(float3 gp, uint sn, Parameters p,
	struct cudaGraphicsResource **vbo_resource);
void launchInitKernel(Parameters p);
void launchMainKernel(float3 gp, uint sn, Parameters p);

void getData(uint n, float4* positions, float3* velocities, int* modes);
void getData(uint n, float4* positions, float3* velocities, int* modes, 
	int* nearest_leader, uint* leader_countdown);

/*********************************************
***** FORWARD DECLARED DEVICE FUNCTIONS ******
*********************************************/

__global__ void init_kernel(float4* pos, float3* vel, int* mode, 
	curandState* rand_state, ulong seed, float2* flow_pos, float2* flow_dir, 
	int* nearest_leader, uint* leader_countdown, Parameters p);

__global__ void side_kernel(float4* pos, int* mode, curandState* rand_state, 
	Parameters p, int* nearest_leader, uint* leader_countdown, uint sn);

__global__ void main_kernel(float4* pos, float3* vel, int* mode, 
	float3 goal_heading, curandState* rand_state, float2* flow_pos, float2* flor_dir, 
	bool* occupancy, Parameters p, uint sn);

__device__ void rendezvous(float4 myPos, float4 nPos, float3 nVel, float3 dist3, 
	float2* min_bounds, float2* max_bounds, float2* repel, Parameters p);

__device__ void flock(float4 myPos, float3 myVel, int myMode, float4 nPos,
	float3 nVel, int nMode, float3 dist4, float2* repel, float2* align, 
	float2* cohere, Parameters p);

__device__ void disperse(float4 myPos, float4 nPos, float3 nVel, float3 dist3, 
	float2* repel, float2* cohere, Parameters p);

__device__ void obstacleAvoidance(float4 myPos, float2* avoid, 
	float* dist_to_obstacle, bool* occupancy, Parameters p);

__device__ bool checkOccupancy(float x, float y, bool* occupancy, Parameters p);

__device__ void setColor(uchar4* color, int mode, Parameters p);

__device__ float euclidean(float2 vector);

__device__ void rescale(float2* vel, float value, bool is_value_limit);

__device__ void normalizeAngle(float* angle);

__device__ void capAngularVelocity(float2 old, float2* goal, float max);

#endif
