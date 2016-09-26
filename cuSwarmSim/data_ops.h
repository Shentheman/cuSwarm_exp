#ifndef DATA_OPS_H
#define DATA_OPS_H

// System includes
#include <vector>

// External library includes
#include <Eigen/Dense>

// Project includes
#include "kernels.cuh"

using namespace std;

/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

struct Data
{
	float2 centroid;
	float4 bounds;
	float score;
	float heading_avg;
	float heading_var;
	float ch_area;
	float connectivity;
};

struct Point {
	float x, y;

	bool operator <(const Point &p) const {
		return x < p.x || (x == p.x && y < p.y);
	}
};

/************************************
**** FORWARD DECLARED FUNCTIONS******
************************************/

// Data processing main function
void processData(uint n, uint ws, float4* positions, float3* velocities, 
	int* explored_grid, int4* laplacian, bool* ap, Data* data);

// Convex hull functions
void convexHull(float4* pos, vector<float4>* points, vector<uint>* indicies,   
	uint n);
float2 convexHullCentroid(vector<float4> points);
float convexHullArea(vector<float4> points);
float cross(const Point &O, const Point &A, const Point &B);

// Eigenvalue functions
float connectivity(uint n, int4* laplacian, uint level);
void articulationPoints(uint n, int4* laplacian, bool* ap, uint level);

#endif
