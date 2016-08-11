#ifndef DATA_OPS_H
#define DATA_OPS_H

#include <math.h>
#include <stack>
#include <vector>
#include <stdlib.h>

#include "kernels.cuh"

using namespace std;

struct Point {
	float x, y;

	bool operator <(const Point &p) const {
		return x < p.x || (x == p.x && y < p.y);
	}
};

// Data processing main function
void processData(uint n, uint ws, float4* positions, float3* velocities, 
	int* explored_grid, float* data, uint data_size);

// Convex hull functions
vector<float4> convexHull(float4* points, uint n);
float2 convexHullCentroid(vector<float4> points);
float convexHullArea(vector<float4> points);
float cross(const Point &O, const Point &A, const Point &B);

#endif
