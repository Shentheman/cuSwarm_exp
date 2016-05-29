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
void processData(uint n, float4* positions, float3* velocities, float* data);

// Convex hull functions
vector<Point> convexHull(vector<Point> points);
float2 convexHullCentroid(vector<Point> points);
float convexHullArea(vector<Point> points);
float cross(const Point &O, const Point &A, const Point &B);

#endif
