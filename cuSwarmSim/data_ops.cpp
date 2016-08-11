#include "data_ops.h"

void processData(uint n, uint ws, float4* positions, float3* velocities, 
	int* explored_grid, float* data, uint data_size)
{
	// Clear old data in the array
	for (uint i = 0; i < data_size; i++) {
		data[i] = 0.0f;
	}

	///// HEADING AVERAGE /////
	float nf = static_cast<float>(n);
	// Get the average velocity
	float2 avg_vel = make_float2(0.0f, 0.0f);
	for (uint i = 0; i < n; i++) {
		// Get heading for this robot
		float h = atan2(velocities[i].y, velocities[i].x);
		avg_vel.x += cos(h);
		avg_vel.y += sin(h);
	}

	// Finish average
	avg_vel.x /= nf;
	avg_vel.y /= nf;

	float avg_h = atan2(avg_vel.y, avg_vel.x);
	data[0] = avg_h;

	///// HEADING VARIANCE /////
	float var = 0.0f;
	for (uint i = 0; i < n; i++) {
		float h = atan2(velocities[i].y, velocities[i].x);
		float diff = abs(avg_h - h);
		while (diff > PI) {
			diff -= static_cast<float>(2.0 * PI);
		}
		var += abs(diff);
	}

	data[1] = var;

	///// CENTROID /////
	for (uint i = 0; i < n; i++) {
		data[2] += positions[i].x;
		data[3] += positions[i].y;
	}

	data[2] = data[2] / nf;
	data[3] = data[3] / nf;

	///// POSITION BOUNDS and VARIANCE /////
	// Initialize position bounds to minima/maxima
	data[6] = FLT_MAX;
	data[7] = -FLT_MAX;
	data[8] = FLT_MAX;
	data[9] = -FLT_MAX;

	for (uint i = 0; i < n; i++) {
		// Update variance calculation for x and y  coordinates
		data[4] += fabsf(positions[i].x - data[2]);
		data[5] += fabsf(positions[i].y - data[3]);

		// Check bounds of the swarm
		data[6] = min(data[6], positions[i].x);
		data[7] = max(data[7], positions[i].x);
		data[8] = min(data[8], positions[i].y);
		data[9] = max(data[9], positions[i].y);
	}

	data[4] = data[4] / nf;
	data[5] = data[5] / nf;

	///// CONVEX HULL /////
	// Place the positions of the robots in an array for convex hull calculations
	vector<Point> robot_points;
	for (uint i = 0; i < n; i++) {
		Point p_temp;
		p_temp.x = positions[i].x;
		p_temp.y = positions[i].y;
		robot_points.push_back(p_temp);
	}
	// Get the convex hull of the robot positions
	vector<float4> robot_points_ch = convexHull(positions, n);
	// Get the area of the robot convex hull
	data[10] = convexHullArea(robot_points_ch);

	///// EXPLORED AREA /////
	int explored = 0;
	for (uint i = 0; i < ws * ws; i++) {
		explored += explored_grid[i];
	}
	data[11] = static_cast<float>(explored);
}

/*********************************
***** CONVEX HULL FUNCTIONS ******
*********************************/

// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
vector<float4> convexHull(float4* points, uint num)
{
	vector<Point> P;
	for (uint i = 0; i < num; i++) {
		Point p_temp;
		p_temp.x = points[i].x;
		p_temp.y = points[i].y;
		P.push_back(p_temp);
	}

	int n = static_cast<int>(P.size()), k = 0;
	vector<Point> ch(2 * n);

	// Sort points lexicographically
	sort(P.begin(), P.end());

	// Build lower hull
	for (int i = 0; i < n; ++i) {
		while (k >= 2 && cross(ch[k - 2], ch[k - 1], P[i]) <= 0) k--;
		ch[k++] = P[i];
	}

	// Build upper hull
	for (int i = n - 2, t = k + 1; i >= 0; i--) {
		while (k >= t && cross(ch[k - 2], ch[k - 1], P[i]) <= 0) k--;
		ch[k++] = P[i];
	}

	// Resize the convex hull point array
	ch.resize(k);

	// Convert to array of float4 and return
	vector<float4> ch_final;
	for (uint i = 0; i < ch.size(); i++) {
		float4 temp_point = make_float4(ch[i].x, ch[i].y, 0.0f, 0.0f);
		ch_final.push_back(temp_point);
	}
	return ch_final;
}

// Compute the centroid (x, y) of the convex hull given by points
float2 convexHullCentroid(vector<float4> points)
{
	float min_x = FLT_MAX, max_x = -FLT_MAX;
	float min_y = FLT_MAX, max_y = -FLT_MAX;
	for (unsigned int i = 0; i < points.size(); i++) {
		if (min_x > points[i].x) {
			min_x = points[i].x;
		}
		else if (max_x < points[i].x) {
			max_x = points[i].x;
		}
		if (min_y > points[i].y) {
			min_y = points[i].y;
		}
		else if (max_y < points[i].y) {
			max_y = points[i].y;
		}
	}
	float2 to_return = make_float2((min_x + max_x) / 2.0f, (min_y + max_y) / 2.0f);
	return to_return;
}

// Compute the area of the convex hull
float convexHullArea(vector<float4> points)
{
	float area = 0.0f;
	for (int a = 0; static_cast<uint>(a) < points.size(); a++) {
		int b = ((a + 1) % points.size());
		float temp = (points[a].x * points[b].y) - (points[b].x * points[a].y);
		area += temp;
	}
	return area;
}

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross 
// product.
// Returns a positive value, if OAB makes a counter-clockwise turn, negative for 
// clockwise turn, and zero if the points are collinear.
float cross(const Point &O, const Point &A, const Point &B)
{
	return (float)(A.x - O.x) * (B.y - O.y) - (float)(A.y - O.y) * (B.x - O.x);
}

// Convert the float4 array to a vector of points
vector<Point> float4toPointArray(float4* points, uint n) {
	vector<Point> vector_points;
	for (uint i = 0; i < n; i++) {
		Point p;
		p.x = points[i].x;
		p.y = points[i].y;
		vector_points.push_back(p);
	}
	// Return the new vector of points
	return vector_points;
}
