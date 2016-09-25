#include "data_ops.h"

///////////////////////
///// GRAPH CLASS /////
///////////////////////

class Graph {
private:
	// V is the number of vertices in the graph
	// time is used to determine when a node has a back-link to one of its ancestors
	int V, time;

	// adjList[u] is the adjacency list of vertex u, 0 <= u < V
	vector <int> *adjList;

	// explored[u] is true if u has been explored
	// articulation_point[u] is true is u is an articulation point
	bool *explored, *articulation_point, done;

	// disc_time[u] is the time at which vertex u was explored
	// parent[u] = v if in the dfs tree, there is an edge from v to u
	// low[u] is the time of the earliest explored vertex reachable from u
	// If low[u] < disc_time[u], then there is a back-link from some node in the 
	// subtree rooted at u to some ancestor of u
	int *disc_time, *parent, *low;

	// articulation_points stores the articulation points/cut vertices in the graph
	vector <int> articulation_points;

	void dfsUtil(int u) {
		explored[u] = true;
		int num_child = 0;
		disc_time[u] = low[u] = ++time;

		for (vector <int>::iterator v = adjList[u].begin(); v != adjList[u].end(); v++)	{
			if (!explored[*v])	{
				num_child++;
				parent[*v] = u;
				dfsUtil(*v);
				low[u] = min(low[u], low[*v]);

				// u is an articulation point iff
				// 1. It the the root and has more than 1 child.
				// 2. It is not the root and no vertex in the subtree rooted at one of its
				//    children has a back-link to its ancestor.
				//    A child has a back-link to an ancestor of its parent when its low
				//    value is less than the discovery time of its parent.
				if (parent[u] == -1 && num_child > 1)
					articulation_point[u] = true;
				else if (parent[u] != -1 && low[*v] >= disc_time[u])
					articulation_point[u] = true;
			}
			else if (*v != parent[u])
				low[u] = min(low[u], disc_time[*v]);
		}
	}

	void dfs()    {
		for (int u = 0; u < V; u++)
			if (!explored[u])
				dfsUtil(u);
	}

public:

	// create an empty undirected graph having V vertices
	Graph(int V) {
		this->V = V;
		time = 0;
		done = false;

		adjList = new vector <int>[V];
		explored = new bool[V];
		articulation_point = new bool[V];
		disc_time = new int[V];
		parent = new int[V];
		low = new int[V];

		memset(explored, false, V * sizeof(bool));
		memset(articulation_point, false, V * sizeof(bool));
		memset(parent, -1, V * sizeof(int));
	}

	~Graph()    {
		delete[] adjList;
		delete[] articulation_point;
		delete[] explored;
		delete[] parent;
		delete[] disc_time;
		delete[] low;
	}

	// add an undirected edge (u, v) to the graph
	// returns false if either u or v is less than 0 or greater than equal to V
	// returns true if the edge was added to the digraph
	bool addEdge(int u, int v)  {
		if (u < 0 || u >= V) return false;
		if (v < 0 || v >= V) return false;
		adjList[u].push_back(v);
		adjList[v].push_back(u);
		return true;
	}

	// Performs dfs over the graph and returns a vector containing
	// the articulation points
	vector <int> getArticulationPoints()	{
		if (done)
			return articulation_points;
		dfs();
		done = true;
		for (int u = 0; u < V; u++)
			if (articulation_point[u])
				articulation_points.push_back(u);
		return articulation_points;
	}
};

///////////////////////////
///// END GRAPH CLASS /////
///////////////////////////

void processData(uint n, uint ws, float4* positions, float3* velocities, 
	int* explored_grid, Data* data)
{
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
	data->heading_avg = avg_h;

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

	data->heading_var = var;

	///// CENTROID /////
	for (uint i = 0; i < n; i++) {
		data->centroid.x += positions[i].x;
		data->centroid.y += positions[i].y;
	}

	data->centroid.x = data->centroid.x / nf;
	data->centroid.y = data->centroid.y / nf;

	///// POSITION BOUNDS and VARIANCE /////
	// Initialize position bounds to minima/maxima
	data->bounds.x = FLT_MAX;
	data->bounds.y = -FLT_MAX;
	data->bounds.z = FLT_MAX;
	data->bounds.w = -FLT_MAX;

	for (uint i = 0; i < n; i++) {
		// Check bounds of the swarm
		data->bounds.x = min(data->bounds.x, positions[i].x);
		data->bounds.y = max(data->bounds.y, positions[i].x);
		data->bounds.z = min(data->bounds.z, positions[i].y);
		data->bounds.w = max(data->bounds.w, positions[i].y);
	}

	///// CONVEX HULL /////
	// Convex hull area is computed in the step() function in run.cpp
	data->ch_area = 0.0f;

	///// EXPLORED AREA /////
	int explored = 0;
	for (uint i = 0; i < ws * ws; i++) {
		explored += abs(explored_grid[i]);
	}
	data->score = static_cast<float>(explored);
}

/*********************************
***** CONVEX HULL FUNCTIONS ******
*********************************/

// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
void convexHull(float4* pos, vector<float4>* points, vector<uint>* indicies, 
	uint num)
{
	int n = static_cast<int>(num), k = 0;
	vector<Point> P;
	for (int i = 0; i < n; i++) {
		Point p_temp;
		p_temp.x = pos[i].x;
		p_temp.y = pos[i].y;
		P.push_back(p_temp);
	}

	vector<Point> ch(2 * n);
	vector<uint> ch_i(2 * n);

	// Sort points lexicographically
	sort(P.begin(), P.end());

	// Build lower hull
	for (int i = 0; i < n; i++) {
		while (k >= 2 && cross(ch[k - 2], ch[k - 1], P[i]) <= 0) k--;
		ch[k] = P[i];
		ch_i[k] = i;
		k++;
	}

	// Build upper hull
	for (int i = n - 2, t = k + 1; i >= 0; i--) {
		while (k >= t && cross(ch[k - 2], ch[k - 1], P[i]) <= 0) k--;
		ch[k] = P[i];
		ch_i[k] = i;
		k++;
	}

	// Resize the convex hull point array
	ch.resize(k);
	ch_i.resize(k);

	// Convert to array of float4/uint
	for (uint i = 0; i < ch.size(); i++) {
		float4 temp_point = make_float4(ch[i].x, ch[i].y, 0.0f, 0.0f);
		points->push_back(temp_point);
	}
	for (uint i = 0; i < ch_i.size(); i++) {
		indicies->push_back(ch_i[i]);
	}
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

float connectivity(uint n, int4* laplacian, uint level)
{
	// Laplacian matrix in Eigen form
	Eigen::MatrixXf A(n, n);
	// Populate Eigen matrix
	for (uint i = 0; i < n; i++) {
		for (uint j = 0; j < n; j++) {
			// Get connectivity based on robot range level specified to compute 
			// laplacian of communication graph (1 = max_a ... 4 = max_d).
			switch (level) {
			case 1:
				A(i, j) = static_cast<float>(laplacian[(i * n) + j].x);
				break;
			case 2:
				A(i, j) = static_cast<float>(laplacian[(i * n) + j].y);
				break;
			case 3:
				A(i, j) = static_cast<float>(laplacian[(i * n) + j].z);
				break;
			case 4:
			default:
				// Report max_d (maximum comm range) by default
				A(i, j) = static_cast<float>(laplacian[(i * n) + j].w);
				break;
			}
			
		}
	}

	// Get eigenvalues of laplacian
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(A);

	// Return second-smallest eigenvalue
	return eigensolver.eigenvalues()[1];
}

void articulationPoints(uint n, int4* laplacian, bool* ap, uint level)
{
	// Clear the articulation point vector
	for (uint i = 0; i < n; i++) {
		ap[i] = false;
	}

	// Graph object
	Graph g(n);

	// Create graph
	for (uint i = 0; i < n; i++) {
		for (uint j = i + 1; j < n; j++) {
			bool connected;
			// Compute articulation points based on the robot range level specified
			// for the laplacian (1 = max_a ... 4 = max_d).
			switch (level) {
			case 1:
				connected = laplacian[(i * n) + j].x == -1;
				break;
			case 2:
				connected = laplacian[(i * n) + j].y == -1;
				break;
			case 3:
				connected = laplacian[(i * n) + j].z == -1;
				break;
			case 4:
			default:
				connected = laplacian[(i * n) + j].w == -1;
				break;
			}
			if (connected) {
				g.addEdge(i, j);
			}
		}
	}

	// Get articulation points of graph
	vector<int> points = g.getArticulationPoints();
	for (uint i = 0; i < points.size(); i++) {
		ap[points[i]] = true;
	}
}
