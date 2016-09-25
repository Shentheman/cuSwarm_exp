#include "planning.h"

SwarmState::SwarmState(float4* positions, float3* velocities, int* modes, int* n_l, 
	uint* l_c, uint sn, uint n, uint world_size)
{
	// Initialize arrays and values
	pos = (float4*)malloc(n * sizeof(float4));
	vel = (float3*)malloc(n * sizeof(float3));
	mode = (int*)malloc(n * sizeof(int));
	nearest_leader = (int*)malloc(n * sizeof(int));
	leader_countdown = (uint*)malloc(n * sizeof(uint));
	explored = (int*)malloc(world_size * world_size * sizeof(int));
	score = 0.0f;
	heuristic = 0.0f;
	connectivity = 0.0f;
	step_num = sn;

	// Copy initial robot data to arrays
	for (uint i = 0; i < n; i++) {
		pos[i] = positions[i];
		vel[i] = velocities[i];
		mode[i] = modes[i];
		nearest_leader[i] = n_l[i];
		leader_countdown[i] = l_c[i];
	}
	// Create a blank explored map
	for (uint i = 0; i < world_size * world_size; i++) {
		explored[i] = 0;
	}
}

SwarmState::SwarmState(float4* positions, float3* velocities, int* modes, int* n_l,
	uint* l_c, int* exp_grid, vector<Decision> seq, uint sn, float s, float h, 
	float c, uint n, uint world_size)
{
	// Initialize arrays and values
	pos = (float4*)malloc(n * sizeof(float4));
	vel = (float3*)malloc(n * sizeof(float3));
	mode = (int*)malloc(n * sizeof(int));
	nearest_leader = (int*)malloc(n * sizeof(int));
	leader_countdown = (uint*)malloc(n * sizeof(uint));
	explored = (int*)malloc(world_size * world_size * sizeof(int));
	score = s;
	heuristic = h;
	connectivity = c;
	step_num = sn;

	// Copy robot data to arrays
	for (uint i = 0; i < n; i++) {
		pos[i] = positions[i];
		vel[i] = velocities[i];
		mode[i] = modes[i];
		nearest_leader[i] = n_l[i];
		leader_countdown[i] = l_c[i];
	}
	// Copy behavior sequence to array
	for (uint i = 0; i < seq.size(); i++) {
		b_seq.push_back(seq[i]);
	}
	// Copy explored data to array
	for (uint i = 0; i < world_size * world_size; i++) {
		explored[i] = exp_grid[i];
	}
}

SwarmState::~SwarmState()
{
	delete[] pos;
	delete[] vel;
	delete[] mode;
	delete[] nearest_leader;
	delete[] leader_countdown;
	delete[] explored;
}
