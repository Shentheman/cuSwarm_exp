#include "planning.h"

SwarmState::SwarmState(float4* positions, float3* velocities, int* modes,
	int** exp_grid, uint decisions, uint n, uint world_size)
{
	// Initialize arrays and values
	pos = (float4*)malloc(n * sizeof(float4));
	vel = (float3*)malloc(n * sizeof(float3));
	mode = (int*)malloc(n * sizeof(int));
	fill(explored, explored + world_size, new int[world_size]);
	for (uint i = 0; i < world_size; i++) {
		explored[i] = new int[world_size];
	}
	b_seq = (Behavior*)malloc(10 * sizeof(Behavior));
	score = 0.0f;
	heuristic = 0.0f;

	// Copy data to arrays
	for (uint i = 0; i < n; i++) {
		pos[i] = positions[i];
		vel[i] = velocities[i];
		mode[i] = modes[i];
	}
	for (uint i = 0; i < world_size; i++) {
		for (uint j = 0; j < world_size; j++) {
			explored[i][j] = exp_grid[i][j];
		}
	}
}
