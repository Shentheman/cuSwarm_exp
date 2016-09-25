#ifndef PLANNING_H
#define PLANNING_H

/********************
***** INCLUDES ******
********************/

using namespace std;

// System includes
#include <queue>

// Project includes
#include "kernels.cuh"

/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

typedef struct {
	uint behavior;
	float flock_dir;	// 0.0 if rendezvous or dispersion
	uint time_start;
	uint time_end;
	float score_start;
	float score_end;
} Decision;

/*******************
***** CLASSES ******
*******************/

// Class for holding a swarm state (with behavior sequence to reach that state)
class SwarmState {
public:
	SwarmState(float4* positions, float3* velocities, int* modes, int* n_l, 
		uint* l_c, uint sn, uint n, uint ws);
	SwarmState(float4* positions, float3* velocities, int* modes, int* n_l,
		uint* l_c, int* exp_grid, vector<Decision> seq, uint sn, float s, 
		float h, float c, uint n, uint ws);
	~SwarmState();
	float4* pos;
	float3* vel;
	int* mode;
	int* nearest_leader;
	uint* leader_countdown;
	int* explored;
	vector<Decision> b_seq;
	float score;
	float heuristic;
	float connectivity;
	uint step_num;
};

// Compare class to use with the State class priority queue
class Compare {
public:
	bool operator() (SwarmState* a, SwarmState* b)
	{
		return (a->score + a->heuristic < b->score + b->heuristic);
	}
};

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

// Main automation loop
void automateExplore();
void automateFlockToGoal();

#endif
