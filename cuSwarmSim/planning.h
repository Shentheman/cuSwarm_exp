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

/*****************
***** ENUMS ******
*****************/

enum Behavior {
	RENDEZVOUS, FLOCK_UP, FLOCK_RIGHT, FLOCK_DOWN, FLOCK_LEFT, DISPERSE
};

/*******************
***** CLASSES ******
*******************/

// Class for holding a swarm state (with behavior sequence to reach that state)
class SwarmState {
public:
	SwarmState(float4* positions, float3* velocities, int* modes, 
		int** exp_grid, uint decisions, uint n, uint ws);
	float4* pos;
	float3* vel;
	int* mode;
	int** explored;
	Behavior* b_seq;
	float score;
	float heuristic;
};

// Compare class to use with the State class priority queue
class Compare {
public:
	bool operator() (SwarmState a, SwarmState b)
	{
		return (a.score > b.score);
	}
};

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

// Main automation loop
void automate();

#endif
