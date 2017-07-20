#include "kernels.cuh"

/*********************
***** VARIABLES ******
*********************/

// Device pointers for simulation agent data arrays
/// This is the back end of d_vbo_resource on device
/// (float x, float y, float z, float color)
float4* d_positions;
/// (float x, float y, float speed)
float3* d_velocities;
int* d_modes;
int* d_leaders;
int* d_nearest_leader;
uint* d_leader_countdown;
int4* d_laplacian;
bool* d_ap;
/// The positions of all the obstacles
/// (float x, float y, int flag, float color) (we replace z with flag)
float4* d_positions_obs;
/* front end = the array on the device which connected to the VBO on the host
 * back end = the array on the device used for updating the VBO on the host
 * The reason to have both front and back is to have the back end save data
 * from the past and update the front end in each step. Without the back end,
 * in each step, the front end will be updated entirely.
 */
// The front end buffer which connects to vbo_resource in the host
float4* d_vbo_resource;

// Device pointers for environment variables
bool* d_occupancy;
float2* d_flow_pos;
float2* d_flow_dir;

// CUDA random states
curandState* d_rand_states;

// Kernel launch parameters
uint grid_dim;
dim3 block;
dim3 grid;

// CUDA streams
cudaStream_t streams[4];

/****************************
***** HELPER FUNCTIONS ******
****************************/

void cudaAllocate(Parameters p) {

  // Allocate space on device for simulation arrays
  cudaMalloc(&d_positions, p.num_robots*sizeof(float4));
  cudaMemset(d_positions, 0, p.num_robots*sizeof(float4));

  cudaMalloc(&d_velocities, p.num_robots * sizeof(float3));
  cudaMemset(d_velocities, 0, p.num_robots*sizeof(float3));

  cudaMalloc(&d_modes, p.num_robots * sizeof(int));
  cudaMemset(d_modes, 0, p.num_robots*sizeof(int));

  cudaMalloc(&d_leaders, p.num_robots * sizeof(int));
  cudaMemset(d_leaders, 0, p.num_robots*sizeof(int));

  cudaMalloc(&d_nearest_leader, p.num_robots * sizeof(int));
  cudaMemset(d_nearest_leader, 0, p.num_robots*sizeof(int));

  cudaMalloc(&d_leader_countdown, p.num_robots * sizeof(uint));
  cudaMemset(d_leader_countdown, 0, p.num_robots*sizeof(uint));

  cudaMalloc(&d_laplacian, p.num_robots*p.num_robots*sizeof(int4));
  cudaMemset(d_laplacian, 0, p.num_robots*p.num_robots*sizeof(int4));

  cudaMalloc(&d_ap, p.num_robots * sizeof(bool));
  cudaMemset(d_ap, 0, p.num_robots*sizeof(bool));

  // Allocate space on device for environment arrays
  cudaMalloc(&d_occupancy, p.world_size*p.world_size*10*10*sizeof(bool));
  cudaMemset(d_occupancy, 0, p.world_size*p.world_size*10*10*sizeof(bool));

  cudaMalloc(&d_flow_pos, 256*sizeof(float2));
  cudaMemset(d_flow_pos, 0, 256*sizeof(float2));

  cudaMalloc(&d_flow_dir, 256*sizeof(float2));
  cudaMemset(d_flow_dir, 0, 256*sizeof(float2));

  // Allocate space on device for random state variables
  cudaMalloc(&d_rand_states, p.num_robots * sizeof(curandState));
  cudaMemset(d_rand_states, 0, p.num_robots * sizeof(curandState));

  /// For each robot, we will have NUM_ANGLE_RAY_TRACE possible obstacles
  /// XXX: we need to use double pointer in cuda
  /// https://stackoverflow.com/questions/7989039/use-of-cudamalloc-why-the-double-pointer
  cudaMalloc(&d_positions_obs, 
      p.num_robots*NUM_ANGLE_RAY_TRACE*sizeof(float4));
  cudaMemset(d_positions_obs, 0, 
      p.num_robots*NUM_ANGLE_RAY_TRACE*sizeof(float4));

  cudaMalloc(&d_vbo_resource, p.num_robots*sizeof(float4));
  cudaMemset(d_vbo_resource, 0, p.num_robots*sizeof(float4));

  // Set kernel launch parameters
  grid_dim = (uint)(ceilf((float)(p.num_robots) / BLOCK_SIZE));
  block = dim3(min(p.num_robots, BLOCK_SIZE), 1, 1);
  grid = dim3(grid_dim, 1, 1);

  // Create streams for simultaneous kernel launches
  cudaStreamCreate(&streams[0]);
  cudaStreamCreate(&streams[1]);
  cudaStreamCreate(&streams[2]);
  cudaStreamCreate(&streams[3]);
}

void cuFree()
{
  // Free arrays from simulation device memory
  cudaFree(d_positions);
  cudaFree(d_velocities);
  cudaFree(d_modes);
  cudaFree(d_leaders);
  cudaFree(d_nearest_leader);
  cudaFree(d_leader_countdown);
  cudaFree(d_laplacian);
  cudaFree(d_occupancy);
  cudaFree(d_flow_pos);
  cudaFree(d_flow_dir);
  cudaFree(d_rand_states);

  cudaFree(d_positions_obs);
  cudaFree(d_vbo_resource);

  // Delete streams for simultaneous kernel launches
  cudaStreamDestroy(streams[0]);
  cudaStreamDestroy(streams[1]);
  cudaStreamDestroy(streams[2]);
  cudaStreamDestroy(streams[3]);
}

void launchInitKernel(Parameters p) {

  // Run initialization kernel to load initial simulation state
  init_kernel <<<grid, block>>>(d_positions, d_velocities, d_modes, 
      d_rand_states, (ulong)(time(NULL)), d_flow_pos, d_flow_dir, 
      d_nearest_leader, d_leader_countdown, p, d_positions_obs);
}

void launchInitKernel(Parameters p, 
        struct cudaGraphicsResource **vbo_resource) {

  // Map 1 OpenGL buffer object (resource) for writing from CUDA 
  //   with 0 stream for synchronization
  int error = cudaGraphicsMapResources(1, vbo_resource, 0);
  if (error != 0) printf("ERROR launchInitKernel 1 = %d\n",error);
  size_t num_bytes;
  // Get an device pointer through which to access a mapped graphics resource.
  error = cudaGraphicsResourceGetMappedPointer((void **)&d_vbo_resource, 
        &num_bytes, *vbo_resource);
  if (error != 0) printf("ERROR launchInitKernel 2 = %d\n",error);

  // Run initialization kernel to load initial simulation state
  launchInitKernel(p);

  /// After update the back end arrays, update the front end arrays
  /// so that the host can draw the new VBO
  /*http://docs.nvidia.com/cuda/cuda-runtime-api/group__CUDART__TYPES.html#group__CUDART__TYPES_1g3f51e3575c2178246db0a94a430e0038*/
  error = cudaMemcpy(d_vbo_resource, d_positions, 
          p.num_robots*sizeof(float4), cudaMemcpyDefault);
  if (error != 0) printf("ERROR launchInitKernel 3 = %d\n",error);
  /// XXX: we cannot add p.number_robots*sizeof(float4) - pointer arithmatic
  /*error = cudaMemcpy(d_vbo_resource+p.num_robots, d_positions_obs, */
          /*GRID_SIZE*sizeof(float4), cudaMemcpyDefault);*/
  /*if (error != 0) printf("ERROR launchInitKernel 4 = %d\n",error);*/

  // Unmap OpenGL buffer object
  error = cudaGraphicsUnmapResources(1, vbo_resource, 0);
  if (error != 0) printf("ERROR launchInitKernel 5 = %d\n",error);
}

void launchMainKernel(float3 gh, float2 gp, uint sn, int* leaders, bool* ap, 
  Parameters p) {

  // Copy leader and articulation point data to GPU
  int error = cudaMemcpy(d_leaders, leaders, p.num_robots * sizeof(int),
    cudaMemcpyHostToDevice);
  if (error != 0) printf("ERROR launchMainKernel 1st = %d\n",error);
  error = cudaMemcpy(d_ap, ap, p.num_robots * sizeof(bool), cudaMemcpyHostToDevice);
  if (error != 0) printf("ERROR launchMainKernel 2nd = %d\n",error);
  
  /// XXX: Now we are still in host, not on device
  /// So we cannot access nor print d_positions_obs

  // Launch the main and side kernels
  main_kernel <<<grid, block, 0, streams[0]>>>(d_positions, d_velocities, 
      d_modes, gh, gp,  d_rand_states, d_ap, d_flow_pos, d_flow_dir, 
      d_occupancy, p, sn, d_positions_obs);

  // Run side kernel for extra computations outside the control loop
  side_kernel <<<grid, block, 0, streams[1]>>>(d_positions, d_modes, 
      d_leaders, d_rand_states, p, d_nearest_leader, d_leader_countdown, 
      d_laplacian, sn);

  // Synchronize kernels on device
  cudaDeviceSynchronize();
}

void launchMainKernel(float3 gh, float2 gp, uint sn, int* leaders, bool* ap, 
  Parameters p, struct cudaGraphicsResource **vbo_resource) {

  // Map OpenGL buffer object for writing from CUDA
  int error = cudaGraphicsMapResources(1, vbo_resource, 0);
  if (error != 0) printf("ERROR launchMainKernel 1 = %d\n",error);
  size_t num_bytes;
  error = cudaGraphicsResourceGetMappedPointer((void **)&d_vbo_resource, 
      &num_bytes, *vbo_resource);
  if (error != 0) printf("ERROR launchMainKernel 2 = %d\n",error);

  launchMainKernel(gh, gp, sn, leaders, ap, p);

  error = cudaMemcpy(d_vbo_resource, d_positions, 
          p.num_robots*sizeof(float4), cudaMemcpyDefault);
  if (error != 0) printf("ERROR launchMainKernel 3 = %d\n",error);
  /*error = cudaMemcpy(d_vbo_resource+p.num_robots, d_positions_obs, */
          /*GRID_SIZE*sizeof(float4), cudaMemcpyDefault);*/
  /*if (error != 0) printf("ERROR launchMainKernel 4 = %d\n",error);*/

  // Unmap OpenGL buffer object
  error = cudaGraphicsUnmapResources(1, vbo_resource, 0);
  if (error != 0) printf("ERROR launchMainKernel 5 = %d\n",error);
}

void getData(uint num_robots, float4* positions, float3* velocities, int* modes,
    float4* positions_obs) {

  // Copy simulation data from device to host arrays
  cudaMemcpy(positions, d_positions, 
      num_robots*sizeof(float4), cudaMemcpyDeviceToHost);
  cudaMemcpy(velocities, d_velocities, 
      num_robots*sizeof(float3), cudaMemcpyDeviceToHost);
  cudaMemcpy(modes, d_modes, 
      num_robots*sizeof(int), cudaMemcpyDeviceToHost);
  /// the positions of all the obstacles
  cudaMemcpy(positions_obs, d_positions_obs, 
      num_robots*NUM_ANGLE_RAY_TRACE*sizeof(float4), cudaMemcpyDeviceToHost);
}

void getData(uint num_robots, uint n_positions_obs, float4* positions, 
    float3* velocities, int* modes, int* nearest_leader, 
    uint* leader_countdown, float4* positions_obs) {
  // Copy simulation data from device to host arrays
  cudaMemcpy(positions, d_positions, 
      num_robots*sizeof(float4), cudaMemcpyDeviceToHost);
  cudaMemcpy(velocities, d_velocities, 
      num_robots*sizeof(float3), cudaMemcpyDeviceToHost);
  cudaMemcpy(modes, d_modes, 
      num_robots*sizeof(int), cudaMemcpyDeviceToHost);
  cudaMemcpy(nearest_leader, d_nearest_leader, 
      num_robots*sizeof(int), cudaMemcpyDeviceToHost);
  cudaMemcpy(leader_countdown, d_leader_countdown, 
      num_robots*sizeof(uint), cudaMemcpyDeviceToHost);
  /// the positions of all the obstacles
  cudaMemcpy(positions_obs, d_positions_obs, 
      num_robots*NUM_ANGLE_RAY_TRACE*sizeof(float4), cudaMemcpyDeviceToHost);
}

void getLaplacian(uint n, int4* laplacian) {
  cudaMemcpy(laplacian, d_laplacian, n * n * sizeof(int4), cudaMemcpyDeviceToHost);
}

void setData(uint n, float4* positions, float3* velocities, int* modes)
{
  // Copy simulation data from host to device arrays
  cudaMemcpy(d_positions, positions, n * sizeof(float4), cudaMemcpyHostToDevice);
  cudaMemcpy(d_velocities, velocities, n * sizeof(float3), cudaMemcpyHostToDevice);
  cudaMemcpy(d_modes, modes, n * sizeof(int), cudaMemcpyHostToDevice);
}

void setData(uint n, float4* positions, float3* velocities, int* modes,
  int* nearest_leader, uint* leader_countdown)
{
  // Copy simulation data from host to device arrays
  cudaMemcpy(d_positions, positions, n * sizeof(float4), cudaMemcpyHostToDevice);
  cudaMemcpy(d_velocities, velocities, n * sizeof(float3), cudaMemcpyHostToDevice);
  cudaMemcpy(d_modes, modes, n * sizeof(int), cudaMemcpyHostToDevice);
  cudaMemcpy(d_nearest_leader, nearest_leader, n * sizeof(int), cudaMemcpyHostToDevice);
  cudaMemcpy(d_leader_countdown, leader_countdown, n * sizeof(uint), cudaMemcpyHostToDevice);
}

void setOccupancy(Parameters p, bool* occupancy) {
  // Copy occupancy data from host to device array
  cudaMemcpy(d_occupancy, occupancy, 
      p.world_size*p.world_size*10*10*sizeof(bool), cudaMemcpyHostToDevice);
}

/**************************
***** CUDA FUNCTIONS ******
**************************/

__global__ void init_kernel(float4* pos, float3* vel, int* mode, 
    curandState* rand_state, ulong seed, float2* flow_pos, float2* flow_dir, 
    int* nearest_leader, uint* leader_countdown, Parameters p,
    float4* pos_obs) {

  // Index of this robot
  uint i = blockIdx.x * blockDim.x + threadIdx.x;

  /*Just let one robot to do it*/
  if (i == 0) {
      for (uint j = 0; j < p.num_robots*NUM_ANGLE_RAY_TRACE; j++) {
        pos_obs[j] = make_float4(-p.world_size, -p.world_size,
            GRID_UNEXPLORED, 0.0f);
      }
      for (uint j = 0; j < p.num_robots*NUM_ANGLE_RAY_TRACE; j++) {
        printf("array[%d]=(%f,%f,%f,%f)",j, pos_obs[j].x, pos_obs[j].y, 
            pos_obs[j].z, pos_obs[j].w);
      }
  }

  __syncthreads();

  // Frequently-used parameters
  float n_f = (float)(p.num_robots);
  float ws = (float)(p.world_size);

  // Seed the RNG
  curand_init(seed, i, 0, &rand_state[i]);
  curandState local_state = rand_state[i];

  // Initialize mode
  // Initially, there are no leaders
  mode[i] = p.hops + 1;
  // Make the first noise % robots have a mode of -1 (noise mode)
  if (i < (int)(p.noise * n_f)) {
    mode[i] = MODE_NOISE;
  }
  
  // Initialize nearest_leader and leader_countdown arrays 
  // for RCC leader selection
  // Initially, there are no leaders
  nearest_leader[i] = LEADER_NON_EXIST;
  leader_countdown[i] = i;

  // Randomly place this robot within the starting circle
  float theta = curand_uniform(&local_state) * 2.0f * PI;
  float unit_r = curand_uniform(&local_state);
  float sqrt_unit_r = sqrtf(unit_r);
  /*10,0, 0.801, 2.88*/
  /*printf ("Robot %d Initial pos = %f, %f, %f\n", i, p.start_size, sqrt_unit_r, theta);*/
  float x_pos = p.start_size * sqrt_unit_r * cosf(theta);
  float y_pos = p.start_size * sqrt_unit_r * sinf(theta);

  // Set the initial color
  Color color;
  /// whether this robot member encounters obstacles
  bool is_obs_encountered = false;
  setColorSwarm(&(color.components), 1, false, i, p, is_obs_encountered);

  // Set speed manually from params.txt
  float speed = p.vel_bound / 60.0f;
  // Set initial position, color and velocity
  pos[i] = make_float4(x_pos, y_pos, 0.0f, color.c);
  vel[i] = make_float3(0.0f, 0.0f, speed);

  // Assign a random flow to 256 points in the environment
  if (i < 256) {
    // First, set flow position
    float x_flow_pos = (curand_uniform(&local_state) * ws) - (ws / 2.0f);
    float y_flow_pos = (curand_uniform(&local_state) * ws) - (ws / 2.0f);
    flow_pos[i] = make_float2(x_flow_pos, y_flow_pos);

    // Second, set flow direction
    float x_flow_dir = (curand_uniform(&local_state) * 2.0f) - 1.0f;
    float y_flow_dir = (curand_uniform(&local_state) * 2.0f) - 1.0f;
    flow_dir[i] = make_float2(x_flow_dir, y_flow_dir);
  }
}

__global__ void side_kernel(float4* pos, int* mode, int* leaders, 
        curandState* rand_state, Parameters p, int* nearest_leader, 
        uint* leader_countdown, int4* laplacian, uint sn) {

  // Index of this robot
  uint i = blockIdx.x * blockDim.x + threadIdx.x;

  // Do not perform any leader calculation if a noise robot, 
  // or in a non-update step
  if (mode[i] != MODE_NOISE) {
    // Perform either RCC or CH leader assignment, depending on parameter

    /// I. Random Competition based Clustering (RCC) leader selection
    if (p.leader_selection == 0) {

      // Holds the new mode and nearest leader during computation
      int new_mode = mode[i];
      int new_nearest_leader = nearest_leader[i];

      // If the leader countdown for this robot has expired,
      // switch leader state and reset the timer;
      if (leader_countdown[i] == 0) {
        // Switch to a leader if not already; else switch to non-leader

        /// Wait and check whether other robots claim leadership
        if (mode[i] == MODE_LEADER) {
          // Assign it as non-leader for 1 second
          new_mode = MODE_NON_LEADER_MAX;
          new_nearest_leader = LEADER_NON_EXIST;
          leader_countdown[i] = 60;
        } 
        // (1) If when the timer ends, the robot i has still not been assigned 
        //     a leader, which means that none of other robots 
        //     have claimed as leaders, then this robot i will claim to be 
        //     the leader. 
        else if (mode[i] > MODE_LEADER) {
          // Assign it as a leader for 5 seconds
          new_mode = MODE_LEADER;
          new_nearest_leader = i;
          leader_countdown[i] = 360;
        }
      }
      // (2) If before the timer ends, the robot i was assigned a leader,
      //     then let robot i follow this leader and reset the timer.
      //
      //     i.e. before the timer ends, one robot A != i, 
      //     claims to be a leader,
      //     then A's mode becomes 0,
      //     A's neighbors' modes becomes 1,
      //     A's neighbors' neighbors' modes becomes 2, 
      //     ......
      //     the last neighbor's mode becomes p.hops-1.
      //
      //     Now the robot i's mode >= p.hops which means that 
      //     robot i does not have any leaders now,
      //     because all the other leaders are too far away from robot i.
      //     So now we can finally assign A as the leader of this robot i.
      else {
        // Iterate through all neighbor robots
        for (int n = 0; n < p.num_robots; n++) {
          // Get the distance between the robots
          float4 me = pos[i];
          float4 them = pos[n];
          float2 dist = make_float2(me.x - them.x, me.y - them.y);
          // Range for hops/leader calculations is max_range / 2
          bool within_range = euclidean(dist) < (p.range_l);

          // Peform operation based on if the robots are within range
          if (within_range && i != n) {
            // If a neighbor with a hop value less than the max and 
            // not greater than this robot is found with a lower ID, 
            // set this robot to non-leader status
            if ((mode[n] < p.hops) && (mode[n] <= mode[i]) && (n < i))
            {
              new_mode = mode[n] + 1;
              new_nearest_leader = nearest_leader[n];
              // Reset leader countdown timer to 1 second
              leader_countdown[i] = 60;
            }
          }
        }
      }

      // Synchronize threads before updating mode and neighbor arrays
      __syncthreads();

      // Update this robot's mode and nearest leader
      mode[i] = new_mode;
      nearest_leader[i] = new_nearest_leader;

      // Decrease countdown timer
      leader_countdown[i]--;
    }
    /// II. Convex Hull leader assignment
    else if (p.leader_selection == 1) {
      // Look at the index of this robot in the leader list to determine if 
      // this robot should be a leader
      if (leaders[i] == 0) {
        mode[i] = 0;
      }
      else {
        mode[i] = 1;
      }
    }
  }

  // Degree for laplacian
  uint degree_a = 0, degree_b = 0, degree_c = 0, degree_d = 0;
  // Iterate through all other robots
  for (uint j = 0; j < p.num_robots; j++) {
    if (i != j) {
      // Get the distance between the robots
      float4 me = pos[i];
      float4 them = pos[j];
      float2 dist_xy = make_float2(me.x - them.x, me.y - them.y);
      float dist = euclidean(dist_xy);
      // Set the non-diagonal values based on whether robots are connected
      // at the four different ranges, -1 means connected, 0 disconnected
      if (dist < p.range) {
        degree_a++;
        laplacian[(i * p.num_robots) + j].x = LAPLACIAN_CONNECTED;
        laplacian[(j * p.num_robots) + i].x = LAPLACIAN_CONNECTED;
      }
      else {
        laplacian[(i * p.num_robots) + j].x = LAPLACIAN_DISCONNECTED;
        laplacian[(j * p.num_robots) + i].x = LAPLACIAN_DISCONNECTED;
      }
      if (dist < p.range_r) {
        degree_b++;
        laplacian[(i * p.num_robots) + j].y = LAPLACIAN_CONNECTED;
        laplacian[(j * p.num_robots) + i].y = LAPLACIAN_CONNECTED;
      }
      else {
        laplacian[(i * p.num_robots) + j].y = LAPLACIAN_DISCONNECTED;
        laplacian[(j * p.num_robots) + i].y = LAPLACIAN_DISCONNECTED;
      }
      if (dist < p.range_f) {
        degree_c++;
        laplacian[(i * p.num_robots) + j].z = LAPLACIAN_CONNECTED;
        laplacian[(j * p.num_robots) + i].z = LAPLACIAN_CONNECTED;
      }
      else {
        laplacian[(i * p.num_robots) + j].z = LAPLACIAN_DISCONNECTED;
        laplacian[(j * p.num_robots) + i].z = LAPLACIAN_DISCONNECTED;
      }
      if (dist < p.range_l) {
        degree_d++;
        laplacian[(i * p.num_robots) + j].w = LAPLACIAN_CONNECTED;
        laplacian[(j * p.num_robots) + i].w = LAPLACIAN_CONNECTED;
      }
      else {
        laplacian[(i * p.num_robots) + j].w = LAPLACIAN_DISCONNECTED;
        laplacian[(j * p.num_robots) + i].w = LAPLACIAN_DISCONNECTED;
      }
    }
  }
  // Set the diagonal of the laplacian to the degree of corresponding robot
  laplacian[(i * p.num_robots) + i].x = degree_a;
  laplacian[(i * p.num_robots) + i].y = degree_b;
  laplacian[(i * p.num_robots) + i].z = degree_c;
  laplacian[(i * p.num_robots) + i].w = degree_d;
}

__global__ void main_kernel(float4* pos, float3* vel, int* mode, 
  float3 goal_heading, float2 goal_point, curandState* rand_state, 
  bool* ap, float2* flow_pos, float2* flow_dir, bool* occupancy, 
  Parameters p, uint sn,
  float4* pos_obs) {

  // Index of this robot
  uint i = blockIdx.x * blockDim.x + threadIdx.x;

  __shared__ float4 s_pos[BLOCK_SIZE];
  __shared__ float3 s_vel[BLOCK_SIZE];
  __shared__ int s_mode[BLOCK_SIZE];
  __shared__ bool s_ap[BLOCK_SIZE];

  __syncthreads();

  // Variables for this robot's data
  float4 myPos = pos[i];
  int myMode = mode[i];
  float mySpeed = p.vel_bound / 60.0f;
  float dist_to_obstacle = p.range_o;
  curandState local_state = rand_state[i];
  /// whether this robot member encounters obstalces
  bool is_obs_encountered = false;

  // Computation variable initializations
  float2 min_bounds = make_float2(0.0f, 0.0f);
  float2 max_bounds = make_float2(0.0f, 0.0f);
  float2 repel = make_float2(0.0f, 0.0f);
  float2 align = make_float2(0.0f, 0.0f);
  float2 cohere = make_float2(0.0f, 0.0f);
  float2 avoid = make_float2(0.0f, 0.0f);
  float2 flow = make_float2(0.0f, 0.0f);
  float2 goal = make_float2(0.0f, 0.0f);

  // Ignore behavior operations if this robot is noise
  if (myMode != MODE_NOISE) {
    // If we are flocking and a leader, set the alignment vector towards 
    // the goal point
    if (p.behavior == BEHAVIOR_FLOCKING && myMode == MODE_LEADER) {
      align.x = goal_heading.x;
      align.y = goal_heading.y;
    }
    /// Followers will only follow the leaders 100% without having a align vector

    // Iterate through blocks to use shared memory within a block
    for (uint tile = 0; tile < gridDim.x; tile++) {

      // Assign shared memory for this block
      uint n = tile * blockDim.x + threadIdx.x;
      s_pos[threadIdx.x] = pos[n];
      s_vel[threadIdx.x] = vel[n];
      s_mode[threadIdx.x] = mode[n];
      s_ap[threadIdx.x] = ap[n];

      // Synchronize threads after shared memory is assigned
      __syncthreads();

      // Iterate through all threads in this block
      for (uint ti = 0; ti < blockDim.x; ti++) {
        // Do not perform an interaction between this robot and itself
        if (i != tile * blockDim.x + ti) {

          // Calculate the distance between the two robots on all axes
          float dist_x = s_pos[ti].x - myPos.x;
          float dist_y = s_pos[ti].y - myPos.y;

          // Calculate the Euclidean distance between the two robots
          float dist = euclidean(make_float2(dist_x, dist_y));

          // Perform interaction for neighbors within range
          if (dist <= p.range) {

            // Create collecte distance variable (for readability)
            float3 dist3 = make_float3(dist_x, dist_y, dist);

            // Perform the interaction for this robot pair based on 
            // the current behavior
            switch (p.behavior) {
            case 0:
              rendezvous(dist3, &min_bounds, &max_bounds, &repel, s_ap[ti], p);
              break;
            case 1:
              flock(myMode, s_vel[ti], s_mode[ti], dist3, &repel, &align, &cohere, s_ap[ti], p);
              break;
            case 2:
              disperse(dist3, &repel, &cohere, s_ap[ti], p);
              break;
            case 3:
              rendezvousToPoint(dist3, &repel, p);
              break;
            }
          }
        }
      }
    }

    /// XXX: We can save the positions of all the obstacles 
    /// detected by this robot member
    // Perform obstacle avoidance computation for this robot
    obstacleAvoidance(myPos, &avoid, &dist_to_obstacle, occupancy, p, 
        pos_obs, i, is_obs_encountered);

    // Finish necessary summary computations for each behavior
    switch (p.behavior) {
      case 0:
        // Finish computation of parallel circumcenter algorithm
        cohere.x = ((min_bounds.x + max_bounds.x) / 2.0f);
        cohere.y = ((min_bounds.y + max_bounds.y) / 2.0f);
        break;
      case 1:
        break;
      case 2:
        break;
      case 3:
        // Set align vector to point toward goal point
        float align_angle = atan2f(goal_point.y - myPos.y, goal_point.x - myPos.x);
        align.x = cosf(align_angle);
        align.y = sinf(align_angle);
        break;
    }

    // If velocity is affected by random flows, calculate flow effect here
    if (p.current > 0.0f) {
      for (uint fi = 0; fi < 256; fi++) {
        float2 dist_v = make_float2(myPos.x - flow_pos[fi].x, myPos.y - flow_pos[fi].y);
        float dist = euclidean(dist_v);
        flow.x += flow_dir[fi].x * (1.0f / dist);
        flow.y += flow_dir[fi].y * (1.0f / dist);
      }
    }

    // Scale all component vectors to their weights and compute goal vector
    rescale(&repel, p.repel_weight, false);
    rescale(&align, p.align_weight, false);
    rescale(&cohere, p.cohere_weight, false);
    rescale(&avoid, 4.0f * powf((p.range_o - dist_to_obstacle), 4.0f), false);
    // Add random currents, if applicable
    if (p.current > 0.0f) {
      rescale(&flow, p.current, false);
    }

    // Combine behavior components to make the new goal vector
    goal.x = repel.x + align.x + cohere.x + avoid.x + flow.x;
    goal.y = repel.y + align.y + cohere.y + avoid.y + flow.y;
  }
  else if (myMode == MODE_NOISE) { // Noise robots
    // Apply error from the normal distribution to the velocity
    goal.x = curand_normal(&local_state);
    goal.y = curand_normal(&local_state);
  }

  // Cap the angular velocity
  capAngularVelocity(make_float2(vel[i].x, vel[i].y), &goal, p.ang_bound / 60.0f);
  // Rescale the goal to the calculated speed
  rescale(&goal, mySpeed, true);

  // Synchronize threads before updating robot state variables
  __syncthreads();

  // Set the color based on current mode
  Color color;
  setColorSwarm(&(color.components), myMode, ap[i], i, p, is_obs_encountered);
  // Update velocity and mode
  vel[i] = make_float3(goal.x, goal.y, mySpeed);

  // Update position
  pos[i] = make_float4(myPos.x + vel[i].x, myPos.y + vel[i].y, 0.0f, color.c);

  ////https://stackoverflow.com/questions/21005845/how-to-get-float-bytes
  ////https://stackoverflow.com/questions/920511/how-to-visualize-bytes-with-c-c

  /*char colorBytes[sizeof(float)];*/
  /*memcpy(colorBytes, &color.c, sizeof(float));*/
  /*printf ("color = [%02x, %02x, %02x, %02x] (%d,%d,%d)\n", */
          /*colorBytes[0], colorBytes[1], colorBytes[2], colorBytes[3],*/
          /*color.components.x, color.components.y, color.components.z,*/
          /*color.components.w);*/

  // Update random state for CUDA RNG
  rand_state[i] = local_state;
}

__device__ void rendezvous(float3 dist3, float2* min_bounds, float2* max_bounds, 
  float2* repel, bool is_ap, Parameters p)
{
  if (dist3.z <= p.range_r) {
    // REPEL
    // Repel from robots within repel range
    float weight = powf(p.range_r - dist3.z, 2.0f);
    repel->x -= weight * dist3.x;
    repel->y -= weight * dist3.y;
  }
  if (dist3.z <= p.range && dist3.z > p.range_r)
  {
    // COHERE
    // Robots cohere to the center of the rectangle that bounds neighbors
    // Do not cohere to neighbors within repel range
    min_bounds->x = fminf(min_bounds->x, dist3.x);
    min_bounds->y = fminf(min_bounds->y, dist3.y);
    max_bounds->x = fmaxf(max_bounds->x, dist3.x);
    max_bounds->y = fmaxf(max_bounds->y, dist3.y);
  }
}

__device__ void flock(int myMode, float3 nVel, int nMode, float3 dist3,
  float2* repel, float2* align, float2* cohere, bool is_ap, Parameters p) {

  // Main flocking section
  if (dist3.z <= p.range_f) {
    // REPEL
    // Robots repel from neighbors within flocking repel range
    float weight = powf(p.range_f - dist3.z, 2.0f);
    repel->x -= weight * dist3.x;
    repel->y -= weight * dist3.y;
  }
  if (myMode != 0) {
    // ALIGN
    float weight;
    (nMode == 0) ? weight = 10.0f : weight = 1.0f;
    align->x += weight * nVel.x;
    align->y += weight * nVel.y;
  }
  if (dist3.z < p.range && dist3.z > p.range_f) {
    // COHERE
    // Do not cohere to neighbors within repel range
    float weight = powf(dist3.z - p.range_f, 2.0f);
    cohere->x += weight * dist3.x;
    cohere->y += weight * dist3.y;
  }
}

__device__ void disperse(float3 dist3, float2* repel, float2* cohere, bool is_ap, 
  Parameters p) {

  // Determine whether we should repel or cohere based on the 
  // distance to the neighbor
  if (dist3.z <= p.range_d) {
    // REPEL
    // Robots repel from neighbors within disperse repel range
    float weight = powf(p.range_d - dist3.z, 2.0f);
    repel->x -= weight * dist3.x;
    repel->y -= weight * dist3.y;
  }
  if (dist3.z <= p.range && dist3.z > p.range_d) {
    // COHERE
    // Do not cohere to robots within disperse repel range
    float weight = 0.0f;// powf(dist3.z - p.range_d, 3.0f);
    cohere->x += weight * dist3.x;
    cohere->y += weight * dist3.y;
  }
}

__device__ void rendezvousToPoint(float3 dist3, float2* repel, Parameters p) {

  if (dist3.z <= p.range_r) {
    // REPEL
    // Repel from robots within repel range
    float weight = powf(p.range_r - dist3.z, 2.0f);
    repel->x -= weight * dist3.x;
    repel->y -= weight * dist3.y;
  }
}

__device__ void obstacleAvoidance(float4 myPos, float2* avoid, 
  float* dist_to_obstacle, bool* occupancy, Parameters p, 
  float4* pos_obs, uint robot_index, bool &is_obs_encountered) {

  /// Checks the collision of one robot member with the obstacles in the map 
  /// and the borders of the world.
  /// But it does not check the collision of the member with other robot members.

  *(dist_to_obstacle) = FLT_MAX;
  int counter = 0;
  for (float i = 0; i < 2.0f * PI; i += RAY_TRACE_INTERVAL) {
    float cos = cosf(i);
    float sin = sinf(i);
    // Ray trace along this angle up to the robot's avoidance range
    for (float r = 0.0f; r < p.range_o; r += 1.0f) {
      float x_check = myPos.x + r * cos;
      float y_check = myPos.y + r * sin;

      int occupancy_ind = occupancySub2Ind(x_check, y_check, p);
      bool occupied = false;
      if (occupancy_ind == -1)
        occupied = true;
      else
        occupied = occupancy[occupancy_ind];

      if (occupied == true) {
        // Get weight for obstacle repulsion force
        float weight = powf(1.0f - (r / p.range_o), 2.0f);
        // Update the distance to the closest obstacle
        if (r < *dist_to_obstacle) {
          *dist_to_obstacle = r;
        }
        // Update the obstacle avoidance vector
        avoid->x += weight * -r * cos;
        avoid->y += weight * -r * sin;

        int tmp = robot_index*NUM_ANGLE_RAY_TRACE+counter;
        counter ++;
        Color color;
        setColorGrid(&(color.components), GRID_EXPLORED_OBS);
        pos_obs[tmp] = make_float4(x_check,y_check,GRID_EXPLORED_OBS,color.c);
        /*printf("obs = array[%d]=%f\n",tmp, pos_obs[tmp].w);*/

        is_obs_encountered = true;
        break;
      }
    }
  }
}

__device__ int occupancySub2Ind(float x, float y, Parameters p) {
  float ws_2 = (float)(p.world_size) / 2.0f;
  float ws_10 = (float)(p.world_size) * 10.0f;
  // Return -1 if the coordinates to check are outside the world boundaries;
  // else, return the index in the occupancy grid for these coordinates
  if (x < -ws_2 || x > ws_2 || y < -ws_2 || y > ws_2) {
    /*return true;*/
    return -1;
  }
  else {
    // Get the 1d index for the occupancy array from the x and y coordinates
    uint x_component = (uint)((x + ws_2) * 10.0f);
    uint y_component = (uint)(floorf(y + ws_2) * ws_10 * 10.0f);
    uint index = x_component + y_component;
    /*return occupancy[index];*/
    return index;
  }
}

/// Set color for swarm robot members
__device__ void setColorSwarm(uchar4* color, int mode, bool is_ap, uint i, 
  Parameters p, bool is_obs_encountered) {

  /*draw non-leaders in (255,255,255)*/
  /*draw leaders in (255,0,0)*/
  /*draw articulation points in (0,200,0)*/
  /*draw noise robots in (100,100,100)*/

  /// uchar is an integer between 0 to 255
  /// https://stackoverflow.com/questions/75191/what-is-an-unsigned-charhttps://stackoverflow.com/questions/75191/what-is-an-unsigned-char
  /// Here uchar4 is a tuple of 4 integers in that range
  /// uchar4* is an array of uchar4

  /// ap = articulation points

  if (mode == MODE_LEADER && p.show_leaders) {
    if (p.highlight_leaders) {
      (is_ap && p.show_ap) ? 
                *color = make_uchar4(0, 200, 0, 255) : 
                *color = make_uchar4(255, 0, 0, 255);
    }
    else {
      (is_ap && p.show_ap) ? 
                *color = make_uchar4(0, 200, 0, 255) : 
                *color = make_uchar4(255, 255, 255, 255);
    }
  }
  else if (mode != MODE_LEADER && p.show_non_leaders) {
    if (mode > MODE_LEADER) {
      (is_ap && p.show_ap) ? 
                *color = make_uchar4(0, 200, 0, 255) : 
                *color = make_uchar4(255, 255, 255, 255);
    }
    else {
      *color = make_uchar4(100, 100, 100, 255);
    }
  }
  else {
    *color = make_uchar4(0, 0, 0, 0);
  }

  /// draw extra color for obstacle encounted robots
  if (p.highlight_pioneers == true && is_obs_encountered == true) {
    *color = make_uchar4(255, 255, 0, 255);
  }
}

/// Set color for grid
__device__ void setColorGrid(uchar4* color, int grid) {

  /*draw explored obstacle in (255,255,0)*/
  /*draw others in (0,0,0)*/
  if (grid == GRID_EXPLORED_OBS) {
    *color = make_uchar4(255, 255, 0, 255);
  }
  else {
    printf("WTF\n");
  }
  /*else if (grid == GRID_UNEXPLORED) {*/
    /**color = make_uchar4(0, 255, 0, 255);*/
  /*}*/
  /*else if (grid == GRID_EXPLORED_FREE) {*/
    /**color = make_uchar4(0, 0, 100, 255);*/
  /*}*/
}



__device__ float euclidean(float2 vector)
{
  return sqrtf(powf(vector.x, 2.0f) + powf(vector.y, 2.0f));
}

__device__ void rescale(float2* vel, float value, bool is_value_limit)
{
  // Determine the scalar of the vector
  float scalar = euclidean(*vel);

  // Normalize the vector if the value given is the limit and the scalar is  
  // above that, or if the value given is not a limit
  if (scalar != 0.0f && ((is_value_limit && scalar > value) || !is_value_limit)) {
    float factor = value / scalar;
    vel->x *= factor;
    vel->y *= factor;
  }
}

__device__ void normalizeAngle(float* angle)
{
  while (*angle > PI) {
    *angle -= 2.0f * PI;
  }
  while (*angle <= -PI) {
    *angle += 2.0f * PI;
  }
}

__device__ void capAngularVelocity(float2 old, float2* goal, float max)
{
  // Get the magnitude of each vector
  float norm_old = euclidean(old);
  float norm_goal = euclidean(*goal);

  float old_angle = atan2f(old.y, old.x);
  float goal_angle = atan2f(goal->y, goal->x);

  // Get the angle from the old to new goal vector
  float angle = goal_angle - old_angle;
  normalizeAngle(&angle);

  // If the angle is greater than the maximum angular velocity, cap the 
  // angular velocity to this maximum
  if (fabsf(angle) > max && norm_old != 0.0f && norm_goal != 0.0f) {
    (angle < 0.0f) ? old_angle -= max : old_angle += max;
    goal->x = norm_goal * cosf(old_angle);
    goal->y = norm_goal * sinf(old_angle);
  }
}
