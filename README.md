# cuSwarm_exp

## CUDA-based simulator of agent swarms
`cuSwarm_exp` is a basic simulator for swarms of agents or robots using the **NVIDIA CUDA** platform used for experiments by the Usability Lab at the University of Pittsburgh School of Information Sciences and Carnegie Mellon University. Please note that this simulator is currently being used to conduct user studies as part of research in human-swarm interaction, and thus is NOT meant for public use, and may be difficult to follow. You are of course free to download it and play around.

If you are interested in a basic version without experimental features, and with better documentation, check out the main version (cuSwarm) at [here](https://github.com/pmwalk/cuSwarm).

## Dependencies
- OpenGL, used for drawing the interface (download at [here](https://www.opengl.org/wiki/Getting_Started#Downloading_OpenGL)).
- Eigen, used for data calculations of the robot communication graph (download at [here](http://eigen.tuxfamily.org/index.php?title=Main_Page)).

## File Structure
- `run.cpp/run.h`: the main file that runs the simulation and draws the interface using OpenGL.

- `kernels.cu/kernels.cuh`: CUDA file containing all device kernels.

- `data.cpp/data.h`: data calculations used for logging and drawing certain GUI elements.

- `utils.cpp/utils.h`: utilities file.


## Instructions
To build it,
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

To run it, `$ ./swarm param.txt output.txt`
