# cuSwarm_exp
CUDA-based simulator of agent swarms

DESCRIPTION

cuSwarm_exp is a basic simulator for swarms of agents or robots using the NVIDIA CUDA platform. Please note that this simulator is currently being used to conduct user studies as part of research in human-swarm interaction, and thus is not meant for public use, although you are free to download it and play around.

REQUIRED LIBRARIES

- OpenGL, used for drawing the interface (download at https://www.opengl.org/wiki/Getting_Started#Downloading_OpenGL)
- Eigen, used for data calculations of the robot communication graph (download at http://eigen.tuxfamily.org/index.php?title=Main_Page)

FILE DESCRIPTIONS

---run.cpp/run.h---
The main file that runs the simulation and draws the interface using OpenGL

---kernels.cu/kernels.cuh---
CUDA file containing all device kernels

---data.cpp/data.h---
Data calculations used for logging and drawing certain GUI elements

---utils.cpp/utils.h---
Utilities file

Questions can be directed to Phillip Walker pmwalk@gmail.com
