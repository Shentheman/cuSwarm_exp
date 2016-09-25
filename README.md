# cuSwarmSim
CUDA-based simulator of agent swarms

DESCRIPTION

cuSwarmSim is a basic simulator for swarms of agents or robots using the NVIDIA CUDA platform. Please note that this simulator is currently being used to conduct user studies as part of research in human-swarm interaction, and thus is not meant for public use, although you are free to download it and play around.

REQUIRED LIBRARIES

- OpenGL, used for drawing the interface (download at https://www.opengl.org/wiki/Getting_Started#Downloading_OpenGL)
- Eigen, used for data calculations of the robot communication graph (download at http://eigen.tuxfamily.org/index.php?title=Main_Page)

OPERATING MODES AND PARAMETERS

The simulator operates in one of three modes, which can be set using the op_mode parameter in the params.txt file. The modes are as follows:
    0: Automated mode. This mode will use an algorithm to search for a good sequence of behaviors to reach the goal of the given environment. The current algorithm is defined in the automateExplore() method in run.cpp
    1: Playback mode. This mode will playback the last sequence of behaviors used for the previous map. To view the results of the automated algorithm from mode 0, run the simulator in mode 0 until completion, then re-run in mode 1.
    2: Manual mode. This mode shows the GUI and allows the user to manually control the swarm

Other parameters can be changed in the params.txt file, including number of robots, world size, number of obstacles, and various robot-specific parameters (such as maximum speed and angular velocity).

FILE DESCRIPTIONS

---run.cpp/run.h---

The main file that runs the simulation and draws the interface using OpenGL

---kernels.cu/kernels.cuh---

CUDA file containing all device kernels

---data.cpp/data.h---

Data calculations used for logging and drawing certain GUI elements

---planning.cpp/planning.h---

Contains structures and variables used by the planning algorithm for op mode 0

---utils.cpp/utils.h---

Utilities file

Questions can be directed to Phillip Walker pmwalk@gmail.com
