#### CMakeLists.txt for CUDA swarmSim build

cmake_minimum_required(VERSION 2.8)
find_package(CUDA QUIET REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)

set(CMAKE_CXX_STANDARD 11)

#Pass options to NVCC
set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -arch=sm_35
    )

list(APPEND CUDA_NVCC_FLAGS "-std=c++11")

include_directories(
    /usr/include/eigen3/
    /usr/local/cuda-8.0/include/
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    )

cuda_add_executable(
    swarm
    data_ops.cpp
    data_ops.h
    kernels.cu
    kernels.cuh
    run.cpp
    run.h
    utils.cpp
    utils.h
    )

#specify target and library to link it with
target_link_libraries(
    swarm
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    ${GLEW_LIBRARY}
    -lstdc++
    -lm
  )

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/params.txt DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
