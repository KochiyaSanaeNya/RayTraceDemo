cmake_minimum_required(VERSION 4.2)
project(RayTrace)

set(CMAKE_CXX_STANDARD 26)

find_package(OpenMP REQUIRED)

add_executable(RayTrace raytrace.cpp)

target_compile_options(RayTrace PRIVATE ${OpenMP_CXX_FLAGS})
target_link_options(RayTrace PRIVATE ${OpenMP_CXX_FLAGS})