cmake_minimum_required(VERSION 2.8.3)
project(movemap)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

add_executable(updatepointoccupancy updatepointoccupancy.cpp)
