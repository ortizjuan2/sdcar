cmake_minimum_required(VERSION 2.8)

project(sdcar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Wextra")

add_executable(sdcar main.cpp ./mpc/mpc.cpp)
