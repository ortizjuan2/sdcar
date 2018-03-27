cmake_minimum_required(VERSION 2.8)

project(mpc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Wextra")

add_executable(mpc mpc.cpp)

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    /home/jom/Documents/prgs/projects/cppad
    )

