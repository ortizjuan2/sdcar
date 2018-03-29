cmake_minimum_required(VERSION 2.8)

project(main)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

add_executable(main main.cpp helpers.cpp model/model.cpp)

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    /home/jom/Documents/prgs/projects/eigen
    /home/jom/Documents/prgs/projects/cppad
    )
