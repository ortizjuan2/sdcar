cmake_minimum_required(VERSION 2.8)

project(main)
FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(IPOPT REQUIRED ipopt)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

add_executable(main main.cpp helpers.cpp)

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    /home/jom/Documents/prgs/projects/eigen
    /home/jom/Documents/prgs/projects/cppad
    /home/jom/Documents/prgs/projects/Ipopt-3.12.9/include
    )

# Local link directories to search, as determined by pkg-config for ipopt
LINK_DIRECTORIES( ${IPOPT_LIBRARY_DIRS} )
#
# libraries to be linked into the specified target,
# as determined by pkg-config for ipopt
TARGET_LINK_LIBRARIES(main
    ${IPOPT_LDFLAGS}
)