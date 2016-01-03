cmake_minimum_required(VERSION 2.8)
project(slam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/lib)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)


# Eigen3 library
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
message("${Eigen_INCLUDE_DIRS}")

# G2O
set(G2O_ROOT "/usr/local/include/g2o")
find_package(G2O)
include_directories(${G2O_INCLUDE_DIR})

# CSparse
find_package(CSparse)
include_directories(${CSPARSE_INCLUDE_DIR})



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse2 -msse3 -std=c++0x -O0")
add_subdirectory(src bin)
