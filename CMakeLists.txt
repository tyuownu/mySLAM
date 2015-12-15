cmake_minimum_required(VERSION 2.8)
project(slam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)

# Eigen3 library
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
message("${Eigen_INCLUDE_DIRS}")


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse2 -msse3 -std=c++0x")
add_subdirectory(src bin)
