cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(PointClouds)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PointClouds pointclouds.cpp)
target_link_libraries (PointClouds ${PCL_LIBRARIES})
