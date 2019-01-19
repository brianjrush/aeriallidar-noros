cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp icp.cpp)
add_executable (combine combine_pcd.cpp)
add_executable (colorize colorize_ply.cpp)
add_executable (filter filter.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})
target_link_libraries (combine ${PCL_LIBRARIES})
target_link_libraries (colorize ${PCL_LIBRARIES})
target_link_libraries (filter ${PCL_LIBRARIES})
