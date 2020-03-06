cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ply2pcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ./include)
aux_source_directory(./src ply2pcd_src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(ply2pcd ${ply2pcd_src})
target_link_libraries(ply2pcd ${PCL_LIBRARIES})

