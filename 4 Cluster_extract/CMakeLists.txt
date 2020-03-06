cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ./src/libForPCL/include)
link_directories(${PCL_LIBRARY_DIRS})

aux_source_directory(./src/libForPCL/src lib_src)
aux_source_directory(./src current_src)

add_definitions(${PCL_DEFINITIONS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(cluster ${current_src} ${lib_src})
target_link_libraries(cluster ${PCL_LIBRARIES})



