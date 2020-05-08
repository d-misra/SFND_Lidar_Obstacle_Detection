cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# PCL 1.10 requires C++ 14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.10 REQUIRED)

# Public header files
set(HEADER_FILES
        src/group_defs.dox)

# Source files and private header files go here.
set(SOURCE_FILES
        src/environment.cpp
        src/render/render.cpp
        src/processPointClouds.cpp)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")



add_executable (environment ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries (environment ${PCL_LIBRARIES})




