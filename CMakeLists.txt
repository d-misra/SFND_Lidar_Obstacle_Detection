cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

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

# Our code uses some specific C++ features;
target_compile_features(environment PRIVATE
        cxx_std_14      # requirement for PCL 1.10
        cxx_std_17      # we're using structured bindings
        # ... cxx_constexpr
        # ... cxx_auto_type
        # ... cxx_defaulted_functions
        )
