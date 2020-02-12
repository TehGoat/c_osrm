cmake_minimum_required(VERSION 3.10)
project(c_osrm)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

link_directories(${LibOSRM_LIBRARY_DIRS})
add_library(c_osrm SHARED c_osrm.cpp c_osrm.h parameters.h)

find_package(LibOSRM REQUIRED)

target_link_libraries(c_osrm ${LibOSRM_LIBRARIES} ${LibOSRM_DEPENDENT_LIBRARIES})
include_directories(SYSTEM ${LibOSRM_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LibOSRM_CXXFLAGS}")