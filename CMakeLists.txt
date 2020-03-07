cmake_minimum_required(VERSION 3.10)
project(c_osrm)

set(CMAKE_CXX_STANDARD 14)
include(ExternalProject)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

execute_process(COMMAND sudo apt install build-essential git cmake pkg-config
                        libbz2-dev libxml2-dev libzip-dev libboost-all-dev
                        lua5.2 liblua5.2-dev libtbb-dev)

link_directories("${PROJECT_SOURCE_DIR}/osrm/lib/")
add_library(c_osrm SHARED c_osrm.cpp)

target_link_libraries(c_osrm "${PROJECT_SOURCE_DIR}/osrm/lib/libosrm.a")
include_directories(SYSTEM "${PROJECT_SOURCE_DIR}/osrm/include" "${PROJECT_SOURCE_DIR}/osrm/include/osrm")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -losrm")