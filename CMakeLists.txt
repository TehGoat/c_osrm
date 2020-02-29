cmake_minimum_required(VERSION 3.10)
project(c_osrm)

set(CMAKE_CXX_STANDARD 14)
include(ExternalProject)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

link_directories(${LibOSRM_LIBRARY_DIRS})
add_library(c_osrm SHARED c_osrm.cpp)

find_package(LibOSRM)

if(NOT ${LibOSRM_FOUND}) 
    message("libosrm not found")
    execute_process(COMMAND sudo apt install build-essential git cmake pkg-config
                            libbz2-dev libxml2-dev libzip-dev libboost-all-dev
                            lua5.2 liblua5.2-dev libtbb-dev)
    execute_process(COMMAND git clone https://github.com/Project-OSRM/osrm-backend)
    execute_process(COMMAND mkdir -p build WORKING_DIRECTORY osrm-backend)
    execute_process(COMMAND cmake .. WORKING_DIRECTORY osrm-backend/build)
    execute_process(COMMAND cmake --build . -j12 WORKING_DIRECTORY osrm-backend/build)
    execute_process(COMMAND sudo cmake --build . --target install WORKING_DIRECTORY osrm-backend/build)
endif()


find_package(LibOSRM REQUIRED)

target_link_libraries(c_osrm ${LibOSRM_LIBRARIES} ${LibOSRM_DEPENDENT_LIBRARIES})
include_directories(SYSTEM ${LibOSRM_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LibOSRM_CXXFLAGS}")