cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

option(VWMINI_BUILD_TESTS "Build the vwmini test suite" ON)

add_library(vwmini
    src/geometry.cpp
    src/triangulation.cpp
    src/mesh_topology.cpp
    src/nav_mesh.cpp
    src/corridor.cpp
    src/pathfinding.cpp
    src/steering.cpp
    src/simulation.cpp)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
set_target_properties(vwmini PROPERTIES
    CXX_STANDARD 23
    CXX_EXTENSIONS OFF
    CXX_STANDARD_REQUIRED ON)

target_include_directories(vwmini PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_options(vwmini PRIVATE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
    -Wold-style-cast -Wconversion-null -Wnon-virtual-dtor -Wcast-align
    -Woverloaded-virtual -Wdouble-promotion)

if(VWMINI_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
