cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

add_library(vwmini src/vwmini.cpp src/geometry.cpp src/nav_mesh.cpp src/simulation.cpp)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
target_include_directories(vwmini PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_options(vwmini PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()

add_subdirectory(tests)
