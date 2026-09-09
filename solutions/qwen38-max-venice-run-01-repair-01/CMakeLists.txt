cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

add_library(vwmini
    src/vwmini.cpp
    src/vwmini/geometry.cpp
    src/vwmini/triangulation.cpp
    src/vwmini/nav_mesh.cpp
    src/vwmini/pathfinding.cpp
    src/vwmini/simulation.cpp
)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
target_include_directories(vwmini PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(vwmini PRIVATE src)

option(VWMINI_WARNINGS "Enable strict compiler warnings" ON)
if(VWMINI_WARNINGS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vwmini PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Tests are opt-out via the conventional BUILD_TESTING flag so consumers can
# build the library alone without GTest being present.
option(BUILD_TESTING "Build the test suite (requires GTest)" ON)
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
