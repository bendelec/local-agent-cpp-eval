# VWmini: immutable 2D navigation mesh, deterministic paths, local disc avoidance.
cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

add_library(vwmini
    src/geometry.cpp
    src/triangulator.cpp
    src/nav_mesh.cpp
    src/simulation.cpp
)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
target_include_directories(vwmini PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Warning surface required by NFR-007. Library code must stay warning-clean here.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vwmini PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Tests are opt-in via BUILD_TESTING so a library-only configure builds no test targets.
option(BUILD_TESTING "Build VWmini unit tests" ON)
if(BUILD_TESTING)
    enable_testing()
    # --- Self-contained, deterministic-order framework; no third-party deps. ---
    add_executable(vwmini_tests
        tests/main.cpp
        tests/testing.hpp
        tests/geometry_tests.cpp
        tests/triangulator_tests.cpp
        tests/navmesh_tests.cpp
        tests/path_tests.cpp
        tests/simulation_tests.cpp
    )
    target_link_libraries(vwmini_tests PRIVATE vwmini)
    target_compile_features(vwmini_tests PRIVATE cxx_std_23)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(vwmini_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    include(CTest)
    add_test(NAME vwmini_tests COMMAND vwmini_tests)
endif()
