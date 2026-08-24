cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

add_library(vwmini
    src/vwmini/geometry.cpp
    src/vwmini/geometry_priv.cpp
    src/vwmini/mesh_priv.cpp
    src/vwmini/nav_mesh.cpp
    src/vwmini/path.cpp
    src/vwmini/simulation.cpp
    src/vwmini/triangulate.cpp
)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
target_include_directories(vwmini
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_options(vwmini PRIVATE -Wall -Wextra -Wpedantic)

option(VWMINI_BUILD_TESTS "Build the VWmini test suite" ON)
if (VWMINI_BUILD_TESTS)
    enable_testing()
    find_package(GTest 1.8 QUIET)
    if (NOT GTest_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
        )
        set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()
    add_subdirectory(tests)
endif ()
