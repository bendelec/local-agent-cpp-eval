cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Library target vwmini::vwmini
# ---------------------------------------------------------------------------
add_library(vwmini
    src/vwmini/geometry.cpp
    src/vwmini/triangulate.cpp
    src/vwmini/nav_mesh.cpp
    src/vwmini/path.cpp
    src/vwmini/simulation.cpp)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
target_include_directories(vwmini
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_options(vwmini PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
option(VWMINI_ENABLE_TESTING "Build unit tests" ON)
if(VWMINI_ENABLE_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)
    add_executable(vwmini_tests
        tests/test_geometry.cpp
        tests/test_triangulate.cpp
        tests/test_navmesh.cpp
        tests/test_path.cpp
        tests/test_simulation.cpp)
    target_link_libraries(vwmini_tests PRIVATE vwmini GTest::gtest_main)
    target_compile_options(vwmini_tests PRIVATE -Wall -Wextra -Wpedantic)
    include(GoogleTest)
    gtest_discover_tests(vwmini_tests)
endif()
