cmake_minimum_required(VERSION 3.23)
project(vwmini_conformance LANGUAGES CXX)

include(CTest)
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

set(VWMINI_CONFORMANCE_SOURCE_DIR "" CACHE PATH
    "Provider project exposing vwmini::vwmini; empty uses evaluator/reference")

if(VWMINI_CONFORMANCE_SOURCE_DIR)
    add_subdirectory("${VWMINI_CONFORMANCE_SOURCE_DIR}" candidate_under_test)
else()
    set(VWMINI_BUILD_REFERENCE_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../reference" reference_under_test)
endif()

function(add_conformance_track target label source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE vwmini::vwmini GTest::gtest_main)
    target_compile_features(${target} PRIVATE cxx_std_23)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/tests)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    gtest_discover_tests(${target} PROPERTIES LABELS ${label})
endfunction()

add_conformance_track(vwmini_conformance_geometry geometry tests/geometry_conformance.cpp)
add_conformance_track(vwmini_conformance_navmesh_path navmesh tests/navmesh_path_conformance.cpp)
add_conformance_track(vwmini_conformance_simulation simulation tests/simulation_lifecycle_conformance.cpp)
add_conformance_track(vwmini_conformance_crowd crowd tests/crowd_conformance.cpp)
