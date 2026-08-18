cmake_minimum_required(VERSION 3.23)
project(vwmini_evaluator LANGUAGES CXX)

include(CTest)

option(VWMINI_BUILD_REFERENCE_TESTS "Build reference-library tests" ON)
option(VWMINI_BUILD_LAB "Build the optional visual lab" OFF)

add_subdirectory(reference)

if(VWMINI_BUILD_LAB)
    add_subdirectory(lab)
endif()
