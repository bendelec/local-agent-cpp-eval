cmake_minimum_required(VERSION 3.23)
project(vwmini_evaluator LANGUAGES CXX)

include(CTest)

option(VWMINI_BUILD_REFERENCE_TESTS "Build reference-library tests" ON)
option(VWMINI_BUILD_LAB "Build the optional visual lab" OFF)
set(VWMINI_EVALUATOR_SOURCE_DIR "" CACHE PATH
    "Provider project exposing vwmini::vwmini; empty uses evaluator/reference")

if(VWMINI_EVALUATOR_SOURCE_DIR)
    add_subdirectory("${VWMINI_EVALUATOR_SOURCE_DIR}" provider_under_test)
else()
    add_subdirectory(reference)
endif()

if(VWMINI_BUILD_LAB)
    add_subdirectory(lab)
endif()
