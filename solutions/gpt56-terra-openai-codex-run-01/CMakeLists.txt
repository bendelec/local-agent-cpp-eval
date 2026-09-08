cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

add_library(vwmini
    src/geometry.cpp
    src/nav_mesh.cpp
    src/simulation.cpp)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
target_include_directories(vwmini PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vwmini PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(vwmini_tests tests/vwmini_tests.cpp)
    target_link_libraries(vwmini_tests PRIVATE vwmini::vwmini)
    target_compile_features(vwmini_tests PRIVATE cxx_std_23)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(vwmini_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME vwmini_tests COMMAND vwmini_tests)
endif()
