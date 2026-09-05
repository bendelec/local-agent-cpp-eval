cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

add_library(vwmini src/vwmini.cpp)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
target_include_directories(vwmini PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

enable_testing()
add_executable(vwmini_tests tests/test_basic.cpp)
target_link_libraries(vwmini_tests PRIVATE vwmini)
add_test(NAME vwmini_tests COMMAND vwmini_tests)
