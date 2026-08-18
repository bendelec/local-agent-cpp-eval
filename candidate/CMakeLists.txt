cmake_minimum_required(VERSION 3.23)
project(vwmini LANGUAGES CXX)

add_library(vwmini src/vwmini.cpp)
add_library(vwmini::vwmini ALIAS vwmini)

target_compile_features(vwmini PUBLIC cxx_std_23)
target_include_directories(vwmini PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Add implementation sources with target_sources(vwmini PRIVATE ...).
# Add tests with CTest after implementing the library.
