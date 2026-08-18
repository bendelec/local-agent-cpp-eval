find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(vwmini_reference_tests
    geometry_tests.cpp
    navmesh_tests.cpp
    simulation_tests.cpp
    crowd_tests.cpp)

target_link_libraries(vwmini_reference_tests PRIVATE vwmini::vwmini GTest::gtest_main)
target_compile_features(vwmini_reference_tests PRIVATE cxx_std_23)
target_include_directories(vwmini_reference_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(vwmini_reference_tests PRIVATE -Wall -Wextra -Wpedantic)

gtest_discover_tests(vwmini_reference_tests)
