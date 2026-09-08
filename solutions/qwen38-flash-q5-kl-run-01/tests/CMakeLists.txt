find_package(GTest REQUIRED)

add_executable(vwmini_tests
    geometry_test.cpp
    triangulation_test.cpp
    nav_mesh_test.cpp
    simulation_test.cpp
    path_test.cpp)

target_compile_features(vwmini_tests PRIVATE cxx_std_23)
target_compile_options(vwmini_tests PRIVATE
    -Wall -Wextra -Wpedantic -Wshadow)
target_link_libraries(vwmini_tests PRIVATE vwmini::vwmini GTest::gtest GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(vwmini_tests)
