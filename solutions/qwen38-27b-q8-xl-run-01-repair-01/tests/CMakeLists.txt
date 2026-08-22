add_executable(vwmini_tests
    geometry_test.cpp
    nav_mesh_test.cpp
    triangulate_test.cpp
    path_test.cpp
    simulation_test.cpp
)
target_link_libraries(vwmini_tests PRIVATE vwmini::vwmini GTest::gtest GTest::gtest_main)
# White-box unit tests include private headers (see tests/geometry_test.cpp).
target_include_directories(vwmini_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_options(vwmini_tests PRIVATE -Wall -Wextra -Wpedantic)

include(GoogleTest)
gtest_discover_tests(vwmini_tests)
