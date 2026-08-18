# vwmini tests: one focused executable per module.
add_executable(vwmini_geometry_test geometry_test.cpp)
target_link_libraries(vwmini_geometry_test PRIVATE vwmini::vwmini)
target_compile_options(vwmini_geometry_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME geometry_test COMMAND vwmini_geometry_test)

add_executable(vwmini_navmesh_test nav_mesh_test.cpp)
target_link_libraries(vwmini_navmesh_test PRIVATE vwmini::vwmini)
target_compile_options(vwmini_navmesh_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME navmesh_test COMMAND vwmini_navmesh_test)

add_executable(vwmini_sim_test simulation_test.cpp)
target_link_libraries(vwmini_sim_test PRIVATE vwmini::vwmini)
target_compile_options(vwmini_sim_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME sim_test COMMAND vwmini_sim_test)
