find_package(SDL3 CONFIG REQUIRED)

add_executable(vwmini_lab
    lab_state.cpp
    main.cpp)

target_link_libraries(vwmini_lab PRIVATE vwmini::vwmini SDL3::SDL3)
target_compile_features(vwmini_lab PRIVATE cxx_std_23)
target_include_directories(vwmini_lab PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(vwmini_lab PRIVATE -Wall -Wextra -Wpedantic)
