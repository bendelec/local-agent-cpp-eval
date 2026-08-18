add_library(vwmini_reference
    src/geometry.cpp
    src/mesh.cpp
    src/path.cpp
    src/simulation.cpp
)
add_library(vwmini::vwmini ALIAS vwmini_reference)

target_compile_features(vwmini_reference PUBLIC cxx_std_23)
target_include_directories(vwmini_reference
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../candidate/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(vwmini_reference PRIVATE -Wall -Wextra -Wpedantic)

if(VWMINI_BUILD_REFERENCE_TESTS)
    add_subdirectory(tests)
endif()
