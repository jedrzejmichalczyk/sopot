cmake_minimum_required(VERSION 3.20)
project(SOPOT VERSION 1.0.0)

# SOPOT: Obviously Physics, Obviously Templates
# A C++20 compile-time physics simulation framework

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Header-only core library
add_library(sopot_core INTERFACE)
target_include_directories(sopot_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR})

# Header-only physics library
add_library(sopot_physics INTERFACE)
target_include_directories(sopot_physics INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/physics)
target_link_libraries(sopot_physics INTERFACE sopot_core)

# Header-only rocket library
add_library(sopot_rocket INTERFACE)
target_include_directories(sopot_rocket INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/rocket
    ${CMAKE_CURRENT_SOURCE_DIR}/io)
target_link_libraries(sopot_rocket INTERFACE sopot_core)

set(ROCKET_HEADERS
    rocket/rocket.hpp
    rocket/rocket_tags.hpp
    rocket/rocket_body.hpp
    rocket/simulation_time.hpp
    rocket/translation_kinematics.hpp
    rocket/translation_dynamics.hpp
    rocket/rotation_kinematics.hpp
    rocket/rotation_dynamics.hpp
    rocket/standard_atmosphere.hpp
    rocket/gravity.hpp
    rocket/interpolated_engine.hpp
    rocket/force_aggregator.hpp
    rocket/axisymmetric_aero.hpp
    rocket/vector3.hpp
    rocket/quaternion.hpp
)
target_sources(sopot_rocket INTERFACE ${ROCKET_HEADERS})

# Test executables
add_executable(compile_time_test tests/compile_time_test.cpp)
target_link_libraries(compile_time_test sopot_core sopot_physics)
target_compile_features(compile_time_test PRIVATE cxx_std_20)

add_executable(autodiff_test tests/autodiff_test.cpp)
target_link_libraries(autodiff_test sopot_core sopot_physics)
target_compile_features(autodiff_test PRIVATE cxx_std_20)

add_executable(rocket_flight_test tests/rocket_flight_test.cpp)
target_link_libraries(rocket_flight_test sopot_core sopot_rocket)
target_compile_features(rocket_flight_test PRIVATE cxx_std_20)

# Custom target to run all tests
add_custom_target(run_tests
    COMMAND compile_time_test
    COMMAND autodiff_test
    COMMAND rocket_flight_test
    DEPENDS compile_time_test autodiff_test rocket_flight_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running SOPOT tests"
)

message(STATUS "SOPOT Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Architecture: Pure compile-time dispatch")
message(STATUS "  Performance: Zero runtime overhead")
