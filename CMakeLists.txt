cmake_minimum_required(VERSION 3.20)
project(SOPOT VERSION 1.0.0)

# SOPOT: Obviously Physics, Obviously Templates
# A C++20 compile-time physics simulation framework

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Compilation time optimizations
# ============================================================================

# Enable ccache if available (speeds up recompilation)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache for faster recompilation")
endif()

# Platform-specific compiler flags
if(MSVC)
    add_compile_options(/W4 /permissive-)
    add_compile_options("$<$<CONFIG:Release>:/O2>")
    # MSVC template compilation optimizations
    add_compile_options(/bigobj)  # Increase object file section limit
    # Note: MSVC uses /constexpr:depth<N> for constexpr recursion depth
    # and doesn't have a direct equivalent to -ftemplate-depth
    # Template instantiation depth is generally higher in MSVC (default ~1500)
    add_compile_options(/constexpr:depth2048)  # Increase constexpr evaluation depth
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options("$<$<CONFIG:Release>:-O3>")
    add_compile_options("$<$<CONFIG:Debug>:-O0;-g>")

    # GCC/Clang template compilation optimizations
    add_compile_options(-ftemplate-depth=2048)  # Increase template instantiation depth

    # Note: With algorithmic optimizations, template depth rarely exceeds 50 levels,
    # so this limit is very conservative. Previously needed for O(N) recursive patterns.

    # Optional: Enable compilation time tracing (Clang only)
    # Uncomment to generate time-trace JSON files for analysis
    # add_compile_options($<$<CXX_COMPILER_ID:Clang>:-ftime-trace>)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# ============================================================================
# Explicit template instantiation libraries (optional, for incremental builds)
# ============================================================================
# These libraries force compilation of commonly used headers to help with
# incremental builds. Since most SOPOT components use inline constexpr methods,
# explicit instantiation provides limited benefits. The main value is creating
# compilation dependencies for build system tracking.
#
# Can be disabled with: cmake -DSOPOT_USE_INSTANTIATION_LIBS=OFF

option(SOPOT_USE_INSTANTIATION_LIBS "Enable instantiation helper libraries (minimal benefit)" ON)

if(SOPOT_USE_INSTANTIATION_LIBS)
    # Core instantiations (Dual numbers, TypedComponent, etc.)
    add_library(sopot_core_inst STATIC src/core_instantiations.cpp)
    target_include_directories(sopot_core_inst PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(sopot_core_inst PRIVATE cxx_std_20)

    # Rocket instantiations (Vector3, Quaternion, rocket components)
    add_library(sopot_rocket_inst STATIC src/rocket_instantiations.cpp)
    target_include_directories(sopot_rocket_inst PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(sopot_rocket_inst PUBLIC sopot_core_inst)
    target_compile_features(sopot_rocket_inst PRIVATE cxx_std_20)

    # Physics instantiations (mass-spring systems)
    add_library(sopot_physics_inst STATIC src/physics_instantiations.cpp)
    target_include_directories(sopot_physics_inst PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(sopot_physics_inst PUBLIC sopot_core_inst)
    target_compile_features(sopot_physics_inst PRIVATE cxx_std_20)

    message(STATUS "Instantiation libraries: Enabled (use -DSOPOT_USE_INSTANTIATION_LIBS=OFF to disable)")
else()
    message(STATUS "Instantiation libraries: Disabled")
endif()

# ============================================================================
# Header-only library interfaces
# ============================================================================

# Header-only core library
add_library(sopot_core INTERFACE)
target_include_directories(sopot_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR})
# Link instantiation library if enabled (INTERFACE can link to static libraries)
if(SOPOT_USE_INSTANTIATION_LIBS)
    target_link_libraries(sopot_core INTERFACE sopot_core_inst)
endif()

# Header-only physics library
add_library(sopot_physics INTERFACE)
target_include_directories(sopot_physics INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/physics)
target_link_libraries(sopot_physics INTERFACE sopot_core)
if(SOPOT_USE_INSTANTIATION_LIBS)
    target_link_libraries(sopot_physics INTERFACE sopot_physics_inst)
endif()

# Header-only rocket library
add_library(sopot_rocket INTERFACE)
target_include_directories(sopot_rocket INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/rocket
    ${CMAKE_CURRENT_SOURCE_DIR}/io)
target_link_libraries(sopot_rocket INTERFACE sopot_core)
if(SOPOT_USE_INSTANTIATION_LIBS)
    target_link_libraries(sopot_rocket INTERFACE sopot_rocket_inst)
endif()

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

# Enable testing
enable_testing()

# Test executables
add_executable(compile_time_test tests/compile_time_test.cpp)
target_link_libraries(compile_time_test sopot_core sopot_physics)
target_compile_features(compile_time_test PRIVATE cxx_std_20)
add_test(NAME compile_time_test COMMAND compile_time_test)

add_executable(autodiff_test tests/autodiff_test.cpp)
target_link_libraries(autodiff_test sopot_core sopot_physics)
target_compile_features(autodiff_test PRIVATE cxx_std_20)
add_test(NAME autodiff_test COMMAND autodiff_test)

add_executable(coupled_oscillator_test tests/coupled_oscillator_test.cpp)
target_link_libraries(coupled_oscillator_test sopot_core sopot_physics)
target_compile_features(coupled_oscillator_test PRIVATE cxx_std_20)
add_test(NAME coupled_oscillator_test COMMAND coupled_oscillator_test)

add_executable(rocket_flight_test tests/rocket_flight_test.cpp)
target_link_libraries(rocket_flight_test sopot_core sopot_rocket)
target_compile_features(rocket_flight_test PRIVATE cxx_std_20)
add_test(NAME rocket_flight_test COMMAND rocket_flight_test)

add_executable(connected_masses_test tests/connected_masses_test.cpp)
target_link_libraries(connected_masses_test sopot_core sopot_physics)
target_compile_features(connected_masses_test PRIVATE cxx_std_20)
add_test(NAME connected_masses_test COMMAND connected_masses_test)

add_executable(connected_masses_validation_test tests/connected_masses_validation_test.cpp)
target_link_libraries(connected_masses_validation_test sopot_core sopot_physics)
target_compile_features(connected_masses_validation_test PRIVATE cxx_std_20)
add_test(NAME connected_masses_validation_test COMMAND connected_masses_validation_test)

add_executable(grid_2d_test tests/grid_2d_test.cpp)
target_link_libraries(grid_2d_test sopot_core sopot_physics)
target_compile_features(grid_2d_test PRIVATE cxx_std_20)
add_test(NAME grid_2d_test COMMAND grid_2d_test)

add_executable(kink_chain_test tests/kink_chain_test.cpp)
target_link_libraries(kink_chain_test sopot_core sopot_physics)
target_compile_features(kink_chain_test PRIVATE cxx_std_20)
add_test(NAME kink_chain_test COMMAND kink_chain_test)

# Experimental: Field-based component architecture
add_executable(simple_reactor_demo experimental/simple_reactor_demo.cpp)
target_include_directories(simple_reactor_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/experimental)
target_compile_features(simple_reactor_demo PRIVATE cxx_std_20)

add_executable(system_builder_demo experimental/system_builder_demo.cpp)
target_include_directories(system_builder_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/experimental)
target_compile_features(system_builder_demo PRIVATE cxx_std_20)

add_executable(auto_system_demo experimental/auto_system_demo.cpp)
target_include_directories(auto_system_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/experimental)
target_compile_features(auto_system_demo PRIVATE cxx_std_20)

# Reflection-based API prototype
add_executable(reflect_two_masses examples/reflect_two_masses.cpp)
target_include_directories(reflect_two_masses PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(reflect_two_masses PRIVATE cxx_std_20)

# Custom target to run all tests
add_custom_target(run_tests
    COMMAND compile_time_test
    COMMAND autodiff_test
    COMMAND coupled_oscillator_test
    COMMAND rocket_flight_test
    COMMAND connected_masses_test
    COMMAND connected_masses_validation_test
    COMMAND grid_2d_test
    COMMAND kink_chain_test
    DEPENDS compile_time_test autodiff_test coupled_oscillator_test rocket_flight_test connected_masses_test connected_masses_validation_test grid_2d_test kink_chain_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running SOPOT tests"
)

# ============================================================================
# Additional compilation time optimizations
# ============================================================================

# Enable unity builds for faster compilation (optional, can be enabled with -DCMAKE_UNITY_BUILD=ON)
# This combines multiple source files into single translation units
# Uncomment to enable by default:
# set(CMAKE_UNITY_BUILD ON)

# Apply precompiled headers to all test executables
set(TEST_TARGETS
    compile_time_test
    autodiff_test
    coupled_oscillator_test
    rocket_flight_test
    connected_masses_test
    connected_masses_validation_test
    grid_2d_test
    kink_chain_test
)

# First target creates the PCH, others reuse it
foreach(target ${TEST_TARGETS})
    if(target STREQUAL "compile_time_test")
        # First test creates the precompiled header
        target_precompile_headers(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core/pch.hpp)
    else()
        # Subsequent tests reuse the PCH
        target_precompile_headers(${target} REUSE_FROM compile_time_test)
    endif()
endforeach()

message(STATUS "SOPOT Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Architecture: Pure compile-time dispatch")
message(STATUS "  Performance: Zero runtime overhead")
message(STATUS "")
message(STATUS "Compilation Optimizations:")
message(STATUS "  Precompiled headers: Enabled")
message(STATUS "  Explicit instantiations: Enabled")
if(CCACHE_PROGRAM)
    message(STATUS "  ccache: Enabled (${CCACHE_PROGRAM})")
else()
    message(STATUS "  ccache: Not found (install ccache for faster rebuilds)")
endif()
if(CMAKE_UNITY_BUILD)
    message(STATUS "  Unity builds: Enabled")
else()
    message(STATUS "  Unity builds: Disabled (enable with -DCMAKE_UNITY_BUILD=ON)")
endif()
