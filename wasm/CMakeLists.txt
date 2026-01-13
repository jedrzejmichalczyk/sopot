cmake_minimum_required(VERSION 3.20)
project(sopot_wasm)

# Ensure we're using Emscripten
if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "This CMakeLists.txt is for Emscripten only. Use: emcmake cmake ..")
endif()

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories (SOPOT is header-only)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# Source files
set(SOURCES
    wasm_rocket.cpp
)

# Emscripten-specific compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")

# Emscripten link flags
set(EMSCRIPTEN_LINK_FLAGS
    -lembind
    -s WASM=1
    -s ALLOW_MEMORY_GROWTH=1
    -s MODULARIZE=1
    -s EXPORT_NAME=createSopotModule
    -s EXPORT_ES6=1
    -s ENVIRONMENT=web,worker
    -s NO_DISABLE_EXCEPTION_CATCHING
    --bind
)

# Optimization flags for release
if(CMAKE_BUILD_TYPE MATCHES Release)
    list(APPEND EMSCRIPTEN_LINK_FLAGS
        -O3
        -s ASSERTIONS=0
    )
else()
    list(APPEND EMSCRIPTEN_LINK_FLAGS
        -O0
        -g
        -s ASSERTIONS=1
        -s SAFE_HEAP=1
    )
endif()

# Join flags into a string
string(REPLACE ";" " " EMSCRIPTEN_LINK_FLAGS_STR "${EMSCRIPTEN_LINK_FLAGS}")

# Create the WebAssembly module
add_executable(sopot ${SOURCES})

# Set linker flags
set_target_properties(sopot PROPERTIES
    LINK_FLAGS "${EMSCRIPTEN_LINK_FLAGS_STR}"
    SUFFIX ".js"
)

# Output to current directory
set_target_properties(sopot PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Install target (optional)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/sopot.js
    ${CMAKE_CURRENT_SOURCE_DIR}/sopot.wasm
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)
