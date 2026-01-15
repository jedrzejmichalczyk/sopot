#!/bin/bash

# SOPOT WebAssembly Build Script
# Builds the SOPOT rocket simulator for WebAssembly using Emscripten

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}SOPOT WebAssembly Build Script${NC}"
echo "================================"

# Check if emcc is available
if ! command -v emcc &> /dev/null; then
    echo -e "${RED}Error: emcc (Emscripten) not found${NC}"
    echo "Please install Emscripten: https://emscripten.org/docs/getting_started/downloads.html"
    echo "Or activate emsdk: source /path/to/emsdk/emsdk_env.sh"
    exit 1
fi

# Show Emscripten version
echo -e "${GREEN}Emscripten version:${NC}"
emcc --version | head -n 1

# Check Emscripten version for C++20 support (requires 3.1.0+)
EMCC_VERSION=$(emcc --version | head -n 1 | grep -oP '\d+\.\d+\.\d+' | head -n 1)
REQUIRED_VERSION="3.1.0"

if [ -n "$EMCC_VERSION" ]; then
    # Compare versions (requires sort -V)
    if printf '%s\n' "$REQUIRED_VERSION" "$EMCC_VERSION" | sort -V -C 2>/dev/null; then
        echo -e "${GREEN}Version check passed (>= ${REQUIRED_VERSION})${NC}"
    else
        echo -e "${YELLOW}Warning: Emscripten ${EMCC_VERSION} detected${NC}"
        echo -e "${YELLOW}C++20 features require Emscripten ${REQUIRED_VERSION} or later${NC}"
        echo -e "${YELLOW}Build may fail if version is too old. Consider upgrading.${NC}"
        echo ""
    fi
else
    echo -e "${YELLOW}Warning: Could not detect Emscripten version${NC}"
    echo ""
fi

# Build mode (default: Release)
BUILD_MODE="${1:-Release}"

echo -e "${GREEN}Build mode: ${BUILD_MODE}${NC}"
echo ""

# Choose build method
BUILD_METHOD="${2:-direct}"

if [ "$BUILD_METHOD" = "cmake" ]; then
    echo -e "${YELLOW}Building with CMake...${NC}"

    # Create build directory
    mkdir -p build
    cd build

    # Configure with emcmake
    emcmake cmake .. -DCMAKE_BUILD_TYPE="$BUILD_MODE"

    # Build
    emmake make -j4

    # Copy outputs to parent directory
    cp sopot.js ../
    cp sopot.wasm ../

    cd ..

else
    echo -e "${YELLOW}Building with direct emcc...${NC}"

    # Optimization flags based on build mode
    if [ "$BUILD_MODE" = "Release" ]; then
        OPT_FLAGS="-O3 -s ASSERTIONS=0"
    else
        OPT_FLAGS="-O0 -g -s ASSERTIONS=1 -s SAFE_HEAP=1"
    fi

    # Build command - includes both rocket and grid2d
    emcc -std=c++20 \
        $OPT_FLAGS \
        -lembind \
        -s WASM=1 \
        -s ALLOW_MEMORY_GROWTH=1 \
        -s MODULARIZE=1 \
        -s 'EXPORT_NAME="createSopotModule"' \
        -s EXPORT_ES6=1 \
        -s ENVIRONMENT=web,worker \
        -s NO_DISABLE_EXCEPTION_CATCHING \
        -fexceptions \
        -ftemplate-depth=512 \
        -fconstexpr-depth=512 \
        -I.. \
        wasm_rocket.cpp \
        wasm_grid2d.cpp \
        -o sopot.js
fi

# Check if build succeeded
if [ -f "sopot.js" ] && [ -f "sopot.wasm" ]; then
    echo ""
    echo -e "${GREEN}Build successful!${NC}"
    echo ""
    echo "Output files:"
    ls -lh sopot.js sopot.wasm
    echo ""
    echo "Wasm module size:"
    wc -c sopot.wasm | awk '{printf "  %.2f KB\n", $1/1024}'
    echo ""
    echo "Usage in JavaScript/TypeScript:"
    echo "  import createSopotModule from './sopot.js';"
    echo "  const Module = await createSopotModule();"
    echo "  const sim = new Module.RocketSimulator();"
    echo ""
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi
