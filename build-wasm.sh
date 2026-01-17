#!/bin/bash
# SOPOT WASM Build Helper
# Auto-detects available build tools (Emscripten or Docker) and builds the WASM module

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}SOPOT WASM Build Helper${NC}"
echo "======================="
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check if Emscripten is available
if command -v emcc &> /dev/null; then
    echo -e "${GREEN}✅ Emscripten found${NC}"
    EMCC_VERSION=$(emcc --version | head -n 1 | grep -oP '\d+\.\d+\.\d+' | head -n 1 || echo "unknown")
    echo "   Version: $EMCC_VERSION"
    echo ""

    # Check version
    REQUIRED_VERSION="3.1.0"
    if [ "$EMCC_VERSION" != "unknown" ]; then
        if printf '%s\n' "$REQUIRED_VERSION" "$EMCC_VERSION" | sort -V -C 2>/dev/null; then
            echo -e "${GREEN}   Version check passed (>= ${REQUIRED_VERSION})${NC}"
        else
            echo -e "${YELLOW}   Warning: Version ${EMCC_VERSION} detected${NC}"
            echo -e "${YELLOW}   C++20 features require Emscripten ${REQUIRED_VERSION} or later${NC}"
            echo -e "${YELLOW}   Build may fail. Consider upgrading.${NC}"
        fi
    fi
    echo ""

    echo "Building with Emscripten..."
    cd wasm
    ./build.sh Release
    cd ..

elif command -v docker &> /dev/null; then
    echo -e "${GREEN}✅ Docker found${NC}"
    echo "   (Emscripten not found, using Docker build)"
    echo ""

    echo "Building with Docker (emscripten/emsdk:3.1.51)..."
    echo "This may take a moment on first run (downloading Docker image)..."
    echo ""

    docker run --rm \
        -v "$SCRIPT_DIR:/src" \
        -w /src/wasm \
        emscripten/emsdk:3.1.51 \
        ./build.sh Release

else
    echo -e "${RED}❌ Neither Emscripten nor Docker found${NC}"
    echo ""
    echo "To fix this issue, you need to build the WASM module."
    echo "Please install one of the following:"
    echo ""
    echo -e "${YELLOW}Option 1: Install Emscripten (Recommended)${NC}"
    echo "  1. Clone emsdk:"
    echo "     git clone https://github.com/emscripten-core/emsdk.git ~/emsdk"
    echo ""
    echo "  2. Install and activate:"
    echo "     cd ~/emsdk"
    echo "     ./emsdk install 3.1.51"
    echo "     ./emsdk activate 3.1.51"
    echo "     source ./emsdk_env.sh"
    echo ""
    echo "  3. Add to your shell profile (optional):"
    echo "     echo 'source ~/emsdk/emsdk_env.sh' >> ~/.bashrc"
    echo ""
    echo -e "${YELLOW}Option 2: Install Docker${NC}"
    echo "  Visit: https://docs.docker.com/get-docker/"
    echo ""
    echo "After installation, run this script again."
    echo ""
    echo -e "${BLUE}For more details, see: WASM_BUILD_GUIDE.md${NC}"
    exit 1
fi

# Copy WASM files to web public directory
echo ""
echo "Copying WASM files to web/public/..."
mkdir -p web/public
cp wasm/sopot.js wasm/sopot.wasm web/public/

# Verify files
if [ -f "web/public/sopot.js" ] && [ -f "web/public/sopot.wasm" ]; then
    echo ""
    echo -e "${GREEN}✅ WASM build complete!${NC}"
    echo ""
    echo "Output files:"
    ls -lh web/public/sopot.js web/public/sopot.wasm
    echo ""

    # Get sizes
    WASM_SIZE=$(stat -c%s "web/public/sopot.wasm" 2>/dev/null || stat -f%z "web/public/sopot.wasm" 2>/dev/null)
    WASM_KB=$((WASM_SIZE / 1024))

    echo "WASM module size: ${WASM_KB} KB"

    if [ $WASM_SIZE -lt 50000 ]; then
        echo -e "${YELLOW}⚠️  Warning: WASM file seems too small (< 50 KB). Build may be incomplete.${NC}"
        echo "   Check wasm/build.sh output for errors."
    fi

    echo ""
    echo -e "${GREEN}Next steps:${NC}"
    echo "  1. cd web"
    echo "  2. npm install  # If you haven't already"
    echo "  3. npm run dev  # Start development server"
    echo ""
    echo "The getCenterOfMass() function should now be available!"

else
    echo -e "${RED}❌ Error: WASM files not found after build${NC}"
    echo "Expected files:"
    echo "  - web/public/sopot.js"
    echo "  - web/public/sopot.wasm"
    echo ""
    echo "Check wasm/build.sh output for errors."
    exit 1
fi
