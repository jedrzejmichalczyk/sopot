# WASM Build Guide for SOPOT

## Issue: `$.getCenterOfMass is not a function`

### Root Cause

The error occurs because the WebAssembly (WASM) module has not been built. The `getCenterOfMass()` function **is properly implemented** in the C++ code (`wasm/wasm_grid2d.cpp:385-406`) and correctly bound via Emscripten (`wasm/wasm_grid2d.cpp:447`), but the WASM files (`sopot.js` and `sopot.wasm`) need to be generated before the web interface can use them.

### Why This Happens

- WASM files are **not committed** to the repository (they're generated artifacts)
- In production, GitHub Actions automatically builds the WASM module during CI/CD
- For local development, you need to build the WASM module manually

## Solutions (Choose One)

### Option 1: Install Emscripten and Build Locally (Recommended)

1. **Install Emscripten SDK:**
   ```bash
   # Clone the emsdk repository
   cd ~
   git clone https://github.com/emscripten-core/emsdk.git
   cd emsdk

   # Install and activate Emscripten 3.1.51 (matches CI version)
   ./emsdk install 3.1.51
   ./emsdk activate 3.1.51

   # Activate PATH and other environment variables
   source ./emsdk_env.sh
   ```

2. **Build the WASM module:**
   ```bash
   cd /home/user/sopot/wasm
   ./build.sh Release
   ```

3. **Copy WASM files to web public directory:**
   ```bash
   cp sopot.js sopot.wasm ../web/public/
   ```

4. **Start the web development server:**
   ```bash
   cd ../web
   npm install  # If you haven't already
   npm run dev
   ```

### Option 2: Use Docker (No Emscripten Installation Required)

Create a Docker-based build script:

```bash
# From the sopot root directory
docker run --rm -v $(pwd):/src -w /src/wasm emscripten/emsdk:3.1.51 \
  ./build.sh Release

# Copy WASM files
cp wasm/sopot.js wasm/sopot.wasm web/public/

# Start web dev server
cd web && npm run dev
```

### Option 3: Download Pre-built WASM from CI

If you have access to the GitHub repository, you can download WASM artifacts from a successful CI run:

1. Go to: https://github.com/jedrzejmichalczyk/sopot/actions
2. Find a successful "CI" workflow run on the master branch
3. Download the build artifacts (if available)
4. Extract `sopot.js` and `sopot.wasm` to `web/public/`

**Note:** This option requires that the CI run has artifact uploads enabled.

### Option 4: Quick Script (Auto-detect and Build)

Save this as `build-wasm.sh` in the repository root:

```bash
#!/bin/bash
set -e

echo "SOPOT WASM Build Helper"
echo "======================="

# Check if Emscripten is available
if command -v emcc &> /dev/null; then
    echo "✅ Emscripten found"
    cd wasm
    ./build.sh Release
    cd ..
elif command -v docker &> /dev/null; then
    echo "✅ Docker found, using Docker build"
    docker run --rm -v $(pwd):/src -w /src/wasm emscripten/emsdk:3.1.51 ./build.sh Release
else
    echo "❌ Neither Emscripten nor Docker found"
    echo ""
    echo "Please install Emscripten or Docker:"
    echo "  - Emscripten: https://emscripten.org/docs/getting_started/downloads.html"
    echo "  - Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Copy WASM files
echo "Copying WASM files to web/public/..."
cp wasm/sopot.js wasm/sopot.wasm web/public/

echo ""
echo "✅ WASM build complete!"
echo ""
echo "Files:"
ls -lh web/public/sopot.{js,wasm}
echo ""
echo "You can now run: cd web && npm run dev"
```

Make it executable:
```bash
chmod +x build-wasm.sh
./build-wasm.sh
```

## Verification

After building, verify the WASM files exist:

```bash
ls -lh web/public/sopot.js web/public/sopot.wasm
```

You should see both files with reasonable sizes:
- `sopot.js`: ~50-100 KB
- `sopot.wasm`: ~200-500 KB

## Troubleshooting

### "emcc: command not found"

You need to activate Emscripten environment:
```bash
source ~/emsdk/emsdk_env.sh
```

Or add it to your shell profile:
```bash
echo 'source ~/emsdk/emsdk_env.sh' >> ~/.bashrc
source ~/.bashrc
```

### Build fails with C++20 errors

Ensure you're using Emscripten 3.1.51 or later:
```bash
emcc --version
```

### WASM files are empty or missing

Check the build output for errors:
```bash
cd wasm
./build.sh Release 2>&1 | tee build.log
```

### getCenterOfMass still not found after building

1. Clear browser cache (hard refresh: Ctrl+F5 or Cmd+Shift+R)
2. Verify files are in `web/public/`: `ls web/public/sopot.*`
3. Restart the development server: `cd web && npm run dev`

## Implementation Details

The `getCenterOfMass()` function is implemented in:
- **C++ Implementation:** `/wasm/wasm_grid2d.cpp:385-406`
- **Emscripten Binding:** `/wasm/wasm_grid2d.cpp:447`
- **TypeScript Types:** `/web/src/types/sopot.d.ts:152`
- **Usage:** `/web/src/hooks/useGrid2DSimulation.ts:90`

The function computes the center of mass as:
```
CoM = Σ(m_i * r_i) / Σ(m_i)
```

Where:
- `m_i` is the mass of particle `i`
- `r_i` is the position vector of particle `i`
- The sum is over all particles in the Grid2D system

## For Maintainers

### CI/CD Workflow

The WASM module is automatically built by GitHub Actions:
- **CI Tests:** `.github/workflows/ci.yml` (lines 10-56)
- **Deployment:** `.github/workflows/deploy-github-pages.yml` (lines 49-86)

### Adding New WASM Functions

When adding new functions to the WASM interface:

1. Implement in C++ (e.g., `wasm/wasm_grid2d.cpp`)
2. Add Emscripten binding:
   ```cpp
   EMSCRIPTEN_BINDINGS(grid2d_module) {
       class_<Grid2DSimulator>("Grid2DSimulator")
           .function("myNewFunction", &Grid2DSimulator::myNewFunction);
   }
   ```
3. Add TypeScript type definition (`web/src/types/sopot.d.ts`)
4. Rebuild WASM: `cd wasm && ./build.sh Release`
5. Copy to web: `cp wasm/sopot.* web/public/`
6. Use in frontend code

## References

- [Emscripten Documentation](https://emscripten.org/docs/)
- [Embind (C++/JS Binding)](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
- [SOPOT Project Documentation](README.md)
