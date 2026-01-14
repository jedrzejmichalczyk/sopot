# SOPOT Build Pipeline Documentation

## Overview

This document describes the complete C++ → WebAssembly → Frontend pipeline for the SOPOT rocket simulation framework, including build processes, deployment, and recent improvements.

## Pipeline Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                           SOURCE CODE                                │
│  C++20 Physics Framework (rocket/, core/, io/)                      │
└────────────────────────┬────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      WASM COMPILATION                                │
│  Emscripten 3.1.51 + embind bindings                                │
│  • Input: wasm/wasm_rocket.cpp                                      │
│  • Output: sopot.js (loader) + sopot.wasm (binary)                  │
│  • Flags: -O3, ES6 modules, memory growth, exceptions              │
└────────────────────────┬────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     FRONTEND INTEGRATION                             │
│  React + TypeScript + Vite                                          │
│  • Dynamic WASM loading with error boundaries                       │
│  • Performance telemetry and monitoring                             │
│  • React Three Fiber 3D visualization                               │
└────────────────────────┬────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         DEPLOYMENT                                   │
│  GitHub Actions → GitHub Pages                                      │
│  • Automated builds on push                                         │
│  • WASM validation and size reporting                               │
│  • Base path configuration for GitHub Pages                         │
└─────────────────────────────────────────────────────────────────────┘
```

## Stage 1: C++ Compilation to WebAssembly

### Build Configuration

**Location**: `wasm/CMakeLists.txt`, `wasm/build.sh`

**Compiler**: Emscripten 3.1.51 (pinned for reproducibility)

**Key Compilation Flags**:
```bash
-std=c++20                        # C++20 standard required
-O3                               # Maximum optimization
-lembind                          # Enable embind JavaScript bindings
-s WASM=1                         # Generate WebAssembly output
-s ALLOW_MEMORY_GROWTH=1          # Dynamic memory allocation
-s MODULARIZE=1                   # ES6 module format
-s EXPORT_ES6=1                   # ES6 export syntax
-s EXPORT_NAME="createSopotModule"
-s ENVIRONMENT=web,worker         # Target: browsers and web workers
-s NO_DISABLE_EXCEPTION_CATCHING  # C++ exception handling
-fexceptions                      # Enable exceptions
```

### Build Process

**Quick build**:
```bash
cd wasm
./build.sh Release
```

**Output**:
- `sopot.js` (~100-200 KB): JavaScript loader and glue code
- `sopot.wasm` (~600-800 KB): Compiled WebAssembly binary

### Embind Bindings

**File**: `wasm/wasm_rocket.cpp`

Exposes C++ `RocketSimulator` class to JavaScript with:
- Configuration methods (launcher angles, diameter, timestep)
- **Phase 2 array-based data loading** ✅ (NEW)
- Simulation control (setup, reset, step)
- State queries (position, velocity, quaternion, altitude, etc.)

**Key Feature**: Array-based loading eliminates file I/O requirements:
```cpp
void loadMassData(const val& time_js, const val& mass_js);
void loadEngineData(const val& time_js, const val& thrust_js);
```

JavaScript arrays are converted to C++ vectors and written to Emscripten's virtual filesystem, then loaded via existing CSV parsers.

## Stage 2: Frontend Integration

### Technology Stack

**Framework**: React 18 with TypeScript
**Bundler**: Vite 5
**3D Rendering**: React Three Fiber + Three.js
**State Management**: React hooks (useState, useEffect, custom hooks)
**UI Components**: Recharts for telemetry graphs

### WASM Loading Hook

**File**: `web/src/hooks/useRocketSimulation.ts`

**Features**:
- ✅ Dynamic module loading with base path handling (GitHub Pages support)
- ✅ **Performance telemetry** - tracks load time and instantiation time
- ✅ **Error handling** - detailed error messages and stack traces
- ✅ Analytics integration (Google Analytics event tracking)
- ✅ Memory leak prevention with cleanup on unmount

**Loading Flow**:
```typescript
1. Construct module URL from BASE_URL environment variable
2. Dynamic import with @vite-ignore for runtime loading
3. Instantiate WASM module
4. Log performance metrics (loader time, instantiation time, total time)
5. Return module instance or error state
```

### Error Boundaries

**Files**:
- `web/src/components/ErrorBoundary.tsx` - Generic React error boundary
- `web/src/components/WasmErrorFallback.tsx` - WASM-specific error UI

**Features**:
- ✅ Graceful degradation on WASM loading failure
- ✅ Detailed error diagnostics (error message, stack trace, component stack)
- ✅ User-friendly troubleshooting guide
- ✅ Reload and report issue buttons
- ✅ Browser compatibility detection

**Integration**:
```tsx
<ErrorBoundary>
  <App />
</ErrorBoundary>
```

### NPM Scripts

**File**: `web/package.json`

```json
{
  "copy-wasm": "cp ../wasm/sopot.* public/",
  "copy-wasm:verify": "npm run copy-wasm && ls -lh public/sopot.*",
  "prebuild": "npm run copy-wasm",  // Auto-copy before build
  "dev": "npm run copy-wasm && vite",
  "build": "tsc && vite build",
  "build:github": "VITE_BASE_PATH=/sopot/ npm run build"
}
```

**Improvement**: Automatic WASM file copying eliminates manual step during development.

## Stage 3: Build Automation (CI/CD)

### GitHub Actions Workflow

**File**: `.github/workflows/deploy-github-pages.yml`

**Triggers**:
- Push to `main` or `master` branches (paths: `web/**`, `wasm/**`, workflow file)
- Manual workflow dispatch

### Build Steps

#### 1. Environment Setup
```yaml
- Checkout repository (actions/checkout@v4)
- Setup Node.js 20 with npm caching
- Setup Emscripten 3.1.51 (PINNED VERSION ✅)
```

**Improvement**: Emscripten version pinned from `latest` to `3.1.51` for reproducible builds.

#### 2. WASM Compilation
```bash
cd wasm && ./build.sh Release
```

#### 3. Frontend Dependencies
```bash
cd web && npm ci
```

#### 4. Copy and Validate WASM Files ✅ (NEW)

**Before** (unreliable):
```bash
cp wasm/sopot.js wasm/sopot.wasm web/public/
```

**After** (validated):
```bash
# Copy files
cp wasm/sopot.js wasm/sopot.wasm web/public/

# Validate existence
test -f web/public/sopot.js || exit 1
test -f web/public/sopot.wasm || exit 1

# Validate non-empty
test -s web/public/sopot.js || exit 1
test -s web/public/sopot.wasm || exit 1
```

**Impact**: Build fails fast if WASM compilation produced invalid output.

#### 5. WASM Metrics Reporting ✅ (NEW)

```bash
# Generate GitHub Step Summary with file sizes
echo "## 📊 WASM Build Metrics" >> $GITHUB_STEP_SUMMARY
# Report sopot.wasm and sopot.js sizes in bytes and human-readable format
# Warn if WASM exceeds 2 MB
```

**Output Example**:
```
## 📊 WASM Build Metrics

| File        | Size        | Size (Human) |
|-------------|-------------|--------------|
| sopot.wasm  | 734,298 bytes | 717K       |
| sopot.js    | 156,401 bytes | 153K       |

**Total Size:** 890,699 bytes (0.85 MB)
```

#### 6. Frontend Build
```bash
cd web
VITE_BASE_PATH=/sopot/ npm run build
```

Base path ensures correct WASM loading on GitHub Pages subdirectory.

#### 7. Deployment
```yaml
- Upload artifact: web/dist
- Deploy to GitHub Pages (actions/deploy-pages@v4)
```

### CI/CD Improvements Summary

| Improvement | Status | Impact |
|-------------|--------|--------|
| Pin Emscripten version | ✅ | Reproducible builds |
| Validate WASM files | ✅ | Fail fast on compilation errors |
| Report WASM sizes | ✅ | Track binary size regressions |
| Auto-copy WASM files | ✅ | Eliminate manual dev steps |
| Performance telemetry | ✅ | Monitor load performance |

## Stage 4: Deployment and Runtime

### GitHub Pages Configuration

**URL**: `https://<username>.github.io/sopot/`

**Base Path Handling**:
```typescript
// Vite config
base: process.env.VITE_BASE_PATH || '/',

// Runtime loading
const basePath = import.meta.env.BASE_URL || '/';
const moduleUrl = `${basePath}sopot.js`;
```

### Browser Requirements

- **WebAssembly support** (all modern browsers)
- **ES6 modules** (Chrome 61+, Firefox 60+, Safari 11+, Edge 16+)
- **WebGL 2** for Three.js rendering

### Performance Characteristics

**WASM Module**:
- Load time: 100-500ms (depending on network)
- Instantiation time: 50-200ms
- Total ready time: 150-700ms

**Simulation**:
- Native C++ performance: ~0.3 μs per derivative evaluation
- WASM performance: ~70-80% of native
- Achievable: 60 FPS with real-time physics

**Monitoring**:
- Console logs: `[WASM] Loading module...`, `[WASM] Module instantiated in Xms`
- Google Analytics events: `wasm_load_complete`, `wasm_load_error`

## Phase 2 Implementation Details

### Array-Based Data Loading

**Problem (Phase 1)**: Web browsers cannot access local files via `file://` protocol. Previous implementation required HTTP server to serve CSV files.

**Solution (Phase 2)**: Accept JavaScript arrays directly, bypassing file I/O.

**Implementation**:
```cpp
// wasm/wasm_rocket.cpp
void loadMassData(const val& time_js, const val& mass_js) {
    // 1. Convert JS arrays to C++ vectors
    std::vector<double> time_vec = vecFromJSArray<double>(time_js);
    std::vector<double> mass_vec = vecFromJSArray<double>(mass_js);

    // 2. Validate inputs
    if (time_vec.size() != mass_vec.size()) {
        throw std::runtime_error("Size mismatch");
    }

    // 3. Write to Emscripten virtual filesystem
    writeCSV("/tmp/sim_mass.csv", {"time", "mass"}, {time_vec, mass_vec});

    // 4. Load via existing CSV parser infrastructure
    m_rocket.loadMassData("/tmp/");
}
```

**Benefits**:
- ✅ No HTTP server required for development
- ✅ Works in sandboxed environments
- ✅ Easier data manipulation from JavaScript
- ✅ Reuses existing C++ CSV parsing infrastructure

### Engine Data Simplification

**Previous**: Required 8-column CSV (time, throat_d, exit_d, p_comb, T_comb, gamma, mol_mass, efficiency)

**New**: Simplified to time + thrust arrays. Engine parameters estimated from thrust curve:
```cpp
void loadEngineData(const val& time_js, const val& thrust_js) {
    // Auto-generate reasonable engine parameters
    // - Throat/exit diameters from thrust magnitude
    // - Standard solid propellant properties (p_c=5MPa, T=3000K, gamma=1.24)
    // - Typical nozzle efficiency (96%)
}
```

**Impact**: Makes web interface accessible to non-rocket-scientist users.

## Troubleshooting Guide

### Build Issues

**"emcc: command not found"**
```bash
# Install Emscripten SDK
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install 3.1.51
./emsdk activate 3.1.51
source ./emsdk_env.sh
```

**WASM file validation fails in CI/CD**
- Check wasm/build.sh output for compilation errors
- Ensure CMake configuration is correct
- Verify all required headers are available

### Frontend Issues

**"Failed to load WebAssembly module"**
- Check browser console for detailed error
- Verify WASM files are in `web/public/` directory
- Run `npm run copy-wasm:verify` to check file presence
- Ensure BASE_URL is configured correctly

**TypeScript errors on WASM imports**
- Check `web/src/types/sopot.d.ts` is present
- Ensure `@ts-ignore` and `/* @vite-ignore */` are used for dynamic imports

**Error boundary shows but no specific error**
- Open browser DevTools console (F12)
- Check Network tab for 404 errors on WASM files
- Verify browser supports WebAssembly: `typeof WebAssembly !== 'undefined'`

### Performance Issues

**Slow WASM loading**
- Check Network tab: WASM file should be <1 MB
- Enable compression on web server (gzip/brotli)
- Consider CDN for production deployment

**Low simulation FPS**
- Reduce timestep (increase dt)
- Lower trajectory history limit
- Check browser DevTools Performance profiler

## Best Practices

### Development Workflow

1. **Make C++ changes** in `core/`, `rocket/`, or `wasm/wasm_rocket.cpp`
2. **Rebuild WASM**: `cd wasm && ./build.sh Release`
3. **Copy to web**: `cd web && npm run copy-wasm:verify`
4. **Test locally**: `npm run dev`
5. **Commit both C++ and WASM changes**

### Performance Optimization

- Use `-O3` for production builds (already enabled)
- Profile with `emcc --profiling` for debug builds
- Monitor WASM size: keep under 1 MB if possible
- Use `console.time()`/`console.timeEnd()` for JS profiling

### Error Handling

- Always wrap WASM operations in try-catch
- Log performance metrics for monitoring
- Use error boundaries for graceful degradation
- Provide actionable error messages to users

## Future Improvements

### Short Term
- [ ] Add WebAssembly streaming compilation (faster instantiation)
- [ ] Implement service worker for offline support
- [ ] Add WASM module caching strategy
- [ ] Create npm package for reusable WASM module

### Medium Term
- [ ] WebGPU compute shaders for trajectory optimization
- [ ] Multi-threaded simulation with Web Workers
- [ ] SIMD optimizations for vector operations
- [ ] Real-time collaborative simulations (WebRTC)

### Long Term
- [ ] WebXR (VR/AR) visualization
- [ ] Cloud-based simulation backend
- [ ] Machine learning integration for parameter optimization
- [ ] Mobile app with React Native + WASM

## Resources

### Documentation
- [Emscripten Documentation](https://emscripten.org/docs/)
- [embind Reference](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
- [Vite Documentation](https://vitejs.dev/)
- [React Three Fiber](https://docs.pmnd.rs/react-three-fiber/)

### Internal Docs
- `CLAUDE.md` - SOPOT framework architecture
- `wasm/README.md` - WASM module API reference
- `web/README.md` - Frontend application guide
- `DEPLOYMENT.md` - Deployment instructions

## Changelog

### 2026-01-14 - Pipeline Improvements

**Phase 2 Implementation Complete**:
- ✅ Array-based data loading (no file I/O)
- ✅ Simplified engine data interface
- ✅ Demo mode with embedded data

**CI/CD Improvements**:
- ✅ Pinned Emscripten version (3.1.51)
- ✅ WASM file validation in build pipeline
- ✅ Size reporting in GitHub Actions summary
- ✅ Auto-copy WASM files in npm scripts

**Frontend Improvements**:
- ✅ React error boundaries with fallback UI
- ✅ Performance telemetry for WASM loading
- ✅ Detailed error diagnostics
- ✅ Browser compatibility checks

---

**Pipeline Status**: ✅ Production Ready

Last Updated: 2026-01-14
