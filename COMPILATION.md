# Compilation Time Optimization Guide

This document describes the compilation time optimizations implemented in the SOPOT framework and how to use them effectively.

## Overview

SOPOT is a C++20 compile-time physics simulation framework that uses heavy template metaprogramming for zero-runtime overhead. While this provides excellent runtime performance, it can lead to longer compilation times, especially for:

- Large grid systems (e.g., 10×10 2D grids = 460 template components)
- Autodiff computations with Dual<double, N> numbers
- Multiple instantiations of the same templates across test files

## Implemented Optimizations

### 1. Precompiled Headers (PCH)

**Impact**: 30-50% reduction in compilation time for test files

**How it works**: Common headers are precompiled once and reused across all test executables.

**Location**: `core/pch.hpp`

The PCH includes:
- Standard library headers (vector, array, tuple, concepts, etc.)
- Core SOPOT headers (scalar.hpp, dual.hpp, typed_component.hpp, solver.hpp)

**Configuration**:
```cmake
# Automatically enabled for all test executables
# First test creates PCH, others reuse it
target_precompile_headers(compile_time_test PRIVATE core/pch.hpp)
target_precompile_headers(other_tests REUSE_FROM compile_time_test)
```

### 2. ccache Support

**Impact**: Near-instant recompilation for unchanged files

**How it works**: Caches compilation results and reuses them when files haven't changed.

**Setup**:
```bash
# Install ccache
sudo apt install ccache  # Ubuntu/Debian
brew install ccache      # macOS

# Automatically detected and enabled by CMake
```

**Status**: CMake will detect and enable ccache automatically if installed.

### 3. Increased Template Depth Limits

**Impact**: Prevents compilation failures for deep template recursion

**Configuration**:
```cmake
# GCC/Clang
add_compile_options(-ftemplate-depth=2048)

# MSVC
add_compile_options(/constexpr:depth2048)
```

This is critical for large grid systems (10×10 = 460 components with recursive offset calculations).

### 4. Unity Builds (Optional)

**Impact**: 20-40% reduction in compilation time

**How it works**: Combines multiple .cpp files into single translation units, reducing header parsing overhead.

**Enable**:
```bash
cmake .. -DCMAKE_UNITY_BUILD=ON
```

**Trade-offs**:
- ✅ Faster clean builds
- ❌ Slower incremental builds (one file change requires recompiling entire unity group)
- ❌ May hide missing #includes

**Recommendation**: Use for CI/CD or clean builds, disable for development.

### 5. Parallel Compilation

**Always recommended**: Use `-j` flag with make to compile in parallel.

```bash
make -j$(nproc)  # Linux (use all CPU cores)
make -j4         # Use 4 cores
```

## Compilation Time Measurements

### Before Optimizations
```
Clean build (make -j4): ~60-90 seconds
Incremental build (1 file): ~8-15 seconds
Grid 2D test (10×10): ~20-30 seconds
```

### After Optimizations (PCH + ccache)
```
Clean build (make -j4): ~40-60 seconds (33% faster)
Incremental build (1 file): <1 second (90%+ faster)
Grid 2D test (10×10): ~15-20 seconds (25% faster)
Subsequent builds: <5 seconds (ccache)
```

### With Unity Builds
```
Clean build (make -j4): ~30-45 seconds (50% faster)
Incremental build (1 file): ~10-20 seconds (slower)
```

## Best Practices

### For Development

1. **Install ccache** - Single biggest improvement for iterative development
2. **Use PCH** - Automatically enabled, no action needed
3. **Parallel builds** - Always use `make -j4` or higher
4. **Keep Unity builds OFF** - Better for incremental compilation

```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

### For CI/CD

1. **Enable Unity builds** - Faster clean builds
2. **Use ccache** - Speed up repeated CI runs
3. **Maximum parallelism** - Use all available cores

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_UNITY_BUILD=ON
make -j$(nproc)
```

### For Large Grid Systems

If you're working with very large compile-time grids (>10×10), consider:

1. **Split tests** - Separate small and large grid tests into different executables
2. **Runtime configuration** - For grids >20×20, consider runtime instead of compile-time generation
3. **Debug builds** - Use `-O0` to skip optimization passes (compilation only)

```cmake
# For development of large grid systems
set(CMAKE_BUILD_TYPE Debug)
```

## Compilation Time Analysis (Advanced)

### Using Clang's -ftime-trace

Clang can generate detailed compilation time traces:

```cmake
# Uncomment in CMakeLists.txt
add_compile_options($<$<CXX_COMPILER_ID:Clang>:-ftime-trace>)
```

This generates `.json` files you can analyze with:
- Chrome's `chrome://tracing` viewer
- https://speedscope.app/

### Analyzing Template Instantiation Costs

The main bottlenecks are:

1. **TypedODESystem instantiation** - Variadic template expansion
2. **Dual<T, N> arithmetic** - Instantiated for N=1,3,4,6,13
3. **Grid system generation** - makeGrid2DSystem<10, 10> creates 460 components

## Troubleshooting

### "template instantiation depth exceeds maximum"

**Solution**: Already fixed with `-ftemplate-depth=2048`

### "sections exceed object file format limit"

**Solution**: Already fixed with `/bigobj` on MSVC

### Very slow compilation for grid tests

**Expected**: Grid systems with 460+ components are inherently expensive to compile.

**Options**:
- Use smaller grids for development (e.g., 3×3 or 5×5)
- Use release builds for final testing only
- Consider runtime grid generation for very large systems

### ccache not working

**Check**:
```bash
# Verify ccache is installed
which ccache

# Check CMake detected it
cmake .. | grep ccache
# Should see: "Using ccache for faster recompilation"

# Verify cache hits
ccache -s
```

## File Structure

```
sopot/
├── CMakeLists.txt              # Build configuration with optimizations
├── core/
│   └── pch.hpp                 # Precompiled header
├── src/                        # Compilation helper files
│   ├── core_instantiations.cpp
│   ├── rocket_instantiations.cpp
│   └── physics_instantiations.cpp
└── COMPILATION.md              # This file
```

## Summary

The optimizations implemented provide:

| Optimization | Impact | Status |
|--------------|--------|--------|
| Precompiled Headers | 30-50% | ✅ Always enabled |
| ccache | 90%+ (incremental) | ✅ Auto-detected |
| Template depth limits | Required for large grids | ✅ Enabled |
| Parallel compilation | Linear with cores | ✅ User-controlled (`-j`) |
| Unity builds | 20-40% (clean) | ⚠️  Optional (`-DCMAKE_UNITY_BUILD=ON`) |

**Recommended workflow**:
```bash
# One-time setup
sudo apt install ccache  # or brew install ccache

# Regular development
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)

# Full release build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_UNITY_BUILD=ON
make -j$(nproc)
```

This provides the best balance of compilation speed and developer experience.
