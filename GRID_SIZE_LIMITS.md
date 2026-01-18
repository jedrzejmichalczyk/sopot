# Triangular Grid Compilation Time Analysis

## Summary

After optimization work on the SOPOT framework, we tested various triangular grid sizes to determine realistic compilation limits. The triangular grid uses the full X-pattern connectivity (horizontal + vertical + both diagonal edges).

## Test Results

| Grid Size | Masses | Springs | State Dim | Compile Time | Status | Memory Usage |
|-----------|--------|---------|-----------|--------------|--------|--------------|
| 3×3       | 9      | 24      | 36        | ~4.2s        | ✓ OK   | Low          |
| 4×4       | 16     | 42      | 64        | ~14.5s       | ✓ OK   | Low          |
| 5×5       | 25     | 72      | 100       | ~55.5s       | ✓ OK   | Moderate     |
| 6×6       | 36     | 110     | 144       | ~5m 14s      | ✓ OK   | ~2-3GB       |
| 7×7       | 49     | 156     | 196       | >20min       | ✗ FAIL | ~10GB        |
| 8×8       | 64     | 210     | 256       | Not tested   | —      | —            |

### Edge Count Formula

For a triangular grid of size R×C (rows × cols):
- **Horizontal edges**: R × (C - 1)
- **Vertical edges**: (R - 1) × C
- **Diagonal edges**: 2 × (R - 1) × (C - 1)
- **Total edges**: R×(C-1) + (R-1)×C + 2×(R-1)×(C-1)

For square grids (N×N):
- **Total edges**: 4N² - 6N + 2

## Compilation Time Growth

The compilation time grows approximately **exponentially** with grid size:

```
3×3:   4 seconds     (baseline)
4×4:   15 seconds    (3.5× increase)
5×5:   55 seconds    (3.8× increase)
6×6:   314 seconds   (5.7× increase)
7×7:   >1200 seconds (~4× increase, extrapolated)
```

## Practical Recommendations

### Development & Iteration (Fast compile times)
- **Use 3×3 or 4×4 grids** (~4-15 seconds)
- Ideal for rapid prototyping, testing new features
- Fast enough for CI/CD pipelines

### Testing & Validation (Moderate compile times)
- **Use 5×5 grids** (~1 minute)
- Good balance between fidelity and compile time
- Suitable for unit tests and validation

### Production Simulations (Slow compile times)
- **Use 6×6 grids maximum** (~5 minutes)
- Upper limit for practical compilation
- Requires significant memory (~3GB)
- Not suitable for rapid iteration

### Not Recommended
- **7×7 and larger**: Compilation times >20 minutes
- Memory usage >10GB
- May fail on systems with limited RAM
- Not practical for development workflow

## Optimization Notes

The framework includes several compilation optimizations:
- Template depth limit: 2048
- Precompiled headers (PCH)
- ccache support for recompilation
- Explicit template instantiation libraries

Even with these optimizations, the combinatorial explosion of template instantiations for large grids makes compilation times impractical beyond 6×6.

## Alternative Approaches for Larger Simulations

For simulations requiring more than 36 masses (6×6):

1. **Runtime connectivity**: Generate connectivity at runtime instead of compile-time
   - Trade compile-time checks for runtime flexibility
   - Significantly faster compilation
   - Small runtime overhead

2. **Subdomain composition**: Break large grids into multiple smaller systems
   - Compile multiple 5×5 or 6×6 subsystems
   - Couple them at boundaries
   - Parallel compilation possible

3. **Hybrid approach**: Use compile-time for critical components, runtime for connectivity
   - Keep type safety for physics components
   - Runtime grid generation for large systems

## Compiler Details

Tests performed with:
- Compiler: g++ (GCC) 13.x
- Flags: `-std=c++20 -O3 -ftemplate-depth=2048`
- Platform: Linux x86_64
- Available Memory: 16GB+

## Conclusion

**Realistic upper limit: 6×6 grid (36 masses, 110 springs)**

This provides:
- Compile time: ~5 minutes (acceptable for production builds)
- Memory usage: ~3GB (within reasonable limits)
- State dimension: 144 (4 states per mass: x, y, vx, vy)
- Sufficient complexity for meaningful cloth/membrane simulations

For rapid development and testing, stick to 4×4 or 5×5 grids.
