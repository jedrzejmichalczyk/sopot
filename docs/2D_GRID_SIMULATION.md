# 2D Mass-Spring Grid Simulation

## Overview

SOPOT now supports 2D mass-spring grid simulations, demonstrating the framework's domain-agnostic architecture. This extends the 1D mass-spring capabilities to 2D, enabling cloth-like and membrane simulations.

## Key Features

- **2D Point Masses**: Each mass has 4 states (x, y, vx, vy)
- **2D Springs**: Connect masses in 2D space with Hooke's law + damping
- **Flexible Topology**: Rectangular grids with optional diagonal springs
- **Compile-Time Type Safety**: All connectivity validated at compile time
- **Zero Runtime Overhead**: Same performance guarantees as other SOPOT components

## Components

### IndexedPointMass2D

```cpp
template<size_t Index, Scalar T = double>
class IndexedPointMass2D : public TypedComponent<4, T>
```

State vector: `[x, y, vx, vy]`

Provides:
- `MassTag2D<Index>::Position` → `std::array<T, 2>` (x, y)
- `MassTag2D<Index>::Velocity` → `std::array<T, 2>` (vx, vy)
- `MassTag2D<Index>::Mass` → `T`
- `MassTag2D<Index>::Force` → `std::array<T, 2>` (Fx, Fy) [via ForceAggregator2D]

### IndexedSpring2D

```cpp
template<size_t I, size_t J, Scalar T = double>
class IndexedSpring2D : public TypedComponent<0, T>
```

Computes spring force between masses I and J:
- **Elastic Force**: `F = -k * (current_length - rest_length) * direction`
- **Damping Force**: `F_damping = -c * relative_velocity * direction`

Provides:
- `SpringTag2D<I,J>::Extension` → `T`
- `SpringTag2D<I,J>::Length` → `T`
- `SpringTag2D<I,J>::Force` → `std::array<T, 2>` (force on mass I)
- `SpringTag2D<I,J>::PotentialEnergy` → `T`

### ForceAggregator2D

```cpp
template<size_t Index, auto Edges, Scalar T = double>
class ForceAggregator2D : public TypedComponent<0, T>
```

Aggregates all spring forces acting on mass `Index`:
- Iterates through edges at compile time
- Sums forces from all connected springs
- Applies Newton's 3rd law automatically

## Usage Examples

### Simple 3x3 Grid

```cpp
#include "physics/connected_masses/connectivity_matrix_2d.hpp"

using namespace sopot::connected_masses;

// Create a 3x3 grid
auto system = makeGrid2DSystem<double, 3, 3, false>(
    1.0,    // mass (kg)
    1.0,    // spacing (m)
    10.0,   // stiffness (N/m)
    0.5     // damping (N·s/m)
);

// Grid layout:
// 0 -- 1 -- 2
// |    |    |
// 3 -- 4 -- 5
// |    |    |
// 6 -- 7 -- 8

auto state = system.getInitialState();

// Query mass positions
auto pos_center = system.computeStateFunction<MassTag2D<4>::Position>(state);
std::cout << "Center mass: (" << pos_center[0] << ", " << pos_center[1] << ")\n";

// Simulate
double t = 0.0, dt = 0.001;
while (t < 2.0) {
    auto derivs = system.computeDerivatives(t, state);
    // ... RK4 integration ...
    t += dt;
}
```

### Cloth Simulation with Pinned Corners

```cpp
// Create 4x4 grid with diagonal springs for stability
auto system = makeGrid2DSystem<double, 4, 4, true>(
    0.1,    // lighter mass
    0.5,    // spacing
    50.0,   // stiffer springs
    1.0     // damping
);

auto state = system.getInitialState();

// Apply gravity (initial downward velocity)
for (size_t i = 0; i < 16; ++i) {
    state[i * 4 + 3] = -2.0;  // vy = -2 m/s
}

// Simulate and pin top row
while (t < t_end) {
    // ... integration ...

    // Pin top corners (override integration)
    for (size_t c = 0; c < 4; ++c) {
        size_t idx = c;  // Top row indices
        state[idx * 4 + 0] = c * 0.5;  // fixed x
        state[idx * 4 + 1] = 0.0;      // fixed y
        state[idx * 4 + 2] = 0.0;      // zero vx
        state[idx * 4 + 3] = 0.0;      // zero vy
    }
}
```

### Custom Topology

```cpp
// Define custom connectivity
constexpr auto edges = std::array{
    std::pair{size_t(0), size_t(1)},
    std::pair{size_t(1), size_t(2)},
    std::pair{size_t(0), size_t(2)}  // Triangle
};

// Set up mass parameters
std::array<MassParams2D, 3> masses = {{
    {1.0, 0.0, 0.0, 0.0, 0.0},   // mass, x, y, vx, vy
    {1.0, 1.0, 0.0, 0.0, 0.0},
    {1.0, 0.5, 1.0, 0.0, 0.0}
}};

// Set up spring parameters
std::array<SpringParams2D, 3> springs = {{
    {10.0, 1.0, 0.5},  // k, L0, c
    {10.0, 1.0, 0.5},
    {10.0, 1.0, 0.5}
}};

auto system = makeConnectedMassSystem2D<double, 3, edges>(masses, springs);
```

## Grid Connectivity Helper

The `grid_2d.hpp` header provides utilities for generating rectangular grid connectivity:

```cpp
// Runtime version
auto edges = makeGrid2DEdges(5, 5, false);  // 5x5 grid, no diagonals

// Compile-time version
constexpr auto edges = makeGrid2DEdgesArray<5, 5, true>();  // With diagonals

// Convert between index and coordinates
size_t idx = gridIndex(row, col, num_cols);
auto [row, col] = gridCoords(idx, num_cols);
```

Grid connectivity patterns:
- **Orthogonal only** (`include_diagonals=false`): 4-neighbor connectivity
  - Horizontal edges: `Rows * (Cols - 1)`
  - Vertical edges: `(Rows - 1) * Cols`
  - Total: `2*Rows*Cols - Rows - Cols`

- **With diagonals** (`include_diagonals=true`): 8-neighbor connectivity
  - Adds: `2 * (Rows - 1) * (Cols - 1)` diagonal springs

## Architecture Highlights

### Type-Safe Indexing

Each mass and spring has a unique compile-time type:
```cpp
// These are different types!
MassTag2D<0>::Position  // Type for mass 0 position
MassTag2D<1>::Position  // Type for mass 1 position
SpringTag2D<0,1>::Force // Type for spring 0-1 force
```

The compiler prevents:
- Querying non-existent masses
- Mixing up mass indices
- Invalid spring connections (e.g., mass to itself)

### Force Aggregation

Forces are collected automatically using compile-time iteration:

```cpp
template<size_t Index, auto Edges, Scalar T>
class ForceAggregator2D {
    template<typename Registry>
    std::array<T, 2> compute(
        typename MassTag2D<Index>::Force,
        std::span<const T> state,
        const Registry& registry
    ) const {
        std::array<T, 2> total_force = {T(0), T(0)};

        // Compile-time iteration over edges
        for each edge (I, J) in Edges:
            if I == Index:
                total_force += registry.computeFunction<SpringTag2D<I,J>::Force>()
            else if J == Index:
                total_force -= registry.computeFunction<SpringTag2D<I,J>::Force>()

        return total_force;
    }
};
```

## Performance

- **Compile-time dispatch**: Zero virtual function overhead
- **Inlining**: Spring force calculations inline completely
- **Cache-friendly**: State is a flat vector
- **Autodiff-ready**: Works with `Dual<T, N>` for Jacobian computation

Benchmark (Release build, -O3):
- 3x3 grid (9 masses, 12 springs): ~2000 RK4 steps in 8ms
- 5x5 grid (25 masses, 40 springs): ~3000 RK4 steps in 25ms

## Testing

Run the comprehensive test suite:

```bash
./grid_2d_test
```

Tests include:
1. **3x3 Grid**: Basic grid functionality with perturbation
2. **4x4 Cloth**: Hanging cloth with pinned top row
3. **Energy Conservation**: Verification (2x2 grid, no damping)
4. **CSV Export**: Animation data for 5x5 grid wave propagation

## Future Enhancements

Potential extensions:
- 3D mass-spring grids (6 states per mass: x, y, z, vx, vy, vz)
- Collision detection and response
- External forces (gravity, wind, user interaction)
- Variable mass/stiffness properties
- Mesh import from standard formats

## Comparison: Rocket vs Mass-Spring Grid

| Feature | Rocket Simulation | 2D Grid Simulation |
|---------|-------------------|-------------------|
| **Domain** | Aerospace | Structural/Cloth |
| **State per entity** | 13 (position, velocity, quaternion, ω) | 4 (x, y, vx, vy) |
| **Components** | 9+ (kinematics, dynamics, aero, propulsion) | 3 (mass, spring, aggregator) |
| **Topology** | Single rigid body | N connected masses |
| **Forces** | Gravity, thrust, drag, lift | Spring forces (elastic + damping) |
| **Coordinate system** | ENU + body frame | 2D Cartesian |

Both use **the same SOPOT framework** with identical TypedComponent/TypedODESystem architecture!

## Conclusion

The 2D grid implementation proves SOPOT's domain-agnostic design. The same framework that simulates rockets can simulate:
- ✅ 1D oscillators
- ✅ 2D cloth/membranes
- ✅ Complex coupled systems
- 🚧 3D structures (future)
- 🚧 Fluid dynamics (future)
- 🚧 Chemical reactions (future)

**SOPOT = Obviously Physics, Obviously Templates** 🎯
