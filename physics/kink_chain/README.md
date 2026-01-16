# Kink Chain Simulation

## Overview

This module implements a 1D kink chain simulation - a chain of rigid segments connected by torsional springs. This system exhibits interesting topological phenomena where "kinks" (regions of large angular discontinuity) can form, propagate, and annihilate.

## Physical System

### Components

1. **RotationalSegment** - A rigid stick with rotational degrees of freedom
   - State: `[θ, ω]` (angle, angular velocity)
   - Dynamics: `dθ/dt = ω`, `dω/dt = τ/I`
   - Moment of inertia `I` determines rotational inertia

2. **TorsionalSpring** - Elastic coupling between adjacent segments
   - Torque: `τ = -k·Δθ - c·Δω`
   - Wraps angles to `[-π, π]` for proper topology
   - Conserves angular momentum (action-reaction pairs)

### Physics Model

The system follows Euler's rotation equation for each segment:
```
I₁ (dω₁/dt) = τ₁
```

Where the torque comes from torsional springs:
```
τ₁ = -k(θ₁ - θ₂) - c(ω₁ - ω₂)
```

### Topological Defects

A **kink** is a localized region where adjacent segments have large angular differences (close to ±π). Key properties:

- **Winding number**: Topological invariant counting net rotations
- **Kink propagation**: Angular discontinuities travel along the chain
- **Kink annihilation**: Opposite-winding kinks cancel when they meet
- **Energy**: Kinks carry both kinetic and potential energy

## Usage Example

```cpp
#include "physics/kink_chain/kink_system.hpp"
#include "core/solver.hpp"

using namespace sopot::physics::kink;

// Create a 3-segment chain with a kink in the middle
auto system = makeTypedODESystem<double>(
    createSegment<0, double>(0.01, 0.0, 0.0),      // θ₀ = 0
    createSegment<1, double>(0.01, π, 0.0),        // θ₁ = π (kink!)
    createSegment<2, double>(0.01, 0.0, 0.0),      // θ₂ = 0
    createTorsionalSpring<0, 1, double>(10.0, 0.1),
    createTorsionalSpring<1, 2, double>(10.0, 0.1)
);

// Integrate forward in time
auto state = system.getInitialState();
for (int step = 0; step < 1000; ++step) {
    state = rk4Step(system, t, state, dt);
    t += dt;
}
```

## Demo Programs

### `kink_chain_test.cpp`

Two demonstration scenarios:

1. **Single Kink (3 segments)**
   - Middle segment rotated 180°
   - Watch kink propagate and dissipate
   - Output: `kink_chain_output.csv`

2. **Multiple Kinks (5 segments)**
   - Alternating pattern: 0, π, 0, π, 0
   - Observe kink interactions and annihilation
   - Output: `kink_multiple_output.csv`

Run with:
```bash
./build/kink_chain_test
```

## Physical Parameters

Typical values:
- Moment of inertia: `I = 0.01 kg·m²`
- Torsional stiffness: `k = 10 N·m/rad`
- Damping: `c = 0.05 - 0.1 N·m·s/rad`

## Compile-Time Features

Like all SOPOT components:
- Zero runtime overhead (compile-time dispatch)
- Automatic differentiation support
- Type-safe component composition
- Compile-time state function verification

## Applications

This simulation is analogous to:
- **Polymer chains** with rotational degrees of freedom
- **Magnetic domain walls** in 1D spin chains
- **Topological solitons** in field theory
- **Twist defects** in DNA or other helical structures

## See Also

- [Coupled Oscillator](../coupled_oscillator/) - Linear spring-mass systems
- [Connected Masses](../connected_masses/) - General mass-spring networks
- [SOPOT Framework Guide](../../CLAUDE.md) - Component architecture
