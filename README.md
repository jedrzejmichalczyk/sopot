<p align="center">
  <h1 align="center">SOPOT</h1>
  <p align="center"><strong>Obviously Physics, Obviously Templates</strong></p>
  <p align="center">A modern C++20 compile-time physics simulation framework</p>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/C%2B%2B-20-blue.svg" alt="C++20">
  <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License: MIT">
  <img src="https://img.shields.io/badge/Header--Only-Yes-brightgreen.svg" alt="Header Only">
  <img src="https://img.shields.io/badge/Dependencies-None-orange.svg" alt="No Dependencies">
</p>

---

## Highlights

- **Zero Runtime Overhead** - All state function dispatch resolved at compile time
- **Automatic Differentiation** - Forward-mode autodiff for Jacobian computation
- **Type-Safe Units** - Compile-time dimensional analysis prevents unit errors
- **Modular Components** - Compose ODE systems from reusable building blocks
- **Header-Only** - Just include and use, no linking required

## Table of Contents

- [Quick Start](#quick-start)
- [Features](#features)
  - [Compile-Time Dispatch](#compile-time-dispatch)
  - [Automatic Differentiation](#automatic-differentiation)
  - [Type-Safe Units](#type-safe-units)
- [Architecture](#architecture)
- [Examples](#examples)
- [Performance](#performance)
- [Requirements](#requirements)
- [License](#license)

## Quick Start

```bash
git clone https://github.com/yourusername/sopot.git
cd sopot
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
./compile_time_test
```

### Minimal Example

```cpp
#include "core/typed_component.hpp"
#include "physics/harmonic_oscillator.hpp"

using namespace sopot;

int main() {
    // Create a damped harmonic oscillator
    auto osc = physics::createDampedOscillator<double>(
        /*mass=*/2.0, /*k=*/9.0, /*damping=*/0.1, /*x0=*/1.5
    );
    auto system = makeTypedODESystem<double>(std::move(osc));

    // Query state functions - resolved at compile time!
    auto state = system.getInitialState();
    double energy = system.computeStateFunction<energy::Total>(state);

    // Integrate with RK4
    auto solver = createRK4Solver();
    auto result = solver.solve(system, 0.0, 10.0, 0.01);
}
```

## Features

### Compile-Time Dispatch

State functions are resolved entirely at compile time using C++20 concepts:

```cpp
// Compile-time check - fails to compile if function not available
static_assert(decltype(system)::hasFunction<kinematics::Position>());
static_assert(decltype(system)::hasFunction<energy::Total>());

// Zero-overhead call - no virtual functions, fully inlined
double pos = system.computeStateFunction<kinematics::Position>(state);
```

### Automatic Differentiation

Forward-mode autodiff for computing Jacobians - essential for control system design:

```cpp
#include "core/dual.hpp"

using Dual2 = Dual<double, 2>;

Dual2 x = Dual2::variable(3.0, 0);  // dx/dx = 1
Dual2 y = Dual2::variable(2.0, 1);  // dy/dy = 1

auto f = x * x + sin(x * y);

double value = f.value();        // f(3, 2)
double df_dx = f.derivative(0);  // df/dx
double df_dy = f.derivative(1);  // df/dy
```

**System Linearization for LQR:**

```cpp
#include "core/linearization.hpp"

auto linearizer = makeLinearizer<2, 1>(dynamics);
auto [A, B] = linearizer.linearize(t, x0, u0);
// A = df/dx, B = df/du - ready for LQR design
```

### Type-Safe Units

Compile-time dimensional analysis catches unit errors before runtime:

```cpp
#include "core/units.hpp"
using namespace sopot::units::literals;

auto distance = 100.0_m;
auto time = 10.0_s;
auto velocity = distance / time;  // Automatically MetersPerSecond

auto mass = 50.0_kg;
auto force = mass * 9.81_m / (1.0_s * 1.0_s);  // Newtons

// auto invalid = distance + mass;  // COMPILE ERROR: incompatible dimensions
```

## Architecture

```
sopot/
├── core/                         # Framework foundation
│   ├── dual.hpp                  # Forward-mode autodiff
│   ├── units.hpp                 # Compile-time units
│   ├── typed_component.hpp       # Component base class
│   ├── linearization.hpp         # System linearization
│   └── solver.hpp                # RK4 integrator
│
├── physics/                      # Example components
│   └── harmonic_oscillator.hpp
│
├── rocket/                       # 6DOF rocket simulation
│   ├── vector3.hpp               # 3D vector (autodiff-compatible)
│   ├── quaternion.hpp            # Attitude representation
│   ├── translation_*.hpp         # Position/velocity ODEs
│   ├── rotation_*.hpp            # Attitude/angular velocity ODEs
│   ├── standard_atmosphere.hpp   # US Standard Atmosphere 1976
│   └── ...
│
└── tests/
    ├── compile_time_test.cpp
    ├── autodiff_test.cpp
    └── rocket_flight_test.cpp
```

## Examples

### Custom Component

```cpp
template<Scalar T>
class SpringMass : public TypedComponent<2, T> {
    T m_k;  // Spring constant

public:
    SpringMass(T k) : m_k(k) {}

    // ODE: dx/dt
    LocalDerivative computeLocalDerivatives(
        T t, const LocalState& x, const std::vector<T>& global
    ) const override {
        return {x[1], -m_k * x[0]};  // x' = v, v' = -k*x
    }

    // State functions (compile-time dispatch)
    T compute(kinematics::Position, const std::vector<T>& s) const {
        return this->getGlobalState(s, 0);
    }

    T compute(energy::Potential, const std::vector<T>& s) const {
        T x = this->getGlobalState(s, 0);
        return T(0.5) * m_k * x * x;
    }
};
```

### 6DOF Rocket Simulation

```cpp
#include "rocket/rocket.hpp"

using namespace sopot::rocket;

// Components compose automatically
auto system = makeTypedODESystem<double>(
    TranslationKinematics<double>(),
    TranslationDynamics<double>(),
    RotationKinematics<double>(),
    RotationDynamics<double>(),
    Gravity<double>(),
    StandardAtmosphere<double>()
);

// Simulate trajectory
auto solver = createRK4Solver();
auto trajectory = solver.solve(system, 0.0, 100.0, 0.01);
```

## Performance

| Metric | Value |
|--------|-------|
| State function call | ~1 ns (fully inlined) |
| RK4 step (13-state rocket) | ~0.3 us |
| Analytical solution error | < 1e-10 |
| Jacobian computation | Automatic (no finite differences) |

Benchmarked on Intel i7, GCC 13 with `-O3`.

## Requirements

- **C++20** compiler:
  - GCC 10+
  - Clang 12+
  - MSVC 19.29+
- **CMake** 3.20+
- **No external dependencies**

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<p align="center">
  <sub>Built with C++20 template metaprogramming for maximum performance and type safety.</sub>
</p>
