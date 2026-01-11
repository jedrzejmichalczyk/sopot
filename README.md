# SOPOT - Obviously Physics, Obviously Templates

A modern C++20 physics simulation framework with **zero runtime overhead**, **automatic differentiation**, and **compile-time dimensional analysis**.

## Key Features

| Feature | Description |
|---------|-------------|
| **Compile-Time Dispatch** | All state function calls resolved at compile time - no virtual functions |
| **Automatic Differentiation** | Forward-mode autodiff for Jacobian computation (LQR control design) |
| **Dimensional Analysis** | Type-safe units prevent dimension errors at compile time |
| **Component Composition** | Modular ODE components with automatic state management |
| **Zero Overhead** | Template metaprogramming for maximum performance |

## Architecture

```
sopot/
├── core/
│   ├── dual.hpp                  # Forward-mode autodiff (Dual<T, N>)
│   ├── units.hpp                 # Compile-time dimensional analysis
│   ├── scalar.hpp                # Scalar concepts and utilities
│   ├── state_function_tags.hpp   # Hierarchical function tag system
│   ├── typed_component.hpp       # Templated components for autodiff
│   ├── linearization.hpp         # System linearization (A,B matrices)
│   └── solver.hpp                # Optimized RK4 solver
├── physics/
│   └── harmonic_oscillator.hpp   # Example physics components
├── rocket/
│   ├── vector3.hpp               # 3D vector with autodiff support
│   ├── quaternion.hpp            # Quaternion for attitude
│   └── ...                       # 6DOF rocket components
└── tests/
    ├── compile_time_test.cpp     # Component system tests
    ├── autodiff_test.cpp         # Autodiff and units tests
    └── rocket_flight_test.cpp    # Rocket simulation tests
```

## Quick Start

### Building

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
./autodiff_test
./compile_time_test
./rocket_flight_test
```

### Basic Usage

```cpp
#include "core/typed_component.hpp"
#include "physics/harmonic_oscillator.hpp"

using namespace sopot;

// Create components and compose into system
auto oscillator = physics::createDampedOscillator(2.0, 9.0, 0.1, 1.5);
auto system = makeTypedODESystem<double>(std::move(oscillator));

// Compile-time function availability checks
static_assert(decltype(system)::hasFunction<kinematics::Position>());
static_assert(decltype(system)::hasFunction<energy::Total>());

// Zero-overhead state function calls
auto state = system.getInitialState();
double energy = system.computeStateFunction<energy::Total>(state);

// Integration
auto solver = createRK4Solver();
auto result = solver.solve(system, 0.0, 10.0, 0.01);
```

## Automatic Differentiation

The `Dual<T, N>` type enables forward-mode autodiff for computing Jacobians:

```cpp
#include "core/dual.hpp"
#include "core/linearization.hpp"

using namespace sopot;

// Dual number with 2 partial derivatives
using Dual2 = Dual<double, 2>;

// Create variable (derivative = 1 for this variable)
Dual2 x = Dual2::variable(3.0, 0);  // d/dx = 1
Dual2 y = Dual2::constant(2.0);     // d/dy = 0

// Compute f = x^2 + sin(x*y)
auto f = x * x + sin(x * y);

// Access value and derivative
double value = f.value();           // f(3, 2)
double df_dx = f.derivative(0);     // df/dx at (3, 2)
```

### System Linearization for LQR

```cpp
// Define rocket pitch dynamics: dx/dt = f(x, u)
auto dynamics = [](DualT t, const DualState& x, const DualInput& u) {
    return DualDerivative{
        x[1],                              // dtheta/dt = omega
        DualT(-k) * x[0] + DualT(b) * u[0] // domega/dt = -k*theta + b*delta
    };
};

// Create linearizer and compute A, B matrices
auto linearizer = makeLinearizer<2, 1>(dynamics);
auto result = linearizer.linearize(t, x0, u0);

// result.A = df/dx (state Jacobian)
// result.B = df/du (input Jacobian)
// Ready for LQR: K = lqr(A, B, Q, R)
```

## Compile-Time Units

Type-safe physical quantities prevent dimension errors:

```cpp
#include "core/units.hpp"
using namespace sopot::units;
using namespace sopot::units::literals;

// Create quantities with units
auto distance = 100.0_m;
auto time = 10.0_s;
auto velocity = distance / time;  // Type: MetersPerSecond<>

// Compile-time dimension checks
auto mass = 50.0_kg;
auto accel = 9.81_m / (1.0_s * 1.0_s);  // MetersPerSecond2
auto force = mass * accel;               // Type: Newtons<>

// Angle conversions
auto angle = 45.0_deg;  // Converts to radians internally
auto sine = units::sin(angle);  // Returns dimensionless

// This would fail at compile time:
// auto invalid = distance + mass;  // Error: incompatible dimensions
```

## Component System

Components provide state functions and ODE derivatives:

```cpp
template<Scalar T>
class MyComponent : public TypedComponent<2, T> {
public:
    // ODE: compute dx/dt
    LocalDerivative computeLocalDerivatives(
        T t, const LocalState& x, const std::vector<T>& global
    ) const override {
        return {x[1], -k * x[0]};  // Harmonic oscillator
    }

    LocalState getInitialLocalState() const override {
        return {T(1.0), T(0.0)};  // x=1, v=0
    }

    // Provide state functions (resolved at compile time)
    T compute(kinematics::Position, const std::vector<T>& state) const {
        return this->getGlobalState(state, 0);
    }

    T compute(energy::Kinetic, const std::vector<T>& state) const {
        T v = this->getGlobalState(state, 1);
        return T(0.5 * m) * v * v;
    }
};
```

## State Function Tags

Hierarchical organization for extensibility:

```cpp
namespace sopot {
    // Base categories
    namespace categories {
        struct Kinematics : StateFunction { ... };
        struct Dynamics : StateFunction { ... };
        struct Energy : StateFunction { ... };
    }

    // Specific functions
    namespace kinematics {
        struct Position : categories::Kinematics { ... };
        struct Velocity : categories::Kinematics { ... };
        struct Acceleration : categories::Kinematics { ... };
    }

    namespace energy {
        struct Kinetic : categories::Energy { ... };
        struct Potential : categories::Energy { ... };
        struct Total : categories::Energy { ... };
    }

    // Add your own domain-specific tags
    namespace rocket {
        struct CenterOfGravity : categories::Dynamics { ... };
        struct ThrustForce : categories::Dynamics { ... };
    }
}
```

## Performance

| Metric | Value |
|--------|-------|
| Function call overhead | ~23 ns (compile-time dispatch) |
| Analytical accuracy | < 1e-10 error vs exact solution |
| Integration speed | 0.05 ms for 200 RK4 steps |
| Jacobian computation | Automatic via autodiff |
| Memory overhead | Zero for function registry |

## Requirements

- C++20 compiler (GCC 10+, Clang 12+, MSVC 19.29+)
- CMake 3.20+

## Design Principles

1. **Zero Runtime Overhead** - All dispatch resolved at compile time
2. **Type Safety** - C++20 concepts and compile-time units
3. **Automatic Differentiation** - Jacobians computed automatically
4. **Component Autonomy** - Each component declares its capabilities
5. **Scalability** - Hierarchical tags support hundreds of functions
6. **Modularity** - Easy to add new physics domains

## License

MIT License - See LICENSE file for details.

---

**SOPOT** achieves the perfect balance: expressive, type-safe simulation with the performance of hand-written code and automatic differentiation for control system design.
