<p align="center">
  <h1 align="center">SOPOT</h1>
  <p align="center"><strong>Obviously Physics, Obviously Templates</strong></p>
  <p align="center">A modern C++20 compile-time physics simulation framework</p>
</p>

<p align="center">
  <a href="https://github.com/jedrzejmichalczyk/sopot/actions/workflows/ci.yml"><img src="https://github.com/jedrzejmichalczyk/sopot/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
  <img src="https://img.shields.io/badge/C%2B%2B-20-blue.svg" alt="C++20">
  <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License: MIT">
  <img src="https://img.shields.io/badge/Header--Only-Yes-brightgreen.svg" alt="Header Only">
  <img src="https://img.shields.io/badge/Dependencies-None-orange.svg" alt="No Dependencies">
  <img src="https://img.shields.io/badge/WebAssembly-Ready-blueviolet.svg" alt="WebAssembly">
</p>

<p align="center">
  <strong>🚀 <a href="https://jedrzejmichalczyk.github.io/sopot/">Try the Interactive 3D Demo!</a> 🚀</strong>
</p>

<p align="center">
  <em>C++20 physics simulation compiled to WebAssembly, running in your browser with real-time 3D visualization</em>
</p>

---

## Highlights

- **🚀 WebAssembly Ready** - Run C++20 physics in the browser with [interactive 3D demo](https://jedrzejmichalczyk.github.io/sopot/)
- **Zero Runtime Overhead** - All state function dispatch resolved at compile time
- **Automatic Differentiation** - Forward-mode autodiff for Jacobian computation
- **Symbolic CAS** - Compile-time computer algebra for automatic constraint Jacobians
- **Type-Safe Units** - Compile-time dimensional analysis prevents unit errors
- **Modular Components** - Compose ODE systems from reusable building blocks
- **Header-Only** - Just include and use, no linking required

## What's New 🎉

**Interactive 3D Web Demo** - Experience SOPOT in your browser with:
- Real-time 6-DOF rocket flight simulation
- Stunning 3D visualization with React Three Fiber
- Live telemetry and trajectory tracking
- Interactive camera controls (rotate, pan, zoom)
- Zero installation required

[**🎮 Try it now →**](https://jedrzejmichalczyk.github.io/sopot/)

## Table of Contents

- [Quick Start](#quick-start)
  - [C++ Native](#c-native)
  - [WebAssembly Browser Demo](#webassembly-browser-demo)
- [Features](#features)
  - [Compile-Time Dispatch](#compile-time-dispatch)
  - [Automatic Differentiation](#automatic-differentiation)
  - [Symbolic CAS](#symbolic-cas)
  - [Type-Safe Units](#type-safe-units)
  - [WebAssembly Integration](#webassembly-integration)
- [Architecture](#architecture)
- [Examples](#examples)
- [Performance](#performance)
- [Requirements](#requirements)
- [License](#license)

## Quick Start

### C++ Native

```bash
git clone https://github.com/jedrzejmichalczyk/sopot.git
cd sopot
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
./compile_time_test
```

### WebAssembly Browser Demo

**Option 1: Use the Live Demo** (Easiest)

Visit [https://jedrzejmichalczyk.github.io/sopot/](https://jedrzejmichalczyk.github.io/sopot/) and start simulating!

**Option 2: Build Locally**

```bash
# Install Emscripten
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk && ./emsdk install latest && ./emsdk activate latest
source ./emsdk_env.sh

# Build WebAssembly module
cd sopot/wasm
./build.sh

# Set up web application
cd ../web
npm install
cp ../wasm/sopot.{js,wasm} public/
npm run dev  # Opens at http://localhost:3000
```

See [`web/README.md`](web/README.md) for detailed instructions.

**Troubleshooting WASM Build:**

If you encounter errors like `$.getCenterOfMass is not a function` or other missing function errors in the web interface, you need to build the WASM module. See [`WASM_BUILD_GUIDE.md`](WASM_BUILD_GUIDE.md) for comprehensive troubleshooting, or use the quick helper script:

```bash
./build-wasm.sh  # Auto-detects Emscripten or Docker
```

### Minimal C++ Example

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

### Symbolic CAS

Compile-time computer algebra system for automatic constraint Jacobian derivation:

```cpp
#include "physics/constraints/symbolic/named_expression.hpp"

using namespace sopot::symbolic;
using namespace sopot::symbolic::cartesian::two_body_2d;

// Define constraints using named symbols - reads like math!
auto g1 = sq(x1) + sq(y1);              // Rod 1: x₁² + y₁² = L₁²
auto g2 = sq(x2 - x1) + sq(y2 - y1);    // Rod 2: (x₂-x₁)² + (y₂-y₁)² = L₂²

// Jacobian computed at COMPILE TIME via template metaprogramming
using J = Jacobian<4, decltype(g1)::type, decltype(g2)::type>;

std::array<double, 4> pos = {0.6, -0.8, 1.4, -1.4};
auto jacobian = J::eval(pos);  // 2x4 Jacobian, zero runtime differentiation overhead
```

**Supported operations:**
- Arithmetic: `+`, `-`, `*`, `/`, `sq()`, `pow<N>()`
- Trigonometric: `sin()`, `cos()`, `sqrt()`
- Automatic differentiation rules: product, quotient, chain rule
- Gradient, Jacobian, and Hessian computation

**Use cases:**
- Constrained dynamics (pendulums, linkages)
- Holonomic constraints with Baumgarte stabilization
- Any algebraic constraint g(q) = 0

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

### WebAssembly Integration

SOPOT compiles seamlessly to WebAssembly with **zero modifications** to the core C++ code:

**Interactive Web Demo:**
- C++20 physics simulation runs in browser at **near-native speed**
- Real-time 3D visualization with React Three Fiber
- ~800 KB WebAssembly module (including full 6-DOF rocket simulation)
- Achieves **10x real-time** simulation speed in browser

**JavaScript/TypeScript Integration:**

```typescript
import createSopotModule from './sopot.js';

// Initialize WebAssembly module
const Module = await createSopotModule();
const sim = new Module.RocketSimulator();

// Configure simulation
sim.setLauncher(85, 0);    // 85° elevation, 0° azimuth
sim.setDiameter(0.16);     // 16cm diameter
sim.setTimestep(0.01);     // 10ms timestep
sim.setup();

// Run simulation loop
requestAnimationFrame(function animate() {
  if (sim.step()) {
    const state = sim.getFullState();
    updateVisualization(state.position, state.quaternion);
    requestAnimationFrame(animate);
  }
});
```

**What Works in WebAssembly:**
- ✅ Full C++20 concepts and constexpr
- ✅ Automatic differentiation (Dual numbers)
- ✅ Template metaprogramming
- ✅ All math functions (sin, cos, exp, etc.)
- ✅ Zero external dependencies
- ✅ 70-80% of native C++ performance

**Web Application Features:**
- Interactive 3D rocket visualization
- Real-time telemetry display (altitude, speed, mass, attitude)
- Trajectory tracking and replay
- Configurable launch parameters
- Playback speed control (0.1x to 10x)
- Orbit camera controls

See [`wasm/README.md`](wasm/README.md) and [`web/README.md`](web/README.md) for details.

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
├── physics/                      # Physics components
│   ├── coupled_oscillator/       # Mass-spring systems
│   ├── connected_masses/         # 1D/2D mass-spring networks
│   ├── pendulum/                 # Double pendulum (Lagrangian & Cartesian)
│   │   ├── double_pendulum.hpp   # Generalized coordinates
│   │   ├── cartesian_pendulum.hpp # Cartesian with Baumgarte
│   │   └── named_constraint_pendulum.hpp # Using named CAS
│   └── constraints/symbolic/     # Compile-time CAS ⭐ NEW
│       ├── expression.hpp        # Expression templates
│       ├── differentiation.hpp   # Symbolic differentiation
│       └── named_expression.hpp  # Named variables API
│
├── rocket/                       # 6DOF rocket simulation
│   ├── vector3.hpp               # 3D vector (autodiff-compatible)
│   ├── quaternion.hpp            # Attitude representation
│   ├── translation_*.hpp         # Position/velocity ODEs
│   ├── rotation_*.hpp            # Attitude/angular velocity ODEs
│   ├── standard_atmosphere.hpp   # US Standard Atmosphere 1976
│   ├── rocket.hpp                # Complete rocket system
│   └── ...
│
├── wasm/                         # WebAssembly bindings ⭐ NEW
│   ├── wasm_rocket.cpp           # Embind wrapper
│   ├── build.sh                  # Emscripten build script
│   ├── CMakeLists.txt            # CMake configuration
│   └── README.md                 # WebAssembly documentation
│
├── web/                          # React web application ⭐ NEW
│   ├── src/
│   │   ├── components/           # React components
│   │   │   ├── RocketVisualization3D.tsx  # Three.js 3D scene
│   │   │   ├── TelemetryPanel.tsx         # Live data display
│   │   │   └── ControlPanel.tsx           # Simulation controls
│   │   ├── hooks/
│   │   │   └── useRocketSimulation.ts     # WebAssembly integration
│   │   └── types/
│   │       └── sopot.d.ts        # TypeScript definitions
│   ├── package.json              # Dependencies
│   └── README.md                 # Web app documentation
│
├── .github/workflows/            # GitHub Actions CI/CD ⭐ NEW
│   └── deploy-github-pages.yml  # Automatic deployment
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

### Double Pendulum with Constraints

```cpp
#include "physics/pendulum/double_pendulum.hpp"

using namespace sopot::pendulum;

// Lagrangian formulation - 4 states: θ₁, θ₂, ω₁, ω₂
DoublePendulum<double> pendulum(
    /*m1=*/1.0, /*m2=*/1.0,
    /*L1=*/1.0, /*L2=*/1.0,
    /*g=*/9.81,
    /*θ1_0=*/M_PI/4, /*θ2_0=*/M_PI/6  // Initial angles
);

auto system = makeTypedODESystem<double>(pendulum);
auto solver = createRK4Solver();
auto trajectory = solver.solve(system, 0.0, 20.0, 0.001);

// Query state functions
double energy = system.computeStateFunction<system::TotalEnergy>(state);
auto pos1 = system.computeStateFunction<mass1::CartesianPosition>(state);
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
