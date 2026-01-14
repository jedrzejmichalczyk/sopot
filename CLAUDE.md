# CLAUDE.md - SOPOT Framework Guide

## Project Overview

SOPOT (Obviously Physics, Obviously Templates) is a C++20 compile-time physics simulation framework with zero runtime overhead, automatic differentiation, and type-safe component composition. The framework is domain-agnostic and currently supports:
- **Rocket flight simulation** (6-DOF with aerodynamics, propulsion, atmosphere)
- **1D mass-spring systems** (coupled oscillators, chains)
- **2D mass-spring grids** (cloth-like simulations, membranes)

## Build Commands

```bash
cd /mnt/c/Users/j.michalczyk/Projekty/sopot
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

# Run tests
./compile_time_test
./autodiff_test
./rocket_flight_test
```

## Architecture Summary

```
sopot/
├── core/                    # Framework core
│   ├── typed_component.hpp  # TypedComponent, TypedRegistry, TypedODESystem
│   ├── dual.hpp             # Forward-mode autodiff (Dual<T, N>)
│   ├── scalar.hpp           # Scalar concept, value_of()
│   ├── state_function_tags.hpp  # Base state function tags
│   └── solver.hpp           # RK4 integrator
├── physics/                 # Generic physics components
│   ├── coupled_oscillator/  # 1D mass-spring systems (2 masses)
│   └── connected_masses/    # Arbitrary 1D/2D mass-spring networks
│       ├── indexed_point_mass.hpp       # 1D point mass (2 states)
│       ├── indexed_point_mass_2d.hpp    # 2D point mass (4 states)
│       ├── indexed_spring.hpp           # 1D spring
│       ├── indexed_spring_2d.hpp        # 2D spring with damping
│       ├── force_aggregator_2d.hpp      # Force collection for 2D
│       ├── grid_2d.hpp                  # 2D grid connectivity helpers
│       ├── connectivity_matrix.hpp      # 1D system builder
│       └── connectivity_matrix_2d.hpp   # 2D system builder
├── rocket/                  # Rocket simulation components
│   ├── rocket_tags.hpp      # Rocket-specific state function tags
│   ├── vector3.hpp          # 3D vector with autodiff support
│   ├── quaternion.hpp       # Quaternion for attitude
│   ├── translation_kinematics.hpp   # Position ODE (3 states)
│   ├── translation_dynamics.hpp     # Velocity ODE (3 states)
│   ├── rotation_kinematics.hpp      # Quaternion ODE (4 states)
│   ├── rotation_dynamics.hpp        # Angular velocity ODE (3 states)
│   ├── gravity.hpp          # Gravity model
│   ├── standard_atmosphere.hpp      # US Standard Atmosphere 1976
│   ├── interpolated_engine.hpp      # Engine thrust from CSV
│   ├── axisymmetric_aero.hpp        # Aerodynamic forces
│   ├── rocket_body.hpp      # Time-varying mass properties
│   └── force_aggregator.hpp # Force/torque aggregation
├── io/                      # Input/output utilities
│   ├── csv_parser.hpp       # CSV file parser
│   └── interpolation.hpp    # Linear/bilinear interpolation
└── tests/                   # Test executables
```

## Key Concepts

### 1. TypedComponent - Base for all simulation components

```cpp
template<size_t StateSize, Scalar T = double>
class MyComponent : public TypedComponent<StateSize, T> {
public:
    using Base = TypedComponent<StateSize, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Base::computeLocalDerivatives;  // CRITICAL: Expose registry-aware method

    // Required: Initial state
    LocalState getInitialLocalState() const override { ... }

    // Required: Component identification
    std::string_view getComponentType() const override { return "MyComponent"; }
    std::string_view getComponentName() const override { return m_name; }

    // Optional: Compute derivatives (for stateful components)
    LocalDerivative computeLocalDerivatives(
        T t, const LocalState& local, const std::vector<T>& global
    ) const override { ... }

    // Optional: Registry-aware derivatives (for cross-component access)
    template<typename Registry>
    LocalDerivative computeLocalDerivatives(
        T t, const LocalState& local, const std::vector<T>& global,
        const Registry& registry
    ) const {
        // Can query other components:
        auto vel = registry.template computeFunction<kinematics::VelocityENU>(global);
        ...
    }

    // Optional: Provide state functions for other components
    T compute(SomeTag, const std::vector<T>& state) const { ... }
    Vector3<T> compute(VectorTag, const std::vector<T>& state) const { ... }
};
```

### 2. TypedODESystem - Compose components into a system

```cpp
// Create system with multiple components
auto system = makeTypedODESystem<double>(
    TranslationKinematics<double>(),
    TranslationDynamics<double>(),
    RotationKinematics<double>(),
    RotationDynamics<double>(),
    Gravity<double>(),
    StandardAtmosphere<double>()
);

// Compile-time checks
static_assert(decltype(system)::hasFunction<kinematics::VelocityENU>());

// Get initial state and integrate
auto state = system.getInitialState();
auto derivs = system.computeDerivatives(t, state);

// Query state functions
auto velocity = system.computeStateFunction<kinematics::VelocityENU>(state);
```

### 3. State Function Tags - Type-safe identifiers

Tags are defined in `rocket/rocket_tags.hpp`:

```cpp
namespace sopot::rocket {
    namespace kinematics {
        struct PositionENU : StateFunction { ... };
        struct VelocityENU : StateFunction { ... };
        struct Altitude : StateFunction { ... };
        struct AttitudeQuaternion : StateFunction { ... };
        struct AngularVelocity : StateFunction { ... };
    }
    namespace dynamics {
        struct Mass : StateFunction { ... };
        struct MomentOfInertia : StateFunction { ... };
        struct TotalForceENU : StateFunction { ... };
        struct TotalTorqueBody : StateFunction { ... };
        struct GravityAcceleration : StateFunction { ... };
    }
    namespace propulsion {
        struct ThrustForceBody : StateFunction { ... };
        struct MassFlowRate : StateFunction { ... };
    }
    namespace environment {
        struct AtmosphericDensity : StateFunction { ... };
        struct AtmosphericPressure : StateFunction { ... };
        struct SpeedOfSound : StateFunction { ... };
    }
}
```

### 4. Cross-Component Dependencies

Components query each other through the registry:

```cpp
template<typename Registry>
LocalDerivative computeLocalDerivatives(
    T t, const LocalState& local, const std::vector<T>& global,
    const Registry& registry
) const {
    // TranslationDynamics queries force and mass
    Vector3<T> force = registry.template computeFunction<dynamics::TotalForceENU>(global);
    T mass = registry.template computeFunction<dynamics::Mass>(global);

    Vector3<T> accel = force / mass;
    return {accel.x, accel.y, accel.z};
}
```

### 5. Autodiff Support

Template on scalar type `T` for autodiff compatibility:

```cpp
// Use with double for simulation
auto system_double = makeTypedODESystem<double>(components...);

// Use with Dual<double, 13> for Jacobian computation
using Dual13 = Dual<double, 13>;
auto system_dual = makeTypedODESystem<Dual13>(components...);
auto jacobian = computeJacobian(system_dual, t, state);
```

## Component Design Checklist

When creating a new component:

- [ ] Template on `Scalar T`
- [ ] Inherit from `TypedComponent<StateSize, T>`
- [ ] Add `using Base::computeLocalDerivatives;` (expose template method)
- [ ] Implement `getInitialLocalState()`, `getComponentType()`, `getComponentName()`
- [ ] Use `T` for all computed values (not `double`)
- [ ] Use `this->extractLocalState(state)` for state access
- [ ] Query other components through registry
- [ ] Add member `std::string m_name` for component naming

## Rocket Simulation Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                      TypedODESystem                              │
│                                                                  │
│  ┌──────────────┐     ┌──────────────┐     ┌────────────────┐  │
│  │ Atmosphere   │────►│ Aerodynamics │────►│                │  │
│  │  density     │     │  drag, lift  │     │                │  │
│  └──────────────┘     └──────────────┘     │ ForceAggregator│  │
│        ▲                    │              │  TotalForceENU │  │
│        │                    │              │  TotalTorque   │  │
│  ┌─────┴────────┐     ┌─────┴────────┐     │                │  │
│  │ Translation  │     │   Engine     │────►│                │  │
│  │ Kinematics   │     │   thrust     │     └───────┬────────┘  │
│  │  (altitude)  │     └──────────────┘             │           │
│  └──────────────┘                                  ▼           │
│        ▲                                   ┌────────────────┐  │
│        │              ┌──────────────┐     │ Translation    │  │
│        └──────────────│ RocketBody   │────►│ Dynamics       │  │
│                       │  mass, I     │     │  (dVel/dt)     │  │
│                       └──────────────┘     └────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Common Patterns

### Pattern 1: Stateless Provider Component

```cpp
template<Scalar T = double>
class StandardAtmosphere : public TypedComponent<0, T> {  // 0 states
    // No own state, just provides computed values
    T compute(environment::AtmosphericDensity, const std::vector<T>& state) const {
        T altitude = /* extract from state */;
        return computeDensity(altitude);
    }
};
```

### Pattern 2: Stateful Dynamics Component

```cpp
template<Scalar T = double>
class TranslationDynamics : public TypedComponent<3, T> {  // 3 states: vx, vy, vz
    template<typename Registry>
    LocalDerivative computeLocalDerivatives(
        T t, const LocalState& local, const std::vector<T>& global,
        const Registry& registry
    ) const {
        Vector3<T> force = registry.template computeFunction<dynamics::TotalForceENU>(global);
        T mass = registry.template computeFunction<dynamics::Mass>(global);
        Vector3<T> accel = force / mass;
        return {accel.x, accel.y, accel.z};
    }

    Vector3<T> compute(kinematics::VelocityENU, const std::vector<T>& state) const {
        auto local = this->extractLocalState(state);
        return {local[0], local[1], local[2]};
    }
};
```

### Pattern 3: Force Aggregator

```cpp
template<Scalar T = double>
class ForceAggregator : public TypedComponent<0, T> {
    template<typename Registry>
    Vector3<T> computeTotalForce(const std::vector<T>& state, const Registry& reg) const {
        Vector3<T> F_gravity = /* from Gravity component */;
        Vector3<T> F_thrust = reg.template computeFunction<propulsion::ThrustForceBody>(state);
        Vector3<T> F_aero = reg.template computeFunction<aero::AeroForceENU>(state);
        return F_gravity + F_thrust + F_aero;
    }

    Vector3<T> compute(dynamics::TotalForceENU, const std::vector<T>& state) const {
        // Simplified version without registry
        return /* default implementation */;
    }
};
```

## Performance Notes

- TypedODESystem uses compile-time dispatch: zero virtual function overhead
- 20,000 RK4 steps complete in ~8 ms
- Autodiff Jacobian computation is automatic when using Dual<T, N>
