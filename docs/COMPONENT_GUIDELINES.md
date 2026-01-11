# Component Design Guidelines for Large Systems

This document provides guidelines for building complex simulation systems with many components while preserving the benefits of compile-time dispatch, autodiff compatibility, and type safety.

## Table of Contents
1. [Component Design Principles](#component-design-principles)
2. [State Function Organization](#state-function-organization)
3. [Cross-Component Dependencies](#cross-component-dependencies)
4. [Autodiff Compatibility](#autodiff-compatibility)
5. [Performance Considerations](#performance-considerations)
6. [Common Pitfalls](#common-pitfalls)
7. [Rocket Simulation Example](#rocket-simulation-example)

---

## Component Design Principles

### 1. Single Responsibility
Each component should own a coherent piece of physics:

```cpp
// GOOD: Focused components
RigidBodyComponent<T>      // Owns: position, velocity, attitude, angular velocity
AerodynamicsComponent<T>   // Owns: nothing (stateless), provides: forces, moments
ThrustComponent<T>         // Owns: fuel mass, provides: thrust force
AtmosphereComponent<T>     // Owns: nothing, provides: density, pressure, temperature

// BAD: Monolithic component
RocketComponent<T>         // Owns everything - hard to test, reuse, or extend
```

### 2. Explicit State Ownership
Declare state size at compile time. Components with no state should use `state_size = 0`:

```cpp
template<Scalar T>
class Aerodynamics : public TypedComponent<0, T> {  // No own state
    // Computes forces from state owned by other components
};

template<Scalar T>
class RigidBody : public TypedComponent<13, T> {   // 13 state variables
    // [0-2]: position (x, y, z)
    // [3-5]: velocity (vx, vy, vz)
    // [6-9]: quaternion (q0, q1, q2, q3)
    // [10-12]: angular velocity (wx, wy, wz)
};
```

### 3. Template on Scalar Type
Always template components on scalar type `T` to support autodiff:

```cpp
template<Scalar T = double>
class MyComponent : public TypedComponent<N, T> {
    // Use T for all computed values
    // Use double for constant parameters (mass, area, etc.)
};
```

### 4. Required Boilerplate
Every component must include:

```cpp
template<Scalar T>
class MyComponent : public TypedComponent<StateSize, T> {
public:
    using Base = TypedComponent<StateSize, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Base::computeLocalDerivatives;  // CRITICAL: Expose registry-aware method

    // ... rest of implementation
};
```

---

## State Function Organization

### 1. Hierarchical Tag Namespaces
Organize tags by physics domain to avoid collisions and improve discoverability:

```cpp
namespace sopot {

// Domain-specific namespaces
namespace rigid_body {
    struct Position : categories::Kinematics { /* ... */ };
    struct Velocity : categories::Kinematics { /* ... */ };
    struct Attitude : categories::Kinematics { /* ... */ };      // Quaternion
    struct AngularVelocity : categories::Kinematics { /* ... */ };
    struct BodyVelocity : categories::Kinematics { /* ... */ };  // In body frame
}

namespace aerodynamics {
    struct DragForce : categories::Dynamics { /* ... */ };
    struct LiftForce : categories::Dynamics { /* ... */ };
    struct AeroMoment : categories::Dynamics { /* ... */ };
    struct DynamicPressure : categories::Dynamics { /* ... */ };
    struct MachNumber : categories::Analysis { /* ... */ };
    struct AngleOfAttack : categories::Analysis { /* ... */ };
}

namespace propulsion {
    struct ThrustForce : categories::Dynamics { /* ... */ };
    struct ThrustMoment : categories::Dynamics { /* ... */ };
    struct FuelMass : categories::Dynamics { /* ... */ };
    struct MassFlowRate : categories::Dynamics { /* ... */ };
}

namespace atmosphere {
    struct Density : categories::Environment { /* ... */ };
    struct Pressure : categories::Environment { /* ... */ };
    struct Temperature : categories::Environment { /* ... */ };
    struct SpeedOfSound : categories::Environment { /* ... */ };
}

}  // namespace sopot
```

### 2. Unique Type IDs
Assign non-overlapping type_id ranges to each domain:

```cpp
// Core kinematics: 1-99
// Energy: 100-199
// Dynamics: 200-299
// Rigid body: 1000-1099
// Aerodynamics: 1100-1199
// Propulsion: 1200-1299
// Atmosphere: 1300-1399
// Navigation: 1400-1499
// Control: 1500-1599
```

### 3. Vector vs Scalar State Functions
For vector quantities, provide both combined and component access:

```cpp
// Combined (returns custom Vector3<T> type)
T compute(rigid_body::Velocity, const std::vector<T>& state) const {
    return Vector3<T>{
        getGlobalState(state, 3),
        getGlobalState(state, 4),
        getGlobalState(state, 5)
    };
}

// Individual components (useful for specific calculations)
T compute(rigid_body::VelocityX, const std::vector<T>& state) const {
    return getGlobalState(state, 3);
}
```

---

## Cross-Component Dependencies

### 1. Use Registry for All Cross-Component Access

```cpp
// GOOD: Query through registry
template<typename Registry>
T computeDragForce(const std::vector<T>& state, const Registry& registry) const {
    auto velocity = registry.template computeFunction<rigid_body::Velocity>(state);
    auto density = registry.template computeFunction<atmosphere::Density>(state);
    // ...
}

// BAD: Hardcoded indices
T computeDragForce(const std::vector<T>& state) const {
    T vx = state[3];  // Assumes rigid body is first component!
    T vy = state[4];  // Breaks if component order changes
    // ...
}
```

### 2. Static Dependency Checking
Use `static_assert` to verify required dependencies at compile time:

```cpp
template<typename Registry>
T computeDragForce(const std::vector<T>& state, const Registry& registry) const {
    static_assert(Registry::template hasFunction<rigid_body::Velocity>(),
                 "Aerodynamics requires a component that provides Velocity");
    static_assert(Registry::template hasFunction<atmosphere::Density>(),
                 "Aerodynamics requires a component that provides Density");
    // ...
}
```

### 3. Conditional Dependencies
Handle optional dependencies gracefully:

```cpp
template<typename Registry>
T computeTotalForce(const std::vector<T>& state, const Registry& registry) const {
    T force = T{0};

    // Required
    force += registry.template computeFunction<propulsion::ThrustForce>(state);

    // Optional: add drag if aerodynamics component present
    if constexpr (Registry::template hasFunction<aerodynamics::DragForce>()) {
        force += registry.template computeFunction<aerodynamics::DragForce>(state);
    }

    return force;
}
```

### 4. Avoid Circular Dependencies
Design state functions to flow in one direction:

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  Atmosphere  │───►│ Aerodynamics │───►│  RigidBody   │
│  (density)   │    │  (forces)    │    │ (derivatives)│
└──────────────┘    └──────────────┘    └──────────────┘
        │                                       ▲
        └───────────────────────────────────────┘
                    (altitude → density)
```

If circular dependency is unavoidable, break the cycle by:
1. Using previous timestep values
2. Splitting the component
3. Using iterative solving within a timestep

---

## Autodiff Compatibility

### 1. Use `T` for All Computed Values
Every intermediate result must use the scalar type:

```cpp
// GOOD
T computeDrag(const std::vector<T>& state) const {
    T velocity = getGlobalState(state, 1);
    T density = T(m_air_density);           // Constant → T
    T coeff = T(0.5 * m_Cd * m_area);       // Combine constants
    return coeff * density * velocity * abs(velocity);
}

// BAD: Loses derivative information
T computeDrag(const std::vector<T>& state) const {
    double velocity = value_of(getGlobalState(state, 1));  // Strips derivatives!
    // ...
}
```

### 2. Avoid Non-Differentiable Operations
Some operations break derivative propagation:

```cpp
// BAD: Conditional on value breaks autodiff
T computeForce(T velocity) const {
    if (value_of(velocity) > 0) {  // Branch based on value
        return velocity * T(2.0);
    } else {
        return velocity * T(1.0);
    }
}

// GOOD: Smooth approximation
T computeForce(T velocity) const {
    // Use smooth functions that work with autodiff
    T sign = tanh(velocity * T(100.0));  // Smooth sign function
    return velocity * (T(1.5) + T(0.5) * sign);
}
```

### 3. Math Functions Must Support Dual Numbers
Use the overloaded math functions from `dual.hpp`:

```cpp
// These work with Dual<T, N>:
sin(x), cos(x), tan(x)
exp(x), log(x)
sqrt(x), pow(x, n)
abs(x), atan2(y, x)

// Standard library functions do NOT work:
std::sin(x)  // Won't compile with Dual
```

---

## Performance Considerations

### 1. Minimize State Vector Copies
The registry passes `const std::vector<T>&` - don't copy unless necessary:

```cpp
// GOOD: Pass by reference
T compute(Tag, const std::vector<T>& state) const {
    return state[m_state_offset];
}

// BAD: Unnecessary copy
T compute(Tag, const std::vector<T>& state) const {
    std::vector<T> local_copy = state;  // Expensive!
    return local_copy[m_state_offset];
}
```

### 2. Cache Expensive Computations
If multiple state functions need the same intermediate result:

```cpp
template<Scalar T>
class Aerodynamics : public TypedComponent<0, T> {
    // Compute all aero quantities at once
    struct AeroQuantities {
        T drag, lift, moment;
        T mach, alpha, dynamic_pressure;
    };

    template<typename Registry>
    AeroQuantities computeAll(const std::vector<T>& state, const Registry& reg) const {
        // Single computation of expensive quantities
        auto vel = reg.template computeFunction<rigid_body::Velocity>(state);
        auto rho = reg.template computeFunction<atmosphere::Density>(state);
        // ... compute everything once
    }

    // Individual getters can cache or recompute as needed
};
```

### 3. Compile-Time vs Runtime Trade-offs
More components = longer compile times but same runtime performance:

```cpp
// Compile time: O(N²) where N = number of components (template instantiation)
// Runtime: O(1) for function dispatch (fully inlined)

// For very large systems (50+ components), consider:
// 1. Grouping related components into subsystems
// 2. Using explicit template instantiation
// 3. Precompiled headers
```

---

## Common Pitfalls

### 1. Forgetting `using Base::computeLocalDerivatives`
```cpp
// BUG: Registry-aware method hidden by override
class MyComponent : public TypedComponent<2, T> {
    LocalDerivative computeLocalDerivatives(...) const override {
        // This hides the base class template method!
    }
};

// FIX: Add using declaration
class MyComponent : public TypedComponent<2, T> {
    using Base::computeLocalDerivatives;  // Expose template method
    LocalDerivative computeLocalDerivatives(...) const override { ... }
};
```

### 2. Wrong State Offset Calculation
```cpp
// BUG: Using wrong offset
T compute(Tag, const std::vector<T>& state) const {
    return state[2];  // Absolute index - wrong!
}

// FIX: Use getGlobalState helper
T compute(Tag, const std::vector<T>& state) const {
    return this->getGlobalState(state, 2);  // Relative to component offset
}
```

### 3. Mixing Scalar Types
```cpp
// BUG: Mixing T and double
T computeEnergy(const std::vector<T>& state) const {
    T velocity = getGlobalState(state, 1);
    double ke = 0.5 * m_mass * velocity * velocity;  // Compile error!
    return ke;
}

// FIX: Use T consistently
T computeEnergy(const std::vector<T>& state) const {
    T velocity = getGlobalState(state, 1);
    T ke = T(0.5 * m_mass) * velocity * velocity;
    return ke;
}
```

### 4. Modifying State in compute() Methods
```cpp
// BUG: State functions must be pure
T compute(Tag, const std::vector<T>& state) const {
    m_cached_value = state[0];  // Side effect!
    return m_cached_value;
}

// State functions are called during autodiff with different derivative seeds
// Side effects will produce incorrect Jacobians
```

---

## Rocket Simulation Example

### Component Structure
```cpp
// Rigid body dynamics (13 states)
template<Scalar T>
class RigidBody6DOF : public TypedComponent<13, T> {
    // States: [x,y,z, vx,vy,vz, q0,q1,q2,q3, wx,wy,wz]
    // Provides: Position, Velocity, Attitude, AngularVelocity, BodyVelocity
    // Queries: TotalForce, TotalMoment, Mass, Inertia
};

// Aerodynamics (0 states)
template<Scalar T>
class AxisymmetricAero : public TypedComponent<0, T> {
    // Provides: DragForce, AeroMoment, DynamicPressure, MachNumber, AoA
    // Queries: Velocity, Attitude, Density, SpeedOfSound
};

// Thrust (1 state: fuel mass)
template<Scalar T>
class SolidMotor : public TypedComponent<1, T> {
    // States: [fuel_mass]
    // Provides: ThrustForce, ThrustMoment, FuelMass, MassFlowRate
    // Queries: (none - lookup table based)
};

// Atmosphere (0 states)
template<Scalar T>
class StandardAtmosphere : public TypedComponent<0, T> {
    // Provides: Density, Pressure, Temperature, SpeedOfSound
    // Queries: Position (for altitude)
};

// Mass properties (0 states, or 1 if tracking CoG)
template<Scalar T>
class MassProperties : public TypedComponent<0, T> {
    // Provides: Mass, Inertia, CenterOfGravity
    // Queries: FuelMass
};

// Force aggregator (0 states)
template<Scalar T>
class ForceAggregator : public TypedComponent<0, T> {
    // Provides: TotalForce, TotalMoment
    // Queries: ThrustForce, DragForce, AeroMoment, ThrustMoment, Gravity
};
```

### System Assembly
```cpp
// Create with autodiff support for 14 state variables
using Dual14 = Dual<double, 14>;

auto rocket = makeTypedODESystem<Dual14>(
    RigidBody6DOF<Dual14>(initial_state),
    AxisymmetricAero<Dual14>(aero_data),
    SolidMotor<Dual14>(motor_data),
    StandardAtmosphere<Dual14>(),
    MassProperties<Dual14>(dry_mass, fuel_mass),
    ForceAggregator<Dual14>()
);

// Verify all dependencies at compile time
static_assert(decltype(rocket)::hasFunction<rigid_body::Position>());
static_assert(decltype(rocket)::hasFunction<propulsion::ThrustForce>());
static_assert(decltype(rocket)::hasFunction<aerodynamics::DragForce>());

// Compute Jacobian for control design
auto state = rocket.getInitialState();
auto jacobian = computeJacobian(rocket, 0.0, state);
```

### Data Flow
```
┌─────────────────────────────────────────────────────────────────┐
│                      TypedODESystem                              │
│                                                                  │
│  ┌────────────┐      ┌──────────────┐      ┌────────────────┐  │
│  │ Atmosphere │─────►│ Aerodynamics │─────►│                │  │
│  │  density   │      │  drag, lift  │      │                │  │
│  └────────────┘      └──────────────┘      │                │  │
│        ▲                    │              │ ForceAggregator│  │
│        │                    │              │  total force   │  │
│  ┌─────┴──────┐      ┌──────┴───────┐      │  total moment  │  │
│  │ RigidBody  │      │  SolidMotor  │─────►│                │  │
│  │  altitude  │      │   thrust     │      │                │  │
│  └────────────┘      └──────────────┘      └───────┬────────┘  │
│        ▲                    │                      │           │
│        │                    ▼                      ▼           │
│        │             ┌──────────────┐      ┌────────────────┐  │
│        │             │MassProperties│      │   RigidBody    │  │
│        └─────────────│  mass, I     │◄─────│  derivatives   │  │
│                      └──────────────┘      └────────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Summary Checklist

When creating a new component:

- [ ] Template on `Scalar T`
- [ ] Inherit from `TypedComponent<StateSize, T>`
- [ ] Add `using Base::computeLocalDerivatives;`
- [ ] Use `T` for all computed values
- [ ] Use `getGlobalState()` for state access
- [ ] Query other components through registry
- [ ] Add `static_assert` for required dependencies
- [ ] Keep state functions pure (no side effects)
- [ ] Organize tags in domain namespaces
- [ ] Assign unique type_id values
