# SimCore Architecture: Component Composition and State Functions

## The Core Problem

In a rocket simulation, we have multiple physics components (rigid body, aerodynamics, thrust, atmosphere) that:
1. Each own part of the ODE state vector (position, velocity, fuel mass, etc.)
2. Need to compute derivatives of their own state (dx/dt)
3. **Need to query computed values from other components** (aerodynamics needs velocity from rigid body)

The naive approach (hardcoding indices like `global_state[3]`) breaks encapsulation and makes the system fragile. The C# version solved this with runtime reflection, but we want **zero-overhead compile-time dispatch** in C++.

## Solution: State Functions + Registry Pattern

### The Key Insight

Every computed quantity in the simulation is a **state function** - a pure function of the ODE state vector:

```
velocity(state) → Vector3
air_density(state) → double
drag_force(state) → Vector3
```

Components **declare** which state functions they provide, and other components **query** them through a registry that dispatches at compile time.

## Architecture Components

### 1. State Function Tags (`state_function_tags.hpp`)

**Purpose**: Type-safe identifiers for state functions. Each tag is a unique type that identifies what value is being requested.

```cpp
namespace kinematics {
    struct Velocity : categories::Kinematics {
        static constexpr std::string_view name() { return "velocity"; }
        static constexpr size_t type_id() { return 2; }
    };
}
```

**Why types instead of strings/enums?**
- Compile-time dispatch (no switch statements, no hash lookups)
- Extensibility (new domains add new tags without modifying core)
- Type safety (can't accidentally query wrong function type)

**Usage**: Tags are used as template parameters for compile-time dispatch:
```cpp
registry.computeFunction<kinematics::Velocity>(state)  // Returns velocity
registry.computeFunction<energy::Total>(state)         // Returns total energy
```

### 2. TypedComponent (`typed_component.hpp`)

**Purpose**: Base class for simulation components that:
- Own a portion of the ODE state vector
- Compute derivatives for their state variables
- Provide state functions to other components
- Can query state functions from other components via registry

```cpp
template<size_t StateSize, Scalar T = double>
class TypedComponent {
    // Compute dx/dt for this component's state
    virtual LocalDerivative computeLocalDerivatives(
        T t, const LocalState& local, const std::vector<T>& global
    ) const;

    // Registry-aware version for cross-component access
    template<typename Registry>
    LocalDerivative computeLocalDerivatives(
        T t, const LocalState& local, const std::vector<T>& global,
        const Registry& registry  // Can query other components!
    ) const;
};
```

**Key features**:
- **Templated on scalar type `T`**: Works with `double`, `Dual<double,N>` (autodiff), or `Quantity<Dim,T>` (units)
- **State offset management**: Each component knows where its state lives in the global vector
- **Two interfaces**: Legacy (standalone) and registry-aware (cross-component)

### 3. TypedRegistry (in `typed_component.hpp`)

**Purpose**: Compile-time dispatch of state function queries. Given a tag, finds the component that provides that function and calls it.

```cpp
template<typename T, TypedComponentConcept... Components>
class TypedRegistry {
    // Check at compile time if ANY component provides this function
    template<StateTagConcept Tag>
    static constexpr bool hasFunction();

    // Zero-overhead dispatch - resolved at compile time
    template<StateTagConcept Tag>
    T computeFunction(const std::vector<T>& state) const;
};
```

**How dispatch works**:
1. At compile time, `findProvider<Tag>()` iterates through component types
2. Uses C++20 concepts to check which component has `compute(Tag{}, state)`
3. Returns reference to that component (compile-time selection)
4. Caller invokes `provider.compute(Tag{}, state)` - direct call, no virtual dispatch

### 4. TypedODESystem (in `typed_component.hpp`)

**Purpose**: Composes components into a complete ODE system. Orchestrates state management and derivative collection.

```cpp
template<typename T, TypedComponentConcept... Components>
class TypedODESystem {
    std::tuple<Components...> m_components;      // Owns components
    TypedRegistry<T, Components...> m_registry;  // Enables cross-component queries

    // Core ODE interface
    std::vector<T> computeDerivatives(T t, const std::vector<T>& state) const;
    std::vector<T> getInitialState() const;

    // State function queries
    template<StateTagConcept Tag>
    T computeStateFunction(const std::vector<T>& state) const;
};
```

**What it does**:
1. **State offset assignment**: Each component gets its slice of the global state
2. **Derivative collection**: Calls each component's `computeLocalDerivatives`, passes registry
3. **State function routing**: Delegates to registry for function queries

## How Cross-Component Access Works

### The Problem
Aerodynamics component needs velocity, but velocity is owned by rigid body:

```
RigidBody: state[0..5] = [x, y, z, vx, vy, vz]
Aerodynamics: state[6..6] = [] (no own state)

Aerodynamics needs vx, vy, vz to compute drag!
```

### The Solution

1. **RigidBody declares it provides Velocity**:
```cpp
class RigidBody : public TypedComponent<6, T> {
    T compute(kinematics::Velocity, const std::vector<T>& state) const {
        return /* extract velocity from state */;
    }
};
```

2. **Aerodynamics queries Velocity through registry**:
```cpp
class Aerodynamics : public TypedComponent<0, T> {
    template<typename Registry>
    LocalDerivative computeLocalDerivatives(
        T t, const LocalState&, const std::vector<T>& state,
        const Registry& registry
    ) const {
        // Query velocity - no hardcoded indices!
        T velocity = registry.template computeFunction<kinematics::Velocity>(state);
        // Use velocity to compute drag...
    }
};
```

3. **TypedODESystem wires them together**:
```cpp
auto system = makeTypedODESystem<Dual<double,6>>(
    RigidBody<Dual6>(...),
    Aerodynamics<Dual6>(...)
);
// Registry automatically knows RigidBody provides Velocity
// Aerodynamics can query it during derivative computation
```

### Autodiff Propagation

Because everything is templated on scalar type `T`:
- When `T = Dual<double, 6>`, derivatives propagate through cross-component calls
- Jacobian computation "just works" - no manual derivative coding
- LQR control design gets automatic linearization

## File Inventory: Do We Need All of Them?

| File | Purpose | Keep? |
|------|---------|-------|
| `state_function_tags.hpp` | Tag types for state functions | **YES** - Essential for type-safe dispatch |
| `typed_component.hpp` | TypedComponent, TypedRegistry, TypedODESystem | **YES** - Core architecture |
| `component.hpp` | Non-templated Component base | **DEPRECATE** - Superseded by TypedComponent |
| `compile_time_registry.hpp` | Non-templated CompileTimeRegistry | **DEPRECATE** - Superseded by TypedRegistry |
| `scalar.hpp` | Scalar concept, value_of, etc. | **YES** - Required for autodiff integration |
| `dual.hpp` | Forward-mode autodiff | **YES** - Core capability |
| `units.hpp` | Compile-time dimensional analysis | **YES** - Type safety for physics |
| `linearization.hpp` | System linearization for LQR | **YES** - Control design |
| `ode_system.hpp` | Non-templated ODE system | **DEPRECATE** - Superseded by TypedODESystem |
| `solver.hpp` | RK4 integrator | **YES** - Needed for simulation |

### Recommendation: Consolidate to Typed Architecture

The codebase has two parallel hierarchies:

**Old (double-only)**:
- `Component<N>` → `CompileTimeRegistry` → `CompileTimeODESystem`

**New (templated on scalar)**:
- `TypedComponent<N, T>` → `TypedRegistry<T, ...>` → `TypedODESystem<T, ...>`

**Action**: Keep only the typed versions. The old non-templated versions exist for backward compatibility with `harmonic_oscillator.hpp` examples but should be migrated.

## Summary

### What Each Module Does

| Module | Role |
|--------|------|
| **State Function Tags** | Identity - "what value am I asking for?" |
| **TypedComponent** | Ownership - "who owns this state and provides this function?" |
| **TypedRegistry** | Dispatch - "given a tag, find the provider and call it" |
| **TypedODESystem** | Orchestration - "compose components, manage state, collect derivatives" |

### The Flow

```
1. User creates components: RigidBody, Aero, Thrust, ...
2. User creates system: makeTypedODESystem<T>(components...)
3. System assigns state offsets: RigidBody[0..5], Aero[6..6], ...
4. System creates registry from components
5. During integration:
   a. Solver calls system.computeDerivatives(t, state)
   b. System iterates components, passes registry
   c. Each component computes its derivatives
   d. Components query other components via registry
   e. All dispatch resolved at compile time - zero overhead
```

### Key Benefits

1. **Zero runtime overhead** - All function dispatch resolved at compile time
2. **Autodiff compatible** - Derivatives propagate through cross-component calls
3. **Type safe** - Wrong tag = compile error, not runtime bug
4. **Extensible** - Add new state functions without modifying core
5. **Encapsulated** - Components don't know each other's state layout

---

## Compile-Time CAS (Computer Algebra System)

### Purpose

For constrained dynamics (pendulums, linkages, etc.), we need constraint Jacobians:
- Constraint: g(q) = 0
- Jacobian: J = ∂g/∂q

Manual derivation is error-prone. SOPOT provides a **compile-time CAS** that derives Jacobians automatically via template metaprogramming.

### Architecture

```
physics/constraints/symbolic/
├── expression.hpp        # Expression template types
├── differentiation.hpp   # Symbolic differentiation rules
└── named_expression.hpp  # Ergonomic named variable API
```

### Expression Templates

Expressions are **types**, not values. The entire expression tree is encoded in the type system:

```cpp
// Var<I> represents variable x_i
using x = Var<0>;
using y = Var<1>;

// Add<L, R> represents L + R
using expr = Add<Mul<x, x>, y>;  // x² + y

// Evaluation happens at runtime, but structure is compile-time
std::array<double, 2> vars = {3.0, 2.0};
double result = eval<expr>(vars);  // = 11
```

### Symbolic Differentiation

Differentiation rules are implemented as template specializations:

```cpp
// d/dx(const) = 0
template<int N, int D, size_t I>
struct Diff<Const<N, D>, I> { using type = Zero; };

// d/dx(x_j) = δ_ij (Kronecker delta)
template<size_t J, size_t I>
struct Diff<Var<J>, I> { using type = std::conditional_t<I==J, One, Zero>; };

// Product rule: d/dx(f·g) = f·(dg/dx) + (df/dx)·g
template<typename L, typename R, size_t I>
struct Diff<Mul<L, R>, I> {
    using type = Add<Mul<L, Diff_t<R, I>>, Mul<Diff_t<L, I>, R>>;
};

// Chain rule: d/dx(sin(f)) = cos(f) · df/dx
template<typename E, size_t I>
struct Diff<Sin<E>, I> {
    using type = Mul<Cos<E>, Diff_t<E, I>>;
};
```

### Named Expressions

For ergonomics, `named_expression.hpp` provides named variables:

```cpp
// Instead of:
using g1 = Add<Square<Var<0>>, Square<Var<1>>>;

// Write:
namespace tb = symbolic::cartesian::two_body_2d;
auto g1 = sq(tb::x1) + sq(tb::y1);  // Same compiled type!
```

### Jacobian Generation

```cpp
// Define constraints
auto g1 = sq(x1) + sq(y1);           // x₁² + y₁²
auto g2 = sq(x2-x1) + sq(y2-y1);     // (x₂-x₁)² + (y₂-y₁)²

// Jacobian type is computed at compile time
using J = Jacobian<4, decltype(g1)::type, decltype(g2)::type>;

// Evaluation is runtime, but no symbolic work needed
auto jacobian = J::eval(position);  // Returns 2x4 matrix
```

### Supported Operations

| Category | Operations |
|----------|------------|
| Arithmetic | `Add`, `Sub`, `Mul`, `Div`, `Neg`, `Pow<E, N>` |
| Trigonometric | `Sin`, `Cos` |
| Other | `Sqrt`, `Square` (alias for `Pow<E, 2>`) |
| Constants | `Const<N, D>`, `Zero`, `One`, `Two` |

### Constraint Types Supported

- **Holonomic constraints**: g(q) = 0 (position-only)
- **Distance constraints**: |r₁ - r₂|² - L² = 0
- **Angle constraints**: via sin/cos
- **Polynomial constraints**: any degree
- **Algebraic constraints**: involving ratios

### Integration with Components

The `SymbolicCartesianPendulum` demonstrates using CAS in a component:

```cpp
template<Scalar T>
class SymbolicCartesianPendulum : public TypedComponent<8, T> {
    // Constraints defined symbolically
    using g1_type = Add<Square<Var<0>>, Square<Var<1>>>;
    using g2_type = Add<Square<Sub<Var<2>, Var<0>>>, Square<Sub<Var<3>, Var<1>>>>;

    // Jacobian computed at compile time
    using ConstraintJacobian = Jacobian<4, g1_type, g2_type>;

    // In derivatives(), use the Jacobian
    auto J = ConstraintJacobian::eval(position);
    // ... use J for Baumgarte stabilization
};
```

### Benefits

1. **Zero symbolic overhead at runtime** - All differentiation at compile time
2. **Type-safe** - Invalid expressions won't compile
3. **Readable** - Named expressions look like mathematical notation
4. **Autodiff compatible** - Works with `Dual<T, N>` for nested derivatives
5. **Extensible** - Add new operations by adding template specializations
