# SOPOT V2: Compiler-Inspired Component Architecture

## Vision: Components as Compiler Modules

**Goal**: Create a component system where components are completely agnostic to their usage context, similar to how compiler modules don't know what program they're compiling.

## Core Principles

### 1. **Declaration Over Interrogation**
Components DECLARE dependencies, not QUERY for them.

### 2. **Compile-Time Dependency Resolution**
System builder acts as "linker" - resolves all dependencies at compile-time.

### 3. **Zero Runtime Overhead**
All wiring, type checking, and optimization happens during compilation.

### 4. **Composability**
Any component can work with any other component if interfaces match.

---

## Proposed API Design

### Component Declaration (User-Facing)

```cpp
// Example: Chemical reaction component
template<Scalar T = double>
class ArrheniusReaction {
public:
    // ===== DECLARE INTERFACE =====

    // What this component NEEDS from the system
    struct Dependencies {
        Field<chemistry::Concentration, "species_A"> conc_A;
        Field<chemistry::Concentration, "species_B"> conc_B;
        Field<environment::Temperature, "temp"> temperature;
        Field<environment::Pressure, "press"> pressure;
    };

    // What this component PROVIDES to the system
    struct Provides {
        Field<chemistry::ReactionRate, "rate"> reaction_rate;
        Field<chemistry::HeatRelease, "heat"> heat_generation;
    };

    // Internal state size (if stateful)
    static constexpr size_t StateSize = 0;  // Stateless provider

    // ===== IMPLEMENTATION =====

    // Compute function receives ONLY declared dependencies
    template<typename Deps>
    auto compute(T t, Deps&& deps) const {
        // Type-safe access to dependencies
        T c_A = deps.conc_A;
        T c_B = deps.conc_B;
        T T_val = deps.temperature;

        // Arrhenius equation
        T k = A * exp(-E_a / (R * T_val));
        T rate = k * c_A * c_B;
        T heat = rate * delta_H;

        // Return provided values (compile-time checked)
        return Provides{
            .reaction_rate = rate,
            .heat_generation = heat
        };
    }

private:
    T A;        // Pre-exponential factor
    T E_a;      // Activation energy
    T delta_H;  // Heat of reaction
    static constexpr T R = 8.314;  // Gas constant
};
```

### System Composition

```cpp
// Build a chemical reactor system
auto reactor = makeSystem(
    // Stream components
    InletStream<double>("feed", flowRate, composition),
    OutletStream<double>("product"),

    // Physical property components
    ThermodynamicProperties<double>(),
    TransportProperties<double>(),

    // Reaction components (many of them!)
    ArrheniusReaction<double>(A1, E1, dH1),
    ArrheniusReaction<double>(A2, E2, dH2),
    ArrheniusReaction<double>(A3, E3, dH3),
    // ... 50 more reactions

    // Mass balance components (one per species)
    SpeciesMassBalance<double>("A"),
    SpeciesMassBalance<double>("B"),
    SpeciesMassBalance<double>("C"),
    // ... 100 species

    // Energy balance
    EnergyBalance<double>(),

    // Momentum balance (if needed)
    PressureDropCalculator<double>()
);

// ===== COMPILE-TIME VERIFICATION =====
static_assert(reactor.allDependenciesSatisfied(),
    "Some component dependencies are not provided");
static_assert(!reactor.hasCircularDependencies(),
    "Circular dependency detected in system");
static_assert(reactor.totalStateSize() > 0,
    "System has no state variables");

// ===== USE IT =====
auto state = reactor.getInitialState();
auto derivatives = reactor.computeDerivatives(t, state);
```

---

## Technical Implementation Strategy

### Phase 1: Type-Level Dependency Graph

```cpp
// Extract interface information at compile-time
template<typename Component>
concept HasDependencies = requires {
    typename Component::Dependencies;
};

template<typename Component>
concept HasProvides = requires {
    typename Component::Provides;
};

// Dependency graph builder
template<typename... Components>
class DependencyGraph {
    // Collect all dependencies and provisions
    using AllDeps = concat_t<typename Components::Dependencies...>;
    using AllProvs = concat_t<typename Components::Provides...>;

    // Build bipartite graph: Components <-> Fields
    static constexpr auto graph = buildGraph<Components...>();

    // Topological sort for execution order
    static constexpr auto executionOrder = topoSort(graph);

    // Verify soundness
    static_assert(allDepsProvided<AllDeps, AllProvs>(),
        "Unmet dependency");
    static_assert(noCycles(graph),
        "Circular dependency");
};
```

### Phase 2: Field Identification System

Replace explicit tags with field names:

```cpp
// Field is identified by (Type, Name) pair
template<typename T, FixedString Name>
struct Field {
    using Type = T;
    static constexpr auto name = Name;

    // Storage
    Type value;

    // Implicit conversion for ergonomics
    operator Type() const { return value; }
};

// Match fields by type AND name
template<typename Field1, typename Field2>
concept MatchingFields =
    std::same_as<typename Field1::Type, typename Field2::Type> &&
    (Field1::name == Field2::name);
```

### Phase 3: Automatic Dependency Injection

```cpp
// System generates specialized accessor for each component
template<typename Component, typename SystemState>
class ComponentAccessor {
    const SystemState& state;

public:
    // Extract dependencies for this component
    auto getDependencies() const {
        typename Component::Dependencies deps;

        // For each field in Dependencies, find provider in system
        [&]<typename... Fields>(TypeList<Fields...>) {
            (injectField<Fields>(deps), ...);
        }(typename Component::Dependencies{});

        return deps;
    }

private:
    template<typename Field>
    void injectField(auto& deps) const {
        // Find component that provides this field
        constexpr auto provider = findProvider<Field, SystemState>();

        // Extract value from provider
        deps.*(&Field::value) = state.template getValue<provider, Field>();
    }
};
```

### Phase 4: Code Generation

```cpp
// Generate optimized derivative computation
template<typename... Components>
class OptimizedSystem {
    // State layout determined at compile-time
    using StateVector = PackedState<Components...>;

    // Generated computation function
    StateVector computeDerivatives(T t, const StateVector& state) const {
        StateVector derivs;

        // Execute in topological order (compile-time constant)
        [&]<size_t... Is>(std::index_sequence<Is...>) {
            (executeComponent<executionOrder[Is]>(t, state, derivs), ...);
        }(std::make_index_sequence<sizeof...(Components)>{});

        return derivs;
    }

private:
    template<size_t ComponentIndex>
    void executeComponent(T t, const StateVector& state, StateVector& derivs) const {
        using Component = nth_type_t<ComponentIndex, Components...>;

        // Inject dependencies
        auto deps = extractDependencies<Component>(state);

        // Compute
        auto results = std::get<ComponentIndex>(components).compute(t, deps);

        // Store results
        storeResults<Component>(results, state, derivs);
    }
};
```

---

## Advanced Features

### 1. **Partial Evaluation**

For components with constant parameters, fold computations:

```cpp
// If temperature is constant, pre-compute exp(-E_a/RT)
template<typename Component>
concept HasConstantInputs = /* detect constant fields */;

template<HasConstantInputs Component>
class OptimizedComponent {
    // Pre-computed values stored as members
    T precomputed_k = A * exp(-E_a / (R * T_const));
};
```

### 2. **Parallel Execution**

Analyze dependencies for parallel execution:

```cpp
// Components with no data dependencies can run in parallel
auto parallelGroups = analyzeDependencies<Components...>();

// Execute each group in parallel
for (const auto& group : parallelGroups) {
    std::for_each(std::execution::par, group.begin(), group.end(),
        [&](auto component) { executeComponent(component); });
}
```

### 3. **Dataflow Visualization**

Generate GraphViz diagrams automatically:

```cpp
auto reactor = makeSystem(...);
reactor.exportDependencyGraph("reactor_topology.dot");

// Generates:
// digraph {
//   "ArrheniusReaction_1" -> "SpeciesMassBalance_A" [label="reaction_rate"]
//   "ThermoProp" -> "ArrheniusReaction_1" [label="temperature"]
//   ...
// }
```

### 4. **Algebraic Optimization**

Detect and eliminate redundant computations:

```cpp
// If multiple components need same value, compute once
auto density = /* provided by ThermoProp */;

// Instead of:
//   comp1.compute(density)
//   comp2.compute(density)  // Re-fetch density
//
// Generate:
//   T cached_density = system.get<Density>();
//   comp1.compute(cached_density)
//   comp2.compute(cached_density)
```

---

## Migration Path from V1

### Step 1: Dual API Support

```cpp
// V1 style (deprecated)
class LegacyComponent : public TypedComponent<3, double> {
    template<typename Registry>
    auto compute(const Registry& reg) const {
        auto val = reg.template computeFunction<SomeTag>(state);
        // ...
    }
};

// V2 style (preferred)
class ModernComponent {
    struct Dependencies {
        Field<SomeTag, "value"> val;
    };
    // ...
};
```

### Step 2: Adapter Pattern

```cpp
// Wrap V1 components for V2 systems
template<typename V1Component>
class V1Adapter {
    // Auto-generate Dependencies from V1 registry queries
    using Dependencies = ExtractedDependencies<V1Component>;

    // Forward to V1 implementation
    auto compute(auto deps) const {
        auto mock_registry = createMockRegistry(deps);
        return v1_component.compute(mock_registry);
    }
};
```

---

## Example: 100-Species Chemical Reactor

```cpp
// Define species
constexpr std::array species = {"H2", "O2", "H2O", "N2", /* ... 96 more */};

// Generate components programmatically
auto reactor = []<size_t... Is>(std::index_sequence<Is...>) {
    return makeSystem(
        // Mass balances (one per species)
        SpeciesMassBalance<double>(species[Is])...,

        // Reactions (many!)
        CombustionReaction<double>("H2", "O2", "H2O"),
        DecompositionReaction<double>("H2O2", "H2O", "O2"),
        // ... generated from reaction database

        // Properties
        ThermodynamicProperties<double>(species),
        TransportProperties<double>(species),

        // Equipment
        CSTRVolumeElement<double>(volume),
        HeatExchanger<double>(area, U),

        // Boundaries
        InletStream<double>("feed"),
        OutletStream<double>("product")
    );
}(std::make_index_sequence<species.size()>{});

// Compile-time verification
static_assert(reactor.totalComponents() == 200+);  // Many components!
static_assert(reactor.totalStateSize() == species.size() + 1);  // 100 species + 1 temp
static_assert(reactor.isWellFormed());  // All deps satisfied

// Simulate
auto state = reactor.getInitialState();
RK4Solver solver;
auto trajectory = solver.integrate(reactor, 0.0, 100.0, 0.01, state);
```

---

## Key Innovations

1. **Type-Level Programming**: Use C++20 concepts, requires, and consteval for compile-time verification
2. **Dependency Injection**: Automatic wiring based on type+name matching
3. **Code Generation**: Specialized functions generated per system configuration
4. **Zero Overhead**: All abstraction compiled away - as fast as hand-written code
5. **Scalability**: Works from 2 components to 1000+ components

## Next Steps

1. Implement `Field<Type, Name>` primitive
2. Build dependency extraction via reflection (or macro-based)
3. Implement topological sort for execution order
4. Generate optimized `computeDerivatives`
5. Add compile-time diagnostics with helpful error messages
6. Benchmark against V1 architecture

---

*This is a living document. As we prototype, we'll refine the design.*
