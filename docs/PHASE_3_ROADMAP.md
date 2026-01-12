# Phase 3 Roadmap: Automatic Derivative Computation

## Current Status (Phase 2 Complete ✓)

We have:
- ✅ `Field<Tag, Name>` primitive
- ✅ `FieldBundle` for dependency declarations
- ✅ `System` class that manages multiple components
- ✅ Compile-time concepts for dependency checking
- ✅ State management with automatic offset computation
- ✅ Component composition in arbitrary order

**Proof**: `system_builder_demo` shows 5 components composed in arbitrary order!

## The Challenge

Currently, derivative computation is **manual**:

```cpp
// User must manually wire dependencies
double F = system.getComponent<4>().provideForce();
double m = system.getComponent<1>().provideMass();
double a = system.getComponent<2>().computeAcceleration(F, m);
double dv_dt = system.getComponent<0>().computeDerivative(0.0, v, a);
```

**Goal**: Make it **automatic**:

```cpp
// System figures out dependencies and execution order!
auto derivs = system.computeDerivatives(t, state);
```

## Implementation Plan

### Step 1: Field Name Extraction (Compile-Time)

Extract field names from `FieldBundle` at compile-time:

```cpp
template<typename FieldBundle>
struct ExtractFieldNames;

template<typename... Fields>
struct ExtractFieldNames<FieldBundle<Fields...>> {
    static constexpr auto names = std::array{Fields::name...};
    static constexpr size_t count = sizeof...(Fields);
};

// Usage
using Deps = typename MyComponent::Dependencies;
constexpr auto depNames = ExtractFieldNames<Deps>::names;
// depNames[0] = "force", depNames[1] = "mass", etc.
```

### Step 2: Dependency Graph Construction

Build adjacency matrix at compile-time:

```cpp
template<typename... Components>
constexpr auto buildDependencyGraph() {
    constexpr size_t N = sizeof...(Components);
    std::array<std::array<bool, N>, N> adj{};

    // For each component i
    for (size_t i = 0; i < N; ++i) {
        // For each dependency of component i
        auto deps = getDependencies<i>();
        for (auto dep : deps) {
            // Find which component j provides this dependency
            size_t j = findProvider(dep);
            // i depends on j
            adj[i][j] = true;
        }
    }

    return adj;
}
```

### Step 3: Topological Sort

Determine execution order using Kahn's algorithm:

```cpp
template<size_t N>
constexpr auto topologicalSort(const std::array<std::array<bool, N>, N>& adj) {
    std::array<size_t, N> order;
    // ... Kahn's algorithm implementation
    return order;
}

// Result: [4, 1, 2, 0, 3]
// Execute: ConstantForce -> ConstantMass -> NewtonSecondLaw -> Velocity -> Position
```

### Step 4: Provision Cache

Runtime storage for computed values:

```cpp
template<typename T>
class ProvisionCache {
    std::map<std::string, T> cache;

public:
    void store(std::string_view fieldName, T value);
    T get(std::string_view fieldName) const;
};

// During execution:
cache.store("force", 10.0);
cache.store("mass", 2.0);
cache.store("accel", 5.0);  // From NewtonSecondLaw
// ...
```

### Step 5: Automatic Dependency Injection

Generate code to inject dependencies:

```cpp
template<size_t ComponentIdx>
auto injectDependencies(const ProvisionCache<T>& cache) const {
    using Comp = nth_type_t<ComponentIdx, Components...>;
    using Deps = typename Comp::Dependencies;

    // Extract field names from Deps
    constexpr auto names = ExtractFieldNames<Deps>::names;

    // Build tuple of values from cache
    return std::make_tuple(cache.get(names[0]), cache.get(names[1]), ...);
}

// Usage:
auto deps = injectDependencies<2>(cache);  // Get (force, mass) for NewtonSecondLaw
auto result = component.compute(t, std::get<0>(deps), std::get<1>(deps));
```

### Step 6: Generated computeDerivatives()

Put it all together:

```cpp
template<typename T, typename... Components>
class AutoDerivSystem {
    static constexpr auto execOrder = topologicalSort(buildDependencyGraph());

    auto computeDerivatives(T t, const State& state) const {
        ProvisionCache<T> cache;
        State derivatives;

        // Execute in topological order
        for (size_t i = 0; i < NumComponents; ++i) {
            size_t compIdx = execOrder[i];

            if (isStateless(compIdx)) {
                // Stateless: compute provisions
                auto deps = injectDependencies(compIdx, cache);
                auto provs = getComponent(compIdx).compute(t, deps);
                storeProvisions(provs, cache);
            } else {
                // Stateful: compute derivative
                auto localState = extractLocalState(compIdx, state);
                auto deps = injectDependencies(compIdx, cache);
                auto deriv = getComponent(compIdx).computeDerivative(t, localState, deps);
                storeDerivative(compIdx, deriv, derivatives);
            }
        }

        return derivatives;
    }
};
```

## Technical Challenges

### Challenge 1: Tuple Unpacking

Need to unpack dependencies tuple to call component functions:

```cpp
// Component expects: compute(t, force, mass)
// We have: tuple<force, mass>

// Solution: std::apply
auto result = std::apply(
    [&](auto... deps) { return component.compute(t, deps...); },
    depsTuple
);
```

### Challenge 2: FieldBundle to Tuple Conversion

Convert `FieldBundle<Field<Tag, Name>...>` to `std::tuple<T, T, ...>`:

```cpp
template<typename FieldBundle>
auto bundleToTuple(const FieldBundle& bundle) {
    return [&]<size_t... Is>(std::index_sequence<Is...>) {
        return std::make_tuple(bundle.template get<Is>().value...);
    }(std::make_index_sequence<FieldBundle::size()>{});
}
```

### Challenge 3: Field Name Comparison

Field names are `FixedString<N>` template parameters - need runtime comparison:

```cpp
// Compile-time
constexpr auto name1 = FixedString{"force"};
constexpr auto name2 = FixedString{"force"};
static_assert(name1 == name2);  // OK!

// Runtime
std::string_view sv1 = name1.view();
std::string_view sv2 = "force";
assert(sv1 == sv2);  // OK!
```

### Challenge 4: Heterogeneous Provision Storage

Components provide different types:

```cpp
// ConstantForce provides: double
// VectorForce provides: Vector3<double>
// MatrixInertia provides: Matrix3x3<double>

// Need type-erased storage:
std::map<std::string, std::any> cache;

// Or typed storage with variant:
std::map<std::string, std::variant<double, Vector3<double>, ...>> cache;
```

## Optimization Opportunities

### Optimization 1: Compile-Time Cache Size

Pre-compute cache size from dependency graph:

```cpp
// Count unique provisions needed
constexpr size_t cacheSize = countUniqueProvisions<Components...>();

// Use fixed-size array instead of map
std::array<T, cacheSize> cache;
std::array<std::string_view, cacheSize> names;
```

### Optimization 2: Direct Field Access

Skip cache for direct dependencies:

```cpp
// If Component A directly depends on Component B output:
auto result = componentB.compute(...);
auto deriv = componentA.computeDerivative(..., result);  // No cache!
```

### Optimization 3: Parallel Execution

Identify independent components:

```cpp
// Components 1 and 4 have no dependencies - execute in parallel
std::async([&]() { return component1.compute(...); });
std::async([&]() { return component4.compute(...); });
```

## Testing Strategy

### Test 1: Simple Chain

```
A -> B -> C
Component A provides X
Component B depends on X, provides Y
Component C depends on Y, provides Z

Test: Verify execution order [A, B, C]
```

### Test 2: Diamond Dependency

```
    A
   / \
  B   C
   \ /
    D

Test: Verify any valid order (e.g., A, B, C, D or A, C, B, D)
```

### Test 3: Circular Dependency Detection

```
A -> B -> C -> A

Test: static_assert should fail at compile-time
```

### Test 4: Missing Dependency

```
A depends on "force"
No component provides "force"

Test: Helpful compile error message
```

## Success Criteria

Phase 3 complete when:

1. ✅ User can call `system.computeDerivatives(t, state)`
2. ✅ System automatically determines execution order
3. ✅ System automatically injects dependencies
4. ✅ Compile-time error for missing/circular dependencies
5. ✅ Works with 10+ components
6. ✅ Zero runtime overhead compared to hand-written code

## Example: Final API

```cpp
// Define components (same as before)
auto system = makeAutoDerivSystem<double>(
    VelocityComponent(0.0),
    ConstantMass(2.0),
    NewtonSecondLaw(),
    PositionComponent(0.0),
    ConstantForce(10.0)
);

// Get initial state
auto state = system.getInitialState();

// Integrate over time
for (double t = 0; t < 10.0; t += 0.01) {
    // ONE LINE - FULLY AUTOMATIC!
    auto derivs = system.computeDerivatives(t, state);

    // Update state (Euler step)
    for (size_t i = 0; i < state.size(); ++i) {
        state[i] += derivs[i] * 0.01;
    }
}
```

**That's the goal!** 🎯

## Implementation Timeline

1. **Week 1**: Field name extraction + dependency graph
2. **Week 2**: Topological sort + cycle detection
3. **Week 3**: Provision cache + dependency injection
4. **Week 4**: Integration + testing + optimization

## Resources Needed

- C++20 features: `requires`, `concepts`, `constexpr`, `consteval`
- Template metaprogramming expertise
- Graph algorithms (topological sort, cycle detection)
- Type erasure techniques (std::any, std::variant)

---

**Let's build it!** 🚀
