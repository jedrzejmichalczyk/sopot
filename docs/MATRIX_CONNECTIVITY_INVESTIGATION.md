# Matrix-Based Component Connectivity Investigation

## Problem Statement

Current SOPOT architecture requires manual component wiring:

```cpp
// Current approach: manually create each connection
auto system = makeTypedODESystem<double>(
    PointMass<mass1, double>(1.0),
    PointMass<mass2, double>(1.0),
    Spring12<double>(10.0, 1.0)  // Connects mass1 to mass2
);
```

For N masses, this requires:
- N explicit PointMass components
- Up to N*(N-1)/2 explicit Spring components
- Unique tag types for each mass (mass1, mass2, ..., massN)

**Goal:** Enable matrix-like connectivity description while maintaining:
- Zero runtime overhead (all resolved at compile time)
- Type safety (no invalid connections)
- Compatibility with existing TypedComponent architecture

## Current Architecture Summary

From coupled oscillator example (`physics/coupled_oscillator/`):

```
┌──────────────┐         ┌──────────────┐
│  Mass1       │◄────────┤  Spring12    │
│  [x1, v1]    │         │  k=10, c=0.5 │
└──────────────┘         └──────────────┘
                               │
                               ▼
                         ┌──────────────┐
                         │  Mass2       │
                         │  [x2, v2]    │
                         └──────────────┘
```

**Key Patterns:**
1. Each mass has unique tag namespace (`mass1`, `mass2`)
2. Spring queries positions via tags: `mass1::Position`, `mass2::Position`
3. Spring provides forces via tags: `mass1::Force`, `mass2::Force`
4. Registry resolves all queries at compile time

## Design Options

### Option 1: Compile-Time Adjacency Matrix with Index-Based Tags

**Concept:** Encode connectivity as constexpr adjacency structure, generate components via template metaprogramming.

```cpp
// Define connectivity at compile time
constexpr std::array<std::pair<size_t, size_t>, 3> edges = {{
    {0, 1},  // Mass 0 connected to Mass 1
    {1, 2},  // Mass 1 connected to Mass 2
    {0, 2}   // Mass 0 connected to Mass 2 (triangular)
}};

// Spring properties for each edge
constexpr std::array<SpringParams, 3> spring_params = {{
    {10.0, 1.0, 0.5},  // k, rest_length, damping
    {15.0, 1.0, 0.3},
    {20.0, 1.5, 0.2}
}};

// Automatically generate system
auto system = makeConnectedMassSystem<double, 3>(
    {1.0, 1.5, 2.0},  // masses
    edges,
    spring_params
);
```

**Implementation Strategy:**

1. **Indexed Tag Generator**
   ```cpp
   // Generate unique tag types for each mass index
   template<size_t Index>
   struct MassTag {
       struct Position : categories::Kinematics {
           static constexpr std::string_view name() {
               return "Mass" + std::to_string(Index) + "::Position";
           }
       };
       struct Velocity : categories::Kinematics { ... };
       struct Force : categories::Dynamics { ... };
       struct Mass : categories::Dynamics { ... };
   };

   // Usage: MassTag<0>::Position, MassTag<1>::Position, etc.
   ```

2. **Generic Indexed Components**
   ```cpp
   // Generic mass that works with any index
   template<size_t Index, Scalar T = double>
   class IndexedPointMass final : public TypedComponent<2, T> {
       using TagSet = MassTag<Index>;
       // ... (same as PointMass but uses TagSet)
   };

   // Generic spring that works with any index pair
   template<size_t Index1, size_t Index2, Scalar T = double>
   class IndexedSpring final : public TypedComponent<0, T> {
       using TagSet1 = MassTag<Index1>;
       using TagSet2 = MassTag<Index2>;
       // ... (same as Spring but uses TagSet1, TagSet2)
   };
   ```

3. **Connection Matrix Generator**
   ```cpp
   // Generate tuple of components from edge list
   template<typename T, size_t NumMasses, size_t NumEdges>
   auto makeConnectedMassSystem(
       const std::array<double, NumMasses>& masses,
       const std::array<std::pair<size_t, size_t>, NumEdges>& edges,
       const std::array<SpringParams, NumEdges>& params
   ) {
       // Generate masses: IndexedPointMass<0>, IndexedPointMass<1>, ...
       auto mass_components = [&]<size_t... Is>(std::index_sequence<Is...>) {
           return std::make_tuple(IndexedPointMass<Is, T>(masses[Is])...);
       }(std::make_index_sequence<NumMasses>{});

       // Generate springs: IndexedSpring<edges[0].first, edges[0].second>, ...
       auto spring_components = [&]<size_t... Is>(std::index_sequence<Is...>) {
           return std::make_tuple(
               makeSpring<T, edges[Is].first, edges[Is].second>(params[Is])...
           );
       }(std::make_index_sequence<NumEdges>{});

       // Merge tuples and create system
       auto all_components = std::tuple_cat(mass_components, spring_components);
       return std::apply([](auto&&... comps) {
           return makeTypedODESystem<T>(std::forward<decltype(comps)>(comps)...);
       }, all_components);
   }
   ```

**Pros:**
- Natural matrix-like specification
- Scales to arbitrary connectivity
- Zero runtime overhead (all resolved at compile time)
- Type-safe (edges validated at compile time)

**Cons:**
- Complex template metaprogramming
- Compile-time cost for large graphs
- Limited to compile-time known connectivity

---

### Option 2: Variadic Connection Template

**Concept:** Explicitly list connections as template parameters.

```cpp
// Define connections in type system
template<size_t I, size_t J> struct Edge {};

// Specify system with connections
using MySystem = ConnectedSystem<
    double,
    NumMasses<3>,
    Edge<0, 1>,
    Edge<1, 2>,
    Edge<0, 2>
>;

auto system = MySystem::make({1.0, 1.5, 2.0}, {
    SpringParams{10.0, 1.0, 0.5},
    SpringParams{15.0, 1.0, 0.3},
    SpringParams{20.0, 1.5, 0.2}
});
```

**Implementation:**
```cpp
template<typename T, typename NumMasses, typename... Edges>
class ConnectedSystem {
    static constexpr size_t N = NumMasses::value;
    static constexpr size_t E = sizeof...(Edges);

    // Extract edge indices at compile time
    template<size_t... Is>
    static auto makeSystem(
        const std::array<double, N>& masses,
        const std::array<SpringParams, E>& params,
        std::index_sequence<Is...>
    ) {
        // Get Ith edge type
        using EdgeList = std::tuple<Edges...>;

        return makeTypedODESystem<T>(
            makeIndexedMasses<T>(masses, std::make_index_sequence<N>{}),
            makeIndexedSprings<T, Edges>(params[Is])...
        );
    }
};
```

**Pros:**
- Clear, explicit connectivity
- Type-safe edge specification
- Easier to debug (edges visible in type)

**Cons:**
- Verbose for dense graphs
- Less "matrix-like" than adjacency matrix
- Still complex metaprogramming

---

### Option 3: Builder Pattern with Compile-Time Validation

**Concept:** Fluent interface for building connectivity, validated at compile time.

```cpp
auto system = MassSystemBuilder<double>()
    .addMass<0>(1.0)
    .addMass<1>(1.5)
    .addMass<2>(2.0)
    .connect<0, 1>(SpringParams{10.0, 1.0, 0.5})
    .connect<1, 2>(SpringParams{15.0, 1.0, 0.3})
    .connect<0, 2>(SpringParams{20.0, 1.5, 0.2})
    .build();
```

**Implementation:**
```cpp
template<typename T, typename... Masses, typename... Springs>
class MassSystemBuilder {
    std::tuple<Masses...> masses;
    std::tuple<Springs...> springs;

public:
    // Add mass with compile-time index
    template<size_t Index>
    auto addMass(double mass) {
        auto new_mass = IndexedPointMass<Index, T>(mass);
        auto new_masses = std::tuple_cat(masses, std::make_tuple(new_mass));
        return MassSystemBuilder<T, Masses..., decltype(new_mass)>(
            new_masses, springs
        );
    }

    // Connect two masses
    template<size_t I, size_t J>
    auto connect(SpringParams params) {
        static_assert(I < sizeof...(Masses), "Mass I not added");
        static_assert(J < sizeof...(Masses), "Mass J not added");

        auto spring = IndexedSpring<I, J, T>(params.k, params.rest, params.damp);
        auto new_springs = std::tuple_cat(springs, std::make_tuple(spring));
        return MassSystemBuilder<T, Masses..., Springs..., decltype(spring)>(
            masses, new_springs
        );
    }

    // Build final system
    auto build() {
        auto all = std::tuple_cat(masses, springs);
        return std::apply([](auto&&... comps) {
            return makeTypedODESystem<T>(std::forward<decltype(comps)>(comps)...);
        }, all);
    }
};
```

**Pros:**
- Most readable and user-friendly
- Natural specification order
- Compile-time validation of references
- Easy to extend (add dampers, constraints, etc.)

**Cons:**
- Less "matrix-like" visually
- Builder state grows with each addition
- Compile times could be slower

---

### Option 4: Constexpr Adjacency Matrix Literal

**Concept:** Directly specify adjacency matrix, with spring properties in corresponding positions.

```cpp
// Symmetric adjacency matrix (0 = no connection, 1 = connection)
constexpr bool adjacency[3][3] = {
    {0, 1, 1},  // Mass 0 connects to Mass 1, Mass 2
    {1, 0, 1},  // Mass 1 connects to Mass 0, Mass 2
    {1, 1, 0}   // Mass 2 connects to Mass 0, Mass 1
};

// Spring properties for each connection (row, col order)
constexpr SpringParams springs[3][3] = {
    {{}, {10.0, 1.0, 0.5}, {20.0, 1.5, 0.2}},  // Row 0
    {{10.0, 1.0, 0.5}, {}, {15.0, 1.0, 0.3}},  // Row 1 (symmetric)
    {{20.0, 1.5, 0.2}, {15.0, 1.0, 0.3}, {}}   // Row 2 (symmetric)
};

auto system = makeMatrixSystem<double, 3>(
    {1.0, 1.5, 2.0},  // masses
    adjacency,
    springs
);
```

**Implementation:**
```cpp
template<typename T, size_t N>
auto makeMatrixSystem(
    const std::array<double, N>& masses,
    const bool (&adjacency)[N][N],
    const SpringParams (&springs)[N][N]
) {
    // Count edges (upper triangular only for undirected graph)
    constexpr size_t num_edges = [&]() {
        size_t count = 0;
        for (size_t i = 0; i < N; ++i)
            for (size_t j = i + 1; j < N; ++j)
                if (adjacency[i][j]) ++count;
        return count;
    }();

    // Extract edge list
    constexpr auto edges = [&]() {
        std::array<std::pair<size_t, size_t>, num_edges> result{};
        size_t idx = 0;
        for (size_t i = 0; i < N; ++i)
            for (size_t j = i + 1; j < N; ++j)
                if (adjacency[i][j])
                    result[idx++] = {i, j};
        return result;
    }();

    // Use Option 1's implementation
    return makeConnectedMassSystem<T, N, num_edges>(masses, edges, ...);
}
```

**Pros:**
- True matrix representation (requested by user!)
- Symmetric matrix natural for undirected connections
- Clear visualization of topology
- Can support directed graphs if needed

**Cons:**
- O(N²) storage (sparse graphs wasteful)
- Redundant specification for symmetric matrices
- More complex constexpr logic

---

## Proof of Concept: Option 1 Implementation

Let me create a working implementation of Option 1 (most general).

### File Structure
```
physics/
  connected_masses/
    indexed_tags.hpp         # MassTag<Index> generator
    indexed_point_mass.hpp   # Generic indexed mass
    indexed_spring.hpp       # Generic indexed spring
    connectivity_matrix.hpp  # Connection matrix builder
    tags.hpp                 # Tag categories
```

### Key Implementation Details

#### 1. Indexed Tag Generator (`indexed_tags.hpp`)
```cpp
template<size_t Index>
struct MassTag {
    static constexpr size_t index = Index;

    struct Position : categories::Kinematics {
        static constexpr std::string_view name() {
            return std::string_view("Mass") + std::to_string(Index) + "::Position";
        }
        static constexpr std::string_view symbol() {
            return std::string_view("x") + std::to_string(Index);
        }
        static constexpr std::string_view description() {
            return std::string_view("Position of mass ") + std::to_string(Index);
        }
    };

    // Similar for Velocity, Force, Mass
};
```

**Challenge:** `std::string_view` from `std::to_string` not constexpr.

**Solution:** Use template-based compile-time string construction:
```cpp
template<size_t N>
struct IndexString {
    static constexpr auto value = []() {
        // Convert N to string at compile time
        constexpr size_t digits = /* count digits */;
        std::array<char, digits + 1> str{};
        // Fill str with digits of N
        return str;
    }();
};

template<size_t Index>
struct Position : categories::Kinematics {
    static constexpr auto name_storage =
        concat("Mass", IndexString<Index>::value, "::Position");
    static constexpr std::string_view name() {
        return std::string_view(name_storage.data(), name_storage.size());
    }
};
```

#### 2. Generic Components

Masses and springs work identically to coupled oscillator, just with `MassTag<Index>`.

#### 3. System Generator

The magic happens here - convert edge list to component tuple:

```cpp
template<typename T, size_t NumMasses, size_t NumEdges>
auto makeConnectedMassSystem(
    const std::array<double, NumMasses>& masses,
    const std::array<std::pair<size_t, size_t>, NumEdges>& edges,
    const std::array<SpringParams, NumEdges>& spring_params
) {
    // Generate masses
    auto make_masses = [&]<size_t... Is>(std::index_sequence<Is...>) {
        return std::make_tuple(IndexedPointMass<Is, T>(masses[Is])...);
    };
    auto mass_tuple = make_masses(std::make_index_sequence<NumMasses>{});

    // Generate springs - THIS IS THE TRICKY PART
    // We need to create IndexedSpring<I, J> where I = edges[k].first, J = edges[k].second
    // But I and J must be template parameters (not runtime values)

    // Solution: Use nested template lambda + constexpr edge extraction
    auto make_springs = [&]<size_t... EdgeIndices>(std::index_sequence<EdgeIndices...>) {
        return std::make_tuple(
            makeSpringForEdge<T, EdgeIndices>(edges, spring_params)...
        );
    };
    auto spring_tuple = make_springs(std::make_index_sequence<NumEdges>{});

    // Combine and create system
    return std::apply([](auto&&... comps) {
        return makeTypedODESystem<T>(std::forward<decltype(comps)>(comps)...);
    }, std::tuple_cat(mass_tuple, spring_tuple));
}

// Helper to create spring for Kth edge
template<typename T, size_t K>
auto makeSpringForEdge(
    const std::array<std::pair<size_t, size_t>, N>& edges,
    const std::array<SpringParams, N>& params
) {
    // Extract I and J at compile time
    constexpr size_t I = edges[K].first;
    constexpr size_t J = edges[K].second;

    return IndexedSpring<I, J, T>(
        params[K].stiffness,
        params[K].rest_length,
        params[K].damping
    );
}
```

**Problem:** `constexpr size_t I = edges[K].first` requires `edges` to be constexpr.

**Solution:** Pass edges as template non-type parameter (C++20):

```cpp
template<typename T, auto Edges, auto SpringParams>
auto makeConnectedMassSystem(const std::array<double, Edges.size()>& masses);

// Usage:
constexpr auto edges = std::array<std::pair<size_t, size_t>, 3>{{
    {0, 1}, {1, 2}, {0, 2}
}};
constexpr auto springs = std::array<SpringParams, 3>{{ ... }};

auto system = makeConnectedMassSystem<double, edges, springs>(masses);
```

---

## Recommended Approach

**Use Option 1 (Adjacency Matrix with Edge List)** because:

1. **Most general:** Supports arbitrary connectivity (sparse or dense)
2. **Efficient:** O(E) storage for E edges, not O(N²)
3. **Matrix-like:** Edge list is the standard sparse matrix format
4. **Compatible:** Works with existing TypedComponent architecture
5. **Scalable:** Compile time grows with E, not N²

**Hybrid with Option 4:** Provide helper to convert adjacency matrix → edge list:

```cpp
// User can write matrix if preferred
constexpr bool matrix[3][3] = {{0, 1, 1}, {1, 0, 1}, {1, 1, 0}};
constexpr auto edges = matrixToEdges<3>(matrix);  // Constexpr conversion

// Or directly write edge list for sparse graphs
constexpr auto edges = std::array{{
    std::pair{0, 1}, std::pair{1, 2}, std::pair{0, 2}
}};
```

---

## Implementation Roadmap

### Phase 1: Core Infrastructure
- [ ] `physics/connected_masses/tags.hpp` - Tag categories
- [ ] `physics/connected_masses/indexed_tags.hpp` - `MassTag<Index>` generator
- [ ] `physics/connected_masses/indexed_point_mass.hpp` - Generic mass
- [ ] `physics/connected_masses/indexed_spring.hpp` - Generic spring
- [ ] Unit tests for individual components

### Phase 2: Connection Matrix Builder
- [ ] `physics/connected_masses/connectivity_matrix.hpp` - System generator
- [ ] `makeConnectedMassSystem()` function
- [ ] Edge list validation (no duplicates, no self-loops)
- [ ] Test with small graphs (2-4 masses)

### Phase 3: Matrix Helpers
- [ ] `matrixToEdges()` - Adjacency matrix → edge list conversion
- [ ] `validateSymmetry()` - Check undirected graph properties
- [ ] Support for directed graphs (asymmetric matrices)
- [ ] Test with dense graphs (N=10, fully connected)

### Phase 4: Advanced Features
- [ ] Heterogeneous connections (different spring types on different edges)
- [ ] Weighted adjacency matrix (spring stiffness in matrix)
- [ ] 3D mass positions (for spatial visualization)
- [ ] Energy monitor for arbitrary N masses

### Phase 5: Performance & Examples
- [ ] Benchmark compile times vs. manual wiring
- [ ] Example: 1D chain (N masses in a line)
- [ ] Example: 2D lattice (square grid of masses)
- [ ] Example: Complete graph (every mass connected)
- [ ] Documentation and user guide

---

## Open Questions

1. **String Concatenation:** How to generate tag names at compile time?
   - Option A: Preprocessor macros (ugly but works)
   - Option B: Custom constexpr string builder (complex)
   - Option C: Runtime names (defeats some type safety)

2. **Edge Validation:** How to prevent duplicate edges at compile time?
   - Check during `makeConnectedMassSystem()`?
   - Requires constexpr sorting/comparison
   - May increase compile time

3. **Scalability:** What's the practical limit on N and E?
   - Test compile times for N=10, N=100, N=1000
   - Measure template instantiation depth
   - Consider compilation memory usage

4. **Heterogeneity:** Should we support different component types?
   - Not just springs, but dampers, cables, constraints?
   - Would require connection type parameter
   - More complex type system

5. **Visualization:** How to help users visualize connectivity?
   - Compile-time DOT graph generation?
   - Runtime graph export to JSON?
   - Integration with visualization tools?

---

## Conclusion

The matrix-based connectivity approach is **feasible and maintainable** within SOPOT's architecture. Option 1 (edge list with indexed components) provides the best balance of:
- User-friendly specification (matrix-like)
- Compile-time efficiency (sparse graphs scale well)
- Type safety (all validation at compile time)
- Zero runtime overhead (template metaprogramming)

Next step: **Implement Phase 1** to validate the approach with a small working example.
