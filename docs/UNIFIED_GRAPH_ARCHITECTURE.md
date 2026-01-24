# Unified Graph Architecture

## Core Insight

**Everything is a node. Edges are pure connectivity. State function resolution determines dataflow.**

This document describes the unified graph architecture for compile-time physics simulation in SOPOT.

## The Problem with Traditional Approaches

Traditional physics engines distinguish between:
- **Nodes**: Things with state (masses, rigid bodies)
- **Edges**: Things that connect nodes (springs, constraints)

This creates an artificial asymmetry and leads to:
- Separate code paths for "nodes" vs "edges"
- Complex type hierarchies
- O(N²) template instantiations when encoding topology in types

## The Unified Model

In the unified model:

```
┌─────────────────────────────────────────────────────────────────┐
│                    EVERYTHING IS A NODE                         │
│                                                                 │
│  [Mass0] ←─edge─→ [Spring] ←─edge─→ [Mass1]                    │
│     ↓                                    ↓                      │
│  [Gravity]                          [Gravity]                   │
│                                                                 │
│  Masses, springs, gravity sources - ALL are nodes.             │
│  Edges are pure connectivity (wires between ports).            │
└─────────────────────────────────────────────────────────────────┘
```

### Key Concepts

#### 1. Nodes

Every component in the system is a node. Nodes have:

- **State** (optional): Variables that evolve over time (e.g., position, velocity)
- **Ports**: Connection points for edges (a spring has 2 ports, a mass has N ports)
- **Required Functions**: State functions the node needs from neighbors
- **Provided Functions**: State functions the node gives to neighbors

#### 2. Edges

Edges are pure connectivity - they carry no physics, only define "who can talk to whom":

```
Edge = (node_a, port_a, node_b, port_b)
```

An edge says: "port_a of node_a is connected to port_b of node_b"

#### 3. State Functions

State functions are the "currency" that flows between nodes:

| Function | Type | Description |
|----------|------|-------------|
| Position2D | `array<T,2>` | Position in 2D space |
| Velocity2D | `array<T,2>` | Velocity in 2D space |
| Force2D | `array<T,2>` | Force vector (additive) |
| MassValue | `T` | Scalar mass |
| Angle | `T` | Angular position |
| AngularVelocity | `T` | Angular velocity |
| Torque | `T` | Torque (additive) |

#### 4. State Function Resolution

At system construction, we verify that every required function can be satisfied:

```
For each node N:
    For each required function F:
        Find neighbor M (via edges) that provides F
        If no such M exists: ERROR - unresolved dependency
```

## Component Specifications

### PointMass2D

```
State: [x, y, vx, vy]  (4 variables)
Ports: N (configurable, default 4)

REQUIRES:
  - Force2D (accumulated from all neighbors that provide it)

PROVIDES:
  - Position2D
  - Velocity2D
  - MassValue

Dynamics:
  dx/dt = vx
  dy/dt = vy
  dvx/dt = Fx / m
  dvy/dt = Fy / m
```

### Spring2D

```
State: none (stateless)
Ports: 2 (connects exactly two neighbors)

REQUIRES (from each port):
  - Position2D
  - Velocity2D

PROVIDES (to each port):
  - Force2D

Physics:
  F = k * (|p1 - p0| - L0) * unit_vector + c * relative_velocity
  Force on port 0: +F (toward port 1 when stretched)
  Force on port 1: -F (Newton's 3rd law)
```

### Damper2D

```
State: none
Ports: 2

REQUIRES: Position2D, Velocity2D (from both ports)
PROVIDES: Force2D (to both ports)

Physics:
  F = c * (v1 - v0) · unit_vector * unit_vector
```

### GravitySource2D

```
State: none
Ports: 1

REQUIRES: MassValue (from port 0)
PROVIDES: Force2D (to port 0)

Physics:
  F = m * g
```

### Pendulum2D

```
State: [theta, omega]  (2 variables)
Ports: N (can connect to springs, dampers, etc.)

REQUIRES: Force2D, Torque
PROVIDES: Position2D, Velocity2D, MassValue, Angle, AngularVelocity

Kinematics:
  x = pivot_x + L * sin(theta)
  y = pivot_y - L * cos(theta)
  vx = L * omega * cos(theta)
  vy = L * omega * sin(theta)

Dynamics:
  d(theta)/dt = omega
  d(omega)/dt = (torque + force_tangent * L) / (m * L²)
```

## Computation Algorithm

### Phase 1: Export Values from Stateful Nodes

```python
for node in nodes:
    if node.state_size > 0:
        if node.provides(Position2D):
            node_values[node].position = node.getPosition(state)
        if node.provides(Velocity2D):
            node_values[node].velocity = node.getVelocity(state)
        if node.provides(MassValue):
            node_values[node].mass = node.getMass()
```

### Phase 2: Compute Outputs from Stateless Nodes

```python
for node in nodes:
    if node.state_size == 0 and node.provides(Force2D):
        # Gather inputs from neighbors
        neighbor_values = []
        for port in node.ports:
            neighbor = get_neighbor(node, port)
            neighbor_values.append(node_values[neighbor])

        # Compute force outputs
        force_outputs = node.computeOutputs(neighbor_values)

        # Distribute forces to neighbors
        for port, force in enumerate(force_outputs):
            neighbor = get_neighbor(node, port)
            node_values[neighbor].force += force
```

### Phase 3: Compute Derivatives for Stateful Nodes

```python
for node in nodes:
    if node.state_size > 0:
        local_state = state[node.offset : node.offset + node.state_size]
        derivs[node.offset:] = node.computeDerivatives(local_state, node_values[node])
```

## Compile-Time Implementation

For maximum performance, we use compile-time batching:

### Type Structure

```cpp
// All components of the same TYPE go in one batch
template<Scalar T, size_t NumMasses>
class PointMassBatch { ... };

template<Scalar T, size_t NumSprings, auto Edges>
class SpringBatch { ... };

// The system holds one batch per component type
template<Scalar T, typename... Batches>
class UnifiedGraphSystem {
    std::tuple<Batches...> m_batches;
    // Graph topology stored as constexpr or runtime data
};
```

### Complexity Analysis

| Aspect | Old (per-instance types) | Unified (per-type batches) |
|--------|-------------------------|---------------------------|
| Component types | O(N + E) | O(K) |
| Template instantiations | O(N² × E) | O(K) |
| Compile time | Exponential | Linear |
| Runtime overhead | Zero | Zero |

Where:
- N = number of nodes
- E = number of edges
- K = number of distinct component types (typically 3-10)

## Example: Spring-Mass Grid

```cpp
// 100x100 grid = 10,000 masses + ~20,000 springs
// Old approach: ~30,000 component types
// Unified approach: 2 component types (PointMassBatch, SpringBatch)

auto system = makeUnifiedSystem<double>(
    PointMassBatch<double, 10000>(...),
    SpringBatch<double, 20000, edges>(...)
);

// Graph topology
for (auto [spring_idx, mass_i, mass_j] : connections) {
    system.connect(spring_node(spring_idx), 0, mass_node(mass_i));
    system.connect(spring_node(spring_idx), 1, mass_node(mass_j));
}
```

## Validation

At construction time, the system validates:

1. **Port count**: Each node has the expected number of ports
2. **Connection completeness**: Required ports are connected
3. **Function resolution**: Every required function has a provider

```cpp
bool validateResolution() {
    for (auto& node : nodes) {
        uint32_t required = node.getRequiredFunctions();
        uint32_t available = 0;

        for (auto& neighbor : getNeighbors(node)) {
            available |= neighbor.getProvidedFunctions();
        }

        if ((required & ~available) != 0) {
            throw UnresolvedDependency(node, required & ~available);
        }
    }
    return true;
}
```

## Extensibility

Adding a new component type:

1. Define the component class with `state_size`, `num_ports`
2. Implement `getRequiredFunctions()`, `getProvidedFunctions()`
3. Implement `computeOutputs()` and/or `computeDerivatives()`
4. Create a batch class for compile-time optimization

The architecture automatically handles:
- Integration into the computation pipeline
- State function resolution
- Force/torque accumulation
- Derivative computation

## Summary

The unified graph architecture provides:

1. **Conceptual clarity**: No artificial node/edge distinction
2. **Generality**: Any physics component is just a node
3. **Compile-time optimization**: O(K) types instead of O(N)
4. **Validation**: Dependency resolution catches errors early
5. **Extensibility**: New components plug in cleanly
