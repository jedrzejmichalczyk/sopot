#pragma once

/**
 * @file constexpr_topology.hpp
 * @brief Compile-time graph topology with static validation
 *
 * This provides:
 *   - O(K) template instantiations (K = number of component types)
 *   - Full compile-time state function resolution validation
 *   - Zero runtime validation overhead
 *
 * Usage:
 * @code
 * constexpr auto topology = makeTopology<3, 2>(
 *     nodes(mass(0), spring(0), mass(1)),
 *     edges(edge(1, 0, 0, 0), edge(1, 1, 2, 0))
 * );
 * static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(topology));
 * @endcode
 */

#include "state_functions.hpp"
#include "components.hpp"
#include <array>
#include <cstdint>

namespace sopot::unified {

// =============================================================================
// CONSTEXPR TOPOLOGY STRUCTURES
// =============================================================================

/**
 * @brief Compile-time node descriptor
 */
struct ConstexprNode {
    size_t type_index;    // Index into component type list
    size_t batch_index;   // Index within that type's batch
};

/**
 * @brief Compile-time edge descriptor
 */
struct ConstexprEdge {
    size_t node_a;
    size_t port_a;
    size_t node_b;
    size_t port_b;
};

/**
 * @brief Compile-time graph topology
 *
 * @tparam NumNodes Number of nodes in the graph
 * @tparam NumEdges Number of edges in the graph
 */
template<size_t NumNodes, size_t NumEdges>
struct GraphTopology {
    std::array<ConstexprNode, NumNodes> nodes{};
    std::array<ConstexprEdge, NumEdges> edges{};

    static constexpr size_t num_nodes = NumNodes;
    static constexpr size_t num_edges = NumEdges;
};

// =============================================================================
// TOPOLOGY BUILDER HELPERS
// =============================================================================

/**
 * @brief Create a node descriptor for a specific type and batch index
 */
constexpr ConstexprNode node(size_t type_index, size_t batch_index = 0) {
    return {type_index, batch_index};
}

/**
 * @brief Create an edge descriptor
 */
constexpr ConstexprEdge edge(size_t node_a, size_t port_a, size_t node_b, size_t port_b) {
    return {node_a, port_a, node_b, port_b};
}

/**
 * @brief Create a topology from arrays of nodes and edges
 */
template<size_t N, size_t E>
constexpr GraphTopology<N, E> makeTopology(
    const std::array<ConstexprNode, N>& nodes,
    const std::array<ConstexprEdge, E>& edges
) {
    GraphTopology<N, E> t;
    for (size_t i = 0; i < N; ++i) t.nodes[i] = nodes[i];
    for (size_t i = 0; i < E; ++i) t.edges[i] = edges[i];
    return t;
}

// =============================================================================
// COMPILE-TIME TYPE PROPERTIES
// =============================================================================

/**
 * @brief Get compile-time properties for a component type by index
 */
template<typename... Components>
struct TypeProperties {
    static constexpr size_t num_types = sizeof...(Components);

    // Arrays of properties for each type
    static constexpr std::array<size_t, num_types> state_sizes = {
        ComponentTraits<Components>::state_size...
    };
    static constexpr std::array<size_t, num_types> num_ports = {
        ComponentTraits<Components>::num_ports...
    };
    static constexpr std::array<uint32_t, num_types> required_masks = {
        ComponentTraits<Components>::required...
    };
    static constexpr std::array<uint32_t, num_types> provided_masks = {
        ComponentTraits<Components>::provided...
    };

    static constexpr size_t getStateSize(size_t type_idx) {
        return state_sizes[type_idx];
    }

    static constexpr size_t getNumPorts(size_t type_idx) {
        return num_ports[type_idx];
    }

    static constexpr uint32_t getRequired(size_t type_idx) {
        return required_masks[type_idx];
    }

    static constexpr uint32_t getProvided(size_t type_idx) {
        return provided_masks[type_idx];
    }
};

// =============================================================================
// CONSTEXPR ADJACENCY LIST (for O(N+E) validation)
// =============================================================================

/**
 * @brief Constexpr adjacency list using flat arrays
 *
 * Stores neighbors in a flat array with offsets for each node.
 * This allows O(1) access to neighbors and O(N+E) total traversal.
 */
template<size_t NumNodes, size_t NumEdges>
struct ConstexprAdjacency {
    // Each edge creates 2 neighbor entries (bidirectional)
    static constexpr size_t MaxNeighborEntries = NumEdges * 2;

    // Flat array of neighbor node indices
    std::array<size_t, MaxNeighborEntries> neighbors{};

    // For each node: [start_offset, end_offset) into neighbors array
    std::array<size_t, NumNodes + 1> offsets{};

    // Get neighbors for a node
    constexpr size_t neighborCount(size_t node) const {
        return offsets[node + 1] - offsets[node];
    }

    constexpr size_t getNeighbor(size_t node, size_t idx) const {
        return neighbors[offsets[node] + idx];
    }
};

/**
 * @brief Build adjacency list from topology in O(N + E) time
 */
template<size_t N, size_t E>
constexpr ConstexprAdjacency<N, E> buildAdjacency(const GraphTopology<N, E>& topology) {
    ConstexprAdjacency<N, E> adj;

    // Pass 1: Count neighbors for each node - O(E)
    std::array<size_t, N> neighbor_counts{};
    for (size_t e = 0; e < E; ++e) {
        neighbor_counts[topology.edges[e].node_a]++;
        neighbor_counts[topology.edges[e].node_b]++;
    }

    // Compute offsets (prefix sum) - O(N)
    adj.offsets[0] = 0;
    for (size_t i = 0; i < N; ++i) {
        adj.offsets[i + 1] = adj.offsets[i] + neighbor_counts[i];
    }

    // Pass 2: Fill neighbor array - O(E)
    // Use a temporary array to track current position for each node
    std::array<size_t, N> current_pos{};
    for (size_t i = 0; i < N; ++i) {
        current_pos[i] = adj.offsets[i];
    }

    for (size_t e = 0; e < E; ++e) {
        size_t a = topology.edges[e].node_a;
        size_t b = topology.edges[e].node_b;

        adj.neighbors[current_pos[a]++] = b;
        adj.neighbors[current_pos[b]++] = a;
    }

    return adj;
}

// =============================================================================
// COMPILE-TIME VALIDATION (Optimized O(N+E))
// =============================================================================

/**
 * @brief Result of graph validation with error information
 */
struct ValidationResult {
    bool valid = true;
    size_t error_node = 0;           // Node with unresolved dependency
    uint32_t missing_functions = 0;  // Bitmask of missing functions

    constexpr operator bool() const { return valid; }
};

/**
 * @brief Validate state function resolution at compile time - O(N + E)
 *
 * For each node, checks that all required state functions can be
 * provided by at least one connected neighbor.
 *
 * @tparam Components... The component types in order (type 0, type 1, ...)
 * @param topology The graph topology to validate
 * @return ValidationResult with success/failure and error details
 */
template<typename... Components, size_t N, size_t E>
constexpr ValidationResult validateStateFunctionResolution(
    const GraphTopology<N, E>& topology
) {
    using Props = TypeProperties<Components...>;

    // Build adjacency list once - O(E)
    auto adj = buildAdjacency(topology);

    // For each node, check requirements - O(N + E) total
    for (size_t i = 0; i < N; ++i) {
        const auto& node = topology.nodes[i];
        uint32_t required = Props::getRequired(node.type_index);

        if (required == 0) continue;  // No requirements to check

        // Collect provided functions from neighbors - O(degree of node)
        uint32_t available = 0;
        size_t num_neighbors = adj.neighborCount(i);

        for (size_t j = 0; j < num_neighbors; ++j) {
            size_t neighbor_idx = adj.getNeighbor(i, j);
            const auto& neighbor = topology.nodes[neighbor_idx];
            available |= Props::getProvided(neighbor.type_index);
        }

        // Check if all required functions are available
        uint32_t missing = required & ~available;
        if (missing != 0) {
            return {false, i, missing};
        }
    }

    return {true, 0, 0};
}

/**
 * @brief Validate port indices are within bounds
 */
template<typename... Components, size_t N, size_t E>
constexpr ValidationResult validatePortBounds(
    const GraphTopology<N, E>& topology
) {
    using Props = TypeProperties<Components...>;

    for (size_t e = 0; e < E; ++e) {
        const auto& edge = topology.edges[e];

        // Check node indices
        if (edge.node_a >= N || edge.node_b >= N) {
            return {false, edge.node_a >= N ? edge.node_a : edge.node_b, 0};
        }

        // Check port indices
        size_t max_ports_a = Props::getNumPorts(topology.nodes[edge.node_a].type_index);
        size_t max_ports_b = Props::getNumPorts(topology.nodes[edge.node_b].type_index);

        if (edge.port_a >= max_ports_a) {
            return {false, edge.node_a, 0};
        }
        if (edge.port_b >= max_ports_b) {
            return {false, edge.node_b, 0};
        }
    }

    return {true, 0, 0};
}

/**
 * @brief Validate type indices are within bounds
 */
template<typename... Components, size_t N, size_t E>
constexpr ValidationResult validateTypeIndices(
    const GraphTopology<N, E>& topology
) {
    constexpr size_t num_types = sizeof...(Components);

    for (size_t i = 0; i < N; ++i) {
        if (topology.nodes[i].type_index >= num_types) {
            return {false, i, 0};
        }
    }

    return {true, 0, 0};
}

/**
 * @brief Complete graph validation
 *
 * Validates:
 *   1. Type indices are valid
 *   2. Port indices are within bounds
 *   3. State function resolution is complete
 */
template<typename... Components, size_t N, size_t E>
constexpr ValidationResult validateGraph(
    const GraphTopology<N, E>& topology
) {
    // Check type indices
    auto type_result = validateTypeIndices<Components...>(topology);
    if (!type_result) return type_result;

    // Check port bounds
    auto port_result = validatePortBounds<Components...>(topology);
    if (!port_result) return port_result;

    // Check state function resolution
    return validateStateFunctionResolution<Components...>(topology);
}

/**
 * @brief Helper macro for static_assert with better error messages
 */
#define SOPOT_VALIDATE_GRAPH(topology, ...) \
    static_assert( \
        ::sopot::unified::validateGraph<__VA_ARGS__>(topology).valid, \
        "Graph validation failed: unresolved state function dependencies" \
    )

// =============================================================================
// GRID TOPOLOGY GENERATORS
// =============================================================================

/**
 * @brief Generate topology for a 2D grid of masses connected by springs
 *
 * Creates a grid where:
 *   - Type 0 = Mass (Rows * Cols nodes)
 *   - Type 1 = Spring (horizontal and vertical connections)
 *
 * @tparam Rows Number of rows
 * @tparam Cols Number of columns
 */
template<size_t Rows, size_t Cols>
constexpr auto makeGridTopology() {
    constexpr size_t NumMasses = Rows * Cols;
    constexpr size_t NumHSprings = Rows * (Cols - 1);      // Horizontal
    constexpr size_t NumVSprings = (Rows - 1) * Cols;      // Vertical
    constexpr size_t NumSprings = NumHSprings + NumVSprings;
    constexpr size_t NumNodes = NumMasses + NumSprings;
    constexpr size_t NumEdges = NumSprings * 2;  // Each spring connects 2 masses

    GraphTopology<NumNodes, NumEdges> t;

    // Add mass nodes (type 0)
    for (size_t i = 0; i < NumMasses; ++i) {
        t.nodes[i] = {0, i};
    }

    // Add spring nodes (type 1) and edges
    size_t spring_idx = NumMasses;
    size_t edge_idx = 0;

    // Horizontal springs
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < Cols - 1; ++c) {
            size_t mass_left = r * Cols + c;
            size_t mass_right = r * Cols + c + 1;

            t.nodes[spring_idx] = {1, spring_idx - NumMasses};

            // Spring port 0 -> left mass, Spring port 1 -> right mass
            t.edges[edge_idx++] = {spring_idx, 0, mass_left, 0};
            t.edges[edge_idx++] = {spring_idx, 1, mass_right, 1};

            spring_idx++;
        }
    }

    // Vertical springs
    for (size_t r = 0; r < Rows - 1; ++r) {
        for (size_t c = 0; c < Cols; ++c) {
            size_t mass_top = r * Cols + c;
            size_t mass_bottom = (r + 1) * Cols + c;

            t.nodes[spring_idx] = {1, spring_idx - NumMasses};

            // Spring port 0 -> top mass, Spring port 1 -> bottom mass
            t.edges[edge_idx++] = {spring_idx, 0, mass_top, 2};
            t.edges[edge_idx++] = {spring_idx, 1, mass_bottom, 3};

            spring_idx++;
        }
    }

    return t;
}

/**
 * @brief Generate topology for a 2D triangulated grid of masses
 *
 * Creates a grid where masses are connected by horizontal, vertical,
 * AND diagonal springs (X pattern in each cell). This provides more
 * stability and better approximates cloth-like behavior.
 *
 *   - Type 0 = Mass (Rows * Cols nodes)
 *   - Type 1 = Spring (horizontal + vertical + diagonal connections)
 *
 * @tparam Rows Number of rows
 * @tparam Cols Number of columns
 */
template<size_t Rows, size_t Cols>
constexpr auto makeTriangleGridTopology() {
    constexpr size_t NumMasses = Rows * Cols;
    constexpr size_t NumHSprings = Rows * (Cols - 1);           // Horizontal
    constexpr size_t NumVSprings = (Rows - 1) * Cols;           // Vertical
    constexpr size_t NumDiagSprings = 2 * (Rows - 1) * (Cols - 1);  // Diagonal (X pattern)
    constexpr size_t NumSprings = NumHSprings + NumVSprings + NumDiagSprings;
    constexpr size_t NumNodes = NumMasses + NumSprings;
    constexpr size_t NumEdges = NumSprings * 2;  // Each spring connects 2 masses

    GraphTopology<NumNodes, NumEdges> t;

    // Add mass nodes (type 0)
    for (size_t i = 0; i < NumMasses; ++i) {
        t.nodes[i] = {0, i};
    }

    // Add spring nodes (type 1) and edges
    size_t spring_idx = NumMasses;
    size_t edge_idx = 0;

    // Horizontal springs
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < Cols - 1; ++c) {
            size_t mass_left = r * Cols + c;
            size_t mass_right = r * Cols + c + 1;

            t.nodes[spring_idx] = {1, spring_idx - NumMasses};
            t.edges[edge_idx++] = {spring_idx, 0, mass_left, 0};
            t.edges[edge_idx++] = {spring_idx, 1, mass_right, 1};
            spring_idx++;
        }
    }

    // Vertical springs
    for (size_t r = 0; r < Rows - 1; ++r) {
        for (size_t c = 0; c < Cols; ++c) {
            size_t mass_top = r * Cols + c;
            size_t mass_bottom = (r + 1) * Cols + c;

            t.nodes[spring_idx] = {1, spring_idx - NumMasses};
            t.edges[edge_idx++] = {spring_idx, 0, mass_top, 2};
            t.edges[edge_idx++] = {spring_idx, 1, mass_bottom, 3};
            spring_idx++;
        }
    }

    // Diagonal springs (X pattern in each cell)
    for (size_t r = 0; r < Rows - 1; ++r) {
        for (size_t c = 0; c < Cols - 1; ++c) {
            size_t mass_tl = r * Cols + c;           // Top-left
            size_t mass_tr = r * Cols + c + 1;       // Top-right
            size_t mass_bl = (r + 1) * Cols + c;     // Bottom-left
            size_t mass_br = (r + 1) * Cols + c + 1; // Bottom-right

            // Diagonal: top-left to bottom-right
            t.nodes[spring_idx] = {1, spring_idx - NumMasses};
            t.edges[edge_idx++] = {spring_idx, 0, mass_tl, 0};
            t.edges[edge_idx++] = {spring_idx, 1, mass_br, 1};
            spring_idx++;

            // Diagonal: top-right to bottom-left
            t.nodes[spring_idx] = {1, spring_idx - NumMasses};
            t.edges[edge_idx++] = {spring_idx, 0, mass_tr, 0};
            t.edges[edge_idx++] = {spring_idx, 1, mass_bl, 1};
            spring_idx++;
        }
    }

    return t;
}

/**
 * @brief Generate topology for a chain of masses connected by springs
 *
 * @tparam NumMasses Number of masses in the chain
 */
template<size_t NumMasses>
constexpr auto makeChainTopology() {
    constexpr size_t NumSprings = NumMasses - 1;
    constexpr size_t NumNodes = NumMasses + NumSprings;
    constexpr size_t NumEdges = NumSprings * 2;

    GraphTopology<NumNodes, NumEdges> t;

    // Add mass nodes (type 0)
    for (size_t i = 0; i < NumMasses; ++i) {
        t.nodes[i] = {0, i};
    }

    // Add spring nodes (type 1) and edges
    for (size_t i = 0; i < NumSprings; ++i) {
        size_t spring_node = NumMasses + i;
        t.nodes[spring_node] = {1, i};

        t.edges[i * 2] = {spring_node, 0, i, 0};
        t.edges[i * 2 + 1] = {spring_node, 1, i + 1, 0};
    }

    return t;
}

} // namespace sopot::unified
