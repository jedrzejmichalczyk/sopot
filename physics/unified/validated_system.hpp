#pragma once

/**
 * @file validated_system.hpp
 * @brief Compile-time validated graph system
 *
 * Combines:
 *   - Constexpr topology with static_assert validation
 *   - O(K) template instantiations
 *   - Runtime execution with compile-time guarantees
 */

#include "constexpr_topology.hpp"
#include "graph_system.hpp"
#include <utility>

namespace sopot::unified {

// =============================================================================
// VALIDATED SYSTEM
// =============================================================================

/**
 * @brief A graph system with compile-time validated topology
 *
 * @tparam T Scalar type
 * @tparam Topology The constexpr topology (GraphTopology<N, E>)
 * @tparam Components... Component types in order
 */
template<Scalar T, auto Topology, typename... Components>
class ValidatedSystem {
public:
    using TopologyType = decltype(Topology);
    static constexpr size_t num_nodes = TopologyType::num_nodes;
    static constexpr size_t num_edges = TopologyType::num_edges;
    static constexpr size_t num_types = sizeof...(Components);

    // Compile-time validation
    static_assert(validateGraph<Components...>(Topology).valid,
        "Graph validation failed: check state function dependencies, type indices, and port bounds");

private:
    // Batches for each component type
    std::tuple<Batch<Components>...> m_batches;

    // Runtime graph structure (built from constexpr topology)
    std::vector<NodeDesc> m_nodes;
    std::vector<Edge> m_edges;
    std::vector<std::vector<std::vector<Neighbor>>> m_adjacency;

    size_t m_total_states = 0;

    // Helper to get state size for a type index at compile time
    template<size_t I>
    static constexpr size_t getStateSizeForType() {
        if constexpr (I < num_types) {
            using CompType = std::tuple_element_t<I, std::tuple<Components...>>;
            return ComponentTraits<CompType>::state_size;
        }
        return 0;
    }

    // Helper to get num ports for a type index at compile time
    template<size_t I>
    static constexpr size_t getNumPortsForType() {
        if constexpr (I < num_types) {
            using CompType = std::tuple_element_t<I, std::tuple<Components...>>;
            return ComponentTraits<CompType>::num_ports;
        }
        return 0;
    }

    // Runtime lookup tables (computed from compile-time data)
    static constexpr std::array<size_t, num_types> state_sizes = {
        ComponentTraits<Components>::state_size...
    };
    static constexpr std::array<size_t, num_types> port_counts = {
        ComponentTraits<Components>::num_ports...
    };

public:
    ValidatedSystem() {
        initializeFromTopology();
    }

    /**
     * @brief Construct with pre-populated batches
     */
    explicit ValidatedSystem(Batch<Components>... batches)
        : m_batches(std::move(batches)...)
    {
        initializeFromTopology();
    }

    // -------------------------------------------------------------------------
    // Batch accessors for adding components
    // -------------------------------------------------------------------------

    template<size_t TypeIndex>
    Batch<std::tuple_element_t<TypeIndex, std::tuple<Components...>>>& getBatch() {
        return std::get<TypeIndex>(m_batches);
    }

    template<size_t TypeIndex>
    const Batch<std::tuple_element_t<TypeIndex, std::tuple<Components...>>>& getBatch() const {
        return std::get<TypeIndex>(m_batches);
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    size_t getStateDimension() const { return m_total_states; }
    size_t getNodeCount() const { return m_nodes.size(); }
    size_t getEdgeCount() const { return m_edges.size(); }

    // -------------------------------------------------------------------------
    // ODE System interface
    // -------------------------------------------------------------------------

    std::vector<T> getInitialState() const {
        std::vector<T> state(m_total_states);

        for (size_t i = 0; i < m_nodes.size(); ++i) {
            const auto& node = m_nodes[i];
            if (node.state_size > 0) {
                std::span<T> node_state(state.data() + node.state_offset, node.state_size);
                visitNode(node, [&](const auto& component) {
                    component.initState(node_state);
                });
            }
        }

        return state;
    }

    std::vector<T> computeDerivatives(T t, const std::vector<T>& state) const {
        return computeDerivatives(t, std::span<const T>(state));
    }

    std::vector<T> computeDerivatives(T /*t*/, std::span<const T> state) const {
        std::vector<T> derivs(m_total_states, T(0));
        std::vector<NodeData<T>> node_data(m_nodes.size());

        // Phase 1: Export values from all nodes
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            const auto& node = m_nodes[i];
            std::span<const T> local_state;
            if (node.state_size > 0) {
                local_state = std::span<const T>(state.data() + node.state_offset, node.state_size);
            }

            visitNode(node, [&](const auto& component) {
                component.exportValues(local_state, node_data[i]);
            });
        }

        // Phase 2: Compute forces and torques from stateless 2-port components
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            const auto& node = m_nodes[i];

            visitNode(node, [&](const auto& component) {
                using CompType = std::decay_t<decltype(component)>;

                if constexpr (CompType::state_size == 0 &&
                              CompType::num_ports == 2 &&
                              hasFunction<Force2D>(CompType::provided)) {

                    if (m_adjacency[i][0].empty() || m_adjacency[i][1].empty()) return;

                    size_t neighbor0 = m_adjacency[i][0][0].node;
                    size_t neighbor1 = m_adjacency[i][1][0].node;

                    // Check if this component also provides torque
                    if constexpr (hasFunction<Torque>(CompType::provided)) {
                        // Use computeForcesAndTorques for components with rotational coupling
                        auto result = component.computeForcesAndTorques(
                            node_data[neighbor0], node_data[neighbor1]);

                        node_data[neighbor0].addForce(result.force_on_0);
                        node_data[neighbor1].addForce(result.force_on_1);
                        node_data[neighbor0].addTorque(result.torque_on_0);
                        node_data[neighbor1].addTorque(result.torque_on_1);
                    } else {
                        // Legacy path for components without torque
                        auto forces = component.computeForces(
                            node_data[neighbor0], node_data[neighbor1]);

                        node_data[neighbor0].addForce(forces[0]);
                        node_data[neighbor1].addForce(forces[1]);
                    }
                }
            });
        }

        // Phase 2b: Compute forces from 1-port components
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            const auto& node = m_nodes[i];

            visitNode(node, [&](const auto& component) {
                using CompType = std::decay_t<decltype(component)>;

                if constexpr (CompType::state_size == 0 &&
                              CompType::num_ports == 1 &&
                              hasFunction<Force2D>(CompType::provided)) {

                    if (m_adjacency[i][0].empty()) return;

                    size_t neighbor = m_adjacency[i][0][0].node;

                    if constexpr (requires { component.computeForce(node_data[neighbor]); }) {
                        auto force = component.computeForce(node_data[neighbor]);
                        node_data[neighbor].addForce(force);
                    }
                }
            });
        }

        // Phase 3: Compute derivatives for stateful nodes
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            const auto& node = m_nodes[i];

            if (node.state_size > 0) {
                std::span<const T> local_state(state.data() + node.state_offset, node.state_size);

                visitNode(node, [&](const auto& component) {
                    if constexpr (requires { component.computeDerivatives(local_state, node_data[i]); }) {
                        auto d = component.computeDerivatives(local_state, node_data[i]);
                        for (size_t j = 0; j < d.size(); ++j) {
                            derivs[node.state_offset + j] = d[j];
                        }
                    }
                });
            }
        }

        return derivs;
    }

private:
    void initializeFromTopology() {
        m_nodes.reserve(num_nodes);
        m_edges.reserve(num_edges);
        m_adjacency.resize(num_nodes);

        // Create nodes from topology
        for (size_t i = 0; i < num_nodes; ++i) {
            const auto& topo_node = Topology.nodes[i];

            NodeDesc node;
            node.type_index = topo_node.type_index;
            node.batch_index = topo_node.batch_index;
            node.state_offset = m_total_states;
            node.state_size = state_sizes[topo_node.type_index];

            m_total_states += node.state_size;
            m_nodes.push_back(node);

            // Initialize adjacency for this node's ports
            m_adjacency[i].resize(port_counts[topo_node.type_index]);
        }

        // Create edges from topology
        for (size_t i = 0; i < num_edges; ++i) {
            const auto& topo_edge = Topology.edges[i];

            m_edges.push_back({
                topo_edge.node_a, topo_edge.port_a,
                topo_edge.node_b, topo_edge.port_b
            });

            // Bidirectional adjacency
            m_adjacency[topo_edge.node_a][topo_edge.port_a].push_back(
                {topo_edge.node_b, topo_edge.port_b}
            );
            m_adjacency[topo_edge.node_b][topo_edge.port_b].push_back(
                {topo_edge.node_a, topo_edge.port_a}
            );
        }
    }

    template<typename Func>
    void visitNode(const NodeDesc& node, Func&& func) const {
        visitNodeImpl<0>(node.type_index, node.batch_index, std::forward<Func>(func));
    }

    template<size_t I, typename Func>
    void visitNodeImpl(size_t type_index, size_t batch_index, Func&& func) const {
        if constexpr (I < num_types) {
            if (type_index == I) {
                func(std::get<I>(m_batches).get(batch_index));
            } else {
                visitNodeImpl<I + 1>(type_index, batch_index, std::forward<Func>(func));
            }
        }
    }
};

// =============================================================================
// FACTORY FUNCTIONS
// =============================================================================

/**
 * @brief Create a validated system from a constexpr topology
 *
 * Usage:
 * @code
 * constexpr auto topology = makeGridTopology<5, 5>();
 *
 * // This static_asserts at compile time!
 * auto system = makeValidatedSystem<double, topology,
 *     PointMass2D<double>,
 *     Spring2D<double>
 * >();
 *
 * // Populate batches
 * for (size_t i = 0; i < 25; ++i) {
 *     system.getBatch<0>().add(PointMass2D<double>(1.0, {x, y}));
 * }
 * for (size_t i = 0; i < 40; ++i) {
 *     system.getBatch<1>().add(Spring2D<double>(100.0, 0.5, 0.1));
 * }
 * @endcode
 */
template<Scalar T, auto Topology, typename... Components>
auto makeValidatedSystem() {
    return ValidatedSystem<T, Topology, Components...>();
}

/**
 * @brief Create a validated system with pre-populated batches
 */
template<Scalar T, auto Topology, typename... Components>
auto makeValidatedSystem(Batch<Components>... batches) {
    return ValidatedSystem<T, Topology, Components...>(std::move(batches)...);
}

} // namespace sopot::unified
