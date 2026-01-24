#pragma once

/**
 * @file graph_system.hpp
 * @brief Compile-time unified graph system
 *
 * This is the fully compile-time implementation with:
 *   - Zero virtual function overhead
 *   - O(K) template instantiations (K = number of component types)
 *   - State function resolution at construction time
 *   - Batched processing for each component type
 */

#include "state_functions.hpp"
#include "components.hpp"
#include "core/scalar.hpp"
#include <vector>
#include <array>
#include <tuple>
#include <span>
#include <stdexcept>

namespace sopot::unified {

// =============================================================================
// GRAPH TOPOLOGY
// =============================================================================

/**
 * @brief Node descriptor in the graph
 */
struct NodeDesc {
    size_t type_index;    // Index into component type list
    size_t batch_index;   // Index within that type's batch
    size_t state_offset;  // Offset into global state vector
    size_t state_size;    // Number of state variables
};

/**
 * @brief Edge connecting two ports
 */
struct Edge {
    size_t node_a, port_a;
    size_t node_b, port_b;
};

/**
 * @brief Neighbor info for adjacency lookup
 */
struct Neighbor {
    size_t node;
    size_t their_port;
};

// =============================================================================
// COMPONENT BATCH
// =============================================================================

/**
 * @brief Batch of components of the same type
 *
 * @tparam Component The component type (e.g., PointMass2D<double>)
 */
template<typename Component>
class Batch {
    std::vector<Component> m_components;

public:
    using component_type = Component;
    using Traits = ComponentTraits<Component>;

    static constexpr size_t state_size = Traits::state_size;
    static constexpr size_t num_ports = Traits::num_ports;
    static constexpr uint32_t required = Traits::required;
    static constexpr uint32_t provided = Traits::provided;

    size_t add(Component comp) {
        size_t idx = m_components.size();
        m_components.push_back(std::move(comp));
        return idx;
    }

    const Component& get(size_t index) const { return m_components[index]; }
    Component& get(size_t index) { return m_components[index]; }

    size_t size() const { return m_components.size(); }
};

// =============================================================================
// UNIFIED GRAPH SYSTEM
// =============================================================================

/**
 * @brief Compile-time unified graph system
 *
 * @tparam T Scalar type (double or Dual for autodiff)
 * @tparam Batches... Component batch types
 */
template<Scalar T, typename... Batches>
class UnifiedGraphSystem {
public:
    using BatchTuple = std::tuple<Batches...>;
    static constexpr size_t num_types = sizeof...(Batches);

private:
    BatchTuple m_batches;

    // Graph structure
    std::vector<NodeDesc> m_nodes;
    std::vector<Edge> m_edges;

    // Adjacency: m_adjacency[node][port] = list of neighbors
    std::vector<std::vector<std::vector<Neighbor>>> m_adjacency;

    size_t m_total_states = 0;

public:
    explicit UnifiedGraphSystem(Batches... batches)
        : m_batches(std::move(batches)...)
    {}

    // -------------------------------------------------------------------------
    // Graph construction
    // -------------------------------------------------------------------------

    /**
     * @brief Add a node from a specific batch
     * @tparam TypeIndex Index into Batches parameter pack
     * @param batch_index Index within that batch
     * @return Global node index
     */
    template<size_t TypeIndex>
    size_t addNode(size_t batch_index) {
        static_assert(TypeIndex < num_types, "Invalid type index");

        using BatchType = std::tuple_element_t<TypeIndex, BatchTuple>;

        NodeDesc node;
        node.type_index = TypeIndex;
        node.batch_index = batch_index;
        node.state_offset = m_total_states;
        node.state_size = BatchType::state_size;

        m_total_states += node.state_size;

        size_t idx = m_nodes.size();
        m_nodes.push_back(node);

        // Initialize adjacency for this node's ports
        m_adjacency.push_back(std::vector<std::vector<Neighbor>>(BatchType::num_ports));

        return idx;
    }

    /**
     * @brief Connect two nodes via their ports
     */
    void connect(size_t node_a, size_t port_a, size_t node_b, size_t port_b) {
        m_edges.push_back({node_a, port_a, node_b, port_b});

        // Bidirectional adjacency
        m_adjacency[node_a][port_a].push_back({node_b, port_b});
        m_adjacency[node_b][port_b].push_back({node_a, port_a});
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    size_t getStateDimension() const { return m_total_states; }
    size_t getNodeCount() const { return m_nodes.size(); }
    size_t getEdgeCount() const { return m_edges.size(); }

    template<size_t TypeIndex>
    const auto& getBatch() const { return std::get<TypeIndex>(m_batches); }

    // -------------------------------------------------------------------------
    // ODE System interface
    // -------------------------------------------------------------------------

    std::vector<T> getInitialState() const {
        std::vector<T> state(m_total_states);

        for (const auto& node : m_nodes) {
            if (node.state_size > 0) {
                std::span<T> node_state(state.data() + node.state_offset, node.state_size);
                visitNode(node, [&](const auto& component) {
                    component.initState(node_state);
                });
            }
        }

        return state;
    }

    // Overload for vector (calls span version)
    std::vector<T> computeDerivatives(T t, const std::vector<T>& state) const {
        return computeDerivatives(t, std::span<const T>(state));
    }

    std::vector<T> computeDerivatives(T /*t*/, std::span<const T> state) const {
        std::vector<T> derivs(m_total_states, T(0));

        // Storage for node data
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

        // Phase 2: Compute forces from stateless 2-port components (springs, dampers)
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            const auto& node = m_nodes[i];

            visitNode(node, [&](const auto& component) {
                using CompType = std::decay_t<decltype(component)>;

                // Check if this is a 2-port force provider
                if constexpr (CompType::state_size == 0 &&
                              CompType::num_ports == 2 &&
                              hasFunction<Force2D>(CompType::provided)) {

                    // Get neighbors at port 0 and port 1
                    if (m_adjacency[i][0].empty() || m_adjacency[i][1].empty()) return;

                    size_t neighbor0 = m_adjacency[i][0][0].node;
                    size_t neighbor1 = m_adjacency[i][1][0].node;

                    auto forces = component.computeForces(node_data[neighbor0], node_data[neighbor1]);

                    node_data[neighbor0].addForce(forces[0]);
                    node_data[neighbor1].addForce(forces[1]);
                }
            });
        }

        // Phase 2b: Compute forces from 1-port components (gravity)
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
    // -------------------------------------------------------------------------
    // Compile-time dispatch helpers
    // -------------------------------------------------------------------------

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
// FACTORY FUNCTION
// =============================================================================

/**
 * @brief Create a unified graph system
 *
 * Usage:
 * @code
 * Batch<PointMass2D<double>> masses;
 * Batch<Spring2D<double>> springs;
 *
 * auto m0 = masses.add(PointMass2D<double>(1.0, {0, 0}));
 * auto m1 = masses.add(PointMass2D<double>(1.0, {1, 0}));
 * auto s0 = springs.add(Spring2D<double>(100.0, 1.0));
 *
 * auto system = makeUnifiedGraphSystem<double>(std::move(masses), std::move(springs));
 *
 * auto node_m0 = system.addNode<0>(m0);  // Type 0 = masses
 * auto node_m1 = system.addNode<0>(m1);
 * auto node_s0 = system.addNode<1>(s0);  // Type 1 = springs
 *
 * system.connect(node_s0, 0, node_m0, 0);
 * system.connect(node_s0, 1, node_m1, 0);
 * @endcode
 */
template<Scalar T, typename... Batches>
auto makeUnifiedGraphSystem(Batches... batches) {
    return UnifiedGraphSystem<T, Batches...>(std::move(batches)...);
}

} // namespace sopot::unified
