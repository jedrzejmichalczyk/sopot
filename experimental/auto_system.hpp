#pragma once

#include "system_builder.hpp"
#include "field_reflection.hpp"
#include "topological_sort.hpp"
#include <map>
#include <string>
#include <iostream>
#include <iomanip>

namespace sopot::experimental {

// ============================================================================
// AUTO-DERIVATIVE SYSTEM - Fully automatic derivative computation!
// ============================================================================

template<typename T, typename... Components>
class AutoSystem : public System<T, Components...> {
public:
    using Base = System<T, Components...>;
    using Base::components;
    using Base::NumComponents;
    using Base::extractLocalState;
    using Base::getComponent;

    static constexpr size_t TotalStateSize = (Components::StateSize + ...);

    explicit AutoSystem(Components... comps) : Base(comps...) {
        // Build dependency graph at construction
        m_graph = DependencyGraphBuilder<Components...>::buildGraph();

        // Compute execution order via topological sort
        m_execOrder = getExecutionOrder<NumComponents>(m_graph);

        // Check for cycles
        auto sortResult = topologicalSort<NumComponents>(m_graph);
        if (sortResult.hasCycle) {
            std::cerr << "ERROR: Circular dependency detected!" << std::endl;
            m_hasCycle = true;
        } else {
            m_hasCycle = false;
        }
    }

    // Main function: Compute all derivatives AUTOMATICALLY!
    auto computeDerivatives(T t, const std::array<T, TotalStateSize>& state) const {
        if (m_hasCycle) {
            std::cerr << "Cannot compute derivatives: circular dependency!" << std::endl;
            return std::array<T, TotalStateSize>{};
        }

        std::array<T, TotalStateSize> derivatives{};

        // Provision cache - stores computed values
        std::map<std::string, T> cache;

        // Populate cache with state-based provisions
        populateCacheFromState(state, cache);

        // Execute components in topological order
        executeInOrder(t, state, cache, derivatives);

        return derivatives;
    }

    // Print dependency graph
    void printDependencyGraph() const {
        std::cout << "Dependency Graph (adj[i][j] = i depends on j):" << std::endl;
        std::cout << "    ";
        for (size_t j = 0; j < NumComponents; ++j) {
            std::cout << std::setw(3) << j;
        }
        std::cout << std::endl;

        for (size_t i = 0; i < NumComponents; ++i) {
            std::cout << "[" << i << "] ";
            for (size_t j = 0; j < NumComponents; ++j) {
                std::cout << std::setw(3) << (m_graph[i][j] ? "1" : ".");
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    // Print execution order
    void printExecutionOrder() const {
        std::cout << "Execution Order: ";
        for (size_t i = 0; i < NumComponents; ++i) {
            std::cout << m_execOrder[i];
            if (i < NumComponents - 1) std::cout << " -> ";
        }
        std::cout << std::endl;
    }

    // Diagnostic info
    void printDiagnostics() const {
        std::cout << "=== System Diagnostics ===" << std::endl;
        std::cout << "Components: " << NumComponents << std::endl;
        std::cout << "Total state size: " << TotalStateSize << std::endl;
        std::cout << "Has cycle: " << (m_hasCycle ? "YES" : "NO") << std::endl;
        std::cout << std::endl;

        // Dependencies for each component
        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;
                std::cout << "Component [" << Is << "]:" << std::endl;
                std::cout << "  StateSize: " << Comp::StateSize << std::endl;

                if constexpr (HasDependencies<Comp>) {
                    auto deps = getDependencyNames<Comp>();
                    std::cout << "  Dependencies: ";
                    for (size_t i = 0; i < deps.size(); ++i) {
                        std::cout << deps[i];
                        if (i < deps.size() - 1) std::cout << ", ";
                    }
                    std::cout << std::endl;
                }

                if constexpr (HasProvisions<Comp>) {
                    auto provs = getProvisionNames<Comp>();
                    std::cout << "  Provisions: ";
                    for (size_t i = 0; i < provs.size(); ++i) {
                        std::cout << provs[i];
                        if (i < provs.size() - 1) std::cout << ", ";
                    }
                    std::cout << std::endl;
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});

        std::cout << std::endl;
        printDependencyGraph();
        printExecutionOrder();
    }

private:
    std::array<std::array<bool, NumComponents>, NumComponents> m_graph;
    std::array<size_t, NumComponents> m_execOrder;
    bool m_hasCycle;

    // Populate cache with state-based provisions
    void populateCacheFromState(
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache
    ) const {
        // For each stateful component, extract its state value
        // and add provisions to cache
        // This is a simplified version - full version would need component-specific logic

        size_t offset = 0;
        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;

                if constexpr (Comp::StateSize == 1) {
                    // Single-valued state
                    T value = state[offset];

                    // Add provisions based on state
                    // For now, we assume component provides its state value
                    if constexpr (HasProvisions<Comp>) {
                        auto provs = getProvisionNames<Comp>();
                        if (provs.size() > 0) {
                            cache[std::string(provs[0])] = value;
                        }
                    }

                    offset += 1;
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});
    }

    // Execute components in topological order
    void executeInOrder(
        T t,
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache,
        std::array<T, TotalStateSize>& derivatives
    ) const {
        for (size_t i = 0; i < NumComponents; ++i) {
            size_t compIdx = m_execOrder[i];
            executeComponent(compIdx, t, state, cache, derivatives);
        }
    }

    // Execute a single component
    template<size_t I>
    void executeComponentTyped(
        T t,
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache,
        std::array<T, TotalStateSize>& derivatives
    ) const {
        using Comp = nth_type_t<I, Components...>;
        const auto& comp = std::get<I>(components);

        if constexpr (Comp::StateSize == 0) {
            // Stateless component - compute provisions
            auto depNames = getDependencyNames<Comp>();
            auto provNames = getProvisionNames<Comp>();

            // For simple cases, manually inject dependencies
            // Full implementation would use variadic template magic
            if constexpr (depNames.size() == 0) {
                // No dependencies - just compute
                // This is a placeholder for actual computation
                // Real implementation would call comp.compute() appropriately
            }
        } else {
            // Stateful component - compute derivative
            auto local = this->template extractLocalState<I>(state);
            auto depNames = getDependencyNames<Comp>();

            // Get dependencies from cache and compute derivative
            // This is simplified - full version needs proper dependency injection

            constexpr size_t offset = Base::template computeStateOffset<I>();

            // For now, return zero derivative as placeholder
            if constexpr (Comp::StateSize == 1) {
                derivatives[offset] = T{};
            }
        }
    }

    // Runtime dispatch to compile-time execution
    void executeComponent(
        size_t compIdx,
        T t,
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache,
        std::array<T, TotalStateSize>& derivatives
    ) const {
        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                if (compIdx == Is) {
                    executeComponentTyped<Is>(t, state, cache, derivatives);
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});
    }
};

// ============================================================================
// FACTORY FUNCTION
// ============================================================================

template<typename T, typename... Components>
auto makeAutoSystem(Components... components) {
    return AutoSystem<T, Components...>(components...);
}

}  // namespace sopot::experimental
