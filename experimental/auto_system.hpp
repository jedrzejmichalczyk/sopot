#pragma once

#include "system_builder.hpp"
#include "field_reflection.hpp"
#include "topological_sort.hpp"
#include <map>
#include <string>
#include <iostream>
#include <iomanip>
#include <stdexcept>

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

        // Compile-time cycle detection
        constexpr auto graph = DependencyGraphBuilder<Components...>::buildGraph();
        constexpr auto sortResult = topologicalSort<NumComponents>(graph);
        static_assert(!sortResult.hasCycle,
            "Circular dependency detected in component graph! Check component dependencies.");

        // Compute execution order via topological sort
        m_execOrder = getExecutionOrder<NumComponents>(m_graph);
        m_hasCycle = false;
    }

    // Main function: Compute all derivatives AUTOMATICALLY!
    auto computeDerivatives(T t, const std::array<T, TotalStateSize>& state) const {
        if (m_hasCycle) {
            throw std::runtime_error("Cannot compute derivatives: circular dependency detected!");
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
        printComponentInfoImpl<0>();

        std::cout << std::endl;
        printDependencyGraph();
        printExecutionOrder();
    }

private:
    std::array<std::array<bool, NumComponents>, NumComponents> m_graph;
    std::array<size_t, NumComponents> m_execOrder;
    bool m_hasCycle;

    // Helper: Print component info recursively
    template<size_t I>
    void printComponentInfoImpl() const {
        if constexpr (I < NumComponents) {
            using Comp = nth_type_t<I, Components...>;
            std::cout << "Component [" << I << "]:" << std::endl;
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

            printComponentInfoImpl<I + 1>();
        }
    }

    // Helper: Populate cache for component I
    template<size_t I>
    void populateCacheForComponent(
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache,
        size_t& offset
    ) const {
        using Comp = nth_type_t<I, Components...>;

        if constexpr (Comp::StateSize > 0) {
            if constexpr (Comp::StateSize == 1) {
                // Single-valued state
                T value = state[offset];

                // Add provisions based on state
                if constexpr (HasProvisions<Comp>) {
                    auto provs = getProvisionNames<Comp>();
                    if (provs.size() > 0) {
                        cache[std::string(provs[0])] = value;
                    }
                }
            }
            // Increment offset for ALL stateful components
            offset += Comp::StateSize;
        }
    }

    // Helper: Process all components recursively
    template<size_t I = 0>
    void populateCacheImpl(
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache,
        size_t& offset
    ) const {
        if constexpr (I < NumComponents) {
            populateCacheForComponent<I>(state, cache, offset);
            populateCacheImpl<I + 1>(state, cache, offset);
        }
    }

    // Populate cache with state-based provisions
    void populateCacheFromState(
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache
    ) const {
        size_t offset = 0;
        populateCacheImpl(state, cache, offset);
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

    // Execute a single component with automatic dependency injection
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
            // Stateless component - compute provisions and store in cache
            executeStateless<I, Comp>(comp, t, cache);
        } else {
            // Stateful component - compute derivative
            executeStateful<I, Comp>(comp, t, state, cache, derivatives);
        }
    }

    // Execute stateless component: inject dependencies, compute, store provisions
    template<size_t I, typename Comp>
    void executeStateless(const Comp& comp, T t, std::map<std::string, T>& cache) const {
        // Get dependency names
        constexpr auto depNames = getDependencyNames<Comp>();

        // Inject dependencies and call compute()
        auto result = injectAndCall(
            [&](auto... deps) { return comp.compute(t, deps...); },
            depNames,
            cache,
            std::make_index_sequence<depNames.size()>{}
        );

        // Store provisions in cache
        constexpr auto provNames = getProvisionNames<Comp>();
        storeProvisions<Comp>(result, provNames, cache, std::make_index_sequence<provNames.size()>{});
    }

    // Execute stateful component: compute derivative
    template<size_t I, typename Comp>
    void executeStateful(
        const Comp& comp,
        T t,
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache,
        std::array<T, TotalStateSize>& derivatives
    ) const {
        auto local = Base::template extractLocalState<I>(state);
        constexpr auto depNames = getDependencyNames<Comp>();

        // Inject dependencies and compute derivative
        auto deriv = injectAndCall(
            [&](auto... deps) { return comp.computeDerivative(t, local, deps...); },
            depNames,
            cache,
            std::make_index_sequence<depNames.size()>{}
        );

        // Store derivative
        constexpr size_t offset = Base::template computeStateOffset<I>();
        if constexpr (Comp::StateSize == 1) {
            derivatives[offset] = deriv;
        } else {
            // Multi-state component
            for (size_t i = 0; i < Comp::StateSize; ++i) {
                derivatives[offset + i] = deriv[i];
            }
        }
    }

    // Generic dependency injection and function calling
    template<typename Func, size_t N, size_t... Is>
    auto injectAndCall(
        Func&& func,
        const std::array<std::string_view, N>& depNames,
        const std::map<std::string, T>& cache,
        std::index_sequence<Is...>
    ) const {
        return func(cache.at(std::string(depNames[Is]))...);
    }

    // Store provisions from FieldBundle into cache
    template<typename Comp, typename ProvisionsBundle, size_t N, size_t... Is>
    void storeProvisions(
        const ProvisionsBundle& provs,
        const std::array<std::string_view, N>& provNames,
        std::map<std::string, T>& cache,
        std::index_sequence<Is...>
    ) const {
        ((cache[std::string(provNames[Is])] = provs.template get<Is>().value), ...);
    }

    // Helper: Execute component recursively
    template<size_t I = 0>
    void executeComponentImpl(
        size_t compIdx,
        T t,
        const std::array<T, TotalStateSize>& state,
        std::map<std::string, T>& cache,
        std::array<T, TotalStateSize>& derivatives
    ) const {
        if constexpr (I < NumComponents) {
            if (compIdx == I) {
                executeComponentTyped<I>(t, state, cache, derivatives);
            } else {
                executeComponentImpl<I + 1>(compIdx, t, state, cache, derivatives);
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
        executeComponentImpl(compIdx, t, state, cache, derivatives);
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
