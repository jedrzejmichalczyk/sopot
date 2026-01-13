#pragma once

#include "field_based_component_simple.hpp"
#include <array>
#include <algorithm>
#include <utility>
#include <iostream>

namespace sopot::experimental {

// ============================================================================
// TYPE-LEVEL UTILITIES
// ============================================================================

// Get the nth type from a parameter pack
template<size_t N, typename... Ts>
struct nth_type;

template<size_t N, typename T, typename... Ts>
struct nth_type<N, T, Ts...> : nth_type<N-1, Ts...> {};

template<typename T, typename... Ts>
struct nth_type<0, T, Ts...> { using type = T; };

template<size_t N, typename... Ts>
using nth_type_t = typename nth_type<N, Ts...>::type;

// ============================================================================
// FIELD MATCHING - Does component provide this field?
// ============================================================================

// Check if a FieldBundle contains a specific field
template<typename Field, typename FieldBundle>
struct ContainsField;

template<typename Field, typename... Fields>
struct ContainsField<Field, FieldBundle<Fields...>> {
    static constexpr bool value = (std::same_as<Field, Fields> || ...);
};

// Check if component provides a field
template<typename Component, typename Field>
concept ProvidesField = HasProvisions<Component> &&
    ContainsField<Field, typename Component::Provides>::value;

// Check if component depends on a field
template<typename Component, typename Field>
concept DependsOnField = HasDependencies<Component> &&
    ContainsField<Field, typename Component::Dependencies>::value;

// ============================================================================
// DEPENDENCY GRAPH - Compile-time adjacency matrix
// ============================================================================

template<typename... Components>
class DependencyGraph {
public:
    static constexpr size_t NumComponents = sizeof...(Components);

    // Find which component provides a given field
    template<typename Field>
    static constexpr int findProvider() {
        int provider = -1;
        size_t count = 0;

        // Check each component
        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;
                if constexpr (ProvidesField<Comp, Field>) {
                    provider = static_cast<int>(Is);
                    count++;
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});

        if (count == 0) {
            // No provider found - will cause compile error with helpful message
            return -1;
        } else if (count > 1) {
            // Multiple providers - ambiguous!
            return -2;
        }

        return provider;
    }

    // Build adjacency matrix: adj[i][j] = true if component i depends on component j
    static constexpr auto buildAdjacencyMatrix() {
        std::array<std::array<bool, NumComponents>, NumComponents> adj{};

        // For each component i
        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;

                // Check each dependency
                if constexpr (HasDependencies<Comp>) {
                    checkDependencies<Is, Comp>(adj);
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});

        return adj;
    }

private:
    // NOTE: Placeholder method - not yet implemented
    // TODO: Implement dependency extraction and adjacency matrix population
    template<size_t I, typename Comp, typename Dependencies = typename Comp::Dependencies>
    static constexpr void checkDependencies(
        std::array<std::array<bool, NumComponents>, NumComponents>& adj
    ) {
        // TODO: Extract fields from Dependencies bundle
        // TODO: Build adjacency matrix based on component dependencies
        // Currently handled in DependencyGraphBuilder in field_reflection.hpp
        (void)adj;  // Suppress unused parameter warning
    }
};

// ============================================================================
// SYSTEM - Compose components with automatic dependency resolution
// ============================================================================

template<typename T, typename... Components>
class System {
public:
    using Scalar = T;
    static constexpr size_t NumComponents = sizeof...(Components);

    // Store components
    std::tuple<Components...> components;

    // Constructor
    explicit System(Components... comps) : components(comps...) {}

    // Total state size
    static constexpr size_t totalStateSize() {
        return (Components::StateSize + ...);
    }

    // Get component by index
    template<size_t I>
    constexpr auto& getComponent() {
        return std::get<I>(components);
    }

    template<size_t I>
    constexpr const auto& getComponent() const {
        return std::get<I>(components);
    }

    // Initial state
    auto getInitialState() const {
        std::array<T, totalStateSize()> state{};
        size_t offset = 0;

        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                const auto& comp = std::get<Is>(components);
                constexpr size_t stateSize = nth_type_t<Is, Components...>::StateSize;

                if constexpr (stateSize > 0) {
                    // Component has state - get initial values
                    if constexpr (requires { comp.getInitialState(); }) {
                        auto initialState = comp.getInitialState();
                        if constexpr (std::is_arithmetic_v<decltype(initialState)>) {
                            // Single value
                            state[offset] = initialState;
                        } else {
                            // Array/vector - copy all
                            for (size_t i = 0; i < stateSize; ++i) {
                                state[offset + i] = initialState[i];
                            }
                        }
                    }
                    offset += stateSize;
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});

        return state;
    }

    // Extract local state for component I from global state
    template<size_t I>
    auto extractLocalState(const std::array<T, totalStateSize()>& globalState) const {
        constexpr size_t stateSize = nth_type_t<I, Components...>::StateSize;
        constexpr size_t offset = computeStateOffset<I>();

        if constexpr (stateSize == 0) {
            return T{};  // Stateless
        } else if constexpr (stateSize == 1) {
            return globalState[offset];  // Single value
        } else {
            std::array<T, stateSize> local;
            for (size_t i = 0; i < stateSize; ++i) {
                local[i] = globalState[offset + i];
            }
            return local;
        }
    }

    // Compute state offset for component I
    template<size_t I>
    static constexpr size_t computeStateOffset() {
        size_t offset = 0;
        [&]<size_t... Js>(std::index_sequence<Js...>) {
            ([&] {
                if (Js < I) {
                    offset += nth_type_t<Js, Components...>::StateSize;
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});
        return offset;
    }

    // Component info (for debugging)
    void printInfo() const {
        std::cout << "System with " << NumComponents << " components:" << std::endl;
        std::cout << "  Total state size: " << totalStateSize() << std::endl;

        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;
                std::cout << "  [" << Is << "] StateSize=" << Comp::StateSize;

                if constexpr (HasDependencies<Comp>) {
                    std::cout << " (has dependencies)";
                }
                if constexpr (HasProvisions<Comp>) {
                    std::cout << " (has provisions)";
                }
                std::cout << std::endl;
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});
    }
};

// ============================================================================
// FACTORY FUNCTION
// ============================================================================

template<typename T, typename... Components>
auto makeSystem(Components... components) {
    return System<T, Components...>(components...);
}

}  // namespace sopot::experimental
