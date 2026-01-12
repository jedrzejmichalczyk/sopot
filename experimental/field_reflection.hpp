#pragma once

#include "field_based_component_simple.hpp"
#include <string_view>
#include <array>

namespace sopot::experimental {

// ============================================================================
// FIELD NAME EXTRACTION - Compile-time reflection
// ============================================================================

// Extract field name from Field<Tag, Name>
template<typename Field>
struct GetFieldName;

template<typename Tag, FixedString Name>
struct GetFieldName<Field<Tag, Name>> {
    static constexpr auto value = Name;
    static constexpr std::string_view view() { return Name.view(); }
};

// Extract all field names from FieldBundle
template<typename Bundle>
struct ExtractFieldNames;

template<typename... Fields>
struct ExtractFieldNames<FieldBundle<Fields...>> {
    static constexpr size_t count = sizeof...(Fields);

    static constexpr auto names() {
        return std::array<std::string_view, count>{
            GetFieldName<Fields>::view()...
        };
    }
};

// Helper: Get dependencies of a component as array of names
template<typename Component>
constexpr auto getDependencyNames() {
    if constexpr (HasDependencies<Component>) {
        return ExtractFieldNames<typename Component::Dependencies>::names();
    } else {
        return std::array<std::string_view, 0>{};
    }
}

// Helper: Get provisions of a component as array of names
template<typename Component>
constexpr auto getProvisionNames() {
    if constexpr (HasProvisions<Component>) {
        return ExtractFieldNames<typename Component::Provides>::names();
    } else {
        return std::array<std::string_view, 0>{};
    }
}

// ============================================================================
// PROVIDER LOOKUP - Find which component provides a field
// ============================================================================

template<typename... Components>
class ProviderLookup {
public:
    static constexpr size_t NumComponents = sizeof...(Components);
    static constexpr size_t NotFound = static_cast<size_t>(-1);

    // Find provider for a field name
    static constexpr size_t findProvider(std::string_view fieldName) {
        size_t provider = NotFound;
        size_t count = 0;

        // Check each component
        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;
                auto provNames = getProvisionNames<Comp>();

                for (auto name : provNames) {
                    if (name == fieldName) {
                        provider = Is;
                        count++;
                        break;
                    }
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});

        if (count == 0) {
            return NotFound;  // No provider
        } else if (count > 1) {
            return NotFound - 1;  // Ambiguous (multiple providers)
        }

        return provider;
    }

    // Check if all dependencies are satisfied
    static constexpr bool allDependenciesSatisfied() {
        bool satisfied = true;

        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;
                auto depNames = getDependencyNames<Comp>();

                for (auto name : depNames) {
                    if (findProvider(name) == NotFound ||
                        findProvider(name) == NotFound - 1) {
                        satisfied = false;
                    }
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});

        return satisfied;
    }
};

// ============================================================================
// DEPENDENCY GRAPH - Build adjacency matrix
// ============================================================================

template<typename... Components>
class DependencyGraphBuilder {
public:
    static constexpr size_t N = sizeof...(Components);

    // Build adjacency matrix: adj[i][j] = true if i depends on j
    static constexpr auto buildGraph() {
        std::array<std::array<bool, N>, N> adj{};

        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;
                auto depNames = getDependencyNames<Comp>();

                for (auto depName : depNames) {
                    size_t provider = ProviderLookup<Components...>::findProvider(depName);
                    if (provider < N) {
                        adj[Is][provider] = true;  // Component Is depends on provider
                    }
                }
            }(), ...);
        }(std::make_index_sequence<N>{});

        return adj;
    }
};

}  // namespace sopot::experimental
