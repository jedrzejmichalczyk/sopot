#pragma once

#include "field_based_component_simple.hpp"
#include "system_builder.hpp"
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

private:
    // Helper: Check if component I provides the field
    template<size_t I>
    static constexpr bool componentProvides(std::string_view fieldName, size_t& provider, size_t& count) {
        using Comp = nth_type_t<I, Components...>;
        auto provNames = getProvisionNames<Comp>();

        for (auto name : provNames) {
            if (name == fieldName) {
                provider = I;
                count++;
                return true;
            }
        }
        return false;
    }

    // Helper: Find provider recursively
    template<size_t I = 0>
    static constexpr void findProviderImpl(std::string_view fieldName, size_t& provider, size_t& count) {
        if constexpr (I < NumComponents) {
            componentProvides<I>(fieldName, provider, count);
            findProviderImpl<I + 1>(fieldName, provider, count);
        }
    }

public:
    // Find provider for a field name
    static constexpr size_t findProvider(std::string_view fieldName) {
        size_t provider = NotFound;
        size_t count = 0;

        findProviderImpl(fieldName, provider, count);

        if (count == 0) {
            return NotFound;  // No provider
        } else if (count > 1) {
            return NotFound - 1;  // Ambiguous (multiple providers)
        }

        return provider;
    }

private:
    // Helper: Check if a single component's dependencies are satisfied
    template<size_t I>
    static constexpr bool checkComponentDependencies() {
        using Comp = nth_type_t<I, Components...>;
        auto depNames = getDependencyNames<Comp>();

        for (auto name : depNames) {
            size_t provider = findProvider(name);
            if (provider == NotFound || provider == NotFound - 1) {
                return false;
            }
        }
        return true;
    }

    // Helper: Check all components recursively
    template<size_t I = 0>
    static constexpr bool checkAllDepsImpl() {
        if constexpr (I >= NumComponents) {
            return true;
        } else {
            if (!checkComponentDependencies<I>()) {
                return false;
            }
            return checkAllDepsImpl<I + 1>();
        }
    }

public:
    // Check if all dependencies are satisfied
    static constexpr bool allDependenciesSatisfied() {
        return checkAllDepsImpl();
    }
};

// ============================================================================
// DEPENDENCY GRAPH - Build adjacency matrix
// ============================================================================

template<typename... Components>
class DependencyGraphBuilder {
public:
    static constexpr size_t N = sizeof...(Components);

private:
    // Helper: Process dependencies for component I
    template<size_t I>
    static constexpr void processComponentDeps(std::array<std::array<bool, N>, N>& adj) {
        using Comp = nth_type_t<I, Components...>;
        auto depNames = getDependencyNames<Comp>();

        for (auto depName : depNames) {
            size_t provider = ProviderLookup<Components...>::findProvider(depName);
            if (provider < N) {
                adj[I][provider] = true;  // Component I depends on provider
            }
        }
    }

    // Helper: Process all components recursively
    template<size_t I = 0>
    static constexpr void buildGraphImpl(std::array<std::array<bool, N>, N>& adj) {
        if constexpr (I < N) {
            processComponentDeps<I>(adj);
            buildGraphImpl<I + 1>(adj);
        }
    }

public:
    // Build adjacency matrix: adj[i][j] = true if i depends on j
    static constexpr auto buildGraph() {
        std::array<std::array<bool, N>, N> adj{};
        buildGraphImpl(adj);
        return adj;
    }
};

}  // namespace sopot::experimental
