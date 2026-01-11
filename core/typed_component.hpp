#pragma once

#include "state_function_tags.hpp"
#include "scalar.hpp"
#include <array>
#include <vector>
#include <span>
#include <type_traits>
#include <concepts>
#include <stdexcept>
#include <string_view>

namespace sopot {

// Forward declarations
template<size_t N, Scalar T> class TypedComponent;

//=============================================================================
// Component Concepts - All compile-time verification
//=============================================================================

// Basic component structure requirements
template<typename C>
concept TypedComponentConcept = requires {
    typename C::scalar_type;
    typename C::LocalState;
    typename C::LocalDerivative;
    { C::state_size } -> std::convertible_to<size_t>;
};

// Check if component has derivatives method (CRTP-style, non-virtual)
// This is the required interface for components with state
template<typename Component, typename T, typename Registry>
concept HasDerivativesMethod = requires(
    const Component& c,
    T t,
    std::span<const T> local,
    std::span<const T> global,
    const Registry& registry
) {
    { c.derivatives(t, local, global, registry) } -> std::same_as<typename Component::LocalDerivative>;
};

// Check if component has getInitialLocalState (non-virtual)
template<typename Component>
concept HasInitialState = requires(const Component& c) {
    { c.getInitialLocalState() } -> std::same_as<typename Component::LocalState>;
};

// Check if component has identification methods (non-virtual)
template<typename Component>
concept HasIdentification = requires(const Component& c) {
    { c.getComponentType() } -> std::convertible_to<std::string_view>;
    { c.getComponentName() } -> std::convertible_to<std::string_view>;
};

// Complete component concept - all required interfaces
template<typename C, typename T, typename Registry>
concept CompleteTypedComponent =
    TypedComponentConcept<C> &&
    HasInitialState<C> &&
    HasIdentification<C> &&
    (C::state_size == 0 || HasDerivativesMethod<C, T, Registry>);

// Check if component provides a specific state function with span (preferred)
template<typename Component, typename Tag, typename T>
concept TypedProvidesStateFunctionSpan = TypedComponentConcept<Component> &&
    StateTagConcept<Tag> &&
    requires(const Component& c, std::span<const T> state) {
        { c.compute(Tag{}, state) };
    };

// Check if component provides registry-aware state function (span)
template<typename Component, typename Tag, typename T, typename Registry>
concept TypedProvidesRegistryAwareStateFunctionSpan = TypedComponentConcept<Component> &&
    StateTagConcept<Tag> &&
    requires(const Component& c, std::span<const T> state, const Registry& reg) {
        { c.compute(Tag{}, state, reg) };
    };

// Combined: component provides state function (simple or registry-aware)
template<typename Component, typename Tag, typename T, typename Registry>
concept TypedProvidesStateFunction =
    TypedProvidesStateFunctionSpan<Component, Tag, T> ||
    TypedProvidesRegistryAwareStateFunctionSpan<Component, Tag, T, Registry>;

//=============================================================================
// query<Tag>() - Simplified state function access
//=============================================================================
// Free function that eliminates the need for 'registry.template' syntax.
//
// Usage:
//   // Old (verbose - 'template' keyword required for dependent type):
//   auto vel = registry.template computeFunction<kinematics::VelocityENU>(state);
//
//   // New (clean - no 'template' keyword needed):
//   auto vel = query<kinematics::VelocityENU>(registry, state);
//
// Why this works: Free functions don't need the 'template' disambiguator
// because they're not members of a dependent type.
//=============================================================================

// Concept: Registry provides state function for Tag
template<typename Registry, typename Tag>
concept RegistryProvides = requires {
    { Registry::template hasFunction<Tag>() } -> std::convertible_to<bool>;
} && Registry::template hasFunction<Tag>();

// Query a state function from a registry (span version)
template<StateTagConcept Tag, typename Registry, typename T>
    requires RegistryProvides<Registry, Tag>
inline auto query(const Registry& registry, std::span<const T> state) {
    return registry.template computeFunction<Tag>(state);
}

// Query a state function from a registry (vector version)
template<StateTagConcept Tag, typename Registry, typename T>
    requires RegistryProvides<Registry, Tag>
inline auto query(const Registry& registry, const std::vector<T>& state) {
    return registry.template computeFunction<Tag>(state);
}

//=============================================================================
// TypedComponent - Non-virtual base class for components
//=============================================================================
// All dispatch is resolved at compile time through concepts and templates.
// No virtual functions - components must provide required methods directly.
//
// Required methods for components:
//   - derivatives(t, local_span, global_span, registry) -> LocalDerivative
//     (only required if state_size > 0)
//   - getInitialLocalState() -> LocalState
//   - getComponentType() -> std::string_view
//   - getComponentName() -> std::string_view
//   - compute(Tag{}, state) or compute(Tag{}, state, registry) for state functions
//
// This base class provides:
//   - Type aliases (scalar_type, LocalState, LocalDerivative)
//   - State offset management
//   - Helper functions for state access
//=============================================================================

template<size_t StateSize, Scalar T = double>
class TypedComponent {
public:
    using scalar_type = T;
    static constexpr size_t state_size = StateSize;
    using LocalState = ScalarState<T, StateSize>;
    using LocalDerivative = ScalarState<T, StateSize>;

    // No virtual destructor needed - no polymorphic deletion through base pointer
    ~TypedComponent() = default;

    // State management
    size_t getStateOffset() const noexcept { return m_state_offset; }
    void setStateOffset(size_t offset) noexcept { m_state_offset = offset; }

    // Alias for CRTP-style components that use setOffset
    void setOffset(size_t offset) noexcept { m_state_offset = offset; }

protected:
    size_t m_state_offset{0};

    // Helper for accessing global state from span
    T getGlobalState(std::span<const T> global_state, size_t index) const {
        size_t actual_index = m_state_offset + index;
        return global_state[actual_index];
    }

    // Helper to extract local state from global state span
    LocalState extractLocalState(std::span<const T> global_state) const {
        LocalState local;
        for (size_t i = 0; i < StateSize; ++i) {
            local[i] = global_state[m_state_offset + i];
        }
        return local;
    }
};

//=============================================================================
// TypedRegistry - Compile-time registry for state function dispatch
//=============================================================================
// All state function resolution happens at compile time.
// Registry-aware compute() methods take precedence over simple compute().
//=============================================================================

template<typename T, TypedComponentConcept... Components>
class TypedRegistry {
    std::tuple<const Components&...> m_components;

    // Self type for concept checks
    using Self = TypedRegistry<T, Components...>;

    // Find first component that provides a given function type (compile-time)
    // CRITICAL: Must return by reference (decltype(auto)) to avoid copying components!
    template<StateTagConcept Tag, size_t I = 0>
    constexpr decltype(auto) findProvider() const {
        if constexpr (I < sizeof...(Components)) {
            using ComponentType = std::tuple_element_t<I, std::tuple<const Components&...>>;
            using DecayedType = std::decay_t<ComponentType>;

            if constexpr (TypedProvidesStateFunction<DecayedType, Tag, T, Self>) {
                return std::get<I>(m_components);
            } else {
                return findProvider<Tag, I + 1>();
            }
        } else {
            return std::get<0>(m_components); // Dummy return (static_assert will catch)
        }
    }

public:
    explicit constexpr TypedRegistry(const Components&... components)
        : m_components(components...) {}

    // Compile-time function availability check
    template<StateTagConcept Tag>
    static constexpr bool hasFunction() {
        return (TypedProvidesStateFunction<Components, Tag, T, Self> || ...);
    }

    // Zero-overhead function dispatch (span interface only)
    // Registry-aware compute() takes precedence over simple compute()
    template<StateTagConcept Tag>
    auto computeFunction(std::span<const T> state) const {
        static_assert(hasFunction<Tag>(), "No component provides this state function");
        const auto& provider = findProvider<Tag>();
        using ProviderType = std::decay_t<decltype(provider)>;

        // Prefer registry-aware compute over simple compute
        if constexpr (TypedProvidesRegistryAwareStateFunctionSpan<ProviderType, Tag, T, Self>) {
            return provider.compute(Tag{}, state, *this);
        } else {
            return provider.compute(Tag{}, state);
        }
    }

    // Convenience overload for vector - converts to span
    template<StateTagConcept Tag>
    auto computeFunction(const std::vector<T>& state) const {
        return computeFunction<Tag>(std::span<const T>(state));
    }

    static constexpr size_t component_count() {
        return sizeof...(Components);
    }

    template<size_t I>
    constexpr const auto& getComponent() const {
        return std::get<I>(m_components);
    }
};

//=============================================================================
// TypedODESystem - Compile-time ODE system composition
//=============================================================================
// Composes multiple components into an ODE system.
// All dispatch is resolved at compile time - no virtual functions.
//
// Components must provide:
//   - derivatives(t, local_span, global_span, registry) -> LocalDerivative
//     (only required if state_size > 0)
//   - getInitialLocalState() -> LocalState
//=============================================================================

template<typename T, TypedComponentConcept... Components>
class TypedODESystem {
private:
    std::tuple<Components...> m_components;
    TypedRegistry<T, Components...> m_registry;

    static constexpr size_t m_total_state_size = (Components::state_size + ...);
    static constexpr size_t m_component_count = sizeof...(Components);
    using RegistryType = TypedRegistry<T, Components...>;

    // Initialize component state offsets
    template<size_t I = 0, size_t Offset = 0>
    constexpr void initializeOffsets() {
        if constexpr (I < sizeof...(Components)) {
            auto& component = std::get<I>(m_components);
            component.setStateOffset(Offset);
            component.setOffset(Offset);  // TypedComponent provides this
            constexpr size_t NextOffset = Offset +
                std::tuple_element_t<I, std::tuple<Components...>>::state_size;
            initializeOffsets<I + 1, NextOffset>();
        }
    }

    // Collect derivatives from all components using compile-time dispatch
    template<size_t I = 0>
    void collectDerivatives(std::vector<T>& derivatives, T t, const std::vector<T>& state) const {
        if constexpr (I < sizeof...(Components)) {
            using ComponentType = std::tuple_element_t<I, std::tuple<Components...>>;
            const auto& component = std::get<I>(m_components);
            constexpr size_t local_size = ComponentType::state_size;
            constexpr size_t off = offset<I>();

            if constexpr (local_size > 0) {
                // Create spans for local and global state
                std::span<const T> local_span(state.data() + off, local_size);
                std::span<const T> global_span(state);

                // Compile-time requirement: component must have derivatives method
                static_assert(
                    HasDerivativesMethod<ComponentType, T, RegistryType>,
                    "Component with state_size > 0 must provide derivatives(t, local, global, registry)"
                );

                auto local_derivs = component.derivatives(t, local_span, global_span, m_registry);
                for (size_t j = 0; j < local_size; ++j) {
                    derivatives[off + j] = local_derivs[j];
                }
            }

            collectDerivatives<I + 1>(derivatives, t, state);
        }
    }

    // Compile-time offset calculation
    template<size_t I>
    static constexpr size_t offset() {
        if constexpr (I == 0) return 0;
        else return offset<I-1>() + std::tuple_element_t<I-1, std::tuple<Components...>>::state_size;
    }

    // Collect initial states from all components
    template<size_t I = 0>
    void collectInitialStates(std::vector<T>& state) const {
        if constexpr (I < sizeof...(Components)) {
            const auto& component = std::get<I>(m_components);
            constexpr size_t local_size =
                std::tuple_element_t<I, std::tuple<Components...>>::state_size;

            if constexpr (local_size > 0) {
                auto local_state = component.getInitialLocalState();
                size_t off = component.getStateOffset();

                for (size_t j = 0; j < local_state.size; ++j) {
                    state[off + j] = local_state[j];
                }
            }

            collectInitialStates<I + 1>(state);
        }
    }

public:
    using scalar_type = T;

    explicit TypedODESystem(Components... components)
        : m_components(std::move(components)...)
        , m_registry(std::get<Components>(m_components)...) {
        initializeOffsets();
    }

    // Core ODE interface
    std::vector<T> computeDerivatives(T t, const std::vector<T>& state) const {
        std::vector<T> derivatives(m_total_state_size);
        collectDerivatives(derivatives, t, state);
        return derivatives;
    }

    static constexpr size_t getStateDimension() noexcept {
        return m_total_state_size;
    }

    std::vector<T> getInitialState() const {
        std::vector<T> state(m_total_state_size);
        collectInitialStates(state);
        return state;
    }

    // State function access
    template<StateTagConcept Tag>
    static constexpr bool hasFunction() {
        return RegistryType::template hasFunction<Tag>();
    }

    template<StateTagConcept Tag>
    auto computeStateFunction(const std::vector<T>& state) const {
        return m_registry.template computeFunction<Tag>(state);
    }

    template<StateTagConcept Tag>
    auto computeStateFunction(std::span<const T> state) const {
        return m_registry.template computeFunction<Tag>(state);
    }

    // Batch function evaluation
    template<StateTagConcept... Tags>
    auto computeStateFunctions(const std::vector<T>& state) const {
        static_assert(sizeof...(Tags) > 0, "Must specify at least one function");
        static_assert((hasFunction<Tags>() && ...),
                     "All requested functions must be available");
        return std::tuple{m_registry.template computeFunction<Tags>(state)...};
    }

    // Component access
    template<size_t I>
    requires (I < sizeof...(Components))
    constexpr const auto& getComponent() const {
        return std::get<I>(m_components);
    }

    static constexpr size_t getComponentCount() noexcept {
        return m_component_count;
    }

    const auto& getRegistry() const {
        return m_registry;
    }

    // Convert state to values (for output)
    std::vector<double> stateValues(const std::vector<T>& state) const {
        std::vector<double> values(state.size());
        for (size_t i = 0; i < state.size(); ++i) {
            values[i] = value_of(state[i]);
        }
        return values;
    }
};

// Factory function
template<typename T = double, TypedComponentConcept... Components>
auto makeTypedODESystem(Components&&... components) {
    return TypedODESystem<T, std::decay_t<Components>...>(
        std::forward<Components>(components)...
    );
}

// Helper to compute Jacobian using autodiff
// Returns a matrix of ∂f_i/∂x_j where f = derivatives
template<size_t N, TypedComponentConcept... Components>
std::array<std::array<double, N>, N> computeJacobian(
    const TypedODESystem<Dual<double, N>, Components...>& system,
    double t,
    const std::array<double, N>& state_values
) {
    static_assert(N == (Components::state_size + ...), "State size mismatch");

    // Create state with derivatives set up for each variable
    std::vector<Dual<double, N>> state(N);
    for (size_t i = 0; i < N; ++i) {
        state[i] = Dual<double, N>::variable(state_values[i], i);
    }

    // Compute derivatives
    auto derivs = system.computeDerivatives(Dual<double, N>::constant(t), state);

    // Extract Jacobian
    std::array<std::array<double, N>, N> jacobian;
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            jacobian[i][j] = derivs[i].derivative(j);
        }
    }

    return jacobian;
}

} // namespace sopot
