#pragma once

#include "state_function_tags.hpp"
#include "scalar.hpp"
#include <array>
#include <vector>
#include <span>
#include <type_traits>
#include <concepts>
#include <stdexcept>

namespace sopot {

// Forward declarations
template<size_t N, Scalar T> class TypedComponent;

// Component concept for compile-time validation
template<typename C>
concept TypedComponentConcept = requires {
    typename C::scalar_type;
    typename C::LocalState;
    typename C::LocalDerivative;
    C::state_size;
};

// Check if component provides a specific state function with span (preferred)
template<typename Component, typename Tag, typename T>
concept TypedProvidesStateFunctionSpan = TypedComponentConcept<Component> &&
    StateTagConcept<Tag> &&
    requires(const Component& c, std::span<const T> state) {
        { c.compute(Tag{}, state) };
    };

// Check if component provides a specific state function with vector (legacy)
template<typename Component, typename Tag, typename T>
concept TypedProvidesStateFunctionVector = TypedComponentConcept<Component> &&
    StateTagConcept<Tag> &&
    requires(const Component& c, const std::vector<T>& state) {
        { c.compute(Tag{}, state) };
    };

// Combined: accepts either span or vector interface
template<typename Component, typename Tag, typename T>
concept TypedProvidesStateFunction =
    TypedProvidesStateFunctionSpan<Component, Tag, T> ||
    TypedProvidesStateFunctionVector<Component, Tag, T>;

// Check if component provides registry-aware state function (span)
template<typename Component, typename Tag, typename T, typename Registry>
concept TypedProvidesRegistryAwareStateFunctionSpan = TypedComponentConcept<Component> &&
    StateTagConcept<Tag> &&
    requires(const Component& c, std::span<const T> state, const Registry& reg) {
        { c.compute(Tag{}, state, reg) };
    };

// Check if component provides registry-aware state function (vector)
template<typename Component, typename Tag, typename T, typename Registry>
concept TypedProvidesRegistryAwareStateFunctionVector = TypedComponentConcept<Component> &&
    StateTagConcept<Tag> &&
    requires(const Component& c, const std::vector<T>& state, const Registry& reg) {
        { c.compute(Tag{}, state, reg) };
    };

// Combined: accepts either interface
template<typename Component, typename Tag, typename T, typename Registry>
concept TypedProvidesRegistryAwareStateFunction =
    TypedProvidesRegistryAwareStateFunctionSpan<Component, Tag, T, Registry> ||
    TypedProvidesRegistryAwareStateFunctionVector<Component, Tag, T, Registry>;

// Check if component has registry-aware computeLocalDerivatives
template<typename Component, typename Registry>
concept HasRegistryAwareDerivatives = requires(
    const Component& c,
    typename Component::scalar_type t,
    const typename Component::LocalState& local,
    const std::vector<typename Component::scalar_type>& global,
    const Registry& reg
) {
    { c.computeLocalDerivatives(t, local, global, reg) }
        -> std::same_as<typename Component::LocalDerivative>;
};

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

// Base component class templated on scalar type
// T can be: double, Dual<double, N>, Quantity<Dim, double>, Quantity<Dim, Dual<...>>
template<size_t StateSize, Scalar T = double>
class TypedComponent {
public:
    using scalar_type = T;
    static constexpr size_t state_size = StateSize;
    using LocalState = ScalarState<T, StateSize>;
    using LocalDerivative = ScalarState<T, StateSize>;

    virtual ~TypedComponent() = default;

    // Legacy interface without registry (for backward compatibility)
    // Override this if component doesn't need cross-component access
    virtual LocalDerivative computeLocalDerivatives(
        T t,
        const LocalState& local_state,
        const std::vector<T>& global_state
    ) const {
        // Default: no derivatives (override in derived class)
        LocalDerivative result;
        for (size_t i = 0; i < StateSize; ++i) {
            result[i] = T{0};
        }
        return result;
    }

    // Registry-aware interface for cross-component state function access
    // Override this if component needs to query state functions from other components
    // The registry provides: registry.template computeFunction<Tag>(global_state)
    template<typename Registry>
    LocalDerivative computeLocalDerivatives(
        T t,
        const LocalState& local_state,
        const std::vector<T>& global_state,
        [[maybe_unused]] const Registry& registry
    ) const {
        // Default: delegate to legacy interface
        return computeLocalDerivatives(t, local_state, global_state);
    }

    // Returns the initial state for this component
    virtual LocalState getInitialLocalState() const = 0;

    // Component identification
    virtual std::string_view getComponentType() const = 0;
    virtual std::string_view getComponentName() const = 0;

    // State management
    size_t getStateOffset() const noexcept { return m_state_offset; }
    void setStateOffset(size_t offset) noexcept { m_state_offset = offset; }

protected:
    size_t m_state_offset{0};

    // Helper for accessing global state with bounds checking
    T getGlobalState(const std::vector<T>& global_state, size_t index) const {
        size_t actual_index = m_state_offset + index;
        if (actual_index >= global_state.size()) {
            throw std::out_of_range("getGlobalState: index out of bounds");
        }
        return global_state[actual_index];
    }

    // Helper to extract local state from global state
    LocalState extractLocalState(const std::vector<T>& global_state) const {
        LocalState local;
        for (size_t i = 0; i < StateSize; ++i) {
            local[i] = getGlobalState(global_state, i);
        }
        return local;
    }
};

// Compile-time registry for typed components
template<typename T, TypedComponentConcept... Components>
class TypedRegistry {
    std::tuple<const Components&...> m_components;

    // Self type for concept checks
    using Self = TypedRegistry<T, Components...>;

    // Find first component that provides a given function type (compile-time)
    // CRITICAL: Must return by reference (decltype(auto)) to avoid copying components!
    // Checks both simple compute(Tag, state) and registry-aware compute(Tag, state, registry)
    template<StateTagConcept Tag, size_t I = 0>
    constexpr decltype(auto) findProvider() const {
        if constexpr (I < sizeof...(Components)) {
            using ComponentType = std::tuple_element_t<I, std::tuple<const Components&...>>;
            using DecayedType = std::decay_t<ComponentType>;

            if constexpr (TypedProvidesStateFunction<DecayedType, Tag, T> ||
                          TypedProvidesRegistryAwareStateFunction<DecayedType, Tag, T, Self>) {
                return std::get<I>(m_components);
            } else {
                return findProvider<Tag, I + 1>();
            }
        } else {
            return std::get<0>(m_components); // Dummy return
        }
    }

public:
    explicit constexpr TypedRegistry(const Components&... components)
        : m_components(components...) {}

    // Compile-time function availability check
    // Returns true if any component provides the state function (either simple or registry-aware)
    template<StateTagConcept Tag>
    static constexpr bool hasFunction() {
        return (TypedProvidesStateFunction<Components, Tag, T> || ...) ||
               (TypedProvidesRegistryAwareStateFunction<Components, Tag, T, Self> || ...);
    }

    // Zero-overhead function dispatch
    // Calls registry-aware compute if available, otherwise simple compute
    // Supports both span and vector interfaces for backward compatibility
    template<StateTagConcept Tag>
    auto computeFunction(std::span<const T> state) const {
        static_assert(hasFunction<Tag>(), "No component provides this state function");
        const auto& provider = findProvider<Tag>();
        using ProviderType = std::decay_t<decltype(provider)>;

        // Check for registry-aware span interface first (preferred)
        if constexpr (TypedProvidesRegistryAwareStateFunctionSpan<ProviderType, Tag, T, Self>) {
            return provider.compute(Tag{}, state, *this);
        } else if constexpr (TypedProvidesStateFunctionSpan<ProviderType, Tag, T>) {
            return provider.compute(Tag{}, state);
        } else if constexpr (TypedProvidesRegistryAwareStateFunctionVector<ProviderType, Tag, T, Self>) {
            std::vector<T> vec(state.begin(), state.end());
            return provider.compute(Tag{}, vec, *this);
        } else {
            std::vector<T> vec(state.begin(), state.end());
            return provider.compute(Tag{}, vec);
        }
    }

    // Convenience overload for vector
    template<StateTagConcept Tag>
    auto computeFunction(const std::vector<T>& state) const {
        static_assert(hasFunction<Tag>(), "No component provides this state function");
        const auto& provider = findProvider<Tag>();
        using ProviderType = std::decay_t<decltype(provider)>;

        // Check for registry-aware span interface first (preferred), then vector
        if constexpr (TypedProvidesRegistryAwareStateFunctionSpan<ProviderType, Tag, T, Self>) {
            return provider.compute(Tag{}, std::span<const T>(state), *this);
        } else if constexpr (TypedProvidesStateFunctionSpan<ProviderType, Tag, T>) {
            return provider.compute(Tag{}, std::span<const T>(state));
        } else if constexpr (TypedProvidesRegistryAwareStateFunctionVector<ProviderType, Tag, T, Self>) {
            return provider.compute(Tag{}, state, *this);
        } else {
            return provider.compute(Tag{}, state);
        }
    }

    static constexpr size_t component_count() {
        return sizeof...(Components);
    }

    template<size_t I>
    constexpr const auto& getComponent() const {
        return std::get<I>(m_components);
    }
};

// Typed ODE system that works with any scalar type
template<typename T, TypedComponentConcept... Components>
class TypedODESystem {
private:
    std::tuple<Components...> m_components;
    TypedRegistry<T, Components...> m_registry;

    static constexpr size_t m_total_state_size = (Components::state_size + ...);
    static constexpr size_t m_component_count = sizeof...(Components);

    // Initialize component state offsets
    template<size_t I = 0, size_t Offset = 0>
    constexpr void initializeOffsets() {
        if constexpr (I < sizeof...(Components)) {
            auto& component = std::get<I>(m_components);
            component.setStateOffset(Offset);
            // Also call setOffset if component has it (for new CRTP-style components)
            if constexpr (requires { component.setOffset(Offset); }) {
                component.setOffset(Offset);
            }
            constexpr size_t NextOffset = Offset +
                std::tuple_element_t<I, std::tuple<Components...>>::state_size;
            initializeOffsets<I + 1, NextOffset>();
        }
    }

    // Collect derivatives from all components using non-virtual dispatch
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

                // Check if component has non-virtual derivatives method
                if constexpr (requires { component.derivatives(t, local_span, global_span, m_registry); }) {
                    auto local_derivs = component.derivatives(t, local_span, global_span, m_registry);
                    for (size_t j = 0; j < local_size; ++j) {
                        derivatives[off + j] = local_derivs[j];
                    }
                } else {
                    // Fall back to legacy virtual interface
                    typename ComponentType::LocalState local_state;
                    for (size_t j = 0; j < local_size; ++j) {
                        local_state[j] = state[off + j];
                    }
                    auto local_derivs = component.computeLocalDerivatives(t, local_state, state, m_registry);
                    for (size_t j = 0; j < local_size; ++j) {
                        derivatives[off + j] = local_derivs[j];
                    }
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
                size_t offset = component.getStateOffset();

                for (size_t j = 0; j < local_state.size; ++j) {
                    state[offset + j] = local_state[j];
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

    constexpr size_t getStateDimension() const noexcept {
        return m_total_state_size;
    }

    std::vector<T> getInitialState() const {
        std::vector<T> state(m_total_state_size);
        collectInitialStates(state);
        return state;
    }

    // State function access
    template<StateTagConcept Tag>
    constexpr bool hasStateFunction() const {
        return m_registry.template hasFunction<Tag>();
    }

    template<StateTagConcept Tag>
    static constexpr bool hasFunction() {
        return TypedRegistry<T, Components...>::template hasFunction<Tag>();
    }

    template<StateTagConcept Tag>
    auto computeStateFunction(const std::vector<T>& state) const {
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

    constexpr size_t getComponentCount() const {
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
