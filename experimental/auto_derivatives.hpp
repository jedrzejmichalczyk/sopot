#pragma once

#include "system_builder.hpp"
#include <map>
#include <string_view>
#include <any>

namespace sopot::experimental {

// ============================================================================
// PROVISION CACHE - Runtime storage for computed values
// ============================================================================

template<typename T>
class ProvisionCache {
public:
    // Store a field value by name
    void store(std::string_view name, T value) {
        cache[std::string(name)] = value;
    }

    // Retrieve a field value by name
    T get(std::string_view name) const {
        auto it = cache.find(std::string(name));
        if (it != cache.end()) {
            return it->second;
        }
        return T{};  // Should never happen if dependencies are satisfied
    }

    // Check if field exists
    bool has(std::string_view name) const {
        return cache.find(std::string(name)) != cache.end();
    }

    void clear() {
        cache.clear();
    }

private:
    std::map<std::string, T> cache;
};

// ============================================================================
// SYSTEM WITH AUTO-DERIVATIVES
// ============================================================================

template<typename T, typename... Components>
class AutoDerivSystem : public System<T, Components...> {
public:
    using Base = System<T, Components...>;
    using Base::components;
    using Base::NumComponents;
    using Base::totalStateSize;

    explicit AutoDerivSystem(Components... comps) : Base(comps...) {}

    // Main function: Compute all derivatives automatically!
    auto computeDerivatives(T t, const std::array<T, totalStateSize()>& state) const {
        std::array<T, totalStateSize()> derivatives{};

        // Provision cache - stores computed values during execution
        ProvisionCache<T> cache;

        // Step 1: Populate cache with current state values
        // (Components with state provide their current values)
        populateCacheFromState(state, cache);

        // Step 2: Execute stateless components in dependency order
        // (They compute and provide values to cache)
        executeStatelessComponents(t, state, cache);

        // Step 3: Execute stateful components to compute derivatives
        // (They use cached provisions and compute their derivatives)
        computeStatefulDerivatives(t, state, cache, derivatives);

        return derivatives;
    }

private:
    // Step 1: Add state-derived provisions to cache
    // NOTE: Placeholder implementation - does not populate cache yet
    // TODO: Use reflection/introspection to automatically extract provisions from state
    void populateCacheFromState(
        const std::array<T, totalStateSize()>& state,
        ProvisionCache<T>& cache
    ) const {
        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;

                if constexpr (Comp::StateSize > 0) {
                    // Stateful component - extract local state
                    auto local = Base::template extractLocalState<Is>(state);

                    // TODO: If component provides based on state, add to cache
                    // TODO: Implement automatic provision extraction
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});
    }

    // Step 2: Execute stateless components to populate provisions
    // NOTE: Placeholder implementation - does not execute components yet
    // TODO: Use topological sort to determine execution order
    // TODO: Automatically execute stateless components with dependency injection
    void executeStatelessComponents(
        T t,
        const std::array<T, totalStateSize()>& state,
        ProvisionCache<T>& cache
    ) const {
        // TODO: Implement automatic execution order using topological sort

        (void)t;      // Suppress unused parameter warning
        (void)state;  // Suppress unused parameter warning
        (void)cache;  // Suppress unused parameter warning

        // Example execution order (not implemented):
        // Component 4: ConstantForce -> provides "force"
        // Component 1: ConstantMass -> provides "mass"
        // Component 2: NewtonSecondLaw -> uses force, mass -> provides "accel"
    }

    // Step 3: Compute derivatives for stateful components
    // NOTE: Placeholder implementation - does not compute derivatives yet
    // TODO: Implement automatic dependency injection from cache
    // TODO: Automatically call component derivative computation methods
    void computeStatefulDerivatives(
        T t,
        const std::array<T, totalStateSize()>& state,
        const ProvisionCache<T>& cache,
        std::array<T, totalStateSize()>& derivatives
    ) const {
        size_t offset = 0;

        [&]<size_t... Is>(std::index_sequence<Is...>) {
            ([&] {
                using Comp = nth_type_t<Is, Components...>;

                if constexpr (Comp::StateSize > 0) {
                    // Stateful component - compute derivative
                    auto local = Base::template extractLocalState<Is>(state);
                    const auto& comp = std::get<Is>(components);

                    // TODO: Get dependencies from cache automatically
                    // TODO: Inject dependencies and compute derivative:
                    // auto deriv = comp.computeDerivative(t, local, ...deps...);

                    // TODO: Store in derivatives array:
                    // derivatives[offset] = deriv;

                    offset += Comp::StateSize;
                }
            }(), ...);
        }(std::make_index_sequence<NumComponents>{});

        (void)t;      // Suppress unused parameter warning
        (void)state;  // Suppress unused parameter warning
        (void)cache;  // Suppress unused parameter warning
    }
};

// ============================================================================
// FACTORY FUNCTION
// ============================================================================

template<typename T, typename... Components>
auto makeAutoDerivSystem(Components... components) {
    return AutoDerivSystem<T, Components...>(components...);
}

}  // namespace sopot::experimental
