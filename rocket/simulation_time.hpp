#pragma once

#include "../core/typed_component.hpp"
#include "rocket_tags.hpp"
#include <span>

namespace sopot::rocket {

/**
 * SimulationTime: Tracks simulation time as a state variable
 *
 * This component allows other components to query current simulation time
 * through the registry pattern, enabling time-dependent computations
 * (like mass interpolation, thrust curves) to work with the state function system.
 *
 * State (1 element): [t]
 * Derivative: dt/dt = 1
 *
 * Provides: propulsion::Time
 */
template<Scalar T = double>
class SimulationTime final : public TypedComponent<1, T> {
public:
    using Base = TypedComponent<1, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    T m_initial_time{T(0)};
    std::string m_name{"simulation_time"};

public:
    SimulationTime(T initial_time = T(0), std::string_view name = "simulation_time")
        : m_initial_time(initial_time), m_name(name) {}

    // Component identification
    std::string_view getComponentType() const override { return "SimulationTime"; }
    std::string_view getComponentName() const override { return m_name; }

    // Initial state
    LocalState getInitialLocalState() const override {
        return {m_initial_time};
    }

    // Derivative: dt/dt = 1 (non-virtual, called directly)
    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        [[maybe_unused]] std::span<const T> local,
        [[maybe_unused]] std::span<const T> global,
        [[maybe_unused]] const Registry& registry
    ) const {
        return {T(1)};
    }

    // State function: Current simulation time
    T compute(propulsion::Time, std::span<const T> state) const {
        return state[0];  // Time is at offset 0 for this component
    }
};

// Factory function
template<Scalar T = double>
SimulationTime<T> createSimulationTime(T initial_time = T(0), std::string_view name = "simulation_time") {
    return SimulationTime<T>(initial_time, name);
}

} // namespace sopot::rocket
