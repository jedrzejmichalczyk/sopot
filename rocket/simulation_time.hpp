#pragma once

#include "../core/typed_component.hpp"
#include "vehicle_tags.hpp"
#include <span>

namespace sopot::rocket {

/**
 * SimulationTime: Tracks simulation time as a state variable.
 *
 * This component is vehicle-agnostic. Every vehicle in a multi-vehicle
 * system queries the same sim::Time.
 *
 * State (1 element): [t], dt/dt = 1
 * Provides: sim::Time
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

    std::string_view getComponentType() const { return "SimulationTime"; }
    std::string_view getComponentName() const { return m_name; }

    LocalState getInitialLocalState() const { return {m_initial_time}; }

    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        [[maybe_unused]] std::span<const T> local,
        [[maybe_unused]] std::span<const T> global,
        [[maybe_unused]] const Registry& registry
    ) const {
        return {T(1)};
    }

    T compute(sim::Time, std::span<const T> state) const {
        return this->getGlobalState(state, 0);
    }
};

template<Scalar T = double>
SimulationTime<T> createSimulationTime(T initial_time = T(0), std::string_view name = "simulation_time") {
    return SimulationTime<T>(initial_time, name);
}

} // namespace sopot::rocket
