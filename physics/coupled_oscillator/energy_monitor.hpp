#pragma once

#include "../../core/typed_component.hpp"
#include "../../core/scalar.hpp"
#include "tags.hpp"
#include <span>
#include <string>

namespace sopot::physics::coupled {

//=============================================================================
// EnergyMonitor - Computes system-level quantities from component states
//=============================================================================
// This stateless component queries both masses and the spring to compute:
//   - Total kinetic energy
//   - Total potential energy
//   - Total energy (should be conserved for undamped system)
//   - Center of mass position
//   - Total momentum (should be conserved)
//
// State (0 elements): No own state - purely computes from other components
//
// Provides: system::TotalEnergy, system::CenterOfMass, system::Momentum
// Requires: mass1::*, mass2::*, spring::PotentialEnergy
//=============================================================================

template<Scalar T = double>
class EnergyMonitor final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    std::string m_name;

public:
    EnergyMonitor(std::string name = "energy_monitor")
        : m_name(std::move(name)) {}

    void setOffset(size_t) const {} // No state

    //=========================================================================
    // Required Component Interface
    //=========================================================================

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "EnergyMonitor"; }
    std::string_view getComponentName() const { return m_name; }

    //=========================================================================
    // State Functions - System-Level Quantities
    //=========================================================================

    // Total energy = KE1 + KE2 + PE_spring
    template<typename Registry>
    T compute(system::TotalEnergy, std::span<const T> state, const Registry& registry) const {
        // Kinetic energy of mass 1: KE1 = 0.5 * m1 * v1^2
        T m1 = registry.template computeFunction<mass1::Mass>(state);
        T v1 = registry.template computeFunction<mass1::Velocity>(state);
        T ke1 = T(0.5) * m1 * v1 * v1;

        // Kinetic energy of mass 2: KE2 = 0.5 * m2 * v2^2
        T m2 = registry.template computeFunction<mass2::Mass>(state);
        T v2 = registry.template computeFunction<mass2::Velocity>(state);
        T ke2 = T(0.5) * m2 * v2 * v2;

        // Spring potential energy
        T pe = registry.template computeFunction<spring::PotentialEnergy>(state);

        return ke1 + ke2 + pe;
    }

    // Center of mass: x_cm = (m1*x1 + m2*x2) / (m1 + m2)
    template<typename Registry>
    T compute(system::CenterOfMass, std::span<const T> state, const Registry& registry) const {
        T m1 = registry.template computeFunction<mass1::Mass>(state);
        T x1 = registry.template computeFunction<mass1::Position>(state);
        T m2 = registry.template computeFunction<mass2::Mass>(state);
        T x2 = registry.template computeFunction<mass2::Position>(state);

        return (m1 * x1 + m2 * x2) / (m1 + m2);
    }

    // Total momentum: p = m1*v1 + m2*v2 (conserved in closed system)
    template<typename Registry>
    T compute(system::Momentum, std::span<const T> state, const Registry& registry) const {
        T m1 = registry.template computeFunction<mass1::Mass>(state);
        T v1 = registry.template computeFunction<mass1::Velocity>(state);
        T m2 = registry.template computeFunction<mass2::Mass>(state);
        T v2 = registry.template computeFunction<mass2::Velocity>(state);

        return m1 * v1 + m2 * v2;
    }
};

//=============================================================================
// Factory Function
//=============================================================================

template<Scalar T = double>
EnergyMonitor<T> createEnergyMonitor() {
    return EnergyMonitor<T>("energy_monitor");
}

} // namespace sopot::physics::coupled
