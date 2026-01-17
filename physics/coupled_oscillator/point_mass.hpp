#pragma once

#include "../../core/typed_component.hpp"
#include "../../core/scalar.hpp"
#include "tags.hpp"
#include <span>
#include <string>

namespace sopot::physics::coupled {

//=============================================================================
// PointMass - A single point mass with position and velocity
//=============================================================================
// Template parameter TagSet must provide:
//   - TagSet::Position - tag for this mass's position
//   - TagSet::Velocity - tag for this mass's velocity
//   - TagSet::Force    - tag for force acting on this mass
//   - TagSet::Mass     - tag for this mass's mass value
//
// State (2 elements): [position, velocity]
//
// Provides: TagSet::Position, TagSet::Velocity, TagSet::Mass
// Requires: TagSet::Force (for derivatives)
//=============================================================================

template<typename TagSet, Scalar T = double>
class PointMass final : public TypedComponent<2, T> {
public:
    using Base = TypedComponent<2, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_mass;
    double m_initial_position;
    double m_initial_velocity;
    std::string m_name;
    mutable size_t m_offset{0};

public:
    PointMass(
        double mass,
        double initial_position = 0.0,
        double initial_velocity = 0.0,
        std::string name = "point_mass"
    ) : m_mass(mass)
      , m_initial_position(initial_position)
      , m_initial_velocity(initial_velocity)
      , m_name(std::move(name)) {
        if (mass <= 0.0) {
            throw std::invalid_argument("Mass must be positive");
        }
    }

    void setOffset(size_t off) const { m_offset = off; }

    //=========================================================================
    // Required Component Interface
    //=========================================================================

    LocalState getInitialLocalState() const {
        return {T(m_initial_position), T(m_initial_velocity)};
    }

    std::string_view getComponentType() const { return "PointMass"; }
    std::string_view getComponentName() const { return m_name; }

    //=========================================================================
    // Derivatives - dx/dt = v, dv/dt = F/m
    //=========================================================================

    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        std::span<const T> local,
        std::span<const T> global,
        const Registry& registry
    ) const {
        // dx/dt = v (from local state)
        T velocity = local[1];

        // dv/dt = F/m (force from registry)
        T force = registry.template computeFunction<typename TagSet::Force>(global);
        T acceleration = force / T(m_mass);

        return {velocity, acceleration};
    }

    //=========================================================================
    // State Functions - Position, Velocity, Mass
    //=========================================================================

    T compute(typename TagSet::Position, std::span<const T> state) const {
        return this->getGlobalState(state, 0);
    }

    T compute(typename TagSet::Velocity, std::span<const T> state) const {
        return this->getGlobalState(state, 1);
    }

    T compute(typename TagSet::Mass, [[maybe_unused]] std::span<const T> state) const {
        return T(m_mass);
    }

    //=========================================================================
    // Parameter Access
    //=========================================================================

    double getMass() const noexcept { return m_mass; }
    double getInitialPosition() const noexcept { return m_initial_position; }
    double getInitialVelocity() const noexcept { return m_initial_velocity; }
};

//=============================================================================
// Pre-defined Mass Types using the tag namespaces
//=============================================================================

template<Scalar T = double>
using Mass1 = PointMass<mass1, T>;

template<Scalar T = double>
using Mass2 = PointMass<mass2, T>;

//=============================================================================
// Factory Functions
//=============================================================================

template<Scalar T = double>
Mass1<T> createMass1(double mass, double x0 = 0.0, double v0 = 0.0) {
    return Mass1<T>(mass, x0, v0, "mass1");
}

template<Scalar T = double>
Mass2<T> createMass2(double mass, double x0 = 0.0, double v0 = 0.0) {
    return Mass2<T>(mass, x0, v0, "mass2");
}

} // namespace sopot::physics::coupled
