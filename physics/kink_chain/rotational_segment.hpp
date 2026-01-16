#pragma once

#include "../../core/typed_component.hpp"
#include "../../core/scalar.hpp"
#include "tags.hpp"
#include <span>
#include <string>
#include <numbers>

namespace sopot::physics::kink {

//=============================================================================
// RotationalSegment - A rigid stick with rotational degrees of freedom
//=============================================================================
// Template parameter TagSet must provide:
//   - TagSet::Angle - tag for this segment's angle
//   - TagSet::AngularVelocity - tag for this segment's angular velocity
//   - TagSet::Torque - tag for torque acting on this segment
//   - TagSet::MomentOfInertia - tag for this segment's moment of inertia
//
// State (2 elements): [angle, angular_velocity]
//
// Provides: TagSet::Angle, TagSet::AngularVelocity, TagSet::MomentOfInertia
// Requires: TagSet::Torque (for derivatives)
//
// Physics: dθ/dt = ω, dω/dt = τ/I
//=============================================================================

template<typename TagSet, Scalar T = double>
class RotationalSegment final : public TypedComponent<2, T> {
public:
    using Base = TypedComponent<2, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_inertia;           // Moment of inertia I [kg·m²]
    double m_initial_angle;     // Initial angle [rad]
    double m_initial_omega;     // Initial angular velocity [rad/s]
    std::string m_name;
    mutable size_t m_offset{0};

public:
    RotationalSegment(
        double inertia,
        double initial_angle = 0.0,
        double initial_omega = 0.0,
        std::string name = "segment"
    ) : m_inertia(inertia)
      , m_initial_angle(initial_angle)
      , m_initial_omega(initial_omega)
      , m_name(std::move(name)) {
        if (inertia <= 0.0) {
            throw std::invalid_argument("Moment of inertia must be positive");
        }
    }

    void setOffset(size_t off) const { m_offset = off; }

    //=========================================================================
    // Required Component Interface
    //=========================================================================

    LocalState getInitialLocalState() const {
        return {T(m_initial_angle), T(m_initial_omega)};
    }

    std::string_view getComponentType() const { return "RotationalSegment"; }
    std::string_view getComponentName() const { return m_name; }

    //=========================================================================
    // Derivatives - dθ/dt = ω, dω/dt = τ/I
    //=========================================================================

    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        std::span<const T> local,
        std::span<const T> global,
        const Registry& registry
    ) const {
        // dθ/dt = ω (from local state)
        T omega = local[1];

        // dω/dt = τ/I (torque from registry)
        T torque = registry.template computeFunction<typename TagSet::Torque>(global);
        T alpha = torque / T(m_inertia);

        return {omega, alpha};
    }

    //=========================================================================
    // State Functions - Angle, AngularVelocity, MomentOfInertia
    //=========================================================================

    T compute(typename TagSet::Angle, std::span<const T> state) const {
        return state[m_offset];
    }

    T compute(typename TagSet::AngularVelocity, std::span<const T> state) const {
        return state[m_offset + 1];
    }

    T compute(typename TagSet::MomentOfInertia, [[maybe_unused]] std::span<const T> state) const {
        return T(m_inertia);
    }

    //=========================================================================
    // Parameter Access
    //=========================================================================

    double getInertia() const noexcept { return m_inertia; }
    double getInitialAngle() const noexcept { return m_initial_angle; }
    double getInitialOmega() const noexcept { return m_initial_omega; }
};

} // namespace sopot::physics::kink
