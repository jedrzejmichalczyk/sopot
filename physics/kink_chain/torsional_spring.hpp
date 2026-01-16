#pragma once

#include "../../core/typed_component.hpp"
#include "../../core/scalar.hpp"
#include "tags.hpp"
#include <span>
#include <string>
#include <cmath>
#include <numbers>

namespace sopot::physics::kink {

//=============================================================================
// TorsionalSpring - Connects two segments with torsional elasticity
//=============================================================================
// Template parameters TagSet1, TagSet2 identify the two segments.
// Each TagSet must provide Angle, AngularVelocity, and Torque tags.
//
// State (0 elements): No own state - purely computes torques from angles
//
// Provides: TagSet1::Torque, TagSet2::Torque, spring::AngleDifference, spring::PotentialEnergy
// Requires: TagSet1::Angle, TagSet2::Angle, TagSet1::AngularVelocity, TagSet2::AngularVelocity
//
// Physics:
//   angle_diff = θ1 - θ2 (wrapped to [-π, π])
//   τ1 = -k * angle_diff - c * (ω1 - ω2)  (torque on segment 1)
//   τ2 = +k * angle_diff + c * (ω1 - ω2)  (torque on segment 2, action-reaction)
//=============================================================================

template<typename TagSet1, typename TagSet2, Scalar T = double>
class TorsionalSpring final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_stiffness;      // Torsional spring constant k [N·m/rad]
    double m_damping;        // Rotational damping coefficient c [N·m·s/rad]
    std::string m_name;

    // Wrap angle to [-π, π]
    static T wrapAngle(T angle) {
        constexpr T pi = T(std::numbers::pi);
        // Normalize to [-π, π]
        while (angle > pi) angle -= T(2.0) * pi;
        while (angle < -pi) angle += T(2.0) * pi;
        return angle;
    }

public:
    TorsionalSpring(
        double stiffness,
        double damping = 0.0,
        std::string name = "torsional_spring"
    ) : m_stiffness(stiffness)
      , m_damping(damping)
      , m_name(std::move(name)) {
        if (stiffness <= 0.0) {
            throw std::invalid_argument("Torsional stiffness must be positive");
        }
        if (damping < 0.0) {
            throw std::invalid_argument("Damping must be non-negative");
        }
    }

    void setOffset(size_t) const {} // No state

    //=========================================================================
    // Required Component Interface
    //=========================================================================

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "TorsionalSpring"; }
    std::string_view getComponentName() const { return m_name; }

    //=========================================================================
    // State Functions - Torques and Spring Properties
    //=========================================================================

    // Angle difference (wrapped to [-π, π])
    template<typename Registry>
    T compute(spring::AngleDifference, std::span<const T> state, const Registry& registry) const {
        T theta1 = registry.template computeFunction<typename TagSet1::Angle>(state);
        T theta2 = registry.template computeFunction<typename TagSet2::Angle>(state);
        return wrapAngle(theta1 - theta2);
    }

    // Torque on segment 1: τ1 = -k * Δθ - c * Δω
    template<typename Registry>
    T compute(typename TagSet1::Torque, std::span<const T> state, const Registry& registry) const {
        T theta1 = registry.template computeFunction<typename TagSet1::Angle>(state);
        T theta2 = registry.template computeFunction<typename TagSet2::Angle>(state);
        T angle_diff = wrapAngle(theta1 - theta2);

        T spring_torque = -T(m_stiffness) * angle_diff;

        // Add damping if present
        if (m_damping > 0.0) {
            T omega1 = registry.template computeFunction<typename TagSet1::AngularVelocity>(state);
            T omega2 = registry.template computeFunction<typename TagSet2::AngularVelocity>(state);
            T damping_torque = -T(m_damping) * (omega1 - omega2);
            return spring_torque + damping_torque;
        }

        return spring_torque;
    }

    // Torque on segment 2: τ2 = -τ1 (action-reaction)
    template<typename Registry>
    T compute(typename TagSet2::Torque, std::span<const T> state, const Registry& registry) const {
        return -compute(typename TagSet1::Torque{}, state, registry);
    }

    // Spring potential energy: U = 0.5 * k * (Δθ)^2
    template<typename Registry>
    T compute(spring::PotentialEnergy, std::span<const T> state, const Registry& registry) const {
        T angle_diff = compute(spring::AngleDifference{}, state, registry);
        return T(0.5 * m_stiffness) * angle_diff * angle_diff;
    }

    // Torsional stiffness (constant)
    T compute(spring::TorsionalStiffness, [[maybe_unused]] std::span<const T> state) const {
        return T(m_stiffness);
    }

    //=========================================================================
    // Parameter Access
    //=========================================================================

    double getStiffness() const noexcept { return m_stiffness; }
    double getDamping() const noexcept { return m_damping; }
};

} // namespace sopot::physics::kink
