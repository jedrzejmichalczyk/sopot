#pragma once

#include "../../core/typed_component.hpp"
#include "../../core/scalar.hpp"
#include "tags.hpp"
#include <span>
#include <string>
#include <cmath>

namespace sopot::physics::coupled {

//=============================================================================
// Spring - Connects two masses with a linear spring
//=============================================================================
// Template parameters TagSet1, TagSet2 identify the two masses.
// Each TagSet must provide Position and Force tags.
//
// State (0 elements): No own state - purely computes forces from positions
//
// Provides: TagSet1::Force, TagSet2::Force, spring::Extension, spring::PotentialEnergy
// Requires: TagSet1::Position, TagSet2::Position
//
// Physics:
//   extension = x1 - x2 - rest_length
//   F1 = -k * extension  (force on mass 1)
//   F2 = +k * extension  (force on mass 2, Newton's 3rd law)
//=============================================================================

template<typename TagSet1, typename TagSet2, Scalar T = double>
class Spring final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_stiffness;      // Spring constant k [N/m]
    double m_rest_length;    // Natural length L0 [m]
    double m_damping;        // Damping coefficient c [N·s/m]
    std::string m_name;

public:
    Spring(
        double stiffness,
        double rest_length = 1.0,
        double damping = 0.0,
        std::string name = "spring"
    ) : m_stiffness(stiffness)
      , m_rest_length(rest_length)
      , m_damping(damping)
      , m_name(std::move(name)) {
        if (stiffness <= 0.0) {
            throw std::invalid_argument("Spring stiffness must be positive");
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
    std::string_view getComponentType() const { return "Spring"; }
    std::string_view getComponentName() const { return m_name; }

    //=========================================================================
    // State Functions - Forces and Spring Properties
    //=========================================================================

    // Spring extension (x1 - x2 - L0)
    template<typename Registry>
    T compute(spring::Extension, std::span<const T> state, const Registry& registry) const {
        T x1 = registry.template computeFunction<typename TagSet1::Position>(state);
        T x2 = registry.template computeFunction<typename TagSet2::Position>(state);
        return x1 - x2 - T(m_rest_length);
    }

    // Force on mass 1: F1 = -k * extension - c * (v1 - v2)
    template<typename Registry>
    T compute(typename TagSet1::Force, std::span<const T> state, const Registry& registry) const {
        T x1 = registry.template computeFunction<typename TagSet1::Position>(state);
        T x2 = registry.template computeFunction<typename TagSet2::Position>(state);
        T extension = x1 - x2 - T(m_rest_length);

        T spring_force = -T(m_stiffness) * extension;

        // Add damping if present
        if (m_damping > 0.0) {
            T v1 = registry.template computeFunction<typename TagSet1::Velocity>(state);
            T v2 = registry.template computeFunction<typename TagSet2::Velocity>(state);
            T damping_force = -T(m_damping) * (v1 - v2);
            return spring_force + damping_force;
        }

        return spring_force;
    }

    // Force on mass 2: F2 = -F1 (Newton's 3rd law)
    template<typename Registry>
    T compute(typename TagSet2::Force, std::span<const T> state, const Registry& registry) const {
        return -compute(typename TagSet1::Force{}, state, registry);
    }

    // Spring potential energy: U = 0.5 * k * extension^2
    template<typename Registry>
    T compute(spring::PotentialEnergy, std::span<const T> state, const Registry& registry) const {
        T ext = compute(spring::Extension{}, state, registry);
        return T(0.5 * m_stiffness) * ext * ext;
    }

    // Spring stiffness (constant)
    T compute(spring::Stiffness, [[maybe_unused]] std::span<const T> state) const {
        return T(m_stiffness);
    }

    //=========================================================================
    // Parameter Access
    //=========================================================================

    double getStiffness() const noexcept { return m_stiffness; }
    double getRestLength() const noexcept { return m_rest_length; }
    double getDamping() const noexcept { return m_damping; }
};

//=============================================================================
// Pre-defined Spring Type connecting mass1 and mass2
//=============================================================================

template<Scalar T = double>
using Spring12 = Spring<mass1, mass2, T>;

//=============================================================================
// Factory Function
//=============================================================================

template<Scalar T = double>
Spring12<T> createSpring(double stiffness, double rest_length = 1.0, double damping = 0.0) {
    return Spring12<T>(stiffness, rest_length, damping, "spring");
}

} // namespace sopot::physics::coupled
