#pragma once

#include "../core/typed_component.hpp"
#include "../core/scalar.hpp"
#include "../core/state_function_tags.hpp"
#include <string>
#include <span>
#include <cmath>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sopot::physics {

//=============================================================================
// HarmonicOscillator - Simple mass-spring system
//=============================================================================
// Classical harmonic oscillator: m*x'' + c*x' + k*x = 0
//
// State (2 elements): [position, velocity]
//
// Provides: kinematics::Position, kinematics::Velocity, kinematics::Acceleration,
//           energy::Kinetic, energy::Potential, energy::Total, dynamics::Mass
//=============================================================================

template<Scalar T = double>
class HarmonicOscillator final : public TypedComponent<2, T> {
public:
    using Base = TypedComponent<2, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_mass;
    double m_spring_constant;
    double m_damping;
    double m_initial_position;
    double m_initial_velocity;
    double m_natural_frequency;
    std::string m_name;
    mutable size_t m_offset{0};

public:
    HarmonicOscillator(
        double mass,
        double spring_constant,
        double damping = 0.0,
        double initial_position = 1.0,
        double initial_velocity = 0.0,
        std::string name = "harmonic_oscillator"
    ) : m_mass(mass)
      , m_spring_constant(spring_constant)
      , m_damping(damping)
      , m_initial_position(initial_position)
      , m_initial_velocity(initial_velocity)
      , m_natural_frequency(std::sqrt(spring_constant / mass))
      , m_name(std::move(name)) {
        if (mass <= 0.0) throw std::invalid_argument("Mass must be positive");
        if (spring_constant <= 0.0) throw std::invalid_argument("Spring constant must be positive");
        if (damping < 0.0) throw std::invalid_argument("Damping must be non-negative");
    }

    void setOffset(size_t off) const { m_offset = off; }

    //=========================================================================
    // Required Component Interface
    //=========================================================================

    LocalState getInitialLocalState() const {
        return {T(m_initial_position), T(m_initial_velocity)};
    }

    std::string_view getComponentType() const { return "HarmonicOscillator"; }
    std::string_view getComponentName() const { return m_name; }

    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        std::span<const T> local,
        [[maybe_unused]] std::span<const T> global,
        [[maybe_unused]] const Registry& registry
    ) const {
        T x = local[0];
        T v = local[1];
        return {v, T(-m_spring_constant / m_mass) * x - T(m_damping / m_mass) * v};
    }

    //=========================================================================
    // State Functions
    //=========================================================================

    T compute(kinematics::Position, std::span<const T> state) const {
        return this->getGlobalState(state, 0);
    }

    T compute(kinematics::Velocity, std::span<const T> state) const {
        return this->getGlobalState(state, 1);
    }

    T compute(kinematics::Acceleration, std::span<const T> state) const {
        T x = this->getGlobalState(state, 0);
        T v = this->getGlobalState(state, 1);
        return T(-m_spring_constant / m_mass) * x - T(m_damping / m_mass) * v;
    }

    T compute(energy::Kinetic, std::span<const T> state) const {
        T v = this->getGlobalState(state, 1);
        return T(0.5 * m_mass) * v * v;
    }

    T compute(energy::Potential, std::span<const T> state) const {
        T x = this->getGlobalState(state, 0);
        return T(0.5 * m_spring_constant) * x * x;
    }

    T compute(energy::Total, std::span<const T> state) const {
        return compute(energy::Kinetic{}, state) + compute(energy::Potential{}, state);
    }

    T compute(dynamics::Mass, [[maybe_unused]] std::span<const T>) const {
        return T(m_mass);
    }

    //=========================================================================
    // Parameter Access
    //=========================================================================

    double getMass() const noexcept { return m_mass; }
    double getSpringConstant() const noexcept { return m_spring_constant; }
    double getDamping() const noexcept { return m_damping; }
    double getNaturalFrequency() const noexcept { return m_natural_frequency; }

    // Analytical solution (undamped case only)
    std::pair<double, double> analyticalSolution(double t) const {
        if (m_damping != 0.0) {
            throw std::runtime_error("Analytical solution only available for undamped case");
        }
        double A = m_initial_position;
        double B = m_initial_velocity / m_natural_frequency;
        double pos = A * std::cos(m_natural_frequency * t) + B * std::sin(m_natural_frequency * t);
        double vel = -A * m_natural_frequency * std::sin(m_natural_frequency * t) +
                      B * m_natural_frequency * std::cos(m_natural_frequency * t);
        return {pos, vel};
    }
};

//=============================================================================
// Factory Functions
//=============================================================================

template<Scalar T = double>
HarmonicOscillator<T> createSimpleOscillator(
    double mass = 1.0,
    double spring_constant = 1.0,
    double initial_position = 1.0
) {
    return HarmonicOscillator<T>(mass, spring_constant, 0.0, initial_position, 0.0);
}

template<Scalar T = double>
HarmonicOscillator<T> createDampedOscillator(
    double mass = 1.0,
    double spring_constant = 1.0,
    double damping = 0.1,
    double initial_position = 1.0
) {
    return HarmonicOscillator<T>(mass, spring_constant, damping, initial_position, 0.0);
}

} // namespace sopot::physics
