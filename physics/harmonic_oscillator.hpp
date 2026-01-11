#pragma once

#include "../core/typed_component.hpp"
#include "../core/scalar.hpp"
#include "../core/state_function_tags.hpp"
#include <string>
#include <span>

// MSVC requires _USE_MATH_DEFINES before cmath for M_PI
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sopot::physics {

// Oscillator-specific state function tags
namespace oscillator_tags {
    struct SpringForce : categories::Dynamics {
        static constexpr std::string_view name() { return "spring_force"; }
        static constexpr size_t type_id() { return 1001; }
    };

    struct NaturalFrequency : categories::Analysis {
        static constexpr std::string_view name() { return "natural_frequency"; }
        static constexpr size_t type_id() { return 2001; }
    };

    struct Period : categories::Analysis {
        static constexpr std::string_view name() { return "period"; }
        static constexpr size_t type_id() { return 2002; }
    };
}

// Aerodynamics-specific state function tags
namespace aero_tags {
    struct DragForce : categories::Dynamics {
        static constexpr std::string_view name() { return "drag_force"; }
        static constexpr size_t type_id() { return 3001; }
    };

    struct DynamicPressure : categories::Dynamics {
        static constexpr std::string_view name() { return "dynamic_pressure"; }
        static constexpr size_t type_id() { return 3002; }
    };
}

// Typed harmonic oscillator component - works with any scalar type (double, Dual, Quantity)
template<Scalar T = double>
class HarmonicOscillator final : public TypedComponent<2, T> {
private:
    double m_mass;
    double m_spring_constant;
    double m_damping;
    double m_initial_position;
    double m_initial_velocity;
    std::string m_name;

    // Cached derived quantities
    double m_natural_frequency;

public:
    using Base = TypedComponent<2, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    mutable size_t m_offset{0};

public:
    void setOffset(size_t off) const { m_offset = off; }

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
      , m_name(std::move(name))
      , m_natural_frequency(std::sqrt(spring_constant / mass)) {

        if (mass <= 0.0) {
            throw std::invalid_argument("Mass must be positive");
        }
        if (spring_constant <= 0.0) {
            throw std::invalid_argument("Spring constant must be positive");
        }
        if (damping < 0.0) {
            throw std::invalid_argument("Damping must be non-negative");
        }
    }

    // Non-virtual derivatives method for CRTP-style dispatch
    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        std::span<const T> local,
        [[maybe_unused]] std::span<const T> global,
        [[maybe_unused]] const Registry& registry
    ) const {
        T x = local[0];
        T v = local[1];

        LocalDerivative result;
        result[0] = v; // dx/dt = v
        result[1] = T(-m_spring_constant / m_mass) * x - T(m_damping / m_mass) * v;
        return result;
    }

    // Required by TypedComponent - non-virtual
    LocalState getInitialLocalState() const {
        LocalState state;
        state[0] = T(m_initial_position);
        state[1] = T(m_initial_velocity);
        return state;
    }

    std::string_view getComponentType() const { return "HarmonicOscillator"; }
    std::string_view getComponentName() const { return m_name; }

    // State functions - all resolved at compile time
    T compute(kinematics::Position, std::span<const T> state) const {
        return state[m_offset];
    }

    T compute(kinematics::Velocity, std::span<const T> state) const {
        return state[m_offset + 1];
    }

    T compute(kinematics::Acceleration, std::span<const T> state) const {
        T x = state[m_offset];
        T v = state[m_offset + 1];
        return T(-m_spring_constant / m_mass) * x - T(m_damping / m_mass) * v;
    }

    T compute(energy::Kinetic, std::span<const T> state) const {
        T v = compute(kinematics::Velocity{}, state);
        return T(0.5 * m_mass) * v * v;
    }

    T compute(energy::Potential, std::span<const T> state) const {
        T x = compute(kinematics::Position{}, state);
        return T(0.5 * m_spring_constant) * x * x;
    }

    T compute(energy::Total, std::span<const T> state) const {
        return compute(energy::Kinetic{}, state) + compute(energy::Potential{}, state);
    }

    T compute(oscillator_tags::SpringForce, std::span<const T> state) const {
        T x = compute(kinematics::Position{}, state);
        return T(-m_spring_constant) * x;
    }

    T compute(dynamics::Force, std::span<const T> state) const {
        return compute(oscillator_tags::SpringForce{}, state);
    }

    T compute(dynamics::Mass, [[maybe_unused]] std::span<const T>) const {
        return T(m_mass);
    }

    T compute(oscillator_tags::NaturalFrequency, [[maybe_unused]] std::span<const T>) const {
        return T(m_natural_frequency);
    }

    T compute(oscillator_tags::Period, [[maybe_unused]] std::span<const T>) const {
        return T(2.0 * M_PI / m_natural_frequency);
    }

    // Parameter access
    double getMass() const noexcept { return m_mass; }
    double getSpringConstant() const noexcept { return m_spring_constant; }
    double getDamping() const noexcept { return m_damping; }
    double getNaturalFrequency() const noexcept { return m_natural_frequency; }

    // Analytical solution for validation (undamped case)
    std::pair<double, double> getAnalyticalSolution(double t) const {
        if (m_damping == 0.0) {
            const double A = m_initial_position;
            const double B = m_initial_velocity / m_natural_frequency;
            const double position = A * std::cos(m_natural_frequency * t) +
                                  B * std::sin(m_natural_frequency * t);
            const double velocity = -A * m_natural_frequency * std::sin(m_natural_frequency * t) +
                                  B * m_natural_frequency * std::cos(m_natural_frequency * t);
            return {position, velocity};
        } else {
            throw std::runtime_error("Analytical solution only available for undamped case");
        }
    }
};

// Simple aerodynamics component that queries velocity from other components
template<Scalar T = double>
class SimpleAerodynamics final : public TypedComponent<0, T> {
private:
    double m_reference_area;
    double m_drag_coefficient;
    double m_air_density;
    std::string m_name;

public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

    SimpleAerodynamics(
        double reference_area,
        double drag_coefficient,
        double air_density = 1.225,
        std::string name = "simple_aero"
    ) : m_reference_area(reference_area)
      , m_drag_coefficient(drag_coefficient)
      , m_air_density(air_density)
      , m_name(std::move(name)) {}

    void setOffset(size_t) const {} // No state

    // Required by TypedComponent - non-virtual
    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "SimpleAerodynamics"; }
    std::string_view getComponentName() const { return m_name; }

    // Drag force state function - queries velocity from registry
    template<typename Registry>
    T computeDragForce(std::span<const T> state, const Registry& registry) const {
        if constexpr (Registry::template hasFunction<kinematics::Velocity>()) {
            T velocity = registry.template computeFunction<kinematics::Velocity>(state);
            T coeff = T(0.5 * m_air_density * m_drag_coefficient * m_reference_area);
            T abs_vel = abs(velocity);
            return -coeff * velocity * abs_vel;
        } else {
            return T{0};
        }
    }

    // Alternative: compute drag given velocity directly (for standalone use)
    T compute(oscillator_tags::SpringForce, std::span<const T> state) const {
        // Legacy interface - assumes velocity is at index 1
        if (state.size() >= 2) {
            T velocity = state[1];
            T coeff = T(0.5 * m_air_density * m_drag_coefficient * m_reference_area);
            T abs_vel = abs(velocity);
            return -coeff * velocity * abs_vel;
        }
        return T{0};
    }

    T compute(oscillator_tags::NaturalFrequency, [[maybe_unused]] std::span<const T>) const {
        return T(m_drag_coefficient); // Reusing tag for demo
    }

    double getReferenceArea() const { return m_reference_area; }
    double getDragCoefficient() const { return m_drag_coefficient; }
    double getAirDensity() const { return m_air_density; }
};

// Factory functions for convenience
template<Scalar T = double>
HarmonicOscillator<T> createSimpleOscillator(
    double mass = 1.0,
    double spring_constant = 1.0,
    double initial_position = 1.0,
    std::string name = "simple_oscillator"
) {
    return HarmonicOscillator<T>(mass, spring_constant, 0.0, initial_position, 0.0, std::move(name));
}

template<Scalar T = double>
HarmonicOscillator<T> createDampedOscillator(
    double mass = 1.0,
    double spring_constant = 1.0,
    double damping = 0.1,
    double initial_position = 1.0,
    std::string name = "damped_oscillator"
) {
    return HarmonicOscillator<T>(mass, spring_constant, damping, initial_position, 0.0, std::move(name));
}

} // namespace sopot::physics
