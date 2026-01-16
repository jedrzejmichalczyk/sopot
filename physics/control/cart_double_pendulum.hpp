#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "control_tags.hpp"
#include <string>
#include <stdexcept>
#include <array>
#include <cmath>

namespace sopot::control {

/**
 * @brief Cart with inverted double pendulum (6-state system)
 *
 * State vector: [x, θ₁, θ₂, ẋ, ω₁, ω₂]
 *   - x:  Cart position (m)
 *   - θ₁: Angle of link 1 from vertical (rad), 0 = upright
 *   - θ₂: Angle of link 2 from vertical (rad), 0 = upright
 *   - ẋ:  Cart velocity (m/s)
 *   - ω₁: Angular velocity of link 1 (rad/s)
 *   - ω₂: Angular velocity of link 2 (rad/s)
 *
 * Control input: Force F on cart (N)
 *
 * Equations derived from Lagrangian mechanics for point masses at link tips.
 * Convention: y-axis points up, θ=0 means vertical (upright).
 *
 * Positions (relative to cart):
 *   Mass 1: (L₁ sin θ₁, L₁ cos θ₁)
 *   Mass 2: (L₁ sin θ₁ + L₂ sin θ₂, L₁ cos θ₁ + L₂ cos θ₂)
 */
template<Scalar T = double>
class CartDoublePendulum final : public TypedComponent<6, T> {
public:
    using Base = TypedComponent<6, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_cart_mass;   // mc: cart mass (kg)
    double m_mass1;       // m1: mass at end of link 1 (kg)
    double m_mass2;       // m2: mass at end of link 2 (kg)
    double m_length1;     // L1: length of link 1 (m)
    double m_length2;     // L2: length of link 2 (m)
    double m_gravity;     // g: gravitational acceleration (m/s²)

    double m_initial_x;       // Initial cart position
    double m_initial_theta1;  // Initial angle of link 1
    double m_initial_theta2;  // Initial angle of link 2
    double m_initial_xdot;    // Initial cart velocity
    double m_initial_omega1;  // Initial angular velocity of link 1
    double m_initial_omega2;  // Initial angular velocity of link 2

    // Control input (set externally)
    mutable T m_control_force = T(0.0);

    std::string m_name;

public:
    explicit CartDoublePendulum(
        double cart_mass,
        double mass1,
        double mass2,
        double length1,
        double length2,
        double gravity = 9.81,
        double x0 = 0.0,
        double theta1_0 = 0.0,
        double theta2_0 = 0.0,
        double xdot_0 = 0.0,
        double omega1_0 = 0.0,
        double omega2_0 = 0.0
    )
        : m_cart_mass(cart_mass)
        , m_mass1(mass1)
        , m_mass2(mass2)
        , m_length1(length1)
        , m_length2(length2)
        , m_gravity(gravity)
        , m_initial_x(x0)
        , m_initial_theta1(theta1_0)
        , m_initial_theta2(theta2_0)
        , m_initial_xdot(xdot_0)
        , m_initial_omega1(omega1_0)
        , m_initial_omega2(omega2_0)
        , m_name("CartDoublePendulum")
    {
        if (cart_mass <= 0.0 || mass1 <= 0.0 || mass2 <= 0.0) {
            throw std::invalid_argument("All masses must be positive");
        }
        if (length1 <= 0.0 || length2 <= 0.0) {
            throw std::invalid_argument("Lengths must be positive");
        }
    }

    // Set control input (force on cart)
    void setControlForce(T force) const {
        m_control_force = force;
    }

    T getControlForce() const {
        return m_control_force;
    }

    LocalState getInitialLocalState() const {
        return {
            T(m_initial_x),
            T(m_initial_theta1),
            T(m_initial_theta2),
            T(m_initial_xdot),
            T(m_initial_omega1),
            T(m_initial_omega2)
        };
    }

    std::string_view getComponentType() const { return "CartDoublePendulum"; }
    std::string_view getComponentName() const { return m_name; }

    /**
     * @brief Compute derivatives using Euler-Lagrange equations
     *
     * The system is: M(q) * q̈ = τ(q, q̇) + B * u
     *
     * where:
     *   q = [x, θ₁, θ₂]ᵀ
     *   τ = Coriolis/centrifugal + gravity terms
     *   B = [1, 0, 0]ᵀ (control only affects cart)
     *   u = F (control force)
     */
    template<typename Registry>
    LocalDerivative derivatives(
        T /*t*/,
        std::span<const T> local,
        std::span<const T> /*global*/,
        const Registry& /*registry*/
    ) const {
        // Extract state
        T x = local[0];
        T theta1 = local[1];
        T theta2 = local[2];
        T xdot = local[3];
        T omega1 = local[4];
        T omega2 = local[5];

        // Suppress unused warning
        (void)x;

        // Parameters
        T mc = T(m_cart_mass);
        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T L1 = T(m_length1);
        T L2 = T(m_length2);
        T g = T(m_gravity);
        T F = m_control_force;

        // Trigonometric functions
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;

        T s1 = sin(theta1);
        T c1 = cos(theta1);
        T s2 = sin(theta2);
        T c2 = cos(theta2);
        T s12 = sin(theta1 - theta2);
        T c12 = cos(theta1 - theta2);

        // Mass matrix M (3x3, symmetric)
        // M11 = mc + m1 + m2
        // M12 = (m1 + m2) * L1 * c1
        // M13 = m2 * L2 * c2
        // M22 = (m1 + m2) * L1²
        // M23 = m2 * L1 * L2 * c12
        // M33 = m2 * L2²

        T M11 = mc + m1 + m2;
        T M12 = (m1 + m2) * L1 * c1;
        T M13 = m2 * L2 * c2;
        T M22 = (m1 + m2) * L1 * L1;
        T M23 = m2 * L1 * L2 * c12;
        T M33 = m2 * L2 * L2;

        // Right-hand side: τ + B*u
        // Coriolis/centrifugal terms + gravity + control

        // For equation 1 (cart): F - (m1+m2)*L1*s1*ω1² - m2*L2*s2*ω2²
        T rhs1 = F + (m1 + m2) * L1 * s1 * omega1 * omega1
                   + m2 * L2 * s2 * omega2 * omega2;

        // For equation 2 (link 1): (m1+m2)*g*L1*s1 + m2*L1*L2*s12*ω2²
        T rhs2 = (m1 + m2) * g * L1 * s1
               - m2 * L1 * L2 * s12 * omega2 * omega2;

        // For equation 3 (link 2): m2*g*L2*s2 - m2*L1*L2*s12*ω1²
        T rhs3 = m2 * g * L2 * s2
               + m2 * L1 * L2 * s12 * omega1 * omega1;

        // Solve M * [ẍ, α1, α2]ᵀ = [rhs1, rhs2, rhs3]ᵀ
        // Using Cramer's rule for 3x3 system

        // Determinant of M
        T det = M11 * (M22 * M33 - M23 * M23)
              - M12 * (M12 * M33 - M23 * M13)
              + M13 * (M12 * M23 - M22 * M13);

        // Cofactors for first row (for ẍ)
        T C11 = M22 * M33 - M23 * M23;
        T C12 = -(M12 * M33 - M13 * M23);
        T C13 = M12 * M23 - M13 * M22;

        // Cofactors for second row (for α1)
        T C21 = -(M12 * M33 - M13 * M23);
        T C22 = M11 * M33 - M13 * M13;
        T C23 = -(M11 * M23 - M12 * M13);

        // Cofactors for third row (for α2)
        T C31 = M12 * M23 - M13 * M22;
        T C32 = -(M11 * M23 - M12 * M13);
        T C33 = M11 * M22 - M12 * M12;

        // Solve using inverse matrix
        if (det == T(0)) {
            throw std::runtime_error("CartDoublePendulum: mass matrix is singular; cannot compute accelerations.");
        }
        T inv_det = T(1.0) / det;

        T xddot = inv_det * (C11 * rhs1 + C12 * rhs2 + C13 * rhs3);
        T alpha1 = inv_det * (C21 * rhs1 + C22 * rhs2 + C23 * rhs3);
        T alpha2 = inv_det * (C31 * rhs1 + C32 * rhs2 + C33 * rhs3);

        return {xdot, omega1, omega2, xddot, alpha1, alpha2};
    }

    // =========================================================================
    // State Functions
    // =========================================================================

    // Cart state functions
    T compute(cart::Position, std::span<const T> state) const {
        return this->getGlobalState(state, 0);
    }

    T compute(cart::Velocity, std::span<const T> state) const {
        return this->getGlobalState(state, 3);
    }

    T compute(cart::Mass, std::span<const T> /*state*/) const {
        return T(m_cart_mass);
    }

    // Link 1 state functions
    T compute(link1::Angle, std::span<const T> state) const {
        return this->getGlobalState(state, 1);
    }

    T compute(link1::AngularVelocity, std::span<const T> state) const {
        return this->getGlobalState(state, 4);
    }

    T compute(link1::Mass, std::span<const T> /*state*/) const {
        return T(m_mass1);
    }

    T compute(link1::Length, std::span<const T> /*state*/) const {
        return T(m_length1);
    }

    std::array<T, 2> compute(link1::TipPosition, std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;

        T x = this->getGlobalState(state, 0);
        T theta1 = this->getGlobalState(state, 1);
        T L1 = T(m_length1);

        return {x + L1 * sin(theta1), L1 * cos(theta1)};
    }

    // Link 2 state functions
    T compute(link2::Angle, std::span<const T> state) const {
        return this->getGlobalState(state, 2);
    }

    T compute(link2::AngularVelocity, std::span<const T> state) const {
        return this->getGlobalState(state, 5);
    }

    T compute(link2::Mass, std::span<const T> /*state*/) const {
        return T(m_mass2);
    }

    T compute(link2::Length, std::span<const T> /*state*/) const {
        return T(m_length2);
    }

    std::array<T, 2> compute(link2::TipPosition, std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;

        T x = this->getGlobalState(state, 0);
        T theta1 = this->getGlobalState(state, 1);
        T theta2 = this->getGlobalState(state, 2);
        T L1 = T(m_length1);
        T L2 = T(m_length2);

        return {
            x + L1 * sin(theta1) + L2 * sin(theta2),
            L1 * cos(theta1) + L2 * cos(theta2)
        };
    }

    // System state functions
    std::array<T, 6> compute(system::FullState, std::span<const T> state) const {
        return {
            this->getGlobalState(state, 0),
            this->getGlobalState(state, 1),
            this->getGlobalState(state, 2),
            this->getGlobalState(state, 3),
            this->getGlobalState(state, 4),
            this->getGlobalState(state, 5)
        };
    }

    T compute(system::TotalEnergy, std::span<const T> state) const {
        return compute(system::KineticEnergy{}, state) + compute(system::PotentialEnergy{}, state);
    }

    T compute(system::KineticEnergy, std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;

        T x = this->getGlobalState(state, 0);
        T theta1 = this->getGlobalState(state, 1);
        T theta2 = this->getGlobalState(state, 2);
        T xdot = this->getGlobalState(state, 3);
        T omega1 = this->getGlobalState(state, 4);
        T omega2 = this->getGlobalState(state, 5);

        (void)x;

        T mc = T(m_cart_mass);
        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T L1 = T(m_length1);
        T L2 = T(m_length2);

        T c1 = cos(theta1);
        T c2 = cos(theta2);

        // Cart kinetic energy
        T KE_cart = T(0.5) * mc * xdot * xdot;

        // Mass 1 velocity: (ẋ + L1*c1*ω1, -L1*s1*ω1)
        // v1² = ẋ² + 2*L1*c1*ẋ*ω1 + L1²*ω1²
        T v1_sq = xdot * xdot + T(2.0) * L1 * c1 * xdot * omega1 + L1 * L1 * omega1 * omega1;
        T KE_m1 = T(0.5) * m1 * v1_sq;

        // Mass 2 velocity: (ẋ + L1*c1*ω1 + L2*c2*ω2, -L1*s1*ω1 - L2*s2*ω2)
        T s1 = sin(theta1);
        T s2 = sin(theta2);
        T vx2 = xdot + L1 * c1 * omega1 + L2 * c2 * omega2;
        T vy2 = -L1 * s1 * omega1 - L2 * s2 * omega2;
        T v2_sq = vx2 * vx2 + vy2 * vy2;
        T KE_m2 = T(0.5) * m2 * v2_sq;

        return KE_cart + KE_m1 + KE_m2;
    }

    T compute(system::PotentialEnergy, std::span<const T> state) const {
        using std::cos;
        using sopot::cos;

        T theta1 = this->getGlobalState(state, 1);
        T theta2 = this->getGlobalState(state, 2);

        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T L1 = T(m_length1);
        T L2 = T(m_length2);
        T g = T(m_gravity);

        // Height of mass 1: L1*cos(θ1) (measured from pivot)
        // Height of mass 2: L1*cos(θ1) + L2*cos(θ2)
        T h1 = L1 * cos(theta1);
        T h2 = L1 * cos(theta1) + L2 * cos(theta2);

        return m1 * g * h1 + m2 * g * h2;
    }

    // Controller interface
    T compute(controller::ControlInput, std::span<const T> /*state*/) const {
        return m_control_force;
    }

    // Parameter accessors
    double getCartMass() const { return m_cart_mass; }
    double getMass1() const { return m_mass1; }
    double getMass2() const { return m_mass2; }
    double getLength1() const { return m_length1; }
    double getLength2() const { return m_length2; }
    double getGravity() const { return m_gravity; }
};

} // namespace sopot::control
