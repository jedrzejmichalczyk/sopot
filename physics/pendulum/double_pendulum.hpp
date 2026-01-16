#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "tags.hpp"
#include <string>
#include <stdexcept>
#include <array>
#include <cmath>

namespace sopot::pendulum {

/**
 * @brief Double pendulum using generalized coordinates (Lagrangian formulation)
 *
 * State vector: [theta1, theta2, omega1, omega2]
 * - theta1: angle of rod 1 from vertical (positive clockwise)
 * - theta2: angle of rod 2 from vertical (positive clockwise)
 * - omega1: angular velocity of rod 1
 * - omega2: angular velocity of rod 2
 *
 * Equations of motion derived from Euler-Lagrange equations:
 *
 *   (m1+m2)L1*alpha1 + m2*L2*alpha2*cos(delta) = m2*L2*omega2^2*sin(delta) - (m1+m2)*g*sin(theta1)
 *   L2*alpha2 + L1*alpha1*cos(delta) = -L1*omega1^2*sin(delta) - g*sin(theta2)
 *
 * where delta = theta1 - theta2
 *
 * Solved using Cramer's rule for alpha1, alpha2.
 *
 * @tparam T Scalar type (double or Dual for autodiff)
 */
template<Scalar T = double>
class DoublePendulum final : public TypedComponent<4, T> {
public:
    using Base = TypedComponent<4, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_mass1;     // Mass of bob 1 (kg)
    double m_mass2;     // Mass of bob 2 (kg)
    double m_length1;   // Length of rod 1 (m)
    double m_length2;   // Length of rod 2 (m)
    double m_gravity;   // Gravitational acceleration (m/s^2)

    double m_initial_theta1;  // Initial angle of rod 1 (rad)
    double m_initial_theta2;  // Initial angle of rod 2 (rad)
    double m_initial_omega1;  // Initial angular velocity of rod 1 (rad/s)
    double m_initial_omega2;  // Initial angular velocity of rod 2 (rad/s)

    std::string m_name;

public:
    /**
     * @brief Construct double pendulum with specified parameters
     *
     * @param mass1 Mass of bob 1 (kg) - must be positive
     * @param mass2 Mass of bob 2 (kg) - must be positive
     * @param length1 Length of rod 1 (m) - must be positive
     * @param length2 Length of rod 2 (m) - must be positive
     * @param gravity Gravitational acceleration (m/s^2) - default 9.81
     * @param theta1_0 Initial angle of rod 1 (rad) - default 0
     * @param theta2_0 Initial angle of rod 2 (rad) - default 0
     * @param omega1_0 Initial angular velocity of rod 1 (rad/s) - default 0
     * @param omega2_0 Initial angular velocity of rod 2 (rad/s) - default 0
     */
    explicit DoublePendulum(
        double mass1,
        double mass2,
        double length1,
        double length2,
        double gravity = 9.81,
        double theta1_0 = 0.0,
        double theta2_0 = 0.0,
        double omega1_0 = 0.0,
        double omega2_0 = 0.0
    )
        : m_mass1(mass1)
        , m_mass2(mass2)
        , m_length1(length1)
        , m_length2(length2)
        , m_gravity(gravity)
        , m_initial_theta1(theta1_0)
        , m_initial_theta2(theta2_0)
        , m_initial_omega1(omega1_0)
        , m_initial_omega2(omega2_0)
        , m_name("DoublePendulum")
    {
        if (mass1 <= 0.0) {
            throw std::invalid_argument("Mass 1 must be positive (got " + std::to_string(mass1) + ")");
        }
        if (mass2 <= 0.0) {
            throw std::invalid_argument("Mass 2 must be positive (got " + std::to_string(mass2) + ")");
        }
        if (length1 <= 0.0) {
            throw std::invalid_argument("Length 1 must be positive (got " + std::to_string(length1) + ")");
        }
        if (length2 <= 0.0) {
            throw std::invalid_argument("Length 2 must be positive (got " + std::to_string(length2) + ")");
        }
    }

    // Initial state [theta1, theta2, omega1, omega2]
    LocalState getInitialLocalState() const {
        return {
            T(m_initial_theta1),
            T(m_initial_theta2),
            T(m_initial_omega1),
            T(m_initial_omega2)
        };
    }

    // Component identification
    std::string_view getComponentType() const { return "DoublePendulum"; }
    std::string_view getComponentName() const { return m_name; }

    /**
     * @brief Compute derivatives using Euler-Lagrange equations
     *
     * d/dt[theta1, theta2, omega1, omega2] = [omega1, omega2, alpha1, alpha2]
     *
     * The angular accelerations are computed by solving the 2x2 linear system
     * from the Euler-Lagrange equations.
     */
    template<typename Registry>
    LocalDerivative derivatives(
        T /*t*/,
        std::span<const T> local,
        std::span<const T> /*global*/,
        const Registry& /*registry*/
    ) const {
        using std::sin;
        using std::cos;
        using sopot::sin;
        using sopot::cos;

        // Extract state
        T theta1 = local[0];
        T theta2 = local[1];
        T omega1 = local[2];
        T omega2 = local[3];

        // Compute angular accelerations
        T delta = theta1 - theta2;
        T sin_delta = sin(delta);
        T cos_delta = cos(delta);

        T sin_theta1 = sin(theta1);
        T sin_theta2 = sin(theta2);

        // Mass and length parameters
        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T L1 = T(m_length1);
        T L2 = T(m_length2);
        T g = T(m_gravity);

        // Coefficient matrix elements
        // [a11  a12] [alpha1]   [b1]
        // [a21  a22] [alpha2] = [b2]
        T a11 = (m1 + m2) * L1;
        T a12 = m2 * L2 * cos_delta;
        T a21 = L1 * cos_delta;
        T a22 = L2;

        // Right-hand side
        T b1 = m2 * L2 * omega2 * omega2 * sin_delta - (m1 + m2) * g * sin_theta1;
        T b2 = -L1 * omega1 * omega1 * sin_delta - g * sin_theta2;

        // Solve via Cramer's rule
        T det = a11 * a22 - a12 * a21;

        T alpha1 = (b1 * a22 - b2 * a12) / det;
        T alpha2 = (a11 * b2 - a21 * b1) / det;

        // Return derivatives: [d(theta1)/dt, d(theta2)/dt, d(omega1)/dt, d(omega2)/dt]
        return {omega1, omega2, alpha1, alpha2};
    }

    // =========================================================================
    // State Functions
    // =========================================================================

    // Angle of rod 1
    T compute(mass1::Angle, std::span<const T> state) const {
        return this->getGlobalState(state, 0);
    }

    // Angular velocity of rod 1
    T compute(mass1::AngularVelocity, std::span<const T> state) const {
        return this->getGlobalState(state, 2);
    }

    // Cartesian position of mass 1: (L1*sin(theta1), -L1*cos(theta1))
    // Note: y is positive downward in this convention
    std::array<T, 2> compute(mass1::CartesianPosition, std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;
        T theta1 = this->getGlobalState(state, 0);
        T L1 = T(m_length1);
        return {L1 * sin(theta1), -L1 * cos(theta1)};
    }

    // Cartesian velocity of mass 1
    std::array<T, 2> compute(mass1::CartesianVelocity, std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;
        T theta1 = this->getGlobalState(state, 0);
        T omega1 = this->getGlobalState(state, 2);
        T L1 = T(m_length1);
        // v1 = d/dt(L1*sin(theta1), -L1*cos(theta1)) = (L1*cos(theta1)*omega1, L1*sin(theta1)*omega1)
        return {L1 * cos(theta1) * omega1, L1 * sin(theta1) * omega1};
    }

    // Mass of bob 1
    T compute(mass1::Mass, std::span<const T> /*state*/) const {
        return T(m_mass1);
    }

    // Angle of rod 2
    T compute(mass2::Angle, std::span<const T> state) const {
        return this->getGlobalState(state, 1);
    }

    // Angular velocity of rod 2
    T compute(mass2::AngularVelocity, std::span<const T> state) const {
        return this->getGlobalState(state, 3);
    }

    // Cartesian position of mass 2
    std::array<T, 2> compute(mass2::CartesianPosition, std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;
        T theta1 = this->getGlobalState(state, 0);
        T theta2 = this->getGlobalState(state, 1);
        T L1 = T(m_length1);
        T L2 = T(m_length2);
        // Position of mass 2: position of mass 1 + L2*(sin(theta2), -cos(theta2))
        T x2 = L1 * sin(theta1) + L2 * sin(theta2);
        T y2 = -L1 * cos(theta1) - L2 * cos(theta2);
        return {x2, y2};
    }

    // Cartesian velocity of mass 2
    std::array<T, 2> compute(mass2::CartesianVelocity, std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;
        T theta1 = this->getGlobalState(state, 0);
        T theta2 = this->getGlobalState(state, 1);
        T omega1 = this->getGlobalState(state, 2);
        T omega2 = this->getGlobalState(state, 3);
        T L1 = T(m_length1);
        T L2 = T(m_length2);
        // v2 = v1 + d/dt(L2*sin(theta2), -L2*cos(theta2))
        T vx2 = L1 * cos(theta1) * omega1 + L2 * cos(theta2) * omega2;
        T vy2 = L1 * sin(theta1) * omega1 + L2 * sin(theta2) * omega2;
        return {vx2, vy2};
    }

    // Mass of bob 2
    T compute(mass2::Mass, std::span<const T> /*state*/) const {
        return T(m_mass2);
    }

    // Rod lengths
    T compute(system::Length1, std::span<const T> /*state*/) const {
        return T(m_length1);
    }

    T compute(system::Length2, std::span<const T> /*state*/) const {
        return T(m_length2);
    }

    // Kinetic energy: T = 0.5*m1*v1^2 + 0.5*m2*v2^2
    T compute(system::KineticEnergy, std::span<const T> state) const {
        using std::cos;
        using sopot::cos;
        T theta1 = this->getGlobalState(state, 0);
        T theta2 = this->getGlobalState(state, 1);
        T omega1 = this->getGlobalState(state, 2);
        T omega2 = this->getGlobalState(state, 3);

        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T L1 = T(m_length1);
        T L2 = T(m_length2);

        // v1^2 = L1^2 * omega1^2
        T v1_sq = L1 * L1 * omega1 * omega1;

        // v2^2 = (L1*cos(theta1)*omega1 + L2*cos(theta2)*omega2)^2 +
        //        (L1*sin(theta1)*omega1 + L2*sin(theta2)*omega2)^2
        // Expanding and simplifying:
        // v2^2 = L1^2*omega1^2 + L2^2*omega2^2 + 2*L1*L2*omega1*omega2*cos(theta1-theta2)
        T cos_delta = cos(theta1 - theta2);
        T v2_sq = L1 * L1 * omega1 * omega1 +
                  L2 * L2 * omega2 * omega2 +
                  T(2.0) * L1 * L2 * omega1 * omega2 * cos_delta;

        return T(0.5) * m1 * v1_sq + T(0.5) * m2 * v2_sq;
    }

    // Potential energy: V = -m1*g*L1*cos(theta1) - m2*g*(L1*cos(theta1) + L2*cos(theta2))
    // Reference: potential = 0 when both rods are horizontal
    T compute(system::PotentialEnergy, std::span<const T> state) const {
        using std::cos;
        using sopot::cos;
        T theta1 = this->getGlobalState(state, 0);
        T theta2 = this->getGlobalState(state, 1);

        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T L1 = T(m_length1);
        T L2 = T(m_length2);
        T g = T(m_gravity);

        // y1 = -L1*cos(theta1), y2 = -L1*cos(theta1) - L2*cos(theta2)
        // V = m1*g*y1 + m2*g*y2 = -g*(m1*L1*cos(theta1) + m2*(L1*cos(theta1) + L2*cos(theta2)))
        T cos_theta1 = cos(theta1);
        T cos_theta2 = cos(theta2);

        return -g * ((m1 + m2) * L1 * cos_theta1 + m2 * L2 * cos_theta2);
    }

    // Total mechanical energy
    T compute(system::TotalEnergy, std::span<const T> state) const {
        return compute(system::KineticEnergy{}, state) + compute(system::PotentialEnergy{}, state);
    }

    // Accessors
    double getMass1() const { return m_mass1; }
    double getMass2() const { return m_mass2; }
    double getLength1() const { return m_length1; }
    double getLength2() const { return m_length2; }
    double getGravity() const { return m_gravity; }
};

} // namespace sopot::pendulum
