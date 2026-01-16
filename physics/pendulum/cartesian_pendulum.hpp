#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "tags.hpp"
#include "physics/constraints/symbolic/expression.hpp"
#include "physics/constraints/symbolic/differentiation.hpp"
#include <string>
#include <stdexcept>
#include <array>
#include <cmath>

namespace sopot::pendulum {

/**
 * @brief Double pendulum using Cartesian coordinates with Baumgarte stabilization
 *
 * This formulation uses Cartesian coordinates (x,y) for each mass instead of
 * generalized coordinates (angles). The rigid rod constraints are enforced
 * using Baumgarte stabilization for numerical stability.
 *
 * State vector: [x1, y1, x2, y2, vx1, vy1, vx2, vy2] (8 states)
 *
 * Constraints:
 *   g1 = x1^2 + y1^2 - L1^2 = 0           (mass 1 on circle of radius L1)
 *   g2 = (x2-x1)^2 + (y2-y1)^2 - L2^2 = 0 (mass 2 at distance L2 from mass 1)
 *
 * Baumgarte stabilization adds damping and stiffness to constraint errors:
 *   J*a = -2*alpha*(J*v) - beta^2*g
 *
 * where alpha and beta are stabilization parameters (typically alpha = beta).
 *
 * @tparam T Scalar type (double or Dual for autodiff)
 */
template<Scalar T = double>
class CartesianDoublePendulum final : public TypedComponent<8, T> {
public:
    using Base = TypedComponent<8, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_mass1;
    double m_mass2;
    double m_length1;
    double m_length2;
    double m_gravity;

    // Baumgarte stabilization parameters
    double m_alpha;  // Damping coefficient
    double m_beta;   // Stiffness coefficient

    // Initial conditions (converted from angles for compatibility)
    double m_initial_theta1;
    double m_initial_theta2;
    double m_initial_omega1;
    double m_initial_omega2;

    std::string m_name;

    // =========================================================================
    // Symbolic constraint definitions
    // =========================================================================
    // Variables: x1=0, y1=1, x2=2, y2=3
    using x1 = symbolic::Var<0>;
    using y1 = symbolic::Var<1>;
    using x2 = symbolic::Var<2>;
    using y2 = symbolic::Var<3>;

    // Constraint 1: g1 = x1^2 + y1^2 - L1^2
    // (L1^2 is a parameter, we handle it at runtime)
    using g1_partial = symbolic::Add<
        symbolic::Add<symbolic::Square<x1>, symbolic::Square<y1>>,
        symbolic::Neg<symbolic::Param<0>>  // -L1^2 as parameter
    >;

    // Constraint 2: g2 = (x2-x1)^2 + (y2-y1)^2 - L2^2
    using dx = symbolic::Sub<x2, x1>;
    using dy = symbolic::Sub<y2, y1>;
    using g2_partial = symbolic::Add<
        symbolic::Add<symbolic::Square<dx>, symbolic::Square<dy>>,
        symbolic::Neg<symbolic::Param<1>>  // -L2^2 as parameter
    >;

public:
    /**
     * @brief Construct Cartesian double pendulum
     *
     * @param mass1 Mass of bob 1 (kg)
     * @param mass2 Mass of bob 2 (kg)
     * @param length1 Length of rod 1 (m)
     * @param length2 Length of rod 2 (m)
     * @param gravity Gravitational acceleration (m/s^2)
     * @param theta1_0 Initial angle of rod 1 (rad)
     * @param theta2_0 Initial angle of rod 2 (rad)
     * @param omega1_0 Initial angular velocity of rod 1 (rad/s)
     * @param omega2_0 Initial angular velocity of rod 2 (rad/s)
     * @param alpha Baumgarte damping parameter (default 10)
     * @param beta Baumgarte stiffness parameter (default 10)
     */
    explicit CartesianDoublePendulum(
        double mass1,
        double mass2,
        double length1,
        double length2,
        double gravity = 9.81,
        double theta1_0 = 0.0,
        double theta2_0 = 0.0,
        double omega1_0 = 0.0,
        double omega2_0 = 0.0,
        double alpha = 10.0,
        double beta = 10.0
    )
        : m_mass1(mass1)
        , m_mass2(mass2)
        , m_length1(length1)
        , m_length2(length2)
        , m_gravity(gravity)
        , m_alpha(alpha)
        , m_beta(beta)
        , m_initial_theta1(theta1_0)
        , m_initial_theta2(theta2_0)
        , m_initial_omega1(omega1_0)
        , m_initial_omega2(omega2_0)
        , m_name("CartesianDoublePendulum")
    {
        if (mass1 <= 0.0 || mass2 <= 0.0) {
            throw std::invalid_argument("Masses must be positive");
        }
        if (length1 <= 0.0 || length2 <= 0.0) {
            throw std::invalid_argument("Lengths must be positive");
        }
    }

    // Initial state [x1, y1, x2, y2, vx1, vy1, vx2, vy2]
    LocalState getInitialLocalState() const {
        // Convert from angles to Cartesian
        double x1_0 = m_length1 * std::sin(m_initial_theta1);
        double y1_0 = -m_length1 * std::cos(m_initial_theta1);
        double x2_0 = x1_0 + m_length2 * std::sin(m_initial_theta2);
        double y2_0 = y1_0 - m_length2 * std::cos(m_initial_theta2);

        // Velocities from angular velocities
        double vx1_0 = m_length1 * std::cos(m_initial_theta1) * m_initial_omega1;
        double vy1_0 = m_length1 * std::sin(m_initial_theta1) * m_initial_omega1;
        double vx2_0 = vx1_0 + m_length2 * std::cos(m_initial_theta2) * m_initial_omega2;
        double vy2_0 = vy1_0 + m_length2 * std::sin(m_initial_theta2) * m_initial_omega2;

        return {
            T(x1_0), T(y1_0), T(x2_0), T(y2_0),
            T(vx1_0), T(vy1_0), T(vx2_0), T(vy2_0)
        };
    }

    std::string_view getComponentType() const { return "CartesianDoublePendulum"; }
    std::string_view getComponentName() const { return m_name; }

    /**
     * @brief Compute derivatives using constrained dynamics with Baumgarte stabilization
     *
     * The equations of motion are:
     *   M*a = F_ext + J^T * lambda
     *   J*a = -2*alpha*(J*v) - beta^2*g
     *
     * Solving for lambda and then a:
     *   lambda = (J*M^(-1)*J^T)^(-1) * (b - J*M^(-1)*F_ext)
     *   a = M^(-1) * (F_ext + J^T * lambda)
     *
     * where b = -2*alpha*(J*v) - beta^2*g
     */
    template<typename Registry>
    LocalDerivative derivatives(
        T /*t*/,
        std::span<const T> local,
        std::span<const T> /*global*/,
        const Registry& /*registry*/
    ) const {
        // Extract state
        T x1 = local[0], y1 = local[1], x2 = local[2], y2 = local[3];
        T vx1 = local[4], vy1 = local[5], vx2 = local[6], vy2 = local[7];

        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T g = T(m_gravity);
        T L1_sq = T(m_length1 * m_length1);
        T L2_sq = T(m_length2 * m_length2);
        T alpha = T(m_alpha);
        T beta = T(m_beta);

        // Compute constraint values
        T g1 = x1 * x1 + y1 * y1 - L1_sq;
        T g2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) - L2_sq;

        // Compute constraint Jacobian J (2x4 for positions)
        // dg1/d[x1,y1,x2,y2] = [2*x1, 2*y1, 0, 0]
        // dg2/d[x1,y1,x2,y2] = [-2*(x2-x1), -2*(y2-y1), 2*(x2-x1), 2*(y2-y1)]
        T J11 = T(2.0) * x1;
        T J12 = T(2.0) * y1;
        T J13 = T(0.0);
        T J14 = T(0.0);

        T dx = x2 - x1;
        T dy = y2 - y1;
        T J21 = T(-2.0) * dx;
        T J22 = T(-2.0) * dy;
        T J23 = T(2.0) * dx;
        T J24 = T(2.0) * dy;

        // Constraint velocity: J * v
        T Jv1 = J11 * vx1 + J12 * vy1;
        T Jv2 = J21 * vx1 + J22 * vy1 + J23 * vx2 + J24 * vy2;

        // Baumgarte stabilization right-hand side
        // b = -2*alpha*(J*v) - beta^2*g
        T b1 = T(-2.0) * alpha * Jv1 - beta * beta * g1;
        T b2 = T(-2.0) * alpha * Jv2 - beta * beta * g2;

        // External forces (gravity)
        T Fx1 = T(0.0), Fy1 = -m1 * g;
        T Fx2 = T(0.0), Fy2 = -m2 * g;

        // Mass matrix is diagonal: M = diag(m1, m1, m2, m2)
        // M^(-1) = diag(1/m1, 1/m1, 1/m2, 1/m2)
        T inv_m1 = T(1.0) / m1;
        T inv_m2 = T(1.0) / m2;

        // Compute J * M^(-1) * J^T (2x2 matrix)
        // JMJt[0][0] = J1 * diag(1/m1,1/m1,1/m2,1/m2) * J1^T
        //            = (J11^2 + J12^2)/m1 + (J13^2 + J14^2)/m2
        T JMJt11 = (J11 * J11 + J12 * J12) * inv_m1;
        T JMJt12 = (J11 * J21 + J12 * J22) * inv_m1;
        T JMJt21 = JMJt12;  // Symmetric
        T JMJt22 = (J21 * J21 + J22 * J22) * inv_m1 + (J23 * J23 + J24 * J24) * inv_m2;

        // Compute J * M^(-1) * F_ext
        T JMinvF1 = (J11 * Fx1 + J12 * Fy1) * inv_m1;
        T JMinvF2 = (J21 * Fx1 + J22 * Fy1) * inv_m1 + (J23 * Fx2 + J24 * Fy2) * inv_m2;

        // Right-hand side: b - J*M^(-1)*F
        T rhs1 = b1 - JMinvF1;
        T rhs2 = b2 - JMinvF2;

        // Solve 2x2 system for lambda: (J*M^(-1)*J^T) * lambda = rhs
        T det = JMJt11 * JMJt22 - JMJt12 * JMJt21;
        T lambda1 = (JMJt22 * rhs1 - JMJt12 * rhs2) / det;
        T lambda2 = (JMJt11 * rhs2 - JMJt21 * rhs1) / det;

        // Compute constraint forces: J^T * lambda
        T Fc_x1 = J11 * lambda1 + J21 * lambda2;
        T Fc_y1 = J12 * lambda1 + J22 * lambda2;
        T Fc_x2 = J13 * lambda1 + J23 * lambda2;
        T Fc_y2 = J14 * lambda1 + J24 * lambda2;

        // Compute accelerations: a = M^(-1) * (F_ext + F_constraint)
        T ax1 = (Fx1 + Fc_x1) * inv_m1;
        T ay1 = (Fy1 + Fc_y1) * inv_m1;
        T ax2 = (Fx2 + Fc_x2) * inv_m2;
        T ay2 = (Fy2 + Fc_y2) * inv_m2;

        // Return derivatives: [vx1, vy1, vx2, vy2, ax1, ay1, ax2, ay2]
        return {vx1, vy1, vx2, vy2, ax1, ay1, ax2, ay2};
    }

    // =========================================================================
    // State Functions
    // =========================================================================

    // Mass 1 angle (derived from Cartesian)
    T compute(mass1::Angle, std::span<const T> state) const {
        T x1 = this->getGlobalState(state, 0);
        T y1 = this->getGlobalState(state, 1);
        using std::atan2;
        return atan2(x1, -y1);
    }

    // Mass 1 angular velocity (derived)
    T compute(mass1::AngularVelocity, std::span<const T> state) const {
        T x1 = this->getGlobalState(state, 0);
        T y1 = this->getGlobalState(state, 1);
        T vx1 = this->getGlobalState(state, 4);
        T vy1 = this->getGlobalState(state, 5);
        T L1 = T(m_length1);
        // omega1 = (x1*vy1 - y1*vx1) / L1^2
        return (x1 * vy1 - y1 * vx1) / (L1 * L1);
    }

    // Cartesian position of mass 1
    std::array<T, 2> compute(mass1::CartesianPosition, std::span<const T> state) const {
        return {this->getGlobalState(state, 0), this->getGlobalState(state, 1)};
    }

    // Cartesian velocity of mass 1
    std::array<T, 2> compute(mass1::CartesianVelocity, std::span<const T> state) const {
        return {this->getGlobalState(state, 4), this->getGlobalState(state, 5)};
    }

    T compute(mass1::Mass, std::span<const T> /*state*/) const {
        return T(m_mass1);
    }

    // Mass 2 angle (derived)
    T compute(mass2::Angle, std::span<const T> state) const {
        T x1 = this->getGlobalState(state, 0);
        T y1 = this->getGlobalState(state, 1);
        T x2 = this->getGlobalState(state, 2);
        T y2 = this->getGlobalState(state, 3);
        T dx = x2 - x1;
        T dy = y2 - y1;
        using std::atan2;
        return atan2(dx, -dy);
    }

    // Mass 2 angular velocity (derived)
    T compute(mass2::AngularVelocity, std::span<const T> state) const {
        T x1 = this->getGlobalState(state, 0);
        T y1 = this->getGlobalState(state, 1);
        T x2 = this->getGlobalState(state, 2);
        T y2 = this->getGlobalState(state, 3);
        T vx1 = this->getGlobalState(state, 4);
        T vy1 = this->getGlobalState(state, 5);
        T vx2 = this->getGlobalState(state, 6);
        T vy2 = this->getGlobalState(state, 7);

        T dx = x2 - x1;
        T dy = y2 - y1;
        T dvx = vx2 - vx1;
        T dvy = vy2 - vy1;
        T L2 = T(m_length2);
        return (dx * dvy - dy * dvx) / (L2 * L2);
    }

    std::array<T, 2> compute(mass2::CartesianPosition, std::span<const T> state) const {
        return {this->getGlobalState(state, 2), this->getGlobalState(state, 3)};
    }

    std::array<T, 2> compute(mass2::CartesianVelocity, std::span<const T> state) const {
        return {this->getGlobalState(state, 6), this->getGlobalState(state, 7)};
    }

    T compute(mass2::Mass, std::span<const T> /*state*/) const {
        return T(m_mass2);
    }

    T compute(system::Length1, std::span<const T> /*state*/) const {
        return T(m_length1);
    }

    T compute(system::Length2, std::span<const T> /*state*/) const {
        return T(m_length2);
    }

    // Kinetic energy
    T compute(system::KineticEnergy, std::span<const T> state) const {
        T vx1 = this->getGlobalState(state, 4);
        T vy1 = this->getGlobalState(state, 5);
        T vx2 = this->getGlobalState(state, 6);
        T vy2 = this->getGlobalState(state, 7);

        T m1 = T(m_mass1);
        T m2 = T(m_mass2);

        T v1_sq = vx1 * vx1 + vy1 * vy1;
        T v2_sq = vx2 * vx2 + vy2 * vy2;

        return T(0.5) * m1 * v1_sq + T(0.5) * m2 * v2_sq;
    }

    // Potential energy
    T compute(system::PotentialEnergy, std::span<const T> state) const {
        T y1 = this->getGlobalState(state, 1);
        T y2 = this->getGlobalState(state, 3);

        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T g = T(m_gravity);

        return m1 * g * y1 + m2 * g * y2;
    }

    T compute(system::TotalEnergy, std::span<const T> state) const {
        return compute(system::KineticEnergy{}, state) + compute(system::PotentialEnergy{}, state);
    }

    // Constraint error (useful for monitoring)
    T getConstraintError(std::span<const T> state) const {
        T x1 = this->getGlobalState(state, 0);
        T y1 = this->getGlobalState(state, 1);
        T x2 = this->getGlobalState(state, 2);
        T y2 = this->getGlobalState(state, 3);

        T L1_sq = T(m_length1 * m_length1);
        T L2_sq = T(m_length2 * m_length2);

        T g1 = x1 * x1 + y1 * y1 - L1_sq;
        T g2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) - L2_sq;

        using std::abs;
        using std::sqrt;
        return sqrt(g1 * g1 + g2 * g2);
    }

    // Accessors
    double getMass1() const { return m_mass1; }
    double getMass2() const { return m_mass2; }
    double getLength1() const { return m_length1; }
    double getLength2() const { return m_length2; }
    double getGravity() const { return m_gravity; }
    double getAlpha() const { return m_alpha; }
    double getBeta() const { return m_beta; }
};

} // namespace sopot::pendulum
