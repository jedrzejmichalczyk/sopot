#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "tags.hpp"
#include "physics/constraints/symbolic/named_expression.hpp"
#include <string>
#include <stdexcept>
#include <array>
#include <cmath>

namespace sopot::pendulum {

/**
 * @brief Double pendulum using NAMED symbolic constraints
 *
 * This version demonstrates the improved ergonomics of the named constraint
 * system. Compare the constraint definitions here vs SymbolicCartesianPendulum:
 *
 * OLD (indexed):
 *   using sym_x1 = symbolic::Var<0>;
 *   using sym_y1 = symbolic::Var<1>;
 *   using g1_base = symbolic::Add<symbolic::Square<sym_x1>, symbolic::Square<sym_y1>>;
 *
 * NEW (named):
 *   using namespace symbolic::cartesian::two_body_2d;
 *   auto g1 = sq(x1) + sq(y1);              // Rod 1 length constraint
 *   auto g2 = sq(x2-x1) + sq(y2-y1);        // Rod 2 length constraint
 *
 * The named version reads like the mathematical definition!
 *
 * State vector: [x1, y1, x2, y2, vx1, vy1, vx2, vy2] (8 states)
 */
template<Scalar T = double>
class NamedConstraintPendulum final : public TypedComponent<8, T> {
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
    double m_alpha;
    double m_beta;

    double m_initial_theta1;
    double m_initial_theta2;
    double m_initial_omega1;
    double m_initial_omega2;

    std::string m_name;

    // =========================================================================
    // NAMED Constraint Definitions - Much more readable!
    // =========================================================================

    // Named symbols for two-body 2D system (using full qualification)
    // Position variables map to: x1=0, y1=1, x2=2, y2=3
    static constexpr symbolic::NamedSymbol<0> x1{"x1"};
    static constexpr symbolic::NamedSymbol<1> y1{"y1"};
    static constexpr symbolic::NamedSymbol<2> x2{"x2"};
    static constexpr symbolic::NamedSymbol<3> y2{"y2"};

    // Constraint 1: Rod 1 connects origin to mass 1
    //   g1 = x1² + y1² - L1² = 0
    //
    // With named symbols, this reads like the mathematical definition:
    static constexpr auto g1_expr = symbolic::sq(x1) + symbolic::sq(y1);
    using g1_type = decltype(g1_expr)::type;

    // Constraint 2: Rod 2 connects mass 1 to mass 2
    //   g2 = (x2-x1)² + (y2-y1)² - L2² = 0
    //
    // Again, reads like the math:
    static constexpr auto g2_expr = symbolic::sq(x2 - x1) + symbolic::sq(y2 - y1);
    using g2_type = decltype(g2_expr)::type;

    // Jacobian is still computed at compile time from these expressions
    using ConstraintJacobian = symbolic::Jacobian<4, g1_type, g2_type>;

    // =========================================================================
    // Helper: Evaluate symbolic Jacobian
    // =========================================================================
    std::array<std::array<T, 4>, 2> computeJacobian(T x1_val, T y1_val, T x2_val, T y2_val) const {
        std::array<T, 4> pos = {x1_val, y1_val, x2_val, y2_val};
        return ConstraintJacobian::eval(pos);
    }

public:
    explicit NamedConstraintPendulum(
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
        , m_name("NamedConstraintPendulum")
    {
        if (mass1 <= 0.0 || mass2 <= 0.0) {
            throw std::invalid_argument("Masses must be positive");
        }
        if (length1 <= 0.0 || length2 <= 0.0) {
            throw std::invalid_argument("Lengths must be positive");
        }
    }

    LocalState getInitialLocalState() const {
        double x1_0 = m_length1 * std::sin(m_initial_theta1);
        double y1_0 = -m_length1 * std::cos(m_initial_theta1);
        double x2_0 = x1_0 + m_length2 * std::sin(m_initial_theta2);
        double y2_0 = y1_0 - m_length2 * std::cos(m_initial_theta2);

        double vx1_0 = m_length1 * std::cos(m_initial_theta1) * m_initial_omega1;
        double vy1_0 = m_length1 * std::sin(m_initial_theta1) * m_initial_omega1;
        double vx2_0 = vx1_0 + m_length2 * std::cos(m_initial_theta2) * m_initial_omega2;
        double vy2_0 = vy1_0 + m_length2 * std::sin(m_initial_theta2) * m_initial_omega2;

        return {
            T(x1_0), T(y1_0), T(x2_0), T(y2_0),
            T(vx1_0), T(vy1_0), T(vx2_0), T(vy2_0)
        };
    }

    std::string_view getComponentType() const { return "NamedConstraintPendulum"; }
    std::string_view getComponentName() const { return m_name; }

    /**
     * @brief Compute derivatives using named symbolic constraints
     */
    template<typename Registry>
    LocalDerivative derivatives(
        T /*t*/,
        std::span<const T> local,
        std::span<const T> /*global*/,
        const Registry& /*registry*/
    ) const {
        // Extract state
        T x1_v = local[0], y1_v = local[1], x2_v = local[2], y2_v = local[3];
        T vx1 = local[4], vy1 = local[5], vx2 = local[6], vy2 = local[7];

        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T g = T(m_gravity);
        T L1_sq = T(m_length1 * m_length1);
        T L2_sq = T(m_length2 * m_length2);
        T alpha = T(m_alpha);
        T beta = T(m_beta);

        // Evaluate constraints using the named expressions
        std::array<T, 4> pos = {x1_v, y1_v, x2_v, y2_v};
        T g1 = g1_expr.eval(pos) - L1_sq;
        T g2 = g2_expr.eval(pos) - L2_sq;

        // Compute Jacobian (automatically differentiated at compile time)
        auto J = computeJacobian(x1_v, y1_v, x2_v, y2_v);

        T J11 = J[0][0], J12 = J[0][1], J13 = J[0][2], J14 = J[0][3];
        T J21 = J[1][0], J22 = J[1][1], J23 = J[1][2], J24 = J[1][3];

        // Constraint velocity: J * v
        T Jv1 = J11 * vx1 + J12 * vy1 + J13 * vx2 + J14 * vy2;
        T Jv2 = J21 * vx1 + J22 * vy1 + J23 * vx2 + J24 * vy2;

        // Baumgarte stabilization
        T b1 = T(-2.0) * alpha * Jv1 - beta * beta * g1;
        T b2 = T(-2.0) * alpha * Jv2 - beta * beta * g2;

        // External forces (gravity)
        T Fx1 = T(0.0), Fy1 = -m1 * g;
        T Fx2 = T(0.0), Fy2 = -m2 * g;

        // Inverse masses
        T inv_m1 = T(1.0) / m1;
        T inv_m2 = T(1.0) / m2;

        // J * M^(-1) * J^T
        T JMJt11 = (J11 * J11 + J12 * J12) * inv_m1 + (J13 * J13 + J14 * J14) * inv_m2;
        T JMJt12 = (J11 * J21 + J12 * J22) * inv_m1 + (J13 * J23 + J14 * J24) * inv_m2;
        T JMJt21 = JMJt12;
        T JMJt22 = (J21 * J21 + J22 * J22) * inv_m1 + (J23 * J23 + J24 * J24) * inv_m2;

        // J * M^(-1) * F_ext
        T JMinvF1 = (J11 * Fx1 + J12 * Fy1) * inv_m1 + (J13 * Fx2 + J14 * Fy2) * inv_m2;
        T JMinvF2 = (J21 * Fx1 + J22 * Fy1) * inv_m1 + (J23 * Fx2 + J24 * Fy2) * inv_m2;

        // Solve for λ
        T rhs1 = b1 - JMinvF1;
        T rhs2 = b2 - JMinvF2;
        T det = JMJt11 * JMJt22 - JMJt12 * JMJt21;
        T lambda1 = (JMJt22 * rhs1 - JMJt12 * rhs2) / det;
        T lambda2 = (JMJt11 * rhs2 - JMJt21 * rhs1) / det;

        // Constraint forces
        T Fc_x1 = J11 * lambda1 + J21 * lambda2;
        T Fc_y1 = J12 * lambda1 + J22 * lambda2;
        T Fc_x2 = J13 * lambda1 + J23 * lambda2;
        T Fc_y2 = J14 * lambda1 + J24 * lambda2;

        // Accelerations
        T ax1 = (Fx1 + Fc_x1) * inv_m1;
        T ay1 = (Fy1 + Fc_y1) * inv_m1;
        T ax2 = (Fx2 + Fc_x2) * inv_m2;
        T ay2 = (Fy2 + Fc_y2) * inv_m2;

        return {vx1, vy1, vx2, vy2, ax1, ay1, ax2, ay2};
    }

    // State functions
    T compute(mass1::Angle, std::span<const T> state) const {
        using std::atan2; using sopot::atan2;
        T x1_v = this->getGlobalState(state, 0);
        T y1_v = this->getGlobalState(state, 1);
        return atan2(x1_v, -y1_v);
    }

    std::array<T, 2> compute(mass1::CartesianPosition, std::span<const T> state) const {
        return {this->getGlobalState(state, 0), this->getGlobalState(state, 1)};
    }

    std::array<T, 2> compute(mass2::CartesianPosition, std::span<const T> state) const {
        return {this->getGlobalState(state, 2), this->getGlobalState(state, 3)};
    }

    T compute(system::TotalEnergy, std::span<const T> state) const {
        T vx1 = this->getGlobalState(state, 4);
        T vy1 = this->getGlobalState(state, 5);
        T vx2 = this->getGlobalState(state, 6);
        T vy2 = this->getGlobalState(state, 7);
        T y1_v = this->getGlobalState(state, 1);
        T y2_v = this->getGlobalState(state, 3);

        T m1 = T(m_mass1);
        T m2 = T(m_mass2);
        T grav = T(m_gravity);

        T KE = T(0.5) * m1 * (vx1*vx1 + vy1*vy1) + T(0.5) * m2 * (vx2*vx2 + vy2*vy2);
        T PE = m1 * grav * y1_v + m2 * grav * y2_v;

        return KE + PE;
    }

    T getConstraintError(std::span<const T> state) const {
        T x1_v = this->getGlobalState(state, 0);
        T y1_v = this->getGlobalState(state, 1);
        T x2_v = this->getGlobalState(state, 2);
        T y2_v = this->getGlobalState(state, 3);

        T L1_sq = T(m_length1 * m_length1);
        T L2_sq = T(m_length2 * m_length2);

        std::array<T, 4> pos = {x1_v, y1_v, x2_v, y2_v};
        T g1 = g1_expr.eval(pos) - L1_sq;
        T g2 = g2_expr.eval(pos) - L2_sq;

        using std::sqrt;
        return sqrt(g1 * g1 + g2 * g2);
    }
};

} // namespace sopot::pendulum
