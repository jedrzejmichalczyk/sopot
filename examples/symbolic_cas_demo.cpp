/**
 * @file symbolic_cas_demo.cpp
 * @brief Demonstrates the compile-time CAS for constraint mechanics
 *
 * This example shows how to:
 * 1. Define constraints symbolically
 * 2. Automatically compute Jacobians at compile time
 * 3. Use them for constrained dynamics
 */

#include "physics/constraints/symbolic/expression.hpp"
#include "physics/constraints/symbolic/differentiation.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>

using namespace sopot::symbolic;

// =============================================================================
// Example 1: Simple Pendulum Constraint
// =============================================================================
void simplePendulumExample() {
    std::cout << "=== Example 1: Simple Pendulum Constraint ===" << std::endl;

    // Define symbolic variables: x, y (Cartesian position)
    using x = Var<0>;
    using y = Var<1>;

    // Constraint: g(x,y) = x² + y² - L² = 0
    // For L=1, this becomes: g = x² + y² - 1
    using g = Sub<Add<Square<x>, Square<y>>, One>;

    // The Jacobian is computed at COMPILE TIME:
    // dg/dx = 2x, dg/dy = 2y
    using dg_dx = Diff_t<g, 0>;  // This is a TYPE representing 2x
    using dg_dy = Diff_t<g, 1>;  // This is a TYPE representing 2y

    // Test point on the circle: (0.6, -0.8)
    std::array<double, 2> pos = {0.6, -0.8};

    // Evaluate constraint (should be 0)
    double g_val = eval<g>(pos);
    std::cout << "  Position: (" << pos[0] << ", " << pos[1] << ")" << std::endl;
    std::cout << "  Constraint g = x² + y² - 1 = " << g_val << std::endl;

    // Evaluate Jacobian (gradient)
    auto J = Gradient<g, 2>::eval(pos);
    std::cout << "  Jacobian: [" << J[0] << ", " << J[1] << "]" << std::endl;
    std::cout << "  Expected: [1.2, -1.6] (= 2*x, 2*y)" << std::endl;

    // Verify: J should be perpendicular to valid velocity
    // For circular motion, v = (-y*ω, x*ω), so J·v = 0
    double omega = 1.0;
    double vx = -pos[1] * omega;  // 0.8
    double vy = pos[0] * omega;   // 0.6
    double J_dot_v = J[0] * vx + J[1] * vy;
    std::cout << "  J · v (should be 0): " << J_dot_v << std::endl;

    std::cout << std::endl;
}

// =============================================================================
// Example 2: Double Pendulum Constraints
// =============================================================================
void doublePendulumExample() {
    std::cout << "=== Example 2: Double Pendulum Constraints ===" << std::endl;

    // Variables: x1, y1, x2, y2
    using x1 = Var<0>;
    using y1 = Var<1>;
    using x2 = Var<2>;
    using y2 = Var<3>;

    // Constraint 1: x1² + y1² = L1² (mass 1 on circle)
    // For L1=1: g1 = x1² + y1² - 1
    using g1 = Sub<Add<Square<x1>, Square<y1>>, One>;

    // Constraint 2: (x2-x1)² + (y2-y1)² = L2² (mass 2 at distance L2 from mass 1)
    // For L2=1: g2 = (x2-x1)² + (y2-y1)² - 1
    using dx = Sub<x2, x1>;
    using dy = Sub<y2, y1>;
    using g2 = Sub<Add<Square<dx>, Square<dy>>, One>;

    // Compute full Jacobian at compile time!
    // J = [dg1/dx1  dg1/dy1  dg1/dx2  dg1/dy2]
    //     [dg2/dx1  dg2/dy1  dg2/dx2  dg2/dy2]
    using ConstraintJacobian = Jacobian<4, g1, g2>;

    // Test configuration
    std::array<double, 4> pos = {0.6, -0.8, 1.4, -1.4};
    // Mass 1 at (0.6, -0.8), Mass 2 at (1.4, -1.4)
    // dx = 0.8, dy = -0.6

    // Evaluate constraints
    double g1_val = eval<g1>(pos);
    double g2_val = eval<g2>(pos);
    std::cout << "  Mass 1 position: (" << pos[0] << ", " << pos[1] << ")" << std::endl;
    std::cout << "  Mass 2 position: (" << pos[2] << ", " << pos[3] << ")" << std::endl;
    std::cout << "  g1 (rod 1 constraint): " << g1_val << std::endl;
    std::cout << "  g2 (rod 2 constraint): " << g2_val << std::endl;

    // Evaluate Jacobian
    auto J = ConstraintJacobian::eval(pos);

    std::cout << "  Jacobian (2x4):" << std::endl;
    std::cout << "    [" << std::setw(8) << J[0][0] << " " << std::setw(8) << J[0][1]
              << " " << std::setw(8) << J[0][2] << " " << std::setw(8) << J[0][3] << "]" << std::endl;
    std::cout << "    [" << std::setw(8) << J[1][0] << " " << std::setw(8) << J[1][1]
              << " " << std::setw(8) << J[1][2] << " " << std::setw(8) << J[1][3] << "]" << std::endl;

    std::cout << "  Expected:" << std::endl;
    std::cout << "    [     1.2     -1.6        0        0]  (2x1, 2y1, 0, 0)" << std::endl;
    std::cout << "    [    -1.6      1.2      1.6     -1.2]  (-2dx, -2dy, 2dx, 2dy)" << std::endl;

    std::cout << std::endl;
}

// =============================================================================
// Example 3: Computing Constraint Forces
// =============================================================================
void constraintForcesExample() {
    std::cout << "=== Example 3: Constraint Force Computation ===" << std::endl;

    // Simple pendulum: find constraint force to keep mass on circle

    using x = Var<0>;
    using y = Var<1>;
    using g = Sub<Add<Square<x>, Square<y>>, One>;

    // Configuration
    double m = 1.0;           // mass
    double grav = 9.81;       // gravity
    std::array<double, 2> pos = {0.6, -0.8};

    // External force (gravity, pointing down)
    double Fx_ext = 0.0;
    double Fy_ext = -m * grav;

    // Get Jacobian
    auto J = Gradient<g, 2>::eval(pos);
    // J = [2x, 2y] = [1.2, -1.6]

    // For constrained dynamics:
    // M*a = F_ext + J^T * λ
    // J*a = -β²*g - 2α*(J*v)  (Baumgarte stabilization)
    //
    // At equilibrium (v=0, a=0, g≈0):
    // J^T * λ = -F_ext
    // λ = -(J*M^(-1)*J^T)^(-1) * J*M^(-1)*F_ext

    // For diagonal mass matrix M = m*I:
    double JMJt = (J[0]*J[0] + J[1]*J[1]) / m;  // J * M^(-1) * J^T
    double JMF = (J[0]*Fx_ext + J[1]*Fy_ext) / m;  // J * M^(-1) * F

    // At static equilibrium, we want J*a = 0, so:
    // λ = -JMF / JMJt = (constraint force multiplier)
    double lambda = -JMF / JMJt;

    // Constraint force: F_c = J^T * λ
    double Fx_c = J[0] * lambda;
    double Fy_c = J[1] * lambda;

    std::cout << "  Position: (" << pos[0] << ", " << pos[1] << ")" << std::endl;
    std::cout << "  External force (gravity): (" << Fx_ext << ", " << Fy_ext << ")" << std::endl;
    std::cout << "  Jacobian: [" << J[0] << ", " << J[1] << "]" << std::endl;
    std::cout << "  Lagrange multiplier λ: " << lambda << std::endl;
    std::cout << "  Constraint force: (" << Fx_c << ", " << Fy_c << ")" << std::endl;

    // Total force should be tangent to circle (perpendicular to J)
    double Fx_total = Fx_ext + Fx_c;
    double Fy_total = Fy_ext + Fy_c;
    double J_dot_F = J[0]*Fx_total + J[1]*Fy_total;
    std::cout << "  Total force: (" << Fx_total << ", " << Fy_total << ")" << std::endl;
    std::cout << "  J · F_total (should be 0): " << J_dot_F << std::endl;

    // The constraint force magnitude is the tension in the rod
    double tension = std::sqrt(Fx_c*Fx_c + Fy_c*Fy_c);
    std::cout << "  Rod tension: " << tension << " N" << std::endl;

    std::cout << std::endl;
}

// =============================================================================
// Example 4: Hessian for Second-Order Analysis
// =============================================================================
void hessianExample() {
    std::cout << "=== Example 4: Hessian Computation ===" << std::endl;

    using x = Var<0>;
    using y = Var<1>;

    // A more complex expression: f(x,y) = x²y + sin(x*y)
    using xy = Mul<x, y>;
    using f = Add<Mul<Square<x>, y>, Sin<xy>>;

    std::array<double, 2> point = {1.0, 2.0};

    // Compute function value
    double f_val = eval<f>(point);
    std::cout << "  f(x,y) = x²y + sin(xy)" << std::endl;
    std::cout << "  f(1, 2) = " << f_val << std::endl;

    // Compute gradient
    auto grad = Gradient<f, 2>::eval(point);
    std::cout << "  Gradient: [" << grad[0] << ", " << grad[1] << "]" << std::endl;

    // Compute Hessian (second derivatives)
    auto H = Hessian<f, 2>::eval(point);
    std::cout << "  Hessian:" << std::endl;
    std::cout << "    [" << std::setw(10) << H[0][0] << " " << std::setw(10) << H[0][1] << "]" << std::endl;
    std::cout << "    [" << std::setw(10) << H[1][0] << " " << std::setw(10) << H[1][1] << "]" << std::endl;

    // Verify symmetry
    std::cout << "  Hessian symmetric (H[0][1] == H[1][0]): "
              << (std::abs(H[0][1] - H[1][0]) < 1e-10 ? "yes" : "no") << std::endl;

    std::cout << std::endl;
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::cout << "======================================" << std::endl;
    std::cout << "Compile-Time CAS Demonstration" << std::endl;
    std::cout << "======================================" << std::endl << std::endl;

    simplePendulumExample();
    doublePendulumExample();
    constraintForcesExample();
    hessianExample();

    std::cout << "======================================" << std::endl;
    std::cout << "Key Points:" << std::endl;
    std::cout << "  1. Expressions are TYPES, not values" << std::endl;
    std::cout << "  2. Diff_t<E, I> computes ∂E/∂x_i at compile time" << std::endl;
    std::cout << "  3. eval<E>(vars) evaluates at runtime" << std::endl;
    std::cout << "  4. Gradient, Jacobian, Hessian are all automatic" << std::endl;
    std::cout << "======================================" << std::endl;

    return 0;
}
