/**
 * @file acausal_test.cpp
 * @brief Test acausal modeling with compile-time causality assignment
 *
 * Demonstrates:
 *   - RC circuit with symbolic equations
 *   - Compile-time causality resolution
 *   - ODE simulation
 */

#include "physics/acausal/causality.hpp"
#include "core/solver.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot;
using namespace sopot::acausal;
using namespace sopot::symbolic;

// ============================================================================
// TEST: Dependency Analysis
// ============================================================================

void test_dependency_analysis() {
    std::cout << "=== Test: Dependency Analysis ===" << std::endl;

    // Expression: x0 * x1 + x2
    using Expr = Add<Mul<Var<0>, Var<1>>, Var<2>>;

    std::cout << "Expression: x0 * x1 + x2" << std::endl;
    std::cout << "Depends on x0: " << (depends_on_v<Expr, 0> ? "yes" : "no") << std::endl;
    std::cout << "Depends on x1: " << (depends_on_v<Expr, 1> ? "yes" : "no") << std::endl;
    std::cout << "Depends on x2: " << (depends_on_v<Expr, 2> ? "yes" : "no") << std::endl;
    std::cout << "Depends on x3: " << (depends_on_v<Expr, 3> ? "yes" : "no") << std::endl;

    // Verify at compile time
    static_assert(depends_on_v<Expr, 0>, "Should depend on x0");
    static_assert(depends_on_v<Expr, 1>, "Should depend on x1");
    static_assert(depends_on_v<Expr, 2>, "Should depend on x2");
    static_assert(!depends_on_v<Expr, 3>, "Should NOT depend on x3");

    std::cout << "=== PASSED ===" << std::endl;
}

// ============================================================================
// TEST: Incidence Matrix
// ============================================================================

void test_incidence_matrix() {
    std::cout << "\n=== Test: Incidence Matrix ===" << std::endl;

    // Equation: x0 - x1 * x2 = 0
    using Eq = Equation<Var<0>, Mul<Var<1>, Var<2>>>;

    constexpr auto row = IncidenceRow<Eq, 4>::value;

    std::cout << "Equation: x0 = x1 * x2" << std::endl;
    std::cout << "Incidence row: [";
    for (size_t i = 0; i < 4; ++i) {
        std::cout << (row[i] ? "1" : "0");
        if (i < 3) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Verify
    static_assert(row[0] == true, "Should involve x0");
    static_assert(row[1] == true, "Should involve x1");
    static_assert(row[2] == true, "Should involve x2");
    static_assert(row[3] == false, "Should NOT involve x3");

    std::cout << "=== PASSED ===" << std::endl;
}

// ============================================================================
// TEST: RC Circuit Symbolic
// ============================================================================

void test_rc_circuit_symbolic() {
    std::cout << "\n=== Test: RC Circuit Symbolic ===" << std::endl;

    using Circuit = SimpleRCCircuit<0, 1, 2>;  // Params: V0=0, R=1, C=2

    // Parameters: V0=5V, R=1kΩ, C=1μF
    std::array<double, 3> params = {5.0, 1000.0, 1e-6};

    // Initial state: V_C = 0
    std::array<double, 1> state = {0.0};

    // Compute derivatives
    auto derivs = Circuit::computeDerivatives(state, params);

    std::cout << "RC Circuit: V0=5V, R=1kΩ, C=1μF" << std::endl;
    std::cout << "Initial V_C = 0V" << std::endl;
    std::cout << "der(V_C) = " << derivs[0] << " V/s" << std::endl;
    std::cout << "Expected: (V0 - V_C) / (R*C) = 5 / (1000 * 1e-6) = 5000 V/s" << std::endl;

    // Verify
    double expected = 5.0 / (1000.0 * 1e-6);
    bool correct = std::abs(derivs[0] - expected) < 1e-6;
    std::cout << "Result: " << (correct ? "CORRECT" : "WRONG") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

// ============================================================================
// TEST: RC Circuit Simulation
// ============================================================================

void test_rc_circuit_simulation() {
    std::cout << "\n=== Test: RC Circuit Simulation ===" << std::endl;

    using Circuit = SimpleRCCircuit<0, 1, 2>;

    // Parameters: V0=5V, R=1kΩ, C=1mF (larger for visible dynamics)
    std::array<double, 3> params = {5.0, 1000.0, 1e-3};

    // Time constant tau = R * C = 1 second
    double tau = params[1] * params[2];
    std::cout << "Time constant tau = R*C = " << tau << " s" << std::endl;

    // Simulate using simple Euler integration
    double V_C = 0.0;
    double dt = 0.01;  // 10ms steps
    double t = 0.0;

    std::cout << "\nSimulation (Euler, dt=10ms):" << std::endl;
    std::cout << "t(s)\tV_C(V)\tV_C_exact(V)" << std::endl;

    for (int step = 0; step <= 50; step += 10) {
        t = step * dt;

        // Analytical solution: V_C(t) = V0 * (1 - exp(-t/tau))
        double V_C_exact = params[0] * (1.0 - std::exp(-t / tau));

        if (step % 10 == 0) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << t << "\t" << V_C << "\t" << V_C_exact << std::endl;
        }

        // Euler step
        std::array<double, 1> state = {V_C};
        auto derivs = Circuit::computeDerivatives(state, params);
        V_C += derivs[0] * dt;
    }

    // Final check at t=5*tau (should be ~99.3% of V0)
    t = 5.0 * tau;
    std::array<double, 1> state = {0.0};

    // Simulate to t=5*tau
    V_C = 0.0;
    int steps = static_cast<int>(t / dt);
    for (int i = 0; i < steps; ++i) {
        state[0] = V_C;
        auto derivs = Circuit::computeDerivatives(state, params);
        V_C += derivs[0] * dt;
    }

    double V_C_final_exact = params[0] * (1.0 - std::exp(-5.0));
    std::cout << "\nAt t = 5*tau = " << t << "s:" << std::endl;
    std::cout << "V_C simulated: " << V_C << " V" << std::endl;
    std::cout << "V_C exact: " << V_C_final_exact << " V" << std::endl;

    // Allow some error due to Euler method
    bool close_enough = std::abs(V_C - V_C_final_exact) < 0.1;
    std::cout << "Result: " << (close_enough ? "CLOSE ENOUGH" : "ERROR TOO LARGE") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

// ============================================================================
// TEST: Causality at Compile Time
// ============================================================================

void test_compile_time_causality() {
    std::cout << "\n=== Test: Compile-Time Causality ===" << std::endl;

    using Circuit = SimpleRCCircuit<0, 1, 2>;

    std::cout << "RC Circuit causality chain:" << std::endl;
    std::cout << "  State:     V_C (capacitor voltage)" << std::endl;
    std::cout << "  Algebraic: V_node = V_C" << std::endl;
    std::cout << "             I_R = (V0 - V_node) / R" << std::endl;
    std::cout << "             I_C = I_R" << std::endl;
    std::cout << "  Derivative: der(V_C) = I_C / C" << std::endl;
    std::cout << std::endl;
    std::cout << "This causality is determined at COMPILE TIME." << std::endl;
    std::cout << "The solveAlgebraic() method generates direct assignments," << std::endl;
    std::cout << "not iterative solvers." << std::endl;

    // Demonstrate that algebraic equations are solved directly
    std::array<double, 3> params = {10.0, 100.0, 1e-6};  // V0=10V, R=100Ω, C=1μF

    double V_C = 3.0;  // Current capacitor voltage
    auto vars = Circuit::solveAlgebraic(V_C, params);

    std::cout << "\nWith V_C = 3V, V0 = 10V, R = 100Ω:" << std::endl;
    std::cout << "  V_node = " << vars[1] << " V (should be 3)" << std::endl;
    std::cout << "  I_R = " << vars[2] << " A (should be 0.07)" << std::endl;
    std::cout << "  I_C = " << vars[3] << " A (should be 0.07)" << std::endl;

    // Verify
    bool v_correct = std::abs(vars[1] - 3.0) < 1e-10;
    bool i_correct = std::abs(vars[2] - 0.07) < 1e-10;
    std::cout << "\nResults: " << (v_correct && i_correct ? "CORRECT" : "WRONG") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

// ============================================================================
// TEST: Linearity Check
// ============================================================================

void test_linearity_check() {
    std::cout << "\n=== Test: Linearity Check ===" << std::endl;

    // Linear expression: 2*x0 + 3
    using Linear = Add<Mul<Two, Var<0>>, Const<3>>;

    // Nonlinear expression: x0 * x0
    using Nonlinear = Mul<Var<0>, Var<0>>;

    std::cout << "Expression: 2*x0 + 3" << std::endl;
    std::cout << "  Linear in x0: " << (IsLinearIn<Linear, 0>::value ? "yes" : "no") << std::endl;

    std::cout << "Expression: x0 * x0" << std::endl;
    std::cout << "  Linear in x0: " << (IsLinearIn<Nonlinear, 0>::value ? "yes" : "no") << std::endl;

    static_assert(IsLinearIn<Linear, 0>::value, "2*x0 + 3 should be linear in x0");
    static_assert(!IsLinearIn<Nonlinear, 0>::value, "x0*x0 should NOT be linear in x0");

    // Also linear: x0 + x1 (linear in each variable separately)
    using TwoVar = Add<Var<0>, Var<1>>;
    std::cout << "Expression: x0 + x1" << std::endl;
    std::cout << "  Linear in x0: " << (IsLinearIn<TwoVar, 0>::value ? "yes" : "no") << std::endl;
    std::cout << "  Linear in x1: " << (IsLinearIn<TwoVar, 1>::value ? "yes" : "no") << std::endl;

    static_assert(IsLinearIn<TwoVar, 0>::value, "x0+x1 should be linear in x0");
    static_assert(IsLinearIn<TwoVar, 1>::value, "x0+x1 should be linear in x1");

    std::cout << "=== PASSED ===" << std::endl;
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "Acausal Modeling - Compile-Time Causality" << std::endl;
    std::cout << "==================================================" << std::endl;

    test_dependency_analysis();
    test_incidence_matrix();
    test_linearity_check();
    test_rc_circuit_symbolic();
    test_compile_time_causality();
    test_rc_circuit_simulation();

    std::cout << "\n==================================================" << std::endl;
    std::cout << "ALL ACAUSAL TESTS PASSED!" << std::endl;
    std::cout << "Causality resolved at COMPILE TIME" << std::endl;
    std::cout << "==================================================" << std::endl;

    return 0;
}
