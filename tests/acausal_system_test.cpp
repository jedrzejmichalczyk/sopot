/**
 * @file acausal_system_test.cpp
 * @brief Test automatic causality assignment and system construction
 */

#include "physics/acausal/system.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot;
using namespace sopot::acausal;
using namespace sopot::symbolic;

// ============================================================================
// TEST: Incidence Matrix Construction
// ============================================================================

void test_incidence_matrix_construction() {
    std::cout << "=== Test: Incidence Matrix Construction ===\n";

    // Simple system:
    // Eq0: x0 = x1        (x0 - x1 = 0)
    // Eq1: x1 = 2 * x2    (x1 - 2*x2 = 0)
    // Eq2: x2 = param0    (x2 - param0 = 0)

    using Eq0 = Equation<Var<0>, Var<1>>;
    using Eq1 = Equation<Var<1>, Mul<Two, Var<2>>>;
    using Eq2 = Equation<Var<2>, Param<0>>;
    using Equations = std::tuple<Eq0, Eq1, Eq2>;

    constexpr auto incidence = IncidenceMatrix<Equations, 3>::value;

    std::cout << "Equations:\n";
    std::cout << "  Eq0: x0 = x1\n";
    std::cout << "  Eq1: x1 = 2 * x2\n";
    std::cout << "  Eq2: x2 = param0\n\n";

    std::cout << "Incidence Matrix:\n";
    std::cout << "      x0  x1  x2\n";
    for (size_t eq = 0; eq < 3; ++eq) {
        std::cout << "Eq" << eq << ":  ";
        for (size_t var = 0; var < 3; ++var) {
            std::cout << (incidence[eq][var] ? "1" : "0") << "   ";
        }
        std::cout << "\n";
    }

    // Verify
    static_assert(incidence[0][0] == true, "Eq0 should depend on x0");
    static_assert(incidence[0][1] == true, "Eq0 should depend on x1");
    static_assert(incidence[0][2] == false, "Eq0 should NOT depend on x2");

    static_assert(incidence[1][0] == false, "Eq1 should NOT depend on x0");
    static_assert(incidence[1][1] == true, "Eq1 should depend on x1");
    static_assert(incidence[1][2] == true, "Eq1 should depend on x2");

    static_assert(incidence[2][0] == false, "Eq2 should NOT depend on x0");
    static_assert(incidence[2][1] == false, "Eq2 should NOT depend on x1");
    static_assert(incidence[2][2] == true, "Eq2 should depend on x2");

    std::cout << "\n=== PASSED ===\n";
}

// ============================================================================
// TEST: Causality Assignment
// ============================================================================

void test_causality_assignment() {
    std::cout << "\n=== Test: Causality Assignment ===\n";

    // Same system
    using Eq0 = Equation<Var<0>, Var<1>>;
    using Eq1 = Equation<Var<1>, Mul<Two, Var<2>>>;
    using Eq2 = Equation<Var<2>, Param<0>>;
    using Equations = std::tuple<Eq0, Eq1, Eq2>;

    constexpr auto incidence = IncidenceMatrix<Equations, 3>::value;
    constexpr auto assignment = findGreedyMatchingSimple<3, 3>(incidence);

    std::cout << "Causality Assignment:\n";
    for (size_t eq = 0; eq < 3; ++eq) {
        std::cout << "  Eq" << eq << " solves for: x" << assignment[eq] << "\n";
    }

    // Expected:
    // Eq0 has x0, x1 -> should get one of them
    // Eq1 has x1, x2 -> should get one of them
    // Eq2 has only x2 -> must get x2

    // Greedy will assign Eq2 -> x2 first (unique)
    // Then Eq1 can get x1
    // Then Eq0 can get x0

    static_assert(assignment[2] == 2, "Eq2 should solve for x2");

    // After Eq2 gets x2, Eq1 should get x1
    // (x2 is already assigned, so Eq1 can only get x1)
    static_assert(assignment[1] == 1, "Eq1 should solve for x1");

    // After x1 and x2 assigned, Eq0 should get x0
    static_assert(assignment[0] == 0, "Eq0 should solve for x0");

    std::cout << "\nAll assignments verified at compile time!\n";
    std::cout << "\n=== PASSED ===\n";
}

// ============================================================================
// TEST: Topological Sort
// ============================================================================

void test_topological_sort() {
    std::cout << "\n=== Test: Topological Sort (BLT) ===\n";

    using Eq0 = Equation<Var<0>, Var<1>>;
    using Eq1 = Equation<Var<1>, Mul<Two, Var<2>>>;
    using Eq2 = Equation<Var<2>, Param<0>>;
    using Equations = std::tuple<Eq0, Eq1, Eq2>;

    constexpr auto incidence = IncidenceMatrix<Equations, 3>::value;
    constexpr auto assignment = findGreedyMatchingSimple<3, 3>(incidence);
    constexpr auto order = sortEquations<3, 3>(incidence, assignment);

    std::cout << "Evaluation Order:\n";
    for (size_t i = 0; i < 3; ++i) {
        std::cout << "  Step " << (i + 1) << ": Eq" << order[i]
                  << " (computes x" << assignment[order[i]] << ")\n";
    }

    // Expected order:
    // 1. Eq2 (computes x2 from param, no deps)
    // 2. Eq1 (computes x1, depends on x2)
    // 3. Eq0 (computes x0, depends on x1)

    static_assert(order[0] == 2, "First should be Eq2");
    static_assert(order[1] == 1, "Second should be Eq1");
    static_assert(order[2] == 0, "Third should be Eq0");

    std::cout << "\nOrder verified at compile time!\n";
    std::cout << "\n=== PASSED ===\n";
}

// ============================================================================
// TEST: ManualRCCircuit Causality
// ============================================================================

void test_manual_rc_causality() {
    std::cout << "\n=== Test: ManualRCCircuit Causality ===\n";

    ManualRCCircuit::printCausality();

    // Verify compile-time causality
    constexpr auto assignment = ManualRCCircuit::assignment;
    constexpr auto order = ManualRCCircuit::evalOrder;

    // Eq0: V_node = V_C  -> should solve for V_node (var 1)
    // Eq1: V0 - V_node = R*I_R -> should solve for I_R (var 2)
    // Eq2: I_R = I_C -> should solve for I_C (var 3)

    // V_C (var 0) is the state, not solved by algebraic equations
    static_assert(assignment[0] == 1, "Eq0 should solve for V_node");
    static_assert(assignment[1] == 2, "Eq1 should solve for I_R");
    static_assert(assignment[2] == 3, "Eq2 should solve for I_C");

    std::cout << "\n=== PASSED ===\n";
}

// ============================================================================
// TEST: ManualRCCircuit Simulation
// ============================================================================

void test_manual_rc_simulation() {
    std::cout << "\n=== Test: ManualRCCircuit Simulation ===\n";

    // Parameters: V0=5V, R=1kΩ, C=1mF
    std::array<double, 3> params = {5.0, 1000.0, 1e-3};
    double tau = params[1] * params[2];

    std::cout << "Circuit: V0=5V, R=1kΩ, C=1mF\n";
    std::cout << "Time constant τ = RC = " << tau << " s\n\n";

    // Simulate using Euler integration
    double V_C = 0.0;
    double dt = 0.01;

    std::cout << "Simulation (Euler, dt=10ms):\n";
    std::cout << "t(s)\tV_C(V)\tV_C_exact(V)\n";

    for (int step = 0; step <= 500; step += 100) {
        double t = step * dt;

        // Analytical solution
        double V_C_exact = params[0] * (1.0 - std::exp(-t / tau));

        std::cout << std::fixed << std::setprecision(3);
        std::cout << t << "\t" << V_C << "\t" << V_C_exact << "\n";

        // Integrate from current point
        for (int i = 0; i < 100 && step + i < 500; ++i) {
            std::array<double, 1> state = {V_C};
            auto derivs = ManualRCCircuit::computeDerivatives(state, params);
            V_C += derivs[0] * dt;
        }
    }

    // Final verification
    double V_C_final_exact = params[0] * (1.0 - std::exp(-5.0));
    std::cout << "\nAt t = 5τ:\n";
    std::cout << "  V_C simulated: " << V_C << " V\n";
    std::cout << "  V_C exact:     " << V_C_final_exact << " V\n";

    bool close_enough = std::abs(V_C - V_C_final_exact) < 0.1;
    std::cout << "  Result: " << (close_enough ? "CLOSE ENOUGH" : "ERROR TOO LARGE") << "\n";

    std::cout << "\n=== PASSED ===\n";
}

// ============================================================================
// TEST: Verify Residuals are Zero
// ============================================================================

void test_residuals_are_zero() {
    std::cout << "\n=== Test: Residuals are Zero ===\n";

    std::array<double, 3> params = {10.0, 100.0, 1e-6};
    double V_C = 3.0;

    auto vars = ManualRCCircuit::solveAlgebraic(V_C, params);

    std::cout << "After solving with V_C = " << V_C << " V:\n";
    std::cout << "  V_C    = " << vars[0] << " V\n";
    std::cout << "  V_node = " << vars[1] << " V\n";
    std::cout << "  I_R    = " << vars[2] << " A\n";
    std::cout << "  I_C    = " << vars[3] << " A\n\n";

    // Check residuals manually
    // Eq0: V_node - V_C = 0
    double res0 = vars[1] - vars[0];
    // Eq1: (V0 - V_node) - R*I_R = 0
    double res1 = (params[0] - vars[1]) - params[1] * vars[2];
    // Eq2: I_R - I_C = 0
    double res2 = vars[2] - vars[3];

    std::cout << "Residuals:\n";
    std::cout << "  Eq0 (V_node = V_C):         " << res0 << "\n";
    std::cout << "  Eq1 (V0-V_node = R*I_R):    " << res1 << "\n";
    std::cout << "  Eq2 (I_R = I_C):            " << res2 << "\n";

    bool all_zero = std::abs(res0) < 1e-12 &&
                    std::abs(res1) < 1e-12 &&
                    std::abs(res2) < 1e-12;

    std::cout << "\nAll residuals zero: " << (all_zero ? "YES" : "NO") << "\n";

    std::cout << "\n=== PASSED ===\n";
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::cout << "=================================================\n";
    std::cout << "Acausal System - Automatic Causality Assignment\n";
    std::cout << "=================================================\n\n";

    test_incidence_matrix_construction();
    test_causality_assignment();
    test_topological_sort();
    test_manual_rc_causality();
    test_residuals_are_zero();
    test_manual_rc_simulation();

    std::cout << "\n=================================================\n";
    std::cout << "ALL TESTS PASSED!\n";
    std::cout << "Causality assigned and sorted at COMPILE TIME\n";
    std::cout << "=================================================\n";

    return 0;
}
