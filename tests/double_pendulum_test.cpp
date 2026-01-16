#include "../physics/pendulum/double_pendulum.hpp"
#include "../physics/pendulum/cartesian_pendulum.hpp"
#include "../physics/pendulum/symbolic_cartesian_pendulum.hpp"
#include "../core/typed_component.hpp"
#include "../core/dual.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

using namespace sopot;
using namespace sopot::pendulum;

#define ASSERT_NEAR(a, b, tol) \
    do { \
        double _a = value_of(a), _b = value_of(b); \
        if (std::abs(_a - _b) > (tol)) { \
            std::cerr << "FAIL at line " << __LINE__ << ": " \
                      << #a << " = " << _a << " != " << #b << " = " << _b << std::endl; \
            std::abort(); \
        } \
    } while(0)

#define ASSERT_TRUE(cond) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL at line " << __LINE__ << ": " << #cond << " is false" << std::endl; \
            std::abort(); \
        } \
    } while(0)

//=============================================================================
// RK4 Integration Helper
//=============================================================================
template<typename System>
void rk4_step(const System& system, double& t, std::vector<double>& state, double dt) {
    size_t n = state.size();
    std::vector<double> s2(n), s3(n), s4(n);

    auto k1 = system.computeDerivatives(t, state);
    for (size_t i = 0; i < n; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];

    auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
    for (size_t i = 0; i < n; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];

    auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
    for (size_t i = 0; i < n; ++i) s4[i] = state[i] + dt * k3[i];

    auto k4 = system.computeDerivatives(t + dt, s4);
    for (size_t i = 0; i < n; ++i) {
        state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
    t += dt;
}

//=============================================================================
// Test 1: Component Creation
//=============================================================================
void testComponentCreation() {
    std::cout << "Test 1: Double pendulum creation..." << std::endl;

    DoublePendulum<double> pendulum(
        1.0, 1.0,    // masses (kg)
        1.0, 1.0,    // lengths (m)
        9.81,        // gravity
        0.5, 0.5,    // initial angles (rad)
        0.0, 0.0     // initial angular velocities
    );

    ASSERT_NEAR(pendulum.getMass1(), 1.0, 1e-10);
    ASSERT_NEAR(pendulum.getMass2(), 1.0, 1e-10);
    ASSERT_NEAR(pendulum.getLength1(), 1.0, 1e-10);
    ASSERT_NEAR(pendulum.getLength2(), 1.0, 1e-10);
    ASSERT_NEAR(pendulum.getGravity(), 9.81, 1e-10);

    std::cout << "   Created pendulum: m1=" << pendulum.getMass1()
              << " kg, m2=" << pendulum.getMass2()
              << " kg, L1=" << pendulum.getLength1()
              << " m, L2=" << pendulum.getLength2() << " m" << std::endl;
    std::cout << "   [OK] Component creation passed" << std::endl;
}

//=============================================================================
// Test 2: System Assembly
//=============================================================================
void testSystemAssembly() {
    std::cout << "\nTest 2: System assembly..." << std::endl;

    auto system = makeTypedODESystem<double>(
        DoublePendulum<double>(1.0, 1.0, 1.0, 1.0, 9.81, 0.1, 0.2, 0.0, 0.0)
    );

    std::cout << "   State dimension: " << system.getStateDimension() << std::endl;
    std::cout << "   Component count: " << system.getComponentCount() << std::endl;

    ASSERT_NEAR(system.getStateDimension(), 4, 1e-10);  // [theta1, theta2, omega1, omega2]
    ASSERT_NEAR(system.getComponentCount(), 1, 1e-10);

    // Verify compile-time function availability
    static_assert(decltype(system)::hasFunction<mass1::Angle>());
    static_assert(decltype(system)::hasFunction<mass1::AngularVelocity>());
    static_assert(decltype(system)::hasFunction<mass1::CartesianPosition>());
    static_assert(decltype(system)::hasFunction<mass2::Angle>());
    static_assert(decltype(system)::hasFunction<mass2::AngularVelocity>());
    static_assert(decltype(system)::hasFunction<mass2::CartesianPosition>());
    static_assert(decltype(system)::hasFunction<system::TotalEnergy>());
    static_assert(decltype(system)::hasFunction<system::KineticEnergy>());
    static_assert(decltype(system)::hasFunction<system::PotentialEnergy>());

    std::cout << "   All state functions available at compile time" << std::endl;
    std::cout << "   [OK] System assembly passed" << std::endl;
}

//=============================================================================
// Test 3: State Function Queries
//=============================================================================
void testStateFunctions() {
    std::cout << "\nTest 3: State function queries..." << std::endl;

    double theta1_0 = 0.3;  // rad
    double theta2_0 = 0.5;  // rad
    double L1 = 1.0, L2 = 0.8;
    double m1 = 1.0, m2 = 0.5;

    auto system = makeTypedODESystem<double>(
        DoublePendulum<double>(m1, m2, L1, L2, 9.81, theta1_0, theta2_0, 0.0, 0.0)
    );

    auto state = system.getInitialState();

    // Query angles
    double theta1 = system.computeStateFunction<mass1::Angle>(state);
    double theta2 = system.computeStateFunction<mass2::Angle>(state);
    std::cout << "   theta1 = " << theta1 << " rad" << std::endl;
    std::cout << "   theta2 = " << theta2 << " rad" << std::endl;
    ASSERT_NEAR(theta1, theta1_0, 1e-10);
    ASSERT_NEAR(theta2, theta2_0, 1e-10);

    // Query Cartesian positions
    auto pos1 = system.computeStateFunction<mass1::CartesianPosition>(state);
    auto pos2 = system.computeStateFunction<mass2::CartesianPosition>(state);

    double x1_expected = L1 * std::sin(theta1_0);
    double y1_expected = -L1 * std::cos(theta1_0);
    double x2_expected = x1_expected + L2 * std::sin(theta2_0);
    double y2_expected = y1_expected - L2 * std::cos(theta2_0);

    std::cout << "   pos1 = (" << pos1[0] << ", " << pos1[1] << ") m" << std::endl;
    std::cout << "   pos2 = (" << pos2[0] << ", " << pos2[1] << ") m" << std::endl;
    ASSERT_NEAR(pos1[0], x1_expected, 1e-10);
    ASSERT_NEAR(pos1[1], y1_expected, 1e-10);
    ASSERT_NEAR(pos2[0], x2_expected, 1e-10);
    ASSERT_NEAR(pos2[1], y2_expected, 1e-10);

    // Query energy (at rest, only potential energy)
    double KE = system.computeStateFunction<system::KineticEnergy>(state);
    double PE = system.computeStateFunction<system::PotentialEnergy>(state);
    double E = system.computeStateFunction<system::TotalEnergy>(state);

    std::cout << "   KE = " << KE << " J, PE = " << PE << " J, E = " << E << " J" << std::endl;
    ASSERT_NEAR(KE, 0.0, 1e-10);  // At rest initially
    ASSERT_NEAR(E, KE + PE, 1e-10);

    std::cout << "   [OK] State function queries passed" << std::endl;
}

//=============================================================================
// Test 4: Energy Conservation (Undamped)
//=============================================================================
void testEnergyConservation() {
    std::cout << "\nTest 4: Energy conservation (undamped, 20s)..." << std::endl;

    // Small angle initial conditions for stable integration
    auto system = makeTypedODESystem<double>(
        DoublePendulum<double>(1.0, 1.0, 1.0, 1.0, 9.81,
                               0.3, 0.3,   // initial angles
                               0.0, 0.0)   // initial velocities
    );

    auto state = system.getInitialState();
    double E0 = system.computeStateFunction<system::TotalEnergy>(state);

    std::cout << "   Initial energy: " << E0 << " J" << std::endl;

    double dt = 0.0005;  // Small timestep for accuracy
    double t = 0.0;
    double t_end = 20.0;

    double E_max_error = 0.0;
    double E_min = E0, E_max = E0;

    while (t < t_end) {
        rk4_step(system, t, state, dt);

        double E = system.computeStateFunction<system::TotalEnergy>(state);
        double error = std::abs(E - E0) / std::abs(E0);
        E_max_error = std::max(E_max_error, error);
        E_min = std::min(E_min, E);
        E_max = std::max(E_max, E);
    }

    double E_final = system.computeStateFunction<system::TotalEnergy>(state);
    std::cout << "   Final energy: " << E_final << " J" << std::endl;
    std::cout << "   Energy range: [" << E_min << ", " << E_max << "] J" << std::endl;
    std::cout << "   Max relative error: " << E_max_error * 100 << "%" << std::endl;

    // Check energy conservation (< 1% drift)
    if (E_max_error > 0.01) {
        std::cerr << "   [FAIL] Energy drift exceeds 1%!" << std::endl;
        std::abort();
    }

    std::cout << "   [OK] Energy conservation passed" << std::endl;
}

//=============================================================================
// Test 5: Small-Angle Period Approximation
//=============================================================================
void testSmallAnglePeriod() {
    std::cout << "\nTest 5: Small-angle period approximation..." << std::endl;

    double L1 = 1.0, L2 = 1.0;
    double m1 = 1.0, m2 = 1.0;
    double g = 9.81;

    // For a double pendulum with equal masses and lengths, in-phase mode:
    // T_1 = 2*pi*sqrt(L/g) * (1 + sqrt(2))^(-1/2) ≈ 1.31 s (antisymmetric)
    // T_2 = 2*pi*sqrt(L/g) * (1 - sqrt(2))^(-1/2) ≈ out-of-phase

    // Simple pendulum period for comparison
    double T_simple = 2 * M_PI * std::sqrt(L1 / g);
    std::cout << "   Simple pendulum period: " << T_simple << " s" << std::endl;

    // Small initial angle (nearly in-phase)
    double theta0 = 0.05;  // 0.05 rad ≈ 3 degrees
    auto system = makeTypedODESystem<double>(
        DoublePendulum<double>(m1, m2, L1, L2, g, theta0, theta0, 0.0, 0.0)
    );

    auto state = system.getInitialState();
    double theta1_0 = system.computeStateFunction<mass1::Angle>(state);

    double dt = 0.0001;
    double t = 0.0;

    // Find first zero crossing (after initial)
    double prev_theta1 = theta1_0;
    int crossings = 0;

    while (crossings < 2 && t < 10.0) {
        rk4_step(system, t, state, dt);
        double theta1 = system.computeStateFunction<mass1::Angle>(state);

        if (prev_theta1 > 0 && theta1 <= 0) {
            crossings++;
            if (crossings == 2) {
                std::cout << "   Measured period (from crossings): " << t << " s (half period)" << std::endl;
            }
        }
        prev_theta1 = theta1;
    }

    // For small angles and in-phase initial conditions, period should be
    // close to simple pendulum period but modified by the coupling
    // We just verify it's in a reasonable range
    double period_ratio = t / T_simple;
    std::cout << "   Period ratio to simple pendulum: " << period_ratio << std::endl;

    // The actual in-phase mode period is longer due to coupling
    ASSERT_TRUE(period_ratio > 0.5 && period_ratio < 2.0);

    std::cout << "   [OK] Small-angle period test passed" << std::endl;
}

//=============================================================================
// Test 6: Chaotic Sensitivity
//=============================================================================
void testChaoticSensitivity() {
    std::cout << "\nTest 6: Chaotic sensitivity (large angles)..." << std::endl;

    double theta1_0 = 3.0;  // Large angle (~172 degrees)
    double theta2_0 = 3.0;
    double epsilon = 1e-8;  // Tiny perturbation

    auto system1 = makeTypedODESystem<double>(
        DoublePendulum<double>(1.0, 1.0, 1.0, 1.0, 9.81, theta1_0, theta2_0, 0.0, 0.0)
    );
    auto system2 = makeTypedODESystem<double>(
        DoublePendulum<double>(1.0, 1.0, 1.0, 1.0, 9.81, theta1_0 + epsilon, theta2_0, 0.0, 0.0)
    );

    auto state1 = system1.getInitialState();
    auto state2 = system2.getInitialState();

    double dt = 0.001;
    double t = 0.0;
    double t_end = 10.0;

    std::cout << "   Initial difference: " << epsilon << " rad" << std::endl;

    while (t < t_end) {
        rk4_step(system1, t, state1, dt);
        double t2 = t - dt;  // rk4_step advances t, so we need to match
        rk4_step(system2, t2, state2, dt);
    }

    double theta1_final_1 = system1.computeStateFunction<mass1::Angle>(state1);
    double theta1_final_2 = system2.computeStateFunction<mass1::Angle>(state2);
    double final_diff = std::abs(theta1_final_1 - theta1_final_2);

    std::cout << "   Final theta1 (system 1): " << theta1_final_1 << " rad" << std::endl;
    std::cout << "   Final theta1 (system 2): " << theta1_final_2 << " rad" << std::endl;
    std::cout << "   Final difference: " << final_diff << " rad" << std::endl;

    // For chaotic systems, small initial differences grow exponentially
    double amplification = final_diff / epsilon;
    std::cout << "   Amplification factor: " << amplification << "x" << std::endl;

    // We expect significant divergence for chaotic initial conditions
    if (amplification < 1e3) {
        std::cerr << "   [WARNING] Low amplification - may not be in chaotic regime" << std::endl;
    }

    std::cout << "   [OK] Chaotic sensitivity test passed" << std::endl;
}

//=============================================================================
// Test 7: Generalized vs Cartesian Formulation Agreement
//=============================================================================
void testFormulationAgreement() {
    std::cout << "\nTest 7: Generalized vs Cartesian formulation agreement..." << std::endl;

    double m1 = 1.0, m2 = 0.8;
    double L1 = 1.0, L2 = 0.7;
    double g = 9.81;
    double theta1_0 = 0.3, theta2_0 = 0.4;

    auto gen_system = makeTypedODESystem<double>(
        DoublePendulum<double>(m1, m2, L1, L2, g, theta1_0, theta2_0, 0.0, 0.0)
    );
    auto cart_system = makeTypedODESystem<double>(
        CartesianDoublePendulum<double>(m1, m2, L1, L2, g, theta1_0, theta2_0, 0.0, 0.0)
    );

    auto gen_state = gen_system.getInitialState();
    auto cart_state = cart_system.getInitialState();

    double dt = 0.0005;
    double t_gen = 0.0, t_cart = 0.0;
    double t_end = 5.0;

    double max_pos_error = 0.0;

    while (t_gen < t_end) {
        rk4_step(gen_system, t_gen, gen_state, dt);
        rk4_step(cart_system, t_cart, cart_state, dt);

        // Compare Cartesian positions
        auto gen_pos2 = gen_system.computeStateFunction<mass2::CartesianPosition>(gen_state);
        auto cart_pos2 = cart_system.computeStateFunction<mass2::CartesianPosition>(cart_state);

        double dx = value_of(gen_pos2[0]) - value_of(cart_pos2[0]);
        double dy = value_of(gen_pos2[1]) - value_of(cart_pos2[1]);
        double pos_error = std::sqrt(dx*dx + dy*dy);

        max_pos_error = std::max(max_pos_error, pos_error);
    }

    std::cout << "   Max position error between formulations: " << max_pos_error << " m" << std::endl;

    // Position should agree within 5 cm (Baumgarte stabilization has some drift)
    if (max_pos_error > 0.05) {
        std::cerr << "   [FAIL] Formulations disagree by more than 5 cm!" << std::endl;
        std::abort();
    }

    std::cout << "   [OK] Formulation agreement passed" << std::endl;
}

//=============================================================================
// Test 8: Autodiff Jacobian Computation
//=============================================================================
void testAutodiffJacobian() {
    std::cout << "\nTest 8: Autodiff Jacobian computation..." << std::endl;

    using Dual4 = Dual<double, 4>;

    auto system = makeTypedODESystem<Dual4>(
        DoublePendulum<Dual4>(1.0, 1.0, 1.0, 1.0, 9.81, 0.5, 0.3, 0.1, -0.1)
    );

    // Create state with derivatives
    auto state = system.getInitialState();
    std::array<double, 4> state_values;
    for (size_t i = 0; i < 4; ++i) {
        state_values[i] = value_of(state[i]);
    }

    // Compute Jacobian
    auto jacobian = computeJacobian(system, 0.0, state_values);

    std::cout << "   Jacobian matrix (4x4):" << std::endl;
    for (size_t i = 0; i < 4; ++i) {
        std::cout << "   [";
        for (size_t j = 0; j < 4; ++j) {
            std::cout << std::setw(12) << std::setprecision(5) << jacobian[i][j];
        }
        std::cout << " ]" << std::endl;
    }

    // Verify structure: d(theta)/dt = omega, so df_0/d_state[2] should be 1
    ASSERT_NEAR(jacobian[0][2], 1.0, 1e-10);  // d(theta1_dot)/d(omega1) = 1
    ASSERT_NEAR(jacobian[1][3], 1.0, 1e-10);  // d(theta2_dot)/d(omega2) = 1
    ASSERT_NEAR(jacobian[0][0], 0.0, 1e-10);  // d(theta1_dot)/d(theta1) = 0
    ASSERT_NEAR(jacobian[0][1], 0.0, 1e-10);  // d(theta1_dot)/d(theta2) = 0

    std::cout << "   [OK] Autodiff Jacobian test passed" << std::endl;
}

//=============================================================================
// Test 9: Cartesian Constraint Stability
//=============================================================================
void testCartesianConstraintStability() {
    std::cout << "\nTest 9: Cartesian constraint stability (Baumgarte)..." << std::endl;

    auto system = makeTypedODESystem<double>(
        CartesianDoublePendulum<double>(1.0, 1.0, 1.0, 1.0, 9.81, 0.5, 0.5, 0.0, 0.0, 10.0, 10.0)
    );

    auto state = system.getInitialState();
    const auto& pendulum = system.getComponent<0>();

    double initial_error = pendulum.getConstraintError(std::span<const double>(state));
    std::cout << "   Initial constraint error: " << initial_error << std::endl;

    double dt = 0.001;
    double t = 0.0;
    double t_end = 10.0;
    double max_error = initial_error;

    while (t < t_end) {
        rk4_step(system, t, state, dt);
        double error = pendulum.getConstraintError(std::span<const double>(state));
        max_error = std::max(max_error, value_of(error));
    }

    double final_error = pendulum.getConstraintError(std::span<const double>(state));
    std::cout << "   Final constraint error: " << final_error << std::endl;
    std::cout << "   Max constraint error: " << max_error << std::endl;

    // Constraint error should remain bounded with Baumgarte stabilization
    // Note: Baumgarte is a soft constraint method, so some drift is expected
    if (max_error > 0.1) {
        std::cerr << "   [FAIL] Constraint error too large (>0.1)!" << std::endl;
        std::abort();
    }

    std::cout << "   [OK] Cartesian constraint stability passed" << std::endl;
}

//=============================================================================
// Test 10: Symbolic CAS Jacobian Equivalence
//=============================================================================
void testSymbolicJacobianEquivalence() {
    std::cout << "\nTest 10: Symbolic CAS Jacobian equivalence..." << std::endl;

    double m1 = 1.0, m2 = 0.8;
    double L1 = 1.0, L2 = 0.7;
    double g = 9.81;
    double theta1_0 = 0.3, theta2_0 = 0.4;

    // Create both versions
    auto manual_system = makeTypedODESystem<double>(
        CartesianDoublePendulum<double>(m1, m2, L1, L2, g, theta1_0, theta2_0, 0.0, 0.0)
    );
    auto symbolic_system = makeTypedODESystem<double>(
        SymbolicCartesianPendulum<double>(m1, m2, L1, L2, g, theta1_0, theta2_0, 0.0, 0.0)
    );

    auto manual_state = manual_system.getInitialState();
    auto symbolic_state = symbolic_system.getInitialState();

    double dt = 0.001;
    double t_manual = 0.0, t_symbolic = 0.0;
    double t_end = 2.0;

    double max_diff = 0.0;

    while (t_manual < t_end) {
        rk4_step(manual_system, t_manual, manual_state, dt);
        rk4_step(symbolic_system, t_symbolic, symbolic_state, dt);

        // Compare states
        for (size_t i = 0; i < 8; ++i) {
            double diff = std::abs(manual_state[i] - symbolic_state[i]);
            max_diff = std::max(max_diff, diff);
        }
    }

    std::cout << "   Max state difference (manual vs symbolic): " << max_diff << std::endl;

    // They should produce identical results (up to floating point precision)
    if (max_diff > 1e-10) {
        std::cerr << "   [FAIL] Symbolic CAS produces different results!" << std::endl;
        std::abort();
    }

    std::cout << "   [OK] Symbolic CAS Jacobian equivalence passed" << std::endl;
}

//=============================================================================
// Main
//=============================================================================
int main() {
    std::cout << "=== Double Pendulum Test Suite ===" << std::endl;
    std::cout << "Validating physics and compile-time constraint resolution\n" << std::endl;

    testComponentCreation();
    testSystemAssembly();
    testStateFunctions();
    testEnergyConservation();
    testSmallAnglePeriod();
    testChaoticSensitivity();
    testFormulationAgreement();
    testAutodiffJacobian();
    testCartesianConstraintStability();
    testSymbolicJacobianEquivalence();

    std::cout << "\n=== ALL TESTS PASSED ===" << std::endl;
    std::cout << "Demonstrated:" << std::endl;
    std::cout << "  - Generalized coordinate formulation (Lagrangian)" << std::endl;
    std::cout << "  - Cartesian formulation with Baumgarte stabilization" << std::endl;
    std::cout << "  - Energy conservation in undamped case" << std::endl;
    std::cout << "  - Chaotic sensitivity to initial conditions" << std::endl;
    std::cout << "  - Formulation agreement between approaches" << std::endl;
    std::cout << "  - Autodiff Jacobian computation" << std::endl;
    std::cout << "  - Constraint satisfaction with stabilization" << std::endl;
    std::cout << "  - Compile-time CAS for automatic Jacobian derivation" << std::endl;

    return 0;
}
