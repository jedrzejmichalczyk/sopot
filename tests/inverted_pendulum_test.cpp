/**
 * @file inverted_pendulum_test.cpp
 * @brief Tests for the cart-double-pendulum control system
 *
 * Verifies:
 * 1. Plant dynamics are correct (uncontrolled fall)
 * 2. LQR gain computation works
 * 3. Closed-loop stabilization works
 */

#include "physics/control/cart_double_pendulum.hpp"
#include "physics/control/lqr.hpp"
#include "physics/control/state_feedback_controller.hpp"
#include "core/typed_component.hpp"
#include "core/solver.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <iomanip>

using namespace sopot;
using namespace sopot::control;

constexpr double EPSILON = 1e-6;
constexpr double PI = 3.14159265358979323846;

bool approx_equal(double a, double b, double eps = EPSILON) {
    return std::abs(a - b) < eps;
}

void test_passed(const char* name) {
    std::cout << "  [PASS] " << name << std::endl;
}

// ============================================================================
// Test: Plant creation and initial state
// ============================================================================
void test_plant_creation() {
    std::cout << "Test: Plant creation..." << std::endl;

    // Create cart-double-pendulum with typical parameters
    CartDoublePendulum<double> plant(
        1.0,   // cart mass (kg)
        0.5,   // mass 1 (kg)
        0.5,   // mass 2 (kg)
        0.5,   // length 1 (m)
        0.5,   // length 2 (m)
        9.81,  // gravity (m/s²)
        0.0,   // initial x
        0.1,   // initial θ₁ (small angle from vertical)
        0.05,  // initial θ₂
        0.0,   // initial ẋ
        0.0,   // initial ω₁
        0.0    // initial ω₂
    );

    auto state = plant.getInitialLocalState();

    assert(approx_equal(state[0], 0.0));   // x
    assert(approx_equal(state[1], 0.1));   // θ₁
    assert(approx_equal(state[2], 0.05));  // θ₂
    assert(approx_equal(state[3], 0.0));   // ẋ
    assert(approx_equal(state[4], 0.0));   // ω₁
    assert(approx_equal(state[5], 0.0));   // ω₂

    test_passed("Plant creation");
}

// ============================================================================
// Test: Energy computation
// ============================================================================
void test_energy_computation() {
    std::cout << "Test: Energy computation..." << std::endl;

    // Create plant at upright equilibrium
    CartDoublePendulum<double> plant(
        1.0, 0.5, 0.5, 0.5, 0.5, 9.81,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0  // All zeros = upright
    );

    auto state = plant.getInitialLocalState();
    std::vector<double> global_state(state.begin(), state.end());

    double KE = plant.compute(system::KineticEnergy{}, global_state);
    double PE = plant.compute(system::PotentialEnergy{}, global_state);
    double TE = plant.compute(system::TotalEnergy{}, global_state);

    // At rest, KE should be zero
    assert(approx_equal(KE, 0.0));

    // PE = m1*g*L1 + m2*g*(L1+L2) when upright (θ=0 means cos(θ)=1)
    double expected_PE = 0.5 * 9.81 * 0.5 + 0.5 * 9.81 * (0.5 + 0.5);
    assert(approx_equal(PE, expected_PE, 0.01));

    // Total energy
    assert(approx_equal(TE, KE + PE, EPSILON));

    test_passed("Energy computation");
}

// ============================================================================
// Test: Uncontrolled fall (pendulum should fall)
// ============================================================================
void test_uncontrolled_dynamics() {
    std::cout << "Test: Uncontrolled dynamics..." << std::endl;

    // Create plant with small initial angle
    CartDoublePendulum<double> plant(
        1.0, 0.5, 0.5, 0.5, 0.5, 9.81,
        0.0, 0.2, 0.1, 0.0, 0.0, 0.0  // Small tilt
    );

    auto system = makeTypedODESystem<double>(std::move(plant));
    auto state = system.getInitialState();

    // Simulate for 0.5 seconds with no control
    double dt = 0.001;
    double t = 0.0;
    double t_end = 0.5;

    while (t < t_end) {
        auto derivs = system.computeDerivatives(t, state);
        for (size_t i = 0; i < state.size(); ++i) {
            state[i] = state[i] + derivs[i] * dt;
        }
        t += dt;
    }

    // After 0.5 seconds, angles should have grown (unstable system)
    double theta1 = state[1];
    double theta2 = state[2];

    // The pendulum should have started falling (angles increasing in magnitude)
    assert(std::abs(theta1) > 0.2 || std::abs(theta2) > 0.1);

    test_passed("Uncontrolled dynamics");
}

// ============================================================================
// Test: Linearization
// ============================================================================
void test_linearization() {
    std::cout << "Test: Linearization..." << std::endl;

    double mc = 1.0, m1 = 0.5, m2 = 0.5, L1 = 0.5, L2 = 0.5, g = 9.81;

    auto [A, B] = linearizeCartDoublePendulum(mc, m1, m2, L1, L2, g);

    // Check structure: velocity rows
    assert(approx_equal(A[0][3], 1.0));  // ẋ depends on xdot
    assert(approx_equal(A[1][4], 1.0));  // θ̇₁ depends on ω₁
    assert(approx_equal(A[2][5], 1.0));  // θ̇₂ depends on ω₂

    // Acceleration rows should have non-zero entries for angles
    // The system is unstable, so A should have positive eigenvalues
    // For now, just check that A[4][1] and A[5][2] are positive (gravity destabilizes)
    assert(A[4][1] > 0);  // α₁ increases when θ₁ > 0
    assert(A[5][2] > 0);  // α₂ increases when θ₂ > 0

    // B matrix: only cart acceleration is directly affected by force
    assert(B[3][0] > 0);  // Force affects cart acceleration

    test_passed("Linearization");
}

// ============================================================================
// Test: LQR solver
// ============================================================================
void test_lqr_solver() {
    std::cout << "Test: LQR solver..." << std::endl;

    double mc = 1.0, m1 = 0.5, m2 = 0.5, L1 = 0.5, L2 = 0.5, g = 9.81;

    auto [A, B] = linearizeCartDoublePendulum(mc, m1, m2, L1, L2, g);

    // State weights: penalize angles more than position
    std::array<std::array<double, 6>, 6> Q{};
    Q[0][0] = 10.0;   // x position
    Q[1][1] = 100.0;  // θ₁
    Q[2][2] = 100.0;  // θ₂
    Q[3][3] = 1.0;    // ẋ
    Q[4][4] = 10.0;   // ω₁
    Q[5][5] = 10.0;   // ω₂

    // Control weight
    std::array<std::array<double, 1>, 1> R{};
    R[0][0] = 0.1;

    LQR<6, 1> lqr;
    bool converged = lqr.solve(A, B, Q, R);

    std::cout << "    LQR converged: " << (converged ? "yes" : "no") << std::endl;
    assert(converged);

    auto K = lqr.getGain();

    std::cout << "    K = [";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(2) << K[0][i];
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Gains should be non-zero and reasonable
    for (size_t i = 0; i < 6; ++i) {
        assert(!std::isnan(K[0][i]));
        assert(!std::isinf(K[0][i]));
    }

    test_passed("LQR solver");
}

// ============================================================================
// Test: Closed-loop stabilization
// ============================================================================
void test_closed_loop_stabilization() {
    std::cout << "Test: Closed-loop stabilization..." << std::endl;

    // System parameters
    double mc = 1.0, m1 = 0.5, m2 = 0.5, L1 = 0.5, L2 = 0.5, g = 9.81;

    // Create controller with LQR gains
    auto controller = InvertedDoublePendulumController::createWithLQR(
        mc, m1, m2, L1, L2, g,
        {10.0, 100.0, 100.0, 1.0, 10.0, 10.0},  // Q diagonal
        0.1  // R
    );

    // Set control saturation to realistic limits
    controller.setSaturationLimits({-100.0}, {100.0});

    // Create plant with initial perturbation (near upright)
    CartDoublePendulum<double> plant(
        mc, m1, m2, L1, L2, g,
        0.0,    // x
        0.15,   // θ₁ = 0.15 rad ≈ 8.6°
        0.1,    // θ₂ = 0.1 rad ≈ 5.7°
        0.0,    // ẋ
        0.0,    // ω₁
        0.0     // ω₂
    );

    // Simulate closed-loop system
    std::vector<double> state = {0.0, 0.15, 0.1, 0.0, 0.0, 0.0};
    double dt = 0.001;
    double t = 0.0;
    double t_end = 5.0;  // 5 seconds

    double max_theta1 = 0.0;
    double max_theta2 = 0.0;
    double max_force = 0.0;

    while (t < t_end) {
        // Compute control
        std::array<double, 6> state_arr = {
            state[0], state[1], state[2], state[3], state[4], state[5]
        };
        double F = controller.compute(state_arr)[0];

        // Apply control to plant
        plant.setControlForce(F);

        // Compute derivatives
        auto local_state = plant.getInitialLocalState();
        for (size_t i = 0; i < 6; ++i) {
            local_state[i] = state[i];
        }

        // Simple Euler integration with derivative computation
        // (We're not using the full ODE system here for simplicity)
        auto system = makeTypedODESystem<double>(
            CartDoublePendulum<double>(mc, m1, m2, L1, L2, g)
        );
        auto& comp = system.template getComponent<0>();
        comp.setControlForce(F);

        auto derivs = system.computeDerivatives(t, state);

        // Update state
        for (size_t i = 0; i < 6; ++i) {
            state[i] = state[i] + derivs[i] * dt;
        }

        // Track maximums
        max_theta1 = std::max(max_theta1, std::abs(state[1]));
        max_theta2 = std::max(max_theta2, std::abs(state[2]));
        max_force = std::max(max_force, std::abs(F));

        t += dt;
    }

    std::cout << "    Final state: x=" << std::fixed << std::setprecision(4) << state[0]
              << ", θ₁=" << state[1] << ", θ₂=" << state[2] << std::endl;
    std::cout << "    Max angles: θ₁=" << max_theta1 << ", θ₂=" << max_theta2 << std::endl;
    std::cout << "    Max force: " << max_force << " N" << std::endl;

    // Success criteria: angles should be small after 5 seconds
    assert(std::abs(state[1]) < 0.05);  // θ₁ < 3°
    assert(std::abs(state[2]) < 0.05);  // θ₂ < 3°

    test_passed("Closed-loop stabilization");
}

// ============================================================================
// Test: State function tags work correctly
// ============================================================================
void test_state_function_tags() {
    std::cout << "Test: State function tags..." << std::endl;

    CartDoublePendulum<double> plant(
        1.0, 0.5, 0.5, 0.5, 0.5, 9.81,
        1.0, 0.2, 0.1, 0.5, 0.3, -0.1
    );

    auto state = plant.getInitialLocalState();
    std::vector<double> global(state.begin(), state.end());

    // Test cart state functions
    assert(approx_equal(plant.compute(cart::Position{}, global), 1.0));
    assert(approx_equal(plant.compute(cart::Velocity{}, global), 0.5));
    assert(approx_equal(plant.compute(cart::Mass{}, global), 1.0));

    // Test link 1 state functions
    assert(approx_equal(plant.compute(link1::Angle{}, global), 0.2));
    assert(approx_equal(plant.compute(link1::AngularVelocity{}, global), 0.3));
    assert(approx_equal(plant.compute(link1::Length{}, global), 0.5));

    // Test link 2 state functions
    assert(approx_equal(plant.compute(link2::Angle{}, global), 0.1));
    assert(approx_equal(plant.compute(link2::AngularVelocity{}, global), -0.1));
    assert(approx_equal(plant.compute(link2::Length{}, global), 0.5));

    // Test tip positions
    auto tip1 = plant.compute(link1::TipPosition{}, global);
    double expected_x1 = 1.0 + 0.5 * std::sin(0.2);  // x + L1*sin(θ1)
    double expected_y1 = 0.5 * std::cos(0.2);        // L1*cos(θ1)
    assert(approx_equal(tip1[0], expected_x1, 0.001));
    assert(approx_equal(tip1[1], expected_y1, 0.001));

    test_passed("State function tags");
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Inverted Double Pendulum Control Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_plant_creation();
    test_energy_computation();
    test_uncontrolled_dynamics();
    test_linearization();
    test_lqr_solver();
    test_closed_loop_stabilization();
    test_state_function_tags();

    std::cout << "========================================" << std::endl;
    std::cout << "All inverted pendulum tests passed!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
