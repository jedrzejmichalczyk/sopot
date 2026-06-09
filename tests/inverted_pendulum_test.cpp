/**
 * @file inverted_pendulum_test.cpp
 * @brief Tests for the cart-N-pendulum control system (configured for 6 links)
 *
 * Verifies:
 * 1. Generic N-link plant reproduces the analytic double pendulum (N=2)
 * 2. Plant creation and energy computation for the 6-link chain
 * 3. Uncontrolled dynamics (the 6 pendulums fall)
 * 4. Linearization structure about the upright equilibrium
 * 5. LQR gain computation for the 14-state system
 * 6. Closed-loop stabilization of all 6 inverted pendulums
 */

#include "physics/control/cart_double_pendulum.hpp"
#include "physics/control/cart_n_pendulum.hpp"
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

// Number of pendulum links balanced on the cart.
constexpr size_t NLINKS = 6;
constexpr size_t NSTATE = 2 * (NLINKS + 1);  // 14

bool approx_equal(double a, double b, double eps = EPSILON) {
    return std::abs(a - b) < eps;
}

void test_passed(const char* name) {
    std::cout << "  [PASS] " << name << std::endl;
}

// Default six-link configuration used across the controlled tests.
struct Config {
    double mc = 2.0;
    double g = 9.81;
    std::array<double, NLINKS> masses;
    std::array<double, NLINKS> lengths;
    Config() {
        masses.fill(0.1);
        lengths.fill(0.3);
    }
};

std::array<double, NSTATE> lqrWeights() {
    std::array<double, NSTATE> q{};
    q[0] = 1.0;                                  // cart position
    for (size_t i = 1; i <= NLINKS; ++i) q[i] = 200.0;          // link angles
    q[NLINKS + 1] = 1.0;                         // cart velocity
    for (size_t i = NLINKS + 2; i < NSTATE; ++i) q[i] = 10.0;   // link rates
    return q;
}

// ============================================================================
// Test: Generic plant matches the analytic double pendulum (N=2)
// ============================================================================
void test_matches_double_pendulum() {
    std::cout << "Test: Generic N=2 matches CartDoublePendulum..." << std::endl;

    double mc = 1.0, m1 = 0.5, m2 = 0.4, L1 = 0.5, L2 = 0.6, g = 9.81;
    CartDoublePendulum<double> ref(mc, m1, m2, L1, L2, g,
                                   0.3, 0.2, -0.1, 0.5, 0.4, -0.3);
    CartNPendulum<2, double> gen(mc, {m1, m2}, {L1, L2}, g,
                                 {0.3, 0.2, -0.1, 0.5, 0.4, -0.3});
    ref.setControlForce(2.0);
    gen.setControlForce(2.0);

    std::vector<double> st = {0.3, 0.2, -0.1, 0.5, 0.4, -0.3};
    std::span<const double> sp(st);
    TypedRegistry<double> reg{};

    auto dref = ref.derivatives(0.0, sp, sp, reg);
    auto dgen = gen.derivatives(0.0, sp, sp, reg);
    for (size_t i = 0; i < 6; ++i) {
        assert(approx_equal(dref[i], dgen[i], 1e-9));
    }

    // Energies must agree as well.
    assert(approx_equal(ref.compute(system::KineticEnergy{}, st), gen.kineticEnergy(sp), 1e-9));
    assert(approx_equal(ref.compute(system::PotentialEnergy{}, st), gen.potentialEnergy(sp), 1e-9));

    test_passed("Generic N=2 matches CartDoublePendulum");
}

// ============================================================================
// Test: Plant creation and initial state (6 links)
// ============================================================================
void test_plant_creation() {
    std::cout << "Test: Six-link plant creation..." << std::endl;

    Config cfg;
    std::array<double, NSTATE> x0{};
    for (size_t i = 1; i <= NLINKS; ++i) x0[i] = 0.05 * static_cast<double>(i);

    CartNPendulum<NLINKS, double> plant(cfg.mc, cfg.masses, cfg.lengths, cfg.g, x0);
    auto state = plant.getInitialLocalState();

    assert(state.size == NSTATE);
    for (size_t i = 0; i < NSTATE; ++i) {
        assert(approx_equal(state[i], x0[i]));
    }
    test_passed("Six-link plant creation");
}

// ============================================================================
// Test: Energy computation at the upright equilibrium
// ============================================================================
void test_energy_computation() {
    std::cout << "Test: Energy computation..." << std::endl;

    Config cfg;
    std::array<double, NSTATE> x0{};  // all upright, at rest
    CartNPendulum<NLINKS, double> plant(cfg.mc, cfg.masses, cfg.lengths, cfg.g, x0);

    std::vector<double> st(x0.begin(), x0.end());
    std::span<const double> sp(st);

    double KE = plant.kineticEnergy(sp);
    double PE = plant.potentialEnergy(sp);
    double TE = plant.totalEnergy(sp);

    assert(approx_equal(KE, 0.0));  // at rest

    // PE = Σ mᵢ g hᵢ with hᵢ = Σ_{j≤i} Lⱼ (all upright, cos=1).
    double expected_PE = 0.0;
    double height = 0.0;
    for (size_t i = 0; i < NLINKS; ++i) {
        height += cfg.lengths[i];
        expected_PE += cfg.masses[i] * cfg.g * height;
    }
    assert(approx_equal(PE, expected_PE, 1e-9));
    assert(approx_equal(TE, KE + PE, EPSILON));

    test_passed("Energy computation");
}

// ============================================================================
// Test: Uncontrolled dynamics (pendulums fall)
// ============================================================================
void test_uncontrolled_dynamics() {
    std::cout << "Test: Uncontrolled dynamics..." << std::endl;

    Config cfg;
    std::array<double, NSTATE> x0{};
    for (size_t i = 1; i <= NLINKS; ++i) x0[i] = 0.1;  // tilted

    auto system = makeTypedODESystem<double>(
        CartNPendulum<NLINKS, double>(cfg.mc, cfg.masses, cfg.lengths, cfg.g, x0)
    );
    auto state = system.getInitialState();

    double dt = 0.001, t = 0.0, t_end = 0.5;
    while (t < t_end) {
        auto derivs = system.computeDerivatives(t, state);
        for (size_t i = 0; i < state.size(); ++i) state[i] += derivs[i] * dt;
        t += dt;
    }

    // With no control, the chain must have departed from its initial tilt.
    double max_deviation = 0.0;
    for (size_t i = 1; i <= NLINKS; ++i) {
        max_deviation = std::max(max_deviation, std::abs(state[i] - 0.1));
    }
    assert(max_deviation > 0.05);

    test_passed("Uncontrolled dynamics");
}

// ============================================================================
// Test: Linearization structure
// ============================================================================
void test_linearization() {
    std::cout << "Test: Linearization..." << std::endl;

    Config cfg;
    auto [A, B] = linearizeCartNPendulum<NLINKS>(cfg.mc, cfg.masses, cfg.lengths, cfg.g);

    constexpr size_t NDof = NLINKS + 1;

    // Upper-right block is identity: position rates equal velocities.
    for (size_t i = 0; i < NDof; ++i) {
        assert(approx_equal(A[i][NDof + i], 1.0));
    }

    // Each angle's acceleration grows with its own angle (gravity destabilizes).
    for (size_t a = 0; a < NLINKS; ++a) {
        size_t accel_row = NDof + 1 + a;  // α_a row
        size_t angle_col = 1 + a;         // θ_a column
        assert(A[accel_row][angle_col] > 0.0);
    }

    // Force enters the cart acceleration.
    assert(B[NDof][0] > 0.0);

    test_passed("Linearization");
}

// ============================================================================
// Test: LQR solver produces finite gains
// ============================================================================
void test_lqr_solver() {
    std::cout << "Test: LQR solver..." << std::endl;

    Config cfg;
    auto [A, B] = linearizeCartNPendulum<NLINKS>(cfg.mc, cfg.masses, cfg.lengths, cfg.g);

    auto q_diag = lqrWeights();
    std::array<std::array<double, NSTATE>, NSTATE> Q{};
    for (size_t i = 0; i < NSTATE; ++i) Q[i][i] = q_diag[i];

    std::array<std::array<double, 1>, 1> R{};
    R[0][0] = 0.001;

    LQR<NSTATE, 1> lqr;
    bool converged = lqr.solve(A, B, Q, R, 0.005);
    std::cout << "    LQR converged: " << (converged ? "yes" : "no") << std::endl;
    assert(converged);

    auto K = lqr.getGain();
    for (size_t i = 0; i < NSTATE; ++i) {
        assert(!std::isnan(K[0][i]));
        assert(!std::isinf(K[0][i]));
    }

    test_passed("LQR solver");
}

// ============================================================================
// Test: Closed-loop stabilization of 6 inverted pendulums
// ============================================================================
void test_closed_loop_stabilization() {
    std::cout << "Test: Closed-loop stabilization (6 links)..." << std::endl;

    Config cfg;
    auto controller = CartNPendulumController<NLINKS>::createWithLQR(
        cfg.mc, cfg.masses, cfg.lengths, cfg.g, lqrWeights(), 0.001, 0.005
    );
    controller.setSaturationLimits({-500.0}, {500.0});

    CartNPendulum<NLINKS, double> plant(cfg.mc, cfg.masses, cfg.lengths, cfg.g);
    TypedRegistry<double> reg{};

    std::vector<double> state(NSTATE, 0.0);
    for (size_t i = 1; i <= NLINKS; ++i) state[i] = 0.03;  // ~1.7° each

    double dt = 0.001, t = 0.0, t_end = 5.0;
    double max_angle = 0.0, max_force = 0.0;

    while (t < t_end) {
        std::array<double, NSTATE> s_arr;
        for (size_t i = 0; i < NSTATE; ++i) s_arr[i] = state[i];
        double F = controller.computeForce(s_arr);
        max_force = std::max(max_force, std::abs(F));

        plant.setControlForce(F);
        std::span<const double> sp(state);
        auto derivs = plant.derivatives(t, sp, sp, reg);
        for (size_t i = 0; i < NSTATE; ++i) state[i] += derivs[i] * dt;

        for (size_t i = 1; i <= NLINKS; ++i) {
            max_angle = std::max(max_angle, std::abs(state[i]));
        }
        t += dt;
    }

    double final_angle = 0.0;
    for (size_t i = 1; i <= NLINKS; ++i) {
        final_angle = std::max(final_angle, std::abs(state[i]));
    }

    std::cout << "    Max angle during run: " << std::fixed << std::setprecision(4)
              << max_angle << " rad" << std::endl;
    std::cout << "    Final max angle:      " << final_angle << " rad" << std::endl;
    std::cout << "    Max force:            " << max_force << " N" << std::endl;
    std::cout << "    Final cart position:  " << state[0] << " m" << std::endl;

    // The controller must keep every link near upright and drive them back.
    assert(max_angle < 0.2);     // never falls
    assert(final_angle < 0.01);  // converges back toward upright

    test_passed("Closed-loop stabilization (6 links)");
}

// ============================================================================
// Test: State queries
// ============================================================================
void test_state_queries() {
    std::cout << "Test: State queries..." << std::endl;

    Config cfg;
    std::array<double, NSTATE> x0{};
    x0[0] = 1.0;                         // cart position
    for (size_t i = 1; i <= NLINKS; ++i) x0[i] = 0.1 * static_cast<double>(i);
    x0[NLINKS + 1] = 0.5;                // cart velocity

    CartNPendulum<NLINKS, double> plant(cfg.mc, cfg.masses, cfg.lengths, cfg.g, x0);
    std::vector<double> st(x0.begin(), x0.end());
    std::span<const double> sp(st);

    assert(approx_equal(plant.cartPosition(sp), 1.0));
    assert(approx_equal(plant.cartVelocity(sp), 0.5));
    assert(approx_equal(plant.compute(cart::Mass{}, st), cfg.mc));

    for (size_t i = 0; i < NLINKS; ++i) {
        assert(approx_equal(plant.linkAngle(i, sp), 0.1 * static_cast<double>(i + 1)));
    }

    // Tip of the first link: cart_x + L₁ sin θ₁, L₁ cos θ₁.
    auto tip0 = plant.linkTipPosition(0, sp);
    double theta1 = 0.1;
    assert(approx_equal(tip0[0], 1.0 + cfg.lengths[0] * std::sin(theta1), 1e-9));
    assert(approx_equal(tip0[1], cfg.lengths[0] * std::cos(theta1), 1e-9));

    // Tip of the last link must accumulate every link's contribution.
    auto tipN = plant.linkTipPosition(NLINKS - 1, sp);
    double px = 1.0, py = 0.0;
    for (size_t i = 0; i < NLINKS; ++i) {
        double th = 0.1 * static_cast<double>(i + 1);
        px += cfg.lengths[i] * std::sin(th);
        py += cfg.lengths[i] * std::cos(th);
    }
    assert(approx_equal(tipN[0], px, 1e-9));
    assert(approx_equal(tipN[1], py, 1e-9));

    test_passed("State queries");
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Cart-N-Pendulum Control Tests (N = " << NLINKS << ")" << std::endl;
    std::cout << "========================================" << std::endl;

    test_matches_double_pendulum();
    test_plant_creation();
    test_energy_computation();
    test_uncontrolled_dynamics();
    test_linearization();
    test_lqr_solver();
    test_closed_loop_stabilization();
    test_state_queries();

    std::cout << "========================================" << std::endl;
    std::cout << "All cart-" << NLINKS << "-pendulum tests passed!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
