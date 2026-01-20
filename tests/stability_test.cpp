#include "core/stability.hpp"
#include "physics/connected_masses/connectivity_matrix_2d.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot;
using namespace sopot::connected_masses;

/**
 * @brief Test 1: Compile-time stability analysis
 */
void test_compile_time_analysis() {
    std::cout << "\n=== Test 1: Compile-Time Stability Analysis ===\n";

    // Example 1: 3x3 rectangular grid (no diagonals)
    constexpr size_t rows = 3;
    constexpr size_t cols = 3;
    constexpr double k = 10.0;  // N/m
    constexpr double m = 1.0;   // kg
    constexpr double c = 0.5;   // N·s/m

    std::cout << "\nRectangular Grid (3x3, no diagonals):\n";
    std::cout << "  k = " << k << " N/m\n";
    std::cout << "  m = " << m << " kg\n";
    std::cout << "  c = " << c << " N·s/m\n";

    constexpr auto stability_rect = GridStabilityInfo<rows, cols, false>::analyze(k, m, c);

    std::cout << "\nStability Analysis:\n";
    std::cout << "  Max connectivity: " << stability_rect.max_connectivity << " springs\n";
    std::cout << "  Effective stiffness: " << stability_rect.effective_stiffness << " N/m\n";
    std::cout << "  Max frequency: " << stability_rect.max_frequency << " rad/s\n";
    std::cout << "  Recommended dt: " << std::fixed << std::setprecision(6)
              << stability_rect.dt_recommended << " s\n";

    // Example 2: Same grid with diagonals
    std::cout << "\nRectangular Grid (3x3, with diagonals):\n";
    constexpr auto stability_diag = GridStabilityInfo<rows, cols, true>::analyze(k, m, c);

    std::cout << "  Max connectivity: " << stability_diag.max_connectivity << " springs\n";
    std::cout << "  Effective stiffness: " << stability_diag.effective_stiffness << " N/m\n";
    std::cout << "  Max frequency: " << stability_diag.max_frequency << " rad/s\n";
    std::cout << "  Recommended dt: " << std::fixed << std::setprecision(6)
              << stability_diag.dt_recommended << " s\n";

    // Example 3: Triangular grid
    std::cout << "\nTriangular Grid (3x3):\n";
    constexpr auto stability_tri = TriangularGridStabilityInfo<rows, cols>::analyze(k, m, c);

    std::cout << "  Max connectivity: " << stability_tri.max_connectivity << " springs\n";
    std::cout << "  Effective stiffness: " << stability_tri.effective_stiffness << " N/m\n";
    std::cout << "  Max frequency: " << stability_tri.max_frequency << " rad/s\n";
    std::cout << "  Recommended dt: " << std::fixed << std::setprecision(6)
              << stability_tri.dt_recommended << " s\n";

    // Validate that diagonal grids require smaller time steps
    static_assert(stability_diag.dt_recommended < stability_rect.dt_recommended,
                  "Grids with diagonals should have more restrictive time step");

    std::cout << "\n✓ Compile-time assertions passed!\n";
}

/**
 * @brief Test 2: Demonstrate stable vs unstable integration
 */
void test_stable_vs_unstable() {
    std::cout << "\n=== Test 2: Stable vs Unstable Time Steps ===\n";

    constexpr size_t Rows = 3;
    constexpr size_t Cols = 3;
    constexpr double k = 50.0;   // Stiffer springs
    constexpr double m = 1.0;
    constexpr double c = 0.5;

    // Get recommended time step
    constexpr auto stability = GridStabilityInfo<Rows, Cols, false>::analyze(k, m, c);
    constexpr double dt_safe = stability.dt_recommended;

    std::cout << "System parameters:\n";
    std::cout << "  k = " << k << " N/m\n";
    std::cout << "  m = " << m << " kg\n";
    std::cout << "  Recommended dt = " << std::fixed << std::setprecision(6)
              << dt_safe << " s\n";

    // Create system
    auto system = makeGrid2DSystem<double, Rows, Cols, false>(m, 1.0, k, c);
    auto state = system.getInitialState();

    // Perturb center mass
    state[4 * 4 + 1] += 0.5;  // y position of mass 4

    // Test Case 1: Safe time step (0.8x recommended)
    {
        std::cout << "\nTest Case 1: Safe dt = " << 0.8 * dt_safe << " s (0.8x recommended)\n";
        auto test_state = state;
        double t = 0.0;
        double dt = 0.8 * dt_safe;
        double t_end = 2.0;
        size_t n = system.getStateDimension();

        bool stable = true;
        int steps = 0;
        while (t < t_end && stable) {
            // RK4 step
            auto k1 = system.computeDerivatives(t, test_state);
            std::vector<double> s2(n), s3(n), s4(n);
            for (size_t i = 0; i < n; ++i) s2[i] = test_state[i] + 0.5 * dt * k1[i];
            auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
            for (size_t i = 0; i < n; ++i) s3[i] = test_state[i] + 0.5 * dt * k2[i];
            auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
            for (size_t i = 0; i < n; ++i) s4[i] = test_state[i] + dt * k3[i];
            auto k4 = system.computeDerivatives(t + dt, s4);
            for (size_t i = 0; i < n; ++i)
                test_state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);

            // Check for numerical blow-up
            for (size_t i = 0; i < n; ++i) {
                if (std::abs(test_state[i]) > 1e6 || std::isnan(test_state[i])) {
                    stable = false;
                    break;
                }
            }

            t += dt;
            ++steps;
        }

        if (stable) {
            double y_final = test_state[4 * 4 + 1];
            std::cout << "  ✓ STABLE: Completed " << steps << " steps\n";
            std::cout << "  Final y-position of center mass: " << y_final << " m\n";
        } else {
            std::cout << "  ✗ UNSTABLE: Blew up after " << steps << " steps\n";
        }
    }

    // Test Case 2: Marginal time step (1.0x recommended)
    {
        std::cout << "\nTest Case 2: Marginal dt = " << dt_safe << " s (1.0x recommended)\n";
        auto test_state = state;
        double t = 0.0;
        double dt = dt_safe;
        double t_end = 2.0;
        size_t n = system.getStateDimension();

        bool stable = true;
        int steps = 0;
        while (t < t_end && stable) {
            auto k1 = system.computeDerivatives(t, test_state);
            std::vector<double> s2(n), s3(n), s4(n);
            for (size_t i = 0; i < n; ++i) s2[i] = test_state[i] + 0.5 * dt * k1[i];
            auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
            for (size_t i = 0; i < n; ++i) s3[i] = test_state[i] + 0.5 * dt * k2[i];
            auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
            for (size_t i = 0; i < n; ++i) s4[i] = test_state[i] + dt * k3[i];
            auto k4 = system.computeDerivatives(t + dt, s4);
            for (size_t i = 0; i < n; ++i)
                test_state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);

            for (size_t i = 0; i < n; ++i) {
                if (std::abs(test_state[i]) > 1e6 || std::isnan(test_state[i])) {
                    stable = false;
                    break;
                }
            }

            t += dt;
            ++steps;
        }

        if (stable) {
            double y_final = test_state[4 * 4 + 1];
            std::cout << "  ✓ STABLE: Completed " << steps << " steps\n";
            std::cout << "  Final y-position of center mass: " << y_final << " m\n";
        } else {
            std::cout << "  ✗ UNSTABLE: Blew up after " << steps << " steps\n";
        }
    }

    // Test Case 3: Unsafe time step (2.0x recommended)
    {
        std::cout << "\nTest Case 3: Unsafe dt = " << 2.0 * dt_safe << " s (2.0x recommended)\n";
        auto test_state = state;
        double t = 0.0;
        double dt = 2.0 * dt_safe;
        double t_end = 2.0;
        size_t n = system.getStateDimension();

        bool stable = true;
        int steps = 0;
        while (t < t_end && stable) {
            auto k1 = system.computeDerivatives(t, test_state);
            std::vector<double> s2(n), s3(n), s4(n);
            for (size_t i = 0; i < n; ++i) s2[i] = test_state[i] + 0.5 * dt * k1[i];
            auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
            for (size_t i = 0; i < n; ++i) s3[i] = test_state[i] + 0.5 * dt * k2[i];
            auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
            for (size_t i = 0; i < n; ++i) s4[i] = test_state[i] + dt * k3[i];
            auto k4 = system.computeDerivatives(t + dt, s4);
            for (size_t i = 0; i < n; ++i)
                test_state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);

            for (size_t i = 0; i < n; ++i) {
                if (std::abs(test_state[i]) > 1e6 || std::isnan(test_state[i])) {
                    stable = false;
                    break;
                }
            }

            t += dt;
            ++steps;
        }

        if (stable) {
            double y_final = test_state[4 * 4 + 1];
            std::cout << "  ⚠ STABLE (unexpectedly): Completed " << steps << " steps\n";
            std::cout << "  Final y-position of center mass: " << y_final << " m\n";
        } else {
            std::cout << "  ✓ UNSTABLE (as expected): Blew up after " << steps << " steps\n";
        }
    }
}

/**
 * @brief Test 3: Safety factor computation
 */
void test_safety_factor() {
    std::cout << "\n=== Test 3: Safety Factor Computation ===\n";

    constexpr double k = 100.0;
    constexpr double m = 1.0;
    constexpr double c = 1.0;

    constexpr auto stability = GridStabilityInfo<4, 4, true>::analyze(k, m, c);

    std::cout << "System: 4x4 grid with diagonals\n";
    std::cout << "  k = " << k << " N/m\n";
    std::cout << "  m = " << m << " kg\n";
    std::cout << "  Recommended dt = " << std::fixed << std::setprecision(6)
              << stability.dt_recommended << " s\n";

    std::cout << "\nSafety factor analysis:\n";

    double test_dts[] = {0.0001, 0.0005, 0.001, 0.002, 0.005, 0.01};
    for (double dt : test_dts) {
        double factor = computeSafetyFactor(dt, stability);
        bool safe = isStableTimeStep(dt, stability);

        std::cout << "  dt = " << std::setw(8) << dt << " s  ->  factor = "
                  << std::setw(6) << std::setprecision(3) << factor
                  << "  [" << (safe ? "✓ SAFE" : "✗ RISKY") << "]\n";
    }
}

/**
 * @brief Test 4: Comparison between grid types
 */
void test_grid_comparison() {
    std::cout << "\n=== Test 4: Grid Type Comparison ===\n";

    constexpr double k = 20.0;
    constexpr double m = 0.5;
    constexpr double c = 0.3;

    std::cout << "Common parameters:\n";
    std::cout << "  k = " << k << " N/m\n";
    std::cout << "  m = " << m << " kg\n";
    std::cout << "  c = " << c << " N·s/m\n";

    std::cout << "\n" << std::setw(20) << "Grid Type"
              << std::setw(15) << "Max Conn"
              << std::setw(15) << "k_eff (N/m)"
              << std::setw(15) << "ω_max (rad/s)"
              << std::setw(15) << "dt_rec (s)\n";
    std::cout << std::string(79, '-') << "\n";

    // 5x5 grids of various types
    constexpr auto rect = GridStabilityInfo<5, 5, false>::analyze(k, m, c);
    std::cout << std::setw(20) << "Rectangular"
              << std::setw(15) << rect.max_connectivity
              << std::setw(15) << std::fixed << std::setprecision(2) << rect.effective_stiffness
              << std::setw(15) << std::fixed << std::setprecision(3) << rect.max_frequency
              << std::setw(15) << std::fixed << std::setprecision(6) << rect.dt_recommended << "\n";

    constexpr auto diag = GridStabilityInfo<5, 5, true>::analyze(k, m, c);
    std::cout << std::setw(20) << "Rect + Diagonals"
              << std::setw(15) << diag.max_connectivity
              << std::setw(15) << std::fixed << std::setprecision(2) << diag.effective_stiffness
              << std::setw(15) << std::fixed << std::setprecision(3) << diag.max_frequency
              << std::setw(15) << std::fixed << std::setprecision(6) << diag.dt_recommended << "\n";

    constexpr auto tri = TriangularGridStabilityInfo<5, 5>::analyze(k, m, c);
    std::cout << std::setw(20) << "Triangular"
              << std::setw(15) << tri.max_connectivity
              << std::setw(15) << std::fixed << std::setprecision(2) << tri.effective_stiffness
              << std::setw(15) << std::fixed << std::setprecision(3) << tri.max_frequency
              << std::setw(15) << std::fixed << std::setprecision(6) << tri.dt_recommended << "\n";

    std::cout << "\n✓ More connections -> Higher frequency -> Smaller time step required\n";
}

int main() {
    std::cout << "========================================\n";
    std::cout << "SOPOT Stability Analysis Test Suite\n";
    std::cout << "========================================\n";
    std::cout << "\nDemonstrating compile-time stability checks for ODE systems:\n";
    std::cout << "- Rectangular grids (with/without diagonals)\n";
    std::cout << "- Triangular grids\n";
    std::cout << "- Stable vs unstable time step selection\n";

    try {
        test_compile_time_analysis();
        test_safety_factor();
        test_grid_comparison();
        test_stable_vs_unstable();  // Run last as it may show instability

        std::cout << "\n========================================\n";
        std::cout << "All tests completed! ✓\n";
        std::cout << "========================================\n";
        std::cout << "\nKey Takeaways:\n";
        std::cout << "  • Stability bounds are computable at compile-time\n";
        std::cout << "  • More connections -> stricter time step requirements\n";
        std::cout << "  • Recommended dt provides ~10% safety margin\n";
        std::cout << "  • Exceeding recommended dt leads to instability\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
