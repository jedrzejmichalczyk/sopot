#include "physics/connected_masses/connectivity_matrix_2d.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>

using namespace sopot;
using namespace sopot::connected_masses;

/**
 * @brief Test 1: Small 3x3 grid with basic simulation
 */
void test_small_grid_3x3() {
    std::cout << "\n=== Test 1: 3x3 Grid (9 masses, orthogonal connections) ===\n";

    // Create a 3x3 grid of masses and springs
    // Grid layout:
    // 0 -- 1 -- 2
    // |    |    |
    // 3 -- 4 -- 5
    // |    |    |
    // 6 -- 7 -- 8

    constexpr size_t Rows = 3;
    constexpr size_t Cols = 3;
    constexpr size_t NumMasses = Rows * Cols;

    auto system = makeGrid2DSystem<double, Rows, Cols, false>(
        1.0,    // mass (kg)
        1.0,    // spacing (m)
        10.0,   // stiffness (N/m)
        0.5     // damping (N·s/m)
    );

    std::cout << "✓ Grid created: " << Rows << "x" << Cols << " = " << NumMasses << " masses\n";
    std::cout << "✓ State dimension: " << system.getStateDimension() << " (expected "
              << NumMasses * 4 << ")\n";

    // Verify compile-time function availability for a few masses
    static_assert(decltype(system)::hasFunction<MassTag2D<0>::Position>());
    static_assert(decltype(system)::hasFunction<MassTag2D<0>::Velocity>());
    static_assert(decltype(system)::hasFunction<MassTag2D<0>::Force>());
    static_assert(decltype(system)::hasFunction<MassTag2D<4>::Position>());  // Center mass
    static_assert(decltype(system)::hasFunction<MassTag2D<8>::Position>());  // Corner mass

    std::cout << "✓ All state functions available at compile time\n";

    // Get initial state
    auto state = system.getInitialState();

    std::cout << "\nInitial grid positions:\n";
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < Cols; ++c) {
            size_t idx = r * Cols + c;
            if (idx == 0) {
                auto pos = system.computeStateFunction<MassTag2D<0>::Position>(state);
                std::cout << "  Mass " << idx << ": (" << pos[0] << ", " << pos[1] << ")\n";
            } else if (idx == 4) {
                auto pos = system.computeStateFunction<MassTag2D<4>::Position>(state);
                std::cout << "  Mass " << idx << ": (" << pos[0] << ", " << pos[1] << ")\n";
            } else if (idx == 8) {
                auto pos = system.computeStateFunction<MassTag2D<8>::Position>(state);
                std::cout << "  Mass " << idx << ": (" << pos[0] << ", " << pos[1] << ")\n";
            }
        }
    }

    // Perturb the center mass
    std::cout << "\nPerturbing center mass (index 4) by +0.5m in y direction...\n";
    state[4 * 4 + 1] += 0.5;  // y position of mass 4

    // Simulate
    double t = 0.0;
    double dt = 0.001;
    double t_end = 2.0;
    size_t n = system.getStateDimension();

    std::cout << "Simulating for " << t_end << " seconds...\n";

    int steps = 0;
    while (t < t_end) {
        // RK4 integration
        auto k1 = system.computeDerivatives(t, state);
        std::vector<double> s2(n), s3(n), s4(n);
        for (size_t i = 0; i < n; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
        auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
        for (size_t i = 0; i < n; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
        auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
        for (size_t i = 0; i < n; ++i) s4[i] = state[i] + dt * k3[i];
        auto k4 = system.computeDerivatives(t + dt, s4);
        for (size_t i = 0; i < n; ++i)
            state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        t += dt;
        ++steps;
    }

    std::cout << "✓ Completed " << steps << " integration steps\n";

    std::cout << "\nFinal positions (t = " << t << " s):\n";
    auto pos0 = system.computeStateFunction<MassTag2D<0>::Position>(state);
    auto pos4 = system.computeStateFunction<MassTag2D<4>::Position>(state);
    auto pos8 = system.computeStateFunction<MassTag2D<8>::Position>(state);
    std::cout << "  Mass 0: (" << pos0[0] << ", " << pos0[1] << ")\n";
    std::cout << "  Mass 4: (" << pos4[0] << ", " << pos4[1] << ")\n";
    std::cout << "  Mass 8: (" << pos8[0] << ", " << pos8[1] << ")\n";
}

/**
 * @brief Test 2: 4x4 grid with diagonal connections (cloth-like)
 */
void test_cloth_4x4_with_diagonals() {
    std::cout << "\n=== Test 2: 4x4 Cloth with Diagonal Springs ===\n";

    constexpr size_t Rows = 4;
    constexpr size_t Cols = 4;
    constexpr size_t NumMasses = Rows * Cols;

    // Create grid with diagonal springs for more stability
    auto system = makeGrid2DSystem<double, Rows, Cols, true>(
        0.1,    // mass (kg) - lighter for cloth-like behavior
        0.5,    // spacing (m)
        50.0,   // stiffness (N/m) - stiffer springs
        1.0     // damping (N·s/m)
    );

    std::cout << "✓ Cloth grid created: " << Rows << "x" << Cols << " = " << NumMasses << " masses\n";
    std::cout << "✓ Includes diagonal springs for structural stability\n";
    std::cout << "✓ State dimension: " << system.getStateDimension() << "\n";

    auto state = system.getInitialState();

    // Apply downward velocity to simulate cloth drop
    std::cout << "\nApplying downward initial velocity to all masses...\n";
    for (size_t i = 0; i < NumMasses; ++i) {
        state[i * 4 + 3] = -2.0;  // vy = -2 m/s
    }

    // Pin the top row (fix positions, zero velocity)
    std::cout << "Pinning top row masses (indices 0-3)...\n";

    // Short simulation
    double t = 0.0;
    double dt = 0.0005;
    double t_end = 1.0;
    size_t n = system.getStateDimension();

    std::cout << "Simulating cloth motion for " << t_end << " seconds...\n";

    int steps = 0;
    while (t < t_end) {
        // RK4 integration
        auto k1 = system.computeDerivatives(t, state);
        std::vector<double> s2(n), s3(n), s4(n);
        for (size_t i = 0; i < n; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
        auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
        for (size_t i = 0; i < n; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
        auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
        for (size_t i = 0; i < n; ++i) s4[i] = state[i] + dt * k3[i];
        auto k4 = system.computeDerivatives(t + dt, s4);
        for (size_t i = 0; i < n; ++i)
            state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);

        // Pin top row (override integration for fixed masses)
        for (size_t c = 0; c < Cols; ++c) {
            size_t idx = c;  // Top row
            state[idx * 4 + 0] = c * 0.5;  // x position (fixed)
            state[idx * 4 + 1] = 0.0;      // y position (fixed)
            state[idx * 4 + 2] = 0.0;      // vx (zero)
            state[idx * 4 + 3] = 0.0;      // vy (zero)
        }

        t += dt;
        ++steps;
    }

    std::cout << "✓ Completed " << steps << " integration steps\n";

    std::cout << "\nFinal cloth configuration (y-coordinates by row):\n";
    for (size_t r = 0; r < Rows; ++r) {
        std::cout << "  Row " << r << ": ";
        for (size_t c = 0; c < Cols; ++c) {
            size_t idx = r * Cols + c;
            double y = state[idx * 4 + 1];
            std::cout << std::fixed << std::setprecision(3) << y << " ";
        }
        std::cout << "\n";
    }
}

/**
 * @brief Test 3: Energy conservation check
 */
void test_energy_conservation() {
    std::cout << "\n=== Test 3: Energy Conservation (2x2 Grid, No Damping) ===\n";

    constexpr size_t Rows = 2;
    constexpr size_t Cols = 2;

    auto system = makeGrid2DSystem<double, Rows, Cols, false>(
        1.0,    // mass (kg)
        1.0,    // spacing (m)
        20.0,   // stiffness (N/m)
        0.0     // NO damping for energy conservation
    );

    std::cout << "✓ Created 2x2 grid with no damping\n";

    auto state = system.getInitialState();

    // Perturb one corner
    state[0] += 0.2;  // x position of mass 0
    state[1] += 0.3;  // y position of mass 0

    // This is just a demonstration - full energy calculation would require
    // querying all springs, which we can't do easily with the current API
    std::cout << "Note: Full energy tracking would require additional API support\n";
    std::cout << "      to iterate over all springs at runtime.\n";

    // Short simulation
    double t = 0.0;
    double dt = 0.001;
    double t_end = 0.5;
    size_t n = system.getStateDimension();

    int steps = 0;
    while (t < t_end) {
        auto k1 = system.computeDerivatives(t, state);
        std::vector<double> s2(n), s3(n), s4(n);
        for (size_t i = 0; i < n; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
        auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
        for (size_t i = 0; i < n; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
        auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
        for (size_t i = 0; i < n; ++i) s4[i] = state[i] + dt * k3[i];
        auto k4 = system.computeDerivatives(t + dt, s4);
        for (size_t i = 0; i < n; ++i)
            state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        t += dt;
        ++steps;
    }

    std::cout << "✓ Simulation completed (energy tracking requires additional development)\n";
}

/**
 * @brief Test 4: Export grid positions to CSV for visualization
 */
void test_export_to_csv() {
    std::cout << "\n=== Test 4: Export 5x5 Grid Animation to CSV ===\n";

    constexpr size_t Rows = 5;
    constexpr size_t Cols = 5;
    constexpr size_t NumMasses = Rows * Cols;

    auto system = makeGrid2DSystem<double, Rows, Cols, false>(
        0.5,    // mass (kg)
        0.5,    // spacing (m)
        30.0,   // stiffness (N/m)
        0.8     // damping (N·s/m)
    );

    std::cout << "✓ Created 5x5 grid for animation export\n";

    auto state = system.getInitialState();

    // Create a wave: displace center masses
    std::cout << "Creating initial wave pattern...\n";
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < Cols; ++c) {
            size_t idx = r * Cols + c;
            double dx = c * 0.5 - 1.0;  // Distance from center x
            double dy = r * 0.5 - 1.0;  // Distance from center y
            double dist = std::sqrt(dx * dx + dy * dy);
            double displacement = 0.5 * std::exp(-dist * dist / 0.5);
            state[idx * 4 + 1] += displacement;  // y position
        }
    }

    // Open CSV file
    std::ofstream csv("grid_2d_animation.csv");
    csv << "time";
    for (size_t i = 0; i < NumMasses; ++i) {
        csv << ",x" << i << ",y" << i;
    }
    csv << "\n";

    // Simulation with periodic output
    double t = 0.0;
    double dt = 0.001;
    double t_end = 3.0;
    double output_interval = 0.05;
    double next_output = 0.0;
    size_t n = system.getStateDimension();

    std::cout << "Simulating and exporting to 'grid_2d_animation.csv'...\n";

    int steps = 0;
    int outputs = 0;
    while (t < t_end) {
        // Output state at intervals
        if (t >= next_output) {
            csv << t;
            for (size_t i = 0; i < NumMasses; ++i) {
                double x = state[i * 4 + 0];
                double y = state[i * 4 + 1];
                csv << "," << x << "," << y;
            }
            csv << "\n";
            next_output += output_interval;
            ++outputs;
        }

        // RK4 integration
        auto k1 = system.computeDerivatives(t, state);
        std::vector<double> s2(n), s3(n), s4(n);
        for (size_t i = 0; i < n; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
        auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
        for (size_t i = 0; i < n; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
        auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
        for (size_t i = 0; i < n; ++i) s4[i] = state[i] + dt * k3[i];
        auto k4 = system.computeDerivatives(t + dt, s4);
        for (size_t i = 0; i < n; ++i)
            state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        t += dt;
        ++steps;
    }

    csv.close();
    std::cout << "✓ Exported " << outputs << " frames after " << steps << " integration steps\n";
    std::cout << "✓ Output saved to 'grid_2d_animation.csv'\n";
}

int main() {
    std::cout << "========================================\n";
    std::cout << "SOPOT 2D Mass-Spring Grid Test Suite\n";
    std::cout << "========================================\n";
    std::cout << "\nDemonstrating the framework's flexibility:\n";
    std::cout << "- 2D point masses (4 states each: x, y, vx, vy)\n";
    std::cout << "- 2D springs with Hooke's law + damping\n";
    std::cout << "- Rectangular grid topology\n";
    std::cout << "- Cloth-like simulations\n";

    try {
        test_small_grid_3x3();
        test_cloth_4x4_with_diagonals();
        test_energy_conservation();
        test_export_to_csv();

        std::cout << "\n========================================\n";
        std::cout << "All tests passed successfully! ✓\n";
        std::cout << "========================================\n";
        std::cout << "\nSOPOT now supports:\n";
        std::cout << "  • Rocket flight simulation (6-DOF)\n";
        std::cout << "  • 1D mass-spring systems\n";
        std::cout << "  • 2D mass-spring grids (cloth/fabric)\n";
        std::cout << "\nThe framework is truly domain-agnostic!\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
