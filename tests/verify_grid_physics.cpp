#include "physics/connected_masses/connectivity_matrix_2d.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot;
using namespace sopot::connected_masses;

/**
 * @brief Verify that spring forces are computed correctly
 *
 * This test creates a simple 2x2 grid and verifies the physics
 * against analytical calculations.
 */
void test_spring_force_calculation() {
    std::cout << "\n=== Physics Verification: Spring Forces ===\n\n";

    // Create a 2x2 grid
    // Grid layout:
    // 0 -- 1
    // |    |
    // 2 -- 3

    constexpr size_t Rows = 2;
    constexpr size_t Cols = 2;

    auto system = makeGrid2DSystem<double, Rows, Cols, false>(
        1.0,    // mass (kg)
        1.0,    // spacing (m)
        20.0,   // stiffness (N/m)
        0.0     // NO damping
    );

    std::cout << "Created 2x2 grid:\n";
    std::cout << "  Mass: 1.0 kg\n";
    std::cout << "  Spacing: 1.0 m\n";
    std::cout << "  Stiffness: 20.0 N/m\n";
    std::cout << "  Damping: 0.0 N·s/m\n\n";

    // Test 1: Equilibrium check
    std::cout << "Test 1: Forces at equilibrium\n";

    auto state = system.getInitialState();
    auto derivs = system.computeDerivatives(0.0, state);

    std::cout << "Initial positions:\n";
    auto pos0 = system.computeStateFunction<MassTag2D<0>::Position>(state);
    auto pos1 = system.computeStateFunction<MassTag2D<1>::Position>(state);
    auto pos2 = system.computeStateFunction<MassTag2D<2>::Position>(state);
    auto pos3 = system.computeStateFunction<MassTag2D<3>::Position>(state);

    std::cout << "  Mass 0: (" << pos0[0] << ", " << pos0[1] << ")\n";
    std::cout << "  Mass 1: (" << pos1[0] << ", " << pos1[1] << ")\n";
    std::cout << "  Mass 2: (" << pos2[0] << ", " << pos2[1] << ")\n";
    std::cout << "  Mass 3: (" << pos3[0] << ", " << pos3[1] << ")\n\n";

    std::cout << "Forces at equilibrium:\n";
    auto force0 = system.computeStateFunction<MassTag2D<0>::Force>(state);
    auto force1 = system.computeStateFunction<MassTag2D<1>::Force>(state);
    auto force2 = system.computeStateFunction<MassTag2D<2>::Force>(state);
    auto force3 = system.computeStateFunction<MassTag2D<3>::Force>(state);

    std::cout << "  Mass 0: (" << force0[0] << ", " << force0[1] << ") N\n";
    std::cout << "  Mass 1: (" << force1[0] << ", " << force1[1] << ") N\n";
    std::cout << "  Mass 2: (" << force2[0] << ", " << force2[1] << ") N\n";
    std::cout << "  Mass 3: (" << force3[0] << ", " << force3[1] << ") N\n";

    // Check if forces are near zero
    double max_force = std::max({
        std::sqrt(force0[0]*force0[0] + force0[1]*force0[1]),
        std::sqrt(force1[0]*force1[0] + force1[1]*force1[1]),
        std::sqrt(force2[0]*force2[0] + force2[1]*force2[1]),
        std::sqrt(force3[0]*force3[0] + force3[1]*force3[1])
    });

    if (max_force < 1e-10) {
        std::cout << "✓ Equilibrium verified (max force: " << max_force << " N)\n\n";
    } else {
        std::cout << "✗ ERROR: Forces not zero at equilibrium (max: " << max_force << " N)\n\n";
    }

    // Test 2: Perturb corner mass
    std::cout << "Test 2: Perturb corner mass 0 by (0.2, 0.3)\n";

    state[0] += 0.2;  // x position of mass 0
    state[1] += 0.3;  // y position of mass 0

    pos0 = system.computeStateFunction<MassTag2D<0>::Position>(state);
    std::cout << "  New position of mass 0: (" << pos0[0] << ", " << pos0[1] << ")\n\n";

    // Compute forces
    force0 = system.computeStateFunction<MassTag2D<0>::Force>(state);
    force1 = system.computeStateFunction<MassTag2D<1>::Force>(state);
    force2 = system.computeStateFunction<MassTag2D<2>::Force>(state);
    force3 = system.computeStateFunction<MassTag2D<3>::Force>(state);

    std::cout << "Forces after perturbation:\n";
    std::cout << "  Mass 0: (" << std::setw(8) << std::fixed << std::setprecision(4)
              << force0[0] << ", " << std::setw(8) << force0[1] << ") N\n";
    std::cout << "  Mass 1: (" << std::setw(8) << force1[0] << ", "
              << std::setw(8) << force1[1] << ") N\n";
    std::cout << "  Mass 2: (" << std::setw(8) << force2[0] << ", "
              << std::setw(8) << force2[1] << ") N\n";
    std::cout << "  Mass 3: (" << std::setw(8) << force3[0] << ", "
              << std::setw(8) << force3[1] << ") N\n\n";

    // Check Newton's 3rd law (sum of forces should be zero)
    double total_fx = force0[0] + force1[0] + force2[0] + force3[0];
    double total_fy = force0[1] + force1[1] + force2[1] + force3[1];
    double total_force = std::sqrt(total_fx * total_fx + total_fy * total_fy);

    std::cout << "Newton's 3rd law check:\n";
    std::cout << "  Total system force: (" << total_fx << ", " << total_fy << ") N\n";
    std::cout << "  Magnitude: " << total_force << " N\n";

    if (total_force < 1e-10) {
        std::cout << "  ✓ Newton's 3rd law verified\n\n";
    } else {
        std::cout << "  ✗ ERROR: Newton's 3rd law violated!\n\n";
    }

    // Manual calculation for verification
    std::cout << "Manual verification:\n";
    std::cout << "  Spring (0,1):\n";
    double dx_01 = 1.0 - 0.2;
    double dy_01 = 0.0 - 0.3;
    double len_01 = std::sqrt(dx_01*dx_01 + dy_01*dy_01);
    double ext_01 = len_01 - 1.0;
    double fx_01 = 20.0 * ext_01 * (dx_01 / len_01);
    double fy_01 = 20.0 * ext_01 * (dy_01 / len_01);
    std::cout << "    Length: " << len_01 << " m (extension: " << ext_01 << " m)\n";
    std::cout << "    Force on mass 0: (" << fx_01 << ", " << fy_01 << ") N\n";

    std::cout << "  Spring (0,2):\n";
    double dx_02 = 0.0 - 0.2;
    double dy_02 = 1.0 - 0.3;
    double len_02 = std::sqrt(dx_02*dx_02 + dy_02*dy_02);
    double ext_02 = len_02 - 1.0;
    double fx_02 = 20.0 * ext_02 * (dx_02 / len_02);
    double fy_02 = 20.0 * ext_02 * (dy_02 / len_02);
    std::cout << "    Length: " << len_02 << " m (extension: " << ext_02 << " m)\n";
    std::cout << "    Force on mass 0: (" << fx_02 << ", " << fy_02 << ") N\n";

    std::cout << "  Total force on mass 0: (" << (fx_01 + fx_02) << ", "
              << (fy_01 + fy_02) << ") N\n";
    std::cout << "  Computed force on mass 0: (" << force0[0] << ", " << force0[1] << ") N\n";

    double error_x = std::abs((fx_01 + fx_02) - force0[0]);
    double error_y = std::abs((fy_01 + fy_02) - force0[1]);

    if (error_x < 1e-10 && error_y < 1e-10) {
        std::cout << "  ✓ Force calculation verified (error < 1e-10)\n\n";
    } else {
        std::cout << "  ✗ ERROR: Force mismatch! (error: " << error_x << ", " << error_y << ")\n\n";
    }
}

/**
 * @brief Verify energy conservation with no damping
 */
void test_energy_conservation() {
    std::cout << "\n=== Physics Verification: Energy Conservation ===\n\n";

    constexpr size_t Rows = 2;
    constexpr size_t Cols = 2;

    auto system = makeGrid2DSystem<double, Rows, Cols, false>(
        1.0,    // mass (kg)
        1.0,    // spacing (m)
        20.0,   // stiffness (N/m)
        0.0     // NO damping for energy conservation
    );

    std::cout << "Testing energy conservation (no damping)\n\n";

    auto state = system.getInitialState();

    // Perturb mass 0
    state[0] += 0.2;  // x
    state[1] += 0.3;  // y

    // Compute initial energy
    double KE_initial = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        double vx = state[i * 4 + 2];
        double vy = state[i * 4 + 3];
        KE_initial += 0.5 * 1.0 * (vx * vx + vy * vy);
    }

    // Compute potential energy (manually for the 4 springs)
    double PE_initial = 0.0;

    // Spring 0-1
    double dx = state[1*4+0] - state[0*4+0];
    double dy = state[1*4+1] - state[0*4+1];
    double ext = std::sqrt(dx*dx + dy*dy) - 1.0;
    PE_initial += 0.5 * 20.0 * ext * ext;

    // Spring 0-2
    dx = state[2*4+0] - state[0*4+0];
    dy = state[2*4+1] - state[0*4+1];
    ext = std::sqrt(dx*dx + dy*dy) - 1.0;
    PE_initial += 0.5 * 20.0 * ext * ext;

    // Spring 1-3
    dx = state[3*4+0] - state[1*4+0];
    dy = state[3*4+1] - state[1*4+1];
    ext = std::sqrt(dx*dx + dy*dy) - 1.0;
    PE_initial += 0.5 * 20.0 * ext * ext;

    // Spring 2-3
    dx = state[3*4+0] - state[2*4+0];
    dy = state[3*4+1] - state[2*4+1];
    ext = std::sqrt(dx*dx + dy*dy) - 1.0;
    PE_initial += 0.5 * 20.0 * ext * ext;

    double E_initial = KE_initial + PE_initial;

    std::cout << "Initial energy:\n";
    std::cout << "  KE = " << KE_initial << " J\n";
    std::cout << "  PE = " << PE_initial << " J\n";
    std::cout << "  Total = " << E_initial << " J\n\n";

    // Simulate for a short time
    double t = 0.0;
    double dt = 0.001;
    size_t n = system.getStateDimension();

    std::cout << "Simulating without damping...\n";
    std::cout << std::setw(10) << "Time (s)" << std::setw(15) << "KE (J)"
              << std::setw(15) << "PE (J)" << std::setw(15) << "Total (J)"
              << std::setw(15) << "Error (%)\n";
    std::cout << std::string(70, '-') << "\n";

    for (int step = 0; step <= 1000; step += 100) {
        // Compute energy
        double KE = 0.0;
        for (size_t i = 0; i < 4; ++i) {
            double vx = state[i * 4 + 2];
            double vy = state[i * 4 + 3];
            KE += 0.5 * 1.0 * (vx * vx + vy * vy);
        }

        double PE = 0.0;

        // Spring 0-1
        dx = state[1*4+0] - state[0*4+0];
        dy = state[1*4+1] - state[0*4+1];
        ext = std::sqrt(dx*dx + dy*dy) - 1.0;
        PE += 0.5 * 20.0 * ext * ext;

        // Spring 0-2
        dx = state[2*4+0] - state[0*4+0];
        dy = state[2*4+1] - state[0*4+1];
        ext = std::sqrt(dx*dx + dy*dy) - 1.0;
        PE += 0.5 * 20.0 * ext * ext;

        // Spring 1-3
        dx = state[3*4+0] - state[1*4+0];
        dy = state[3*4+1] - state[1*4+1];
        ext = std::sqrt(dx*dx + dy*dy) - 1.0;
        PE += 0.5 * 20.0 * ext * ext;

        // Spring 2-3
        dx = state[3*4+0] - state[2*4+0];
        dy = state[3*4+1] - state[2*4+1];
        ext = std::sqrt(dx*dx + dy*dy) - 1.0;
        PE += 0.5 * 20.0 * ext * ext;

        double E_total = KE + PE;
        double error = 100.0 * std::abs(E_total - E_initial) / E_initial;

        std::cout << std::setw(10) << std::fixed << std::setprecision(3) << t
                  << std::setw(15) << std::setprecision(6) << KE
                  << std::setw(15) << PE
                  << std::setw(15) << E_total
                  << std::setw(15) << error << "\n";

        // RK4 integration
        if (step < 1000) {
            for (int substep = 0; substep < 100; ++substep) {
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
            }
        }
    }

    std::cout << "\n✓ Energy conservation test completed\n";
    std::cout << "  Note: Small energy drift (<1%) is expected due to numerical integration\n\n";
}

int main() {
    std::cout << "============================================================\n";
    std::cout << "SOPOT 2D Grid Physics Verification Suite\n";
    std::cout << "============================================================\n";

    try {
        test_spring_force_calculation();
        test_energy_conservation();

        std::cout << "============================================================\n";
        std::cout << "Physics verification complete!\n";
        std::cout << "============================================================\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
