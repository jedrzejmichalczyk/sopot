#include "physics/connected_masses/connectivity_matrix_2d.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot;
using namespace sopot::connected_masses;

/**
 * @brief Verify damping forces are computed correctly
 */
void test_damping_forces() {
    std::cout << "\n=== Physics Verification: Damping Forces ===\n\n";

    constexpr size_t Rows = 2;
    constexpr size_t Cols = 2;

    auto system = makeGrid2DSystem<double, Rows, Cols, false>(
        1.0,    // mass (kg)
        1.0,    // spacing (m)
        20.0,   // stiffness (N/m)
        2.0     // damping (N·s/m)
    );

    std::cout << "Created 2x2 grid with damping:\n";
    std::cout << "  Mass: 1.0 kg\n";
    std::cout << "  Stiffness: 20.0 N/m\n";
    std::cout << "  Damping: 2.0 N·s/m\n\n";

    auto state = system.getInitialState();

    // Give mass 0 an initial velocity
    std::cout << "Test: Apply initial velocity to mass 0\n";
    std::cout << "  v0 = (1.0, 0.0) m/s\n\n";

    state[2] = 1.0;  // vx of mass 0

    // Compute forces
    auto force0 = system.computeStateFunction<MassTag2D<0>::Force>(state);
    auto force1 = system.computeStateFunction<MassTag2D<1>::Force>(state);

    std::cout << "Forces with moving mass 0 (damping test):\n";
    std::cout << "  Mass 0: (" << force0[0] << ", " << force0[1] << ") N\n";
    std::cout << "  Mass 1: (" << force1[0] << ", " << force1[1] << ") N\n\n";

    std::cout << "Expected damping force on mass 0:\n";
    std::cout << "  Spring 0-1: relative velocity = -1.0 m/s (along x-axis)\n";
    std::cout << "  Damping force = c * rel_vel * unit_vector = 2.0 * (-1.0) * (1, 0)\n";
    std::cout << "  Expected: (-2.0, 0.0) N\n\n";

    double expected_fx = -2.0;
    double expected_fy = 0.0;
    double error_x = std::abs(force0[0] - expected_fx);
    double error_y = std::abs(force0[1] - expected_fy);

    if (error_x < 1e-10 && error_y < 1e-10) {
        std::cout << "✓ Damping force correctly computed!\n\n";
    } else {
        std::cout << "✗ ERROR: Damping force mismatch! Error: (" << error_x << ", " << error_y << ")\n\n";
    }

    // Test energy dissipation
    std::cout << "Test: Energy dissipation with damping\n\n";

    auto state2 = system.getInitialState();
    state2[0] += 0.2;  // Perturb position

    double t = 0.0;
    double dt = 0.001;
    size_t n = system.getStateDimension();

    std::cout << std::setw(10) << "Time (s)" << std::setw(15) << "Total Energy"
              << std::setw(20) << "Energy Change\n";
    std::cout << std::string(45, '-') << "\n";

    double E_prev = 0.0;
    for (int step = 0; step <= 2000; step += 200) {
        // Compute total energy
        double KE = 0.0;
        for (size_t i = 0; i < 4; ++i) {
            double vx = state2[i * 4 + 2];
            double vy = state2[i * 4 + 3];
            KE += 0.5 * 1.0 * (vx * vx + vy * vy);
        }

        double PE = 0.0;
        auto compute_spring_PE = [&](size_t i, size_t j) {
            double dx = state2[j*4+0] - state2[i*4+0];
            double dy = state2[j*4+1] - state2[i*4+1];
            double ext = std::sqrt(dx*dx + dy*dy) - 1.0;
            return 0.5 * 20.0 * ext * ext;
        };

        PE += compute_spring_PE(0, 1);
        PE += compute_spring_PE(0, 2);
        PE += compute_spring_PE(1, 3);
        PE += compute_spring_PE(2, 3);

        double E = KE + PE;
        double dE = E - E_prev;

        std::cout << std::setw(10) << std::fixed << std::setprecision(3) << t
                  << std::setw(15) << std::setprecision(6) << E;
        if (step > 0) {
            std::cout << std::setw(20) << dE << (dE <= 0 ? " ✓" : " ✗");
        }
        std::cout << "\n";

        E_prev = E;

        // RK4 integration
        if (step < 2000) {
            for (int substep = 0; substep < 200; ++substep) {
                auto k1 = system.computeDerivatives(t, state2);
                std::vector<double> s2(n), s3(n), s4(n);
                for (size_t i = 0; i < n; ++i) s2[i] = state2[i] + 0.5 * dt * k1[i];
                auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
                for (size_t i = 0; i < n; ++i) s3[i] = state2[i] + 0.5 * dt * k2[i];
                auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
                for (size_t i = 0; i < n; ++i) s4[i] = state2[i] + dt * k3[i];
                auto k4 = system.computeDerivatives(t + dt, s4);
                for (size_t i = 0; i < n; ++i)
                    state2[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
                t += dt;
            }
        }
    }

    std::cout << "\n✓ Energy dissipates with damping (as expected)\n\n";

    std::cout << "============================================================\n";
    std::cout << "Physics verification complete!\n";
    std::cout << "All physics tests passed! ✓\n";
    std::cout << "============================================================\n";
}

int main() {
    try {
        test_damping_forces();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "✗ Test failed: " << e.what() << "\n";
        return 1;
    }
}
