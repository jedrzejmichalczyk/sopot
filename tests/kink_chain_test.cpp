#include "../physics/kink_chain/kink_system.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <numbers>

using namespace sopot;
using namespace sopot::physics::kink;

// RK4 integration step
template<typename System>
std::vector<double> rk4Step(const System& system, double t, const std::vector<double>& state, double dt) {
    auto k1 = system.computeDerivatives(t, state);

    std::vector<double> s2(state.size()), s3(state.size()), s4(state.size());
    for (size_t i = 0; i < state.size(); ++i) {
        s2[i] = state[i] + 0.5 * dt * k1[i];
    }

    auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
    for (size_t i = 0; i < state.size(); ++i) {
        s3[i] = state[i] + 0.5 * dt * k2[i];
    }

    auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
    for (size_t i = 0; i < state.size(); ++i) {
        s4[i] = state[i] + dt * k3[i];
    }

    auto k4 = system.computeDerivatives(t + dt, s4);

    std::vector<double> result(state.size());
    for (size_t i = 0; i < state.size(); ++i) {
        result[i] = state[i] + dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }

    return result;
}

//=============================================================================
// Test: 3-Segment Kink Chain
//=============================================================================
// Create a chain of 3 rigid segments connected by torsional springs.
// We'll initialize with a kink (one segment rotated) and watch it propagate.
//=============================================================================

void testKinkPropagation() {
    std::cout << "=============================================================================\n";
    std::cout << "Kink Chain Simulation - 3 Segments\n";
    std::cout << "=============================================================================\n\n";

    // Physical parameters
    constexpr double I = 0.01;       // Moment of inertia [kg·m²]
    constexpr double k = 10.0;       // Torsional stiffness [N·m/rad]
    constexpr double c = 0.1;        // Damping [N·m·s/rad]

    // Initial configuration: create a kink by rotating the middle segment
    constexpr double theta0 = 0.0;
    constexpr double theta1 = std::numbers::pi;  // Middle segment rotated 180°
    constexpr double theta2 = 0.0;

    std::cout << "Physical Parameters:\n";
    std::cout << "  Moment of inertia: I = " << I << " kg·m²\n";
    std::cout << "  Torsional stiffness: k = " << k << " N·m/rad\n";
    std::cout << "  Damping coefficient: c = " << c << " N·m·s/rad\n\n";

    std::cout << "Initial Configuration (Kink):\n";
    std::cout << "  Segment 0: θ₀ = " << theta0 << " rad (" << (theta0 * 180.0 / std::numbers::pi) << "°)\n";
    std::cout << "  Segment 1: θ₁ = " << theta1 << " rad (" << (theta1 * 180.0 / std::numbers::pi) << "°)\n";
    std::cout << "  Segment 2: θ₂ = " << theta2 << " rad (" << (theta2 * 180.0 / std::numbers::pi) << "°)\n\n";

    // Build the system
    auto system = makeTypedODESystem<double>(
        createSegment<0, double>(I, theta0, 0.0),
        createSegment<1, double>(I, theta1, 0.0),
        createSegment<2, double>(I, theta2, 0.0),
        createTorsionalSpring<0, 1, double>(k, c),
        createTorsionalSpring<1, 2, double>(k, c)
    );

    std::cout << "System Information:\n";
    std::cout << "  State dimension: " << system.getStateDimension() << " (3 segments × 2 states)\n";
    std::cout << "  Component count: " << system.getComponentCount() << " (3 segments + 2 springs)\n\n";

    // Verify compile-time function availability
    static_assert(decltype(system)::hasFunction<segment<0>::Angle>());
    static_assert(decltype(system)::hasFunction<segment<1>::Angle>());
    static_assert(decltype(system)::hasFunction<segment<2>::Angle>());
    static_assert(decltype(system)::hasFunction<segment<0>::AngularVelocity>());
    static_assert(decltype(system)::hasFunction<segment<0>::Torque>());

    // Get initial state
    auto state = system.getInitialState();

    // Simulation parameters
    constexpr double dt = 0.001;     // Time step [s]
    constexpr double t_max = 5.0;    // Total time [s]
    constexpr size_t steps = static_cast<size_t>(t_max / dt);
    constexpr size_t output_interval = 100;  // Output every 0.1s

    std::cout << "Simulation Parameters:\n";
    std::cout << "  Time step: dt = " << dt << " s\n";
    std::cout << "  Total time: " << t_max << " s\n";
    std::cout << "  Total steps: " << steps << "\n\n";

    std::cout << "Starting simulation...\n\n";

    // Open output file for detailed data
    std::ofstream outfile("kink_chain_output.csv");
    outfile << "time,theta0,theta1,theta2,omega0,omega1,omega2,angle_diff_01,angle_diff_12\n";

    // Integrate
    double t = 0.0;
    for (size_t step = 0; step <= steps; ++step) {
        // Query current state
        double theta_0 = system.computeStateFunction<segment<0>::Angle>(state);
        double theta_1 = system.computeStateFunction<segment<1>::Angle>(state);
        double theta_2 = system.computeStateFunction<segment<2>::Angle>(state);

        double omega_0 = system.computeStateFunction<segment<0>::AngularVelocity>(state);
        double omega_1 = system.computeStateFunction<segment<1>::AngularVelocity>(state);
        double omega_2 = system.computeStateFunction<segment<2>::AngularVelocity>(state);

        // Compute angle differences (wrapped)
        auto wrapAngle = [](double angle) {
            while (angle > std::numbers::pi) angle -= 2.0 * std::numbers::pi;
            while (angle < -std::numbers::pi) angle += 2.0 * std::numbers::pi;
            return angle;
        };

        double diff_01 = wrapAngle(theta_0 - theta_1);
        double diff_12 = wrapAngle(theta_1 - theta_2);

        // Output to file
        outfile << std::fixed << std::setprecision(6)
                << t << ","
                << theta_0 << "," << theta_1 << "," << theta_2 << ","
                << omega_0 << "," << omega_1 << "," << omega_2 << ","
                << diff_01 << "," << diff_12 << "\n";

        // Console output at intervals
        if (step % output_interval == 0) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "t = " << std::setw(5) << t << " s | ";
            std::cout << "θ₀ = " << std::setw(7) << (theta_0 * 180.0 / std::numbers::pi) << "° | ";
            std::cout << "θ₁ = " << std::setw(7) << (theta_1 * 180.0 / std::numbers::pi) << "° | ";
            std::cout << "θ₂ = " << std::setw(7) << (theta_2 * 180.0 / std::numbers::pi) << "° | ";
            std::cout << "Δθ₀₁ = " << std::setw(7) << (diff_01 * 180.0 / std::numbers::pi) << "° | ";
            std::cout << "Δθ₁₂ = " << std::setw(7) << (diff_12 * 180.0 / std::numbers::pi) << "°\n";
        }

        // Take integration step (RK4)
        if (step < steps) {
            state = rk4Step(system, t, state, dt);
            t += dt;
        }
    }

    outfile.close();

    std::cout << "\n=============================================================================\n";
    std::cout << "Simulation complete!\n";
    std::cout << "Output written to: kink_chain_output.csv\n";
    std::cout << "=============================================================================\n\n";

    std::cout << "Physics Explanation:\n";
    std::cout << "  • The middle segment started at 180° (π rad), creating a 'kink'\n";
    std::cout << "  • Torsional springs exert restoring torques proportional to angle differences\n";
    std::cout << "  • The kink propagates as angular momentum transfers between segments\n";
    std::cout << "  • Damping gradually dissipates energy, smoothing out the kink\n";
    std::cout << "  • This is analogous to topological solitons in field theory\n\n";
}

//=============================================================================
// Test: 5-Segment Chain with Multiple Kinks
//=============================================================================

void testMultipleKinks() {
    std::cout << "=============================================================================\n";
    std::cout << "Multiple Kinks Simulation - 5 Segments\n";
    std::cout << "=============================================================================\n\n";

    constexpr double I = 0.01;
    constexpr double k = 10.0;
    constexpr double c = 0.05;

    // Create alternating kinks: 0, π, 0, π, 0
    std::cout << "Initial Configuration (Alternating Pattern):\n";
    for (int i = 0; i < 5; ++i) {
        double angle = (i % 2 == 0) ? 0.0 : std::numbers::pi;
        std::cout << "  Segment " << i << ": θ = " << angle << " rad ("
                  << (angle * 180.0 / std::numbers::pi) << "°)\n";
    }
    std::cout << "\n";

    auto system = makeTypedODESystem<double>(
        createSegment<0, double>(I, 0.0, 0.0),
        createSegment<1, double>(I, std::numbers::pi, 0.0),
        createSegment<2, double>(I, 0.0, 0.0),
        createSegment<3, double>(I, std::numbers::pi, 0.0),
        createSegment<4, double>(I, 0.0, 0.0),
        createTorsionalSpring<0, 1, double>(k, c),
        createTorsionalSpring<1, 2, double>(k, c),
        createTorsionalSpring<2, 3, double>(k, c),
        createTorsionalSpring<3, 4, double>(k, c)
    );

    std::cout << "System: " << system.getStateDimension() << " states, "
              << system.getComponentCount() << " components\n\n";

    auto state = system.getInitialState();

    constexpr double dt = 0.001;
    constexpr double t_max = 10.0;
    constexpr size_t steps = static_cast<size_t>(t_max / dt);
    constexpr size_t output_interval = 200;

    std::ofstream outfile("kink_multiple_output.csv");
    outfile << "time,theta0,theta1,theta2,theta3,theta4\n";

    std::cout << "Simulating for " << t_max << " seconds...\n\n";

    double t = 0.0;
    for (size_t step = 0; step <= steps; ++step) {
        double theta_0 = system.computeStateFunction<segment<0>::Angle>(state);
        double theta_1 = system.computeStateFunction<segment<1>::Angle>(state);
        double theta_2 = system.computeStateFunction<segment<2>::Angle>(state);
        double theta_3 = system.computeStateFunction<segment<3>::Angle>(state);
        double theta_4 = system.computeStateFunction<segment<4>::Angle>(state);

        outfile << std::fixed << std::setprecision(6)
                << t << ","
                << theta_0 << "," << theta_1 << "," << theta_2 << ","
                << theta_3 << "," << theta_4 << "\n";

        if (step % output_interval == 0) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "t = " << std::setw(5) << t << " s | ";
            for (int i = 0; i < 5; ++i) {
                double theta = (i == 0) ? theta_0 : (i == 1) ? theta_1 :
                              (i == 2) ? theta_2 : (i == 3) ? theta_3 : theta_4;
                std::cout << "θ" << i << " = " << std::setw(7)
                         << (theta * 180.0 / std::numbers::pi) << "° ";
                if (i < 4) std::cout << "| ";
            }
            std::cout << "\n";
        }

        if (step < steps) {
            state = rk4Step(system, t, state, dt);
            t += dt;
        }
    }

    outfile.close();

    std::cout << "\n=============================================================================\n";
    std::cout << "Simulation complete! Output: kink_multiple_output.csv\n";
    std::cout << "=============================================================================\n\n";

    std::cout << "Observations:\n";
    std::cout << "  • Adjacent kinks interact through torsional coupling\n";
    std::cout << "  • Kinks can annihilate when they meet (opposite winding)\n";
    std::cout << "  • The system eventually relaxes to a uniform configuration\n";
    std::cout << "  • Energy is conserved (minus damping losses)\n\n";
}

//=============================================================================
// Main
//=============================================================================

int main() {
    try {
        std::cout << "\n";
        std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║          SOPOT Kink Chain Simulation Test Suite                 ║\n";
        std::cout << "║  Topological Defects in 1D Rotational Chains                     ║\n";
        std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";

        // Run tests
        testKinkPropagation();
        std::cout << "\n";
        testMultipleKinks();

        std::cout << "\n";
        std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                    ALL TESTS COMPLETED                           ║\n";
        std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
