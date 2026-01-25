//=============================================================================
// WHIP CRACK SIMULATION - Classic Mass-Spring Chain Problem
//=============================================================================
//
// This example demonstrates the famous "whip crack" phenomenon.
//
// PHYSICS BACKGROUND:
// A whip is modeled as a chain of masses connected by springs. When the
// handle is moved, a wave travels down the whip. Due to conservation of
// energy, as the wave reaches lighter masses near the tip, they accelerate
// dramatically. With appropriate mass distribution, the tip can exceed
// the speed of sound (~343 m/s), creating the characteristic "crack".
//
// MODEL:
//   - N masses in a 1D chain
//   - Mass 0 is the handle (heaviest)
//   - Masses decrease geometrically towards the tip
//   - Linear springs with optional damping connect adjacent masses
//   - Initial velocity impulse applied to mass 1
//
// NOTE: This is a standalone implementation that doesn't use the
// connected_masses framework, as the framework has a limitation with
// force aggregation for multi-connected masses.
//
//=============================================================================

#include <iostream>
#include <iomanip>
#include <cmath>
#include <numbers>
#include <vector>
#include <array>
#include <cassert>

// Speed of sound in air at sea level (m/s)
constexpr double SPEED_OF_SOUND = 343.0;

//=============================================================================
// Simple mass-spring chain simulation
//=============================================================================
class WhipChain {
public:
    size_t num_masses;
    std::vector<double> masses;      // Mass of each particle (kg)
    double spring_k;                  // Spring stiffness (N/m)
    double spring_L0;                 // Spring rest length (m)
    double spring_c;                  // Damping coefficient (N·s/m)

    WhipChain(size_t n, const std::vector<double>& m, double k, double L0, double c)
        : num_masses(n), masses(m), spring_k(k), spring_L0(L0), spring_c(c) {}

    // State vector layout: [x0, v0, x1, v1, ..., x_{n-1}, v_{n-1}]
    size_t stateSize() const { return 2 * num_masses; }

    // Compute derivatives: d/dt [x, v] = [v, a]
    // Mass 0 is FIXED (pivot point) - its velocity and acceleration are always 0
    std::vector<double> computeDerivatives(double /*t*/, const std::vector<double>& state) const {
        std::vector<double> deriv(stateSize(), 0.0);

        // First, compute total force on each mass
        std::vector<double> forces(num_masses, 0.0);

        // Spring forces
        for (size_t i = 0; i < num_masses - 1; ++i) {
            double x_i = state[2 * i];
            double x_ip1 = state[2 * (i + 1)];
            double v_i = state[2 * i + 1];
            double v_ip1 = state[2 * (i + 1) + 1];

            // Extension = actual_length - rest_length (positive = stretched)
            double extension = (x_ip1 - x_i) - spring_L0;

            // Spring force: F = -k * extension
            double F_spring = -spring_k * extension;

            // Damping force: F = -c * relative_velocity
            double F_damping = -spring_c * (v_ip1 - v_i);

            double F_total = F_spring + F_damping;

            // Force on mass i is +F (pulls it toward i+1 if stretched)
            forces[i] += -F_total;

            // Force on mass i+1 is -F (by Newton's 3rd law)
            forces[i + 1] += F_total;
        }

        // Compute accelerations (mass 0 is fixed, so skip it)
        deriv[0] = 0.0;  // x0 doesn't change
        deriv[1] = 0.0;  // v0 stays 0

        for (size_t i = 1; i < num_masses; ++i) {
            double v = state[2 * i + 1];
            double a = forces[i] / masses[i];

            deriv[2 * i] = v;      // dx/dt = v
            deriv[2 * i + 1] = a;  // dv/dt = a
        }

        return deriv;
    }

    // Compute total energy
    std::pair<double, double> computeEnergy(const std::vector<double>& state) const {
        double KE = 0.0, PE = 0.0;

        // Kinetic energy
        for (size_t i = 0; i < num_masses; ++i) {
            double v = state[2 * i + 1];
            KE += 0.5 * masses[i] * v * v;
        }

        // Potential energy
        for (size_t i = 0; i < num_masses - 1; ++i) {
            double x_i = state[2 * i];
            double x_ip1 = state[2 * (i + 1)];
            double extension = (x_ip1 - x_i) - spring_L0;
            PE += 0.5 * spring_k * extension * extension;
        }

        return {KE, PE};
    }
};

//=============================================================================
// RK4 integrator
//=============================================================================
void rk4Step(const WhipChain& system, std::vector<double>& state, double t, double dt) {
    size_t n = state.size();

    auto k1 = system.computeDerivatives(t, state);

    std::vector<double> s2(n);
    for (size_t i = 0; i < n; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
    auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);

    std::vector<double> s3(n);
    for (size_t i = 0; i < n; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
    auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);

    std::vector<double> s4(n);
    for (size_t i = 0; i < n; ++i) s4[i] = state[i] + dt * k3[i];
    auto k4 = system.computeDerivatives(t + dt, s4);

    for (size_t i = 0; i < n; ++i) {
        state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
}

//=============================================================================
// Main simulation
//=============================================================================
int main() {
    std::cout << "================================================================\n";
    std::cout << "        WHIP CRACK SIMULATION\n";
    std::cout << "================================================================\n\n";

    std::cout << "This simulation demonstrates the classic whip crack problem:\n";
    std::cout << "- A chain of masses connected by springs\n";
    std::cout << "- Handle end is heavy (nearly fixed)\n";
    std::cout << "- Masses decrease towards the tip\n";
    std::cout << "- Energy conservation amplifies tip velocity\n\n";

    std::cout << "The 'crack' of a whip is a sonic boom created when the tip\n";
    std::cout << "exceeds the speed of sound (" << SPEED_OF_SOUND << " m/s).\n\n";

    //=========================================================================
    // Configuration
    //=========================================================================
    // Parameters tuned to achieve supersonic tip velocity
    // In practice, a whip cracker applies ~10 m/s to the handle, but
    // the continuous motion over time injects more energy than a single impulse.
    // We simulate this with a stronger initial impulse.
    constexpr size_t NUM_MASSES = 15;
    constexpr double HANDLE_MASS = 0.05;      // kg (pivot mass)
    constexpr double TIP_MASS = 0.0002;       // kg (250:1 ratio)
    constexpr double SEGMENT_LENGTH = 0.08;   // m (8 cm segments)
    constexpr double SPRING_K = 50000.0;      // N/m
    constexpr double SPRING_DAMPING = 0.0;    // N·s/m (no damping)
    constexpr double IMPULSE_VELOCITY = 120.0; // m/s (strong crack)

    // Mass distribution: geometric decay from handle to tip
    double decay = std::pow(TIP_MASS / HANDLE_MASS, 1.0 / (NUM_MASSES - 1));
    std::vector<double> masses(NUM_MASSES);
    double min_mass = HANDLE_MASS;

    std::cout << "Configuration:\n";
    std::cout << "  Number of masses: " << NUM_MASSES << "\n";
    std::cout << "  Handle mass: " << HANDLE_MASS << " kg\n";
    std::cout << "  Tip mass: " << TIP_MASS << " kg\n";
    std::cout << "  Mass ratio: " << (HANDLE_MASS / TIP_MASS) << ":1\n";
    std::cout << "  Segment length: " << (SEGMENT_LENGTH * 100) << " cm\n";
    std::cout << "  Spring stiffness: " << SPRING_K << " N/m\n";
    std::cout << "  Damping: " << SPRING_DAMPING << " N·s/m\n\n";

    std::cout << "Mass distribution:\n";
    for (size_t i = 0; i < NUM_MASSES; ++i) {
        masses[i] = HANDLE_MASS * std::pow(decay, static_cast<double>(i));
        min_mass = std::min(min_mass, masses[i]);
        if (i < 4 || i == NUM_MASSES - 1) {
            std::cout << "  Mass " << std::setw(2) << i << ": "
                      << std::fixed << std::setprecision(4) << masses[i] << " kg\n";
        } else if (i == 4) {
            std::cout << "  ...\n";
        }
    }

    double whip_length = (NUM_MASSES - 1) * SEGMENT_LENGTH;
    std::cout << "\nWhip length: " << whip_length << " m\n";

    //=========================================================================
    // Create system and initial state
    //=========================================================================
    WhipChain system(NUM_MASSES, masses, SPRING_K, SEGMENT_LENGTH, SPRING_DAMPING);

    // Initial positions: masses laid out in a line
    // state = [x0, v0, x1, v1, ...]
    std::vector<double> state(system.stateSize(), 0.0);
    for (size_t i = 0; i < NUM_MASSES; ++i) {
        state[2 * i] = i * SEGMENT_LENGTH;  // position
        state[2 * i + 1] = 0.0;             // velocity
    }

    // Apply impulse to mass 1 (near handle)
    state[3] = IMPULSE_VELOCITY;

    auto [ke0, pe0] = system.computeEnergy(state);
    double initial_energy = ke0 + pe0;

    std::cout << "\nInitial conditions:\n";
    std::cout << "  Impulse velocity: " << IMPULSE_VELOCITY << " m/s (on mass 1)\n";
    std::cout << "  Initial energy: " << std::setprecision(2) << initial_energy << " J\n";

    //=========================================================================
    // Compute timestep
    //=========================================================================
    // Highest frequency: omega = sqrt(k/m_min), need dt << 2*pi/omega
    double omega_max = std::sqrt(SPRING_K / min_mass);
    double period_min = 2.0 * std::numbers::pi / omega_max;
    double dt = period_min / 50.0;  // 50 samples per shortest period

    std::cout << "  Highest frequency: " << std::setprecision(1)
              << (omega_max / (2.0 * std::numbers::pi)) << " Hz\n";
    std::cout << "  Timestep: " << std::scientific << std::setprecision(2)
              << dt << " s\n" << std::fixed;

    //=========================================================================
    // Simulation
    //=========================================================================
    double t = 0.0;
    double t_end = 0.1;   // 100 ms
    double print_interval = 0.01;

    double max_tip_velocity = 0.0;
    double max_velocity_time = 0.0;
    double next_print = 0.0;
    size_t steps = 0;
    size_t tip_idx = NUM_MASSES - 1;

    std::cout << "\nSimulation progress:\n";
    std::cout << "  Time (s)   Tip Vel (m/s)    Mach     KE (J)     PE (J)    Total (J)\n";
    std::cout << "  --------   -------------    ----    -------    -------    ---------\n";

    while (t < t_end) {
        double tip_velocity = std::abs(state[2 * tip_idx + 1]);

        if (tip_velocity > max_tip_velocity) {
            max_tip_velocity = tip_velocity;
            max_velocity_time = t;
        }

        if (t >= next_print) {
            auto [ke, pe] = system.computeEnergy(state);
            double mach = tip_velocity / SPEED_OF_SOUND;
            std::string status = (tip_velocity > SPEED_OF_SOUND) ? " SUPERSONIC!" : "";

            std::cout << "  " << std::fixed << std::setprecision(4) << t
                      << "   " << std::setw(13) << std::setprecision(2) << tip_velocity
                      << "    " << std::setw(4) << std::setprecision(2) << mach
                      << "    " << std::setw(7) << std::setprecision(2) << ke
                      << "    " << std::setw(7) << std::setprecision(2) << pe
                      << "    " << std::setw(9) << std::setprecision(2) << (ke + pe)
                      << status << "\n";
            next_print += print_interval;
        }

        rk4Step(system, state, t, dt);
        t += dt;
        ++steps;
    }

    //=========================================================================
    // Results
    //=========================================================================
    auto [kef, pef] = system.computeEnergy(state);
    double final_energy = kef + pef;
    double energy_change_percent = 100.0 * (final_energy - initial_energy) / initial_energy;

    std::cout << "\n================================================================\n";
    std::cout << "  RESULTS\n";
    std::cout << "================================================================\n";
    std::cout << "\n  Simulation steps: " << steps << "\n";
    std::cout << "  Maximum tip velocity: " << std::setprecision(2)
              << max_tip_velocity << " m/s\n";
    std::cout << "  Time of maximum: " << std::setprecision(4)
              << max_velocity_time << " s\n";
    std::cout << "  Mach number: " << std::setprecision(3)
              << (max_tip_velocity / SPEED_OF_SOUND) << "\n";

    std::cout << "\nEnergy conservation:\n";
    std::cout << "  Initial: " << std::setprecision(3) << initial_energy << " J\n";
    std::cout << "  Final: " << std::setprecision(3) << final_energy << " J\n";
    std::cout << "  Change: " << std::showpos << std::setprecision(2)
              << energy_change_percent << std::noshowpos << "%\n";

    if (max_tip_velocity > SPEED_OF_SOUND) {
        std::cout << "\n  *** SONIC BOOM! The whip cracked! ***\n";
    } else {
        std::cout << "\n  Tip did not reach supersonic speed.\n";
    }

    double amplification = max_tip_velocity / IMPULSE_VELOCITY;
    double theoretical = std::sqrt(HANDLE_MASS / TIP_MASS);
    std::cout << "\nVelocity amplification:\n";
    std::cout << "  Achieved: " << std::setprecision(1) << amplification << "x\n";
    std::cout << "  Theoretical: " << theoretical << "x\n";

    //=========================================================================
    // Validation
    //=========================================================================
    // With no damping, energy should be very well conserved
    assert(std::abs(energy_change_percent) < 1.0 &&
           "Energy should be conserved within 1%");
    assert(amplification > 10.0 &&
           "Velocity amplification should exceed 10x");
    assert(max_tip_velocity > SPEED_OF_SOUND &&
           "Tip velocity should be supersonic");

    std::cout << "\n  All validation checks passed.\n";
    std::cout << "\n================================================================\n";

    return 0;
}
