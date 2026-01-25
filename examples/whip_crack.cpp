//=============================================================================
// WHIP CRACK SIMULATION - Classic Mass-Spring Chain Problem
//=============================================================================
//
// This example demonstrates the famous "whip crack" phenomenon using the
// SOPOT connected masses framework.
//
// PHYSICS BACKGROUND:
// A whip is modeled as a chain of masses connected by springs. When the
// handle is moved, a wave travels down the whip. Due to conservation of
// energy, as the wave reaches lighter masses near the tip, they accelerate
// dramatically. With appropriate mass distribution, the tip can exceed
// the speed of sound (~343 m/s), creating the characteristic "crack".
//
// MODEL:
//   - N masses in a 1D chain: m0 -- m1 -- m2 -- ... -- m(N-1)
//   - Mass 0 is the handle (very heavy, effectively fixed)
//   - Masses decrease geometrically towards the tip
//   - Stiff springs with light damping connect adjacent masses
//   - Initial velocity impulse applied to mass near handle
//
// KEY INSIGHT:
// For a wave traveling down a tapered whip, the tip velocity scales as:
//   v_tip ~ v_handle * sqrt(m_handle / m_tip)
// With a 1000:1 mass ratio, tip speeds can reach ~30x handle speed.
//
//=============================================================================

#include "physics/connected_masses/connectivity_matrix.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace sopot;
using namespace sopot::connected_masses;

// Speed of sound in air at sea level (m/s)
constexpr double SPEED_OF_SOUND = 343.0;

//=============================================================================
// Helper: RK4 integration step
//=============================================================================
template<typename System>
void rk4_step(System& system, std::vector<double>& state, double t, double dt) {
    size_t n = system.getStateDimension();

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
// Helper: Get velocity of mass at index (compile-time dispatch)
//=============================================================================
template<size_t Index, typename System>
double getVelocity(const System& system, const std::vector<double>& state) {
    return system.template computeStateFunction<typename MassTag<Index>::Velocity>(state);
}

template<size_t Index, typename System>
double getPosition(const System& system, const std::vector<double>& state) {
    return system.template computeStateFunction<typename MassTag<Index>::Position>(state);
}

//=============================================================================
// Whip simulation with compile-time number of masses
//=============================================================================
template<size_t NumMasses>
void runWhipSimulation() {
    static_assert(NumMasses >= 3, "Whip needs at least 3 masses");
    constexpr size_t NumSprings = NumMasses - 1;

    std::cout << "\n";
    std::cout << "================================================================\n";
    std::cout << "  WHIP CRACK SIMULATION - " << NumMasses << " Masses\n";
    std::cout << "================================================================\n\n";

    //=========================================================================
    // Build edge list for linear chain at compile time
    //=========================================================================
    constexpr auto edges = []() {
        std::array<std::pair<size_t, size_t>, NumSprings> e{};
        for (size_t i = 0; i < NumSprings; ++i) {
            e[i] = {i, i + 1};
        }
        return e;
    }();

    //=========================================================================
    // Configure mass parameters
    //=========================================================================
    // Mass decreases geometrically: m_i = m_handle * (mass_ratio)^(i/(N-1))
    // This gives realistic whip taper

    constexpr double handle_mass = 10.0;      // Heavy handle (kg)
    constexpr double tip_mass = 0.01;         // Light tip (kg)
    constexpr double mass_ratio = tip_mass / handle_mass;

    // Compute mass decay factor: mass[i] = handle_mass * decay^i
    const double decay = std::pow(mass_ratio, 1.0 / (NumMasses - 1));

    std::array<MassParams, NumMasses> masses;
    double total_length = 0.0;
    const double segment_length = 0.1;  // 10 cm between masses

    std::cout << "Mass distribution (handle to tip):\n";
    std::cout << "  Index   Mass (kg)    Position (m)\n";
    std::cout << "  -----   ---------    ------------\n";

    for (size_t i = 0; i < NumMasses; ++i) {
        double mass = handle_mass * std::pow(decay, static_cast<double>(i));
        double position = i * segment_length;
        total_length = position;

        masses[i] = {mass, position, 0.0};  // All start at rest

        if (i < 5 || i == NumMasses - 1) {
            std::cout << "  " << std::setw(5) << i
                      << "   " << std::setw(9) << std::fixed << std::setprecision(4) << mass
                      << "    " << std::setw(12) << std::setprecision(3) << position << "\n";
        } else if (i == 5) {
            std::cout << "  ...     ...          ...\n";
        }
    }

    std::cout << "\nWhip length: " << total_length << " m\n";
    std::cout << "Mass ratio (handle:tip): " << (handle_mass/tip_mass) << ":1\n";

    //=========================================================================
    // Configure spring parameters
    //=========================================================================
    // Stiff springs to simulate nearly inextensible whip segments

    constexpr double spring_stiffness = 10000.0;  // Very stiff (N/m)
    constexpr double spring_damping = 1.0;        // Light damping (N*s/m)

    std::array<SpringParams, NumSprings> springs;
    for (size_t i = 0; i < NumSprings; ++i) {
        springs[i] = {spring_stiffness, segment_length, spring_damping};
    }

    std::cout << "\nSpring parameters:\n";
    std::cout << "  Stiffness: " << spring_stiffness << " N/m\n";
    std::cout << "  Rest length: " << segment_length << " m\n";
    std::cout << "  Damping: " << spring_damping << " N*s/m\n";

    //=========================================================================
    // Create the system
    //=========================================================================
    auto system = makeConnectedMassSystem<double, NumMasses, edges>(masses, springs);

    std::cout << "\nSystem state dimension: " << system.getStateDimension()
              << " (" << NumMasses << " positions + " << NumMasses << " velocities)\n";

    //=========================================================================
    // Apply initial impulse
    //=========================================================================
    auto state = system.getInitialState();

    // Apply impulse to mass 1 (just after handle)
    // The handle (mass 0) stays nearly fixed due to its large inertia
    constexpr double impulse_velocity = 10.0;  // m/s - realistic whip crack speed
    state[3] = impulse_velocity;  // v1 (velocity of mass 1)

    std::cout << "\nInitial conditions:\n";
    std::cout << "  All masses at rest except mass 1\n";
    std::cout << "  Mass 1 initial velocity: " << impulse_velocity << " m/s\n";
    std::cout << "  Speed of sound: " << SPEED_OF_SOUND << " m/s\n";

    //=========================================================================
    // Simulation
    //=========================================================================
    std::cout << "\nRunning simulation...\n";
    std::cout << "Tracking maximum tip velocity:\n";
    std::cout << "\n  Time (s)    Tip Velocity (m/s)    Mach Number    Status\n";
    std::cout << "  --------    ------------------    -----------    ------\n";

    double t = 0.0;
    const double dt = 1e-6;      // Very small timestep for stiff system
    const double t_end = 0.1;    // 100 ms simulation
    const double print_interval = 0.01;  // Print every 10 ms

    double max_tip_velocity = 0.0;
    double max_velocity_time = 0.0;
    double next_print = 0.0;

    size_t steps = 0;
    const size_t tip_index = NumMasses - 1;

    while (t < t_end) {
        // Get tip velocity (using state array directly: velocity is at odd indices)
        double tip_velocity = std::abs(state[2 * tip_index + 1]);

        // Track maximum
        if (tip_velocity > max_tip_velocity) {
            max_tip_velocity = tip_velocity;
            max_velocity_time = t;
        }

        // Print progress
        if (t >= next_print) {
            double mach = tip_velocity / SPEED_OF_SOUND;
            std::string status = (tip_velocity > SPEED_OF_SOUND) ? "SUPERSONIC!" : "";

            std::cout << "  " << std::fixed << std::setprecision(4) << t
                      << "    " << std::setw(18) << std::setprecision(2) << tip_velocity
                      << "    " << std::setw(11) << std::setprecision(3) << mach
                      << "    " << status << "\n";

            next_print += print_interval;
        }

        // RK4 step
        rk4_step(system, state, t, dt);
        t += dt;
        ++steps;
    }

    //=========================================================================
    // Results
    //=========================================================================
    std::cout << "\n";
    std::cout << "================================================================\n";
    std::cout << "  RESULTS\n";
    std::cout << "================================================================\n";
    std::cout << "\n  Total simulation steps: " << steps << "\n";
    std::cout << "  Maximum tip velocity: " << std::fixed << std::setprecision(2)
              << max_tip_velocity << " m/s\n";
    std::cout << "  Time of maximum: " << std::setprecision(5)
              << max_velocity_time << " s\n";
    std::cout << "  Mach number achieved: " << std::setprecision(3)
              << (max_tip_velocity / SPEED_OF_SOUND) << "\n";

    if (max_tip_velocity > SPEED_OF_SOUND) {
        std::cout << "\n  *** SONIC BOOM! The whip cracked! ***\n";
    } else {
        std::cout << "\n  The tip did not reach supersonic speed.\n";
        std::cout << "  Try increasing mass ratio or impulse velocity.\n";
    }

    // Velocity amplification
    double amplification = max_tip_velocity / impulse_velocity;
    double theoretical = std::sqrt(handle_mass / tip_mass);

    std::cout << "\nVelocity amplification:\n";
    std::cout << "  Achieved: " << std::setprecision(1) << amplification << "x\n";
    std::cout << "  Theoretical maximum: " << std::setprecision(1) << theoretical << "x\n";
    std::cout << "  (Theoretical = sqrt(m_handle / m_tip) for ideal energy transfer)\n";
}

//=============================================================================
// Main
//=============================================================================
int main() {
    std::cout << "================================================================\n";
    std::cout << "        SOPOT WHIP CRACK SIMULATION\n";
    std::cout << "================================================================\n";
    std::cout << "\n";
    std::cout << "This simulation demonstrates the classic whip crack problem:\n";
    std::cout << "- A chain of masses connected by springs\n";
    std::cout << "- Handle end is heavy (nearly fixed)\n";
    std::cout << "- Masses decrease towards the tip\n";
    std::cout << "- Energy conservation amplifies tip velocity\n";
    std::cout << "\n";
    std::cout << "The 'crack' of a whip is actually a sonic boom created when\n";
    std::cout << "the tip exceeds the speed of sound (~343 m/s).\n";

    try {
        // Run simulation with 20 masses
        // (More masses = smoother approximation of continuous whip)
        runWhipSimulation<20>();

        std::cout << "\n================================================================\n";
        std::cout << "  Simulation complete!\n";
        std::cout << "================================================================\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\nSimulation failed: " << e.what() << "\n";
        return 1;
    }
}
