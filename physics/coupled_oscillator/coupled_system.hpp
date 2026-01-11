#pragma once

//=============================================================================
// Coupled Oscillator System - Two masses connected by a spring
//=============================================================================
//
// This example demonstrates SOPOT's modular architecture:
//
//   ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
//   │   Mass1     │     │   Spring    │     │   Mass2     │
//   │             │     │             │     │             │
//   │ State: x1,v1│◄────│ Force1,Force2│────►│ State: x2,v2│
//   │             │     │             │     │             │
//   │ Provides:   │     │ Provides:   │     │ Provides:   │
//   │  Position1  │────►│  Force1     │◄────│  Position2  │
//   │  Velocity1  │     │  Force2     │     │  Velocity2  │
//   │  Mass1      │     │  Extension  │     │  Mass2      │
//   └─────────────┘     └─────────────┘     └─────────────┘
//                              │
//                              ▼
//                       ┌─────────────┐
//                       │EnergyMonitor│
//                       │             │
//                       │ Provides:   │
//                       │  TotalEnergy│
//                       │  CenterOfMass│
//                       │  Momentum   │
//                       └─────────────┘
//
// Key Design Principles:
//
// 1. SEPARATION OF CONCERNS
//    - Each component has a single responsibility
//    - Mass: position/velocity integration, mass property
//    - Spring: force computation, energy storage
//    - EnergyMonitor: system-level quantities
//
// 2. COMPILE-TIME DISPATCH
//    - All component interactions through typed tags
//    - Zero runtime overhead - no virtual functions
//    - Type errors caught at compile time
//
// 3. TAGGED STATE FUNCTIONS
//    - mass1::Position vs mass2::Position distinguished by type
//    - Spring knows which mass is which through template parameters
//    - Prevents accidental mixing of component states
//
// Usage:
//   auto system = makeTypedODESystem<double>(
//       createMass1<double>(1.0, 0.0, 0.0),   // m=1, x0=0, v0=0
//       createMass2<double>(1.0, 2.0, 0.0),   // m=1, x0=2, v0=0
//       createSpring<double>(4.0, 1.0),       // k=4, L0=1
//       createEnergyMonitor<double>()
//   );
//
//   auto state = system.getInitialState();
//   auto energy = system.computeStateFunction<system::TotalEnergy>(state);
//
//=============================================================================

#include "tags.hpp"
#include "point_mass.hpp"
#include "spring.hpp"
#include "energy_monitor.hpp"
#include "../../core/typed_component.hpp"

namespace sopot::physics::coupled {

//=============================================================================
// Convenience Factory for Complete System
//=============================================================================

template<Scalar T = double>
auto createCoupledOscillator(
    double m1, double x1_0, double v1_0,
    double m2, double x2_0, double v2_0,
    double k, double L0 = 1.0, double damping = 0.0
) {
    return makeTypedODESystem<T>(
        Mass1<T>(m1, x1_0, v1_0, "mass1"),
        Mass2<T>(m2, x2_0, v2_0, "mass2"),
        Spring12<T>(k, L0, damping, "spring"),
        EnergyMonitor<T>("energy_monitor")
    );
}

//=============================================================================
// Analytical Solution for Equal Masses (for validation)
//=============================================================================
// For m1 = m2 = m, undamped:
//   Normal modes:
//     1. Center-of-mass mode: x_cm = const (if initial momentum = 0)
//     2. Relative mode: r = x1 - x2 oscillates at omega = sqrt(2k/m)
//
//   General solution (for zero initial velocities, symmetric initial positions):
//     x1(t) = x_cm + 0.5 * r0 * cos(omega * t)
//     x2(t) = x_cm - 0.5 * r0 * cos(omega * t)
//   where r0 = x1_0 - x2_0 - L0 (initial extension)
//
struct AnalyticalSolution {
    double m;           // Mass of each particle (must be equal)
    double k;           // Spring constant
    double L0;          // Rest length
    double x1_0, x2_0;  // Initial positions
    double omega;       // Angular frequency sqrt(2k/m)
    double x_cm;        // Center of mass (constant)
    double r0;          // Initial extension

    AnalyticalSolution(double mass, double stiffness, double rest_length,
                       double initial_x1, double initial_x2)
        : m(mass), k(stiffness), L0(rest_length)
        , x1_0(initial_x1), x2_0(initial_x2)
        , omega(std::sqrt(2.0 * k / m))
        , x_cm(0.5 * (x1_0 + x2_0))
        , r0(x1_0 - x2_0 - L0) {}

    std::pair<double, double> positions(double t) const {
        double half_r = 0.5 * r0 * std::cos(omega * t);
        return {x_cm + half_r + 0.5 * L0, x_cm - half_r - 0.5 * L0};
    }

    std::pair<double, double> velocities(double t) const {
        double half_v = -0.5 * r0 * omega * std::sin(omega * t);
        return {half_v, -half_v};
    }

    double period() const { return 2.0 * 3.14159265358979323846 / omega; }
};

} // namespace sopot::physics::coupled
