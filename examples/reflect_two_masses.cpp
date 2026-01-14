//=============================================================================
// REFLECTION-BASED API PROTOTYPE - Two Masses with Spring
//=============================================================================
//
// This example demonstrates the simplicity of the reflection-based API.
//
// COMPARISON:
//
// ┌────────────────────────────────────────────────────────────────────────┐
// │ CURRENT SOPOT API (physics/coupled_oscillator/)                       │
// ├────────────────────────────────────────────────────────────────────────┤
// │                                                                        │
// │  // User must define tag namespaces                                   │
// │  namespace mass1 {                                                     │
// │      struct Position : StateFunction { using ValueType = double; };   │
// │      struct Velocity : StateFunction { using ValueType = double; };   │
// │      struct Force    : StateFunction { using ValueType = double; };   │
// │      struct Mass     : StateFunction { using ValueType = double; };   │
// │  }                                                                     │
// │                                                                        │
// │  // User must create templated component                              │
// │  template<typename TagSet, Scalar T = double>                         │
// │  class PointMass final : public TypedComponent<2, T> {                │
// │      using Base = TypedComponent<2, T>;                               │
// │      using typename Base::LocalState;                                 │
// │      using typename Base::LocalDerivative;                            │
// │      ...                                                              │
// │      template<typename Registry>                                      │
// │      LocalDerivative derivatives(T t, std::span<const T> local,       │
// │          std::span<const T> global, const Registry& registry) const { │
// │          T force = registry.template                                  │
// │              computeFunction<typename TagSet::Force>(global);         │
// │          ...                                                          │
// │      }                                                                 │
// │  };                                                                    │
// │                                                                        │
// │  // User must wire everything together                                │
// │  auto system = makeTypedODESystem<double>(                            │
// │      Mass1<double>(1.0, 0.0, 0.0, "mass1"),                           │
// │      Mass2<double>(1.0, 1.5, 0.0, "mass2"),                           │
// │      Spring12<double>(10.0, 1.0, 0.1, "spring")                       │
// │  );                                                                    │
// │                                                                        │
// │  TOTAL: ~250 lines across 5 files, requires template expertise        │
// │                                                                        │
// └────────────────────────────────────────────────────────────────────────┘
//
// ┌────────────────────────────────────────────────────────────────────────┐
// │ REFLECTION-BASED API (this file)                                       │
// ├────────────────────────────────────────────────────────────────────────┤
// │                                                                        │
// │  struct TwoMassSpring {                                                │
// │      // State variables                                                │
// │      double x1 = 0.0, v1 = 0.0;                                       │
// │      double x2 = 1.0, v2 = 0.0;                                       │
// │                                                                        │
// │      // Parameters                                                     │
// │      double m1 = 1.0, m2 = 1.0;                                       │
// │      double k = 10.0, c = 0.1, L0 = 1.0;                              │
// │                                                                        │
// │      // Physics                                                        │
// │      double extension() const { return x2 - x1 - L0; }                │
// │      double spring_force() const { return -k * extension(); }         │
// │                                                                        │
// │      // Derivatives (functions ending in _dot)                        │
// │      double x1_dot() const { return v1; }                             │
// │      double v1_dot() const { return -spring_force() / m1; }           │
// │      double x2_dot() const { return v2; }                             │
// │      double v2_dot() const { return spring_force() / m2; }            │
// │  };                                                                    │
// │                                                                        │
// │  TOTAL: ~25 lines, requires NO template knowledge                     │
// │                                                                        │
// └────────────────────────────────────────────────────────────────────────┘
//
//=============================================================================

#include "../reflect/two_masses.hpp"
#include <iostream>
#include <iomanip>

int main() {
    using namespace sopot::reflect;

    std::cout << "=== SOPOT Reflection-Based API Prototype ===\n\n";
    std::cout << "System: Two masses connected by a spring\n";
    std::cout << "  Mass 1: m1 = 1.0 kg at x1 = 0.0 m\n";
    std::cout << "  Mass 2: m2 = 1.0 kg at x2 = 1.5 m\n";
    std::cout << "  Spring: k = 10.0 N/m, L0 = 1.0 m, c = 0.1 N·s/m\n";
    std::cout << "  Initial extension: 0.5 m (stretched)\n\n";

    //=========================================================================
    // Simple usage - one function call
    //=========================================================================

    auto result = run_two_mass_spring(
        1.0,   // m1
        1.0,   // m2
        10.0,  // k
        0.1,   // c (damping)
        1.0,   // L0
        0.0,   // x1_init
        0.0,   // v1_init
        1.5,   // x2_init (stretched by 0.5m)
        0.0,   // v2_init
        10.0,  // t_end
        0.001  // dt
    );

    std::cout << "Simulation complete: " << result.size() << " time steps\n\n";

    //=========================================================================
    // Print trajectory (every 1000 steps = every 1 second)
    //=========================================================================

    std::cout << "Trajectory (sampled every 1s):\n";
    std::cout << "----------------------------------------\n";
    result.print(1000);

    //=========================================================================
    // Advanced usage - direct system access
    //=========================================================================

    std::cout << "\n=== Advanced Usage ===\n\n";

    TwoMassSpringSimulation sim;

    // Configure system directly
    sim.sys.m1 = 2.0;   // Heavier mass 1
    sim.sys.m2 = 0.5;   // Lighter mass 2
    sim.sys.k = 50.0;   // Stiffer spring
    sim.sys.c = 0.0;    // No damping (energy conserved)
    sim.sys.L0 = 1.0;

    // Initial conditions: mass 2 displaced
    sim.sys.x1 = 0.0;
    sim.sys.v1 = 0.0;
    sim.sys.x2 = 2.0;  // 1m extension
    sim.sys.v2 = 0.0;

    std::cout << "System 2: Asymmetric masses, no damping\n";
    std::cout << "  m1 = " << sim.sys.m1 << " kg, m2 = " << sim.sys.m2 << " kg\n";
    std::cout << "  k = " << sim.sys.k << " N/m, c = " << sim.sys.c << " N·s/m\n";
    std::cout << "  Initial extension: " << sim.sys.extension() << " m\n";
    std::cout << "  Initial energy: " << sim.sys.total_energy() << " J\n\n";

    auto result2 = simulate(sim, 5.0, 0.0001);

    // Check energy conservation
    std::cout << "Energy conservation check (no damping):\n";
    std::cout << "  t=0.0s: E = " << sim.sys.total_energy() << " J\n";

    // Get final state
    sim.sys.x1 = result2.states.back()[0];
    sim.sys.v1 = result2.states.back()[1];
    sim.sys.x2 = result2.states.back()[2];
    sim.sys.v2 = result2.states.back()[3];
    std::cout << "  t=5.0s: E = " << sim.sys.total_energy() << " J\n";

    //=========================================================================
    // What reflection will enable
    //=========================================================================

    std::cout << "\n=== What C++26 Reflection Enables ===\n\n";
    std::cout << "With P2996 (C++26), the user code becomes:\n\n";
    std::cout << "  struct TwoMassSpring {\n";
    std::cout << "      double x1 = 0.0, v1 = 0.0;\n";
    std::cout << "      double x2 = 1.0, v2 = 0.0;\n";
    std::cout << "      double m1 = 1.0, m2 = 1.0, k = 10.0;\n";
    std::cout << "      \n";
    std::cout << "      double x1_dot() { return v1; }\n";
    std::cout << "      double v1_dot() { return -k*(x1-x2+L0)/m1; }\n";
    std::cout << "      double x2_dot() { return v2; }\n";
    std::cout << "      double v2_dot() { return -k*(x2-x1-L0)/m2; }\n";
    std::cout << "  };\n";
    std::cout << "  \n";
    std::cout << "  auto result = sopot::simulate<TwoMassSpring>(t_end);\n\n";

    std::cout << "The framework will:\n";
    std::cout << "  1. Reflect on struct members → find state variables\n";
    std::cout << "  2. Reflect on methods → find *_dot() derivatives\n";
    std::cout << "  3. Generate ODE system automatically\n";
    std::cout << "  4. Build dependency graph at compile time\n";
    std::cout << "  5. Provide visualization of the graph\n\n";

    std::cout << "=== END ===\n";

    return 0;
}
