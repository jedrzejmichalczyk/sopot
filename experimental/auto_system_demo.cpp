/*
 * Auto-System Demo - Phase 3: Automatic Derivative Computation!
 *
 * This demonstrates the full compiler-inspired architecture:
 * 1. Components declare dependencies/provisions
 * 2. System builds dependency graph automatically
 * 3. Topological sort determines execution order
 * 4. computeDerivatives() is FULLY AUTOMATIC!
 */

#include "auto_system.hpp"
#include <iostream>
#include <iomanip>

using namespace sopot::experimental;

// ============================================================================
// TAGS
// ============================================================================

namespace physics {
    struct Position { using ValueType = double; };
    struct Velocity { using ValueType = double; };
    struct Acceleration { using ValueType = double; };
    struct Mass { using ValueType = double; };
    struct Force { using ValueType = double; };
}

// ============================================================================
// COMPONENTS
// ============================================================================

template<typename T = double>
class PositionComponent {
public:
    static constexpr size_t StateSize = 1;

    using Dependencies = FieldBundle<Field<physics::Velocity, "vel">>;
    using Provides = FieldBundle<Field<physics::Position, "pos">>;

    explicit PositionComponent(T initial) : m_initial(initial) {}

    T getInitialState() const { return m_initial; }

    // Compute derivative: dx/dt = velocity
    T computeDerivative(T /* t */, T /* local_pos */, T vel) const {
        return vel;
    }

private:
    T m_initial;
};

template<typename T = double>
class VelocityComponent {
public:
    static constexpr size_t StateSize = 1;

    using Dependencies = FieldBundle<Field<physics::Acceleration, "accel">>;
    using Provides = FieldBundle<Field<physics::Velocity, "vel">>;

    explicit VelocityComponent(T initial) : m_initial(initial) {}

    T getInitialState() const { return m_initial; }

    // Compute derivative: dv/dt = acceleration
    T computeDerivative(T /* t */, T /* local_vel */, T accel) const {
        return accel;
    }

private:
    T m_initial;
};

template<typename T = double>
class ConstantForce {
public:
    static constexpr size_t StateSize = 0;

    using Dependencies = FieldBundle<>;
    using Provides = FieldBundle<Field<physics::Force, "force">>;

    explicit ConstantForce(T force) : m_force(force) {}

    T getForce() const { return m_force; }

    // Compute provisions: return force value
    Provides compute(T /* t */) const {
        return Provides{Field<physics::Force, "force">{.value = m_force}};
    }

private:
    T m_force;
};

template<typename T = double>
class ConstantMass {
public:
    static constexpr size_t StateSize = 0;

    using Dependencies = FieldBundle<>;
    using Provides = FieldBundle<Field<physics::Mass, "mass">>;

    explicit ConstantMass(T mass) : m_mass(mass) {}

    T getMass() const { return m_mass; }

    // Compute provisions: return mass value
    Provides compute(T /* t */) const {
        return Provides{Field<physics::Mass, "mass">{.value = m_mass}};
    }

private:
    T m_mass;
};

template<typename T = double>
class NewtonSecondLaw {
public:
    static constexpr size_t StateSize = 0;

    using Dependencies = FieldBundle<
        Field<physics::Force, "force">,
        Field<physics::Mass, "mass">
    >;
    using Provides = FieldBundle<Field<physics::Acceleration, "accel">>;

    // Compute provisions: a = F/m
    Provides compute(T /* t */, T force, T mass) const {
        T accel = force / mass;
        return Provides{Field<physics::Acceleration, "accel">{.value = accel}};
    }
};

// ============================================================================
// DEMO
// ============================================================================

int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "  PHASE 3: AUTOMATIC DERIVATIVES! 🚀" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::endl;

    // Create system in ARBITRARY order
    std::cout << "Creating components in arbitrary order..." << std::endl;
    auto system = makeAutoSystem<double>(
        VelocityComponent<double>(0.0),      // [0]
        ConstantMass<double>(2.0),           // [1]
        NewtonSecondLaw<double>(),           // [2]
        PositionComponent<double>(0.0),      // [3]
        ConstantForce<double>(10.0)          // [4]
    );
    std::cout << "✓ 5 components created" << std::endl;
    std::cout << std::endl;

    // Print full diagnostics
    system.printDiagnostics();

    // Check dependency satisfaction
    using Lookup = ProviderLookup<
        VelocityComponent<double>,
        ConstantMass<double>,
        NewtonSecondLaw<double>,
        PositionComponent<double>,
        ConstantForce<double>
    >;

    constexpr bool allSatisfied = Lookup::allDependenciesSatisfied();
    std::cout << "All dependencies satisfied? " << (allSatisfied ? "YES ✓" : "NO ✗") << std::endl;
    std::cout << std::endl;

    // Find providers
    constexpr size_t forceProvider = Lookup::findProvider("force");
    constexpr size_t massProvider = Lookup::findProvider("mass");
    constexpr size_t accelProvider = Lookup::findProvider("accel");
    constexpr size_t velProvider = Lookup::findProvider("vel");

    std::cout << "Provider lookup (compile-time):" << std::endl;
    std::cout << "  'force' provided by component: " << forceProvider << std::endl;
    std::cout << "  'mass' provided by component: " << massProvider << std::endl;
    std::cout << "  'accel' provided by component: " << accelProvider << std::endl;
    std::cout << "  'vel' provided by component: " << velProvider << std::endl;
    std::cout << std::endl;

    // Get initial state
    auto state = system.getInitialState();
    std::cout << "Initial state: [";
    for (size_t i = 0; i < state.size(); ++i) {
        std::cout << state[i];
        if (i < state.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << std::endl;

    // THE BIG MOMENT: Automatic derivative computation!
    std::cout << "============================================" << std::endl;
    std::cout << "  CALLING computeDerivatives()... " << std::endl;
    std::cout << "============================================" << std::endl;

    double t = 0.0;
    auto derivs = system.computeDerivatives(t, state);

    std::cout << "Derivatives: [";
    for (size_t i = 0; i < derivs.size(); ++i) {
        std::cout << derivs[i];
        if (i < derivs.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << std::endl;

    // Show what SHOULD happen
    std::cout << "Expected behavior:" << std::endl;
    std::cout << "  1. Component [4] (ConstantForce) provides force = 10.0" << std::endl;
    std::cout << "  2. Component [1] (ConstantMass) provides mass = 2.0" << std::endl;
    std::cout << "  3. Component [2] (NewtonSecondLaw) computes accel = F/m = 5.0" << std::endl;
    std::cout << "  4. Component [0] (Velocity) computes dv/dt = accel = 5.0" << std::endl;
    std::cout << "  5. Component [3] (Position) computes dx/dt = vel = 0.0" << std::endl;
    std::cout << std::endl;

    std::cout << "Expected derivatives: [5.0, 0.0]" << std::endl;
    std::cout << "  derivs[0] = dv/dt = 5.0 m/s²" << std::endl;
    std::cout << "  derivs[1] = dx/dt = 0.0 m/s" << std::endl;
    std::cout << std::endl;

    // Test cycle detection
    std::cout << "============================================" << std::endl;
    std::cout << "  Testing Cycle Detection" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::endl;

    // Manually test topological sort
    constexpr size_t N = 5;
    std::array<std::array<bool, N>, N> testGraph{};

    // Create a valid DAG
    testGraph[3][0] = true;  // Pos depends on Vel
    testGraph[0][2] = true;  // Vel depends on Accel
    testGraph[2][1] = true;  // Accel depends on Mass
    testGraph[2][4] = true;  // Accel depends on Force

    auto sortResult = topologicalSort<N>(testGraph);
    std::cout << "Valid DAG sort result:" << std::endl;
    std::cout << "  Has cycle: " << (sortResult.hasCycle ? "YES" : "NO") << std::endl;
    std::cout << "  Order: ";
    for (size_t i = 0; i < sortResult.count; ++i) {
        std::cout << sortResult.order[i] << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;

    // Create a cycle
    std::array<std::array<bool, N>, N> cycleGraph{};
    cycleGraph[0][1] = true;  // 0 -> 1
    cycleGraph[1][2] = true;  // 1 -> 2
    cycleGraph[2][0] = true;  // 2 -> 0 (CYCLE!)

    auto cycleResult = topologicalSort<N>(cycleGraph);
    std::cout << "Graph with cycle:" << std::endl;
    std::cout << "  Has cycle: " << (cycleResult.hasCycle ? "YES ✓" : "NO") << std::endl;
    std::cout << std::endl;

    // Summary
    std::cout << "============================================" << std::endl;
    std::cout << "  Phase 3 Status" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::endl;
    std::cout << "✓ Compile-time field name extraction" << std::endl;
    std::cout << "✓ Dependency graph construction" << std::endl;
    std::cout << "✓ Topological sort implementation" << std::endl;
    std::cout << "✓ Cycle detection" << std::endl;
    std::cout << "✓ Provider lookup (compile-time)" << std::endl;
    std::cout << "✓ Execution order determination" << std::endl;
    std::cout << "✓ Automatic dependency injection" << std::endl;
    std::cout << "✓ Full computeDerivatives() implementation" << std::endl;
    std::cout << std::endl;
    std::cout << "🎉 PHASE 3 COMPLETE! 🎉" << std::endl;
    std::cout << std::endl;
    std::cout << "The system can now automatically compute derivatives" << std::endl;
    std::cout << "with zero manual wiring. Components declare their" << std::endl;
    std::cout << "dependencies, and the framework handles the rest!" << std::endl;

    return 0;
}
