/*
 * System Builder Demo
 * Demonstrates automatic dependency resolution and system composition
 */

#include "system_builder.hpp"
#include <iostream>
#include <iomanip>

using namespace sopot::experimental;

// ============================================================================
// DEFINE TAGS
// ============================================================================

namespace physics {
    struct Position { using ValueType = double; };
    struct Velocity { using ValueType = double; };
    struct Acceleration { using ValueType = double; };
    struct Mass { using ValueType = double; };
    struct Force { using ValueType = double; };
}

// ============================================================================
// COMPONENT 1: Position (Kinematics)
// ============================================================================

template<typename T = double>
class PositionComponent {
public:
    static constexpr size_t StateSize = 1;  // 1 state: position

    using Dependencies = FieldBundle<
        Field<physics::Velocity, "vel">
    >;

    using Provides = FieldBundle<
        Field<physics::Position, "pos">
    >;

    explicit PositionComponent(T initial_pos) : m_initial(initial_pos) {}

    T getInitialState() const { return m_initial; }

    T providePosition(const T& state) const { return state; }

    // dx/dt = v
    T computeDerivative(T /* t */, const T& x, T v) const {
        return v;
    }

private:
    T m_initial;
};

// ============================================================================
// COMPONENT 2: Velocity (Dynamics)
// ============================================================================

template<typename T = double>
class VelocityComponent {
public:
    static constexpr size_t StateSize = 1;  // 1 state: velocity

    using Dependencies = FieldBundle<
        Field<physics::Acceleration, "accel">
    >;

    using Provides = FieldBundle<
        Field<physics::Velocity, "vel">
    >;

    explicit VelocityComponent(T initial_vel) : m_initial(initial_vel) {}

    T getInitialState() const { return m_initial; }

    T provideVelocity(const T& state) const { return state; }

    // dv/dt = a
    T computeDerivative(T /* t */, const T& v, T a) const {
        return a;
    }

private:
    T m_initial;
};

// ============================================================================
// COMPONENT 3: Force Provider (Constant force)
// ============================================================================

template<typename T = double>
class ConstantForce {
public:
    static constexpr size_t StateSize = 0;  // Stateless

    using Dependencies = FieldBundle<>;  // No dependencies

    using Provides = FieldBundle<
        Field<physics::Force, "force">
    >;

    explicit ConstantForce(T force) : m_force(force) {}

    T provideForce() const { return m_force; }

private:
    T m_force;
};

// ============================================================================
// COMPONENT 4: Mass Provider (Constant mass)
// ============================================================================

template<typename T = double>
class ConstantMass {
public:
    static constexpr size_t StateSize = 0;  // Stateless

    using Dependencies = FieldBundle<>;  // No dependencies

    using Provides = FieldBundle<
        Field<physics::Mass, "mass">
    >;

    explicit ConstantMass(T mass) : m_mass(mass) {}

    T provideMass() const { return m_mass; }

private:
    T m_mass;
};

// ============================================================================
// COMPONENT 5: Acceleration Computer (F = ma)
// ============================================================================

template<typename T = double>
class NewtonSecondLaw {
public:
    static constexpr size_t StateSize = 0;  // Stateless

    using Dependencies = FieldBundle<
        Field<physics::Force, "force">,
        Field<physics::Mass, "mass">
    >;

    using Provides = FieldBundle<
        Field<physics::Acceleration, "accel">
    >;

    T computeAcceleration(T force, T mass) const {
        return force / mass;
    }
};

// ============================================================================
// DEMONSTRATION
// ============================================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "System Builder Demo" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Create components in ARBITRARY ORDER!
    // System builder will figure out execution order
    auto system = makeSystem<double>(
        VelocityComponent<double>(0.0),      // Component 0
        ConstantMass<double>(2.0),           // Component 1
        NewtonSecondLaw<double>(),           // Component 2
        PositionComponent<double>(0.0),      // Component 3
        ConstantForce<double>(10.0)          // Component 4
    );

    std::cout << "Components created in arbitrary order:" << std::endl;
    std::cout << "  [0] VelocityComponent (depends on: accel)" << std::endl;
    std::cout << "  [1] ConstantMass (provides: mass)" << std::endl;
    std::cout << "  [2] NewtonSecondLaw (depends on: force, mass; provides: accel)" << std::endl;
    std::cout << "  [3] PositionComponent (depends on: vel)" << std::endl;
    std::cout << "  [4] ConstantForce (provides: force)" << std::endl;
    std::cout << std::endl;

    // System info
    system.printInfo();
    std::cout << std::endl;

    // Dependency analysis
    std::cout << "Dependency Analysis:" << std::endl;
    std::cout << "  Execution order should be:" << std::endl;
    std::cout << "    1. ConstantForce (provides force)" << std::endl;
    std::cout << "    2. ConstantMass (provides mass)" << std::endl;
    std::cout << "    3. NewtonSecondLaw (uses force, mass → produces accel)" << std::endl;
    std::cout << "    4. VelocityComponent (uses accel → produces vel)" << std::endl;
    std::cout << "    5. PositionComponent (uses vel → produces pos)" << std::endl;
    std::cout << std::endl;

    // Type-level dependency checking
    using VelComp = VelocityComponent<double>;
    using ForceComp = ConstantForce<double>;
    using AccelField = Field<physics::Acceleration, "accel">;
    using ForceField = Field<physics::Force, "force">;

    std::cout << "Compile-time dependency verification:" << std::endl;
    std::cout << "  VelocityComponent depends on accel? "
              << (DependsOnField<VelComp, AccelField> ? "YES" : "NO") << std::endl;
    std::cout << "  ConstantForce provides force? "
              << (ProvidesField<ForceComp, ForceField> ? "YES" : "NO") << std::endl;
    std::cout << std::endl;

    // Initial state
    auto state = system.getInitialState();
    std::cout << "Initial state (size = " << state.size() << "):" << std::endl;
    for (size_t i = 0; i < state.size(); ++i) {
        std::cout << "  state[" << i << "] = " << state[i] << std::endl;
    }
    std::cout << std::endl;

    // Extract local states
    std::cout << "Extract local states:" << std::endl;
    auto pos_state = system.extractLocalState<3>(state);  // PositionComponent
    auto vel_state = system.extractLocalState<0>(state);  // VelocityComponent
    std::cout << "  Position state: " << pos_state << std::endl;
    std::cout << "  Velocity state: " << vel_state << std::endl;
    std::cout << std::endl;

    // Component access
    std::cout << "Component access:" << std::endl;
    auto& force_comp = system.getComponent<4>();
    auto& mass_comp = system.getComponent<1>();
    std::cout << "  Force = " << force_comp.provideForce() << " N" << std::endl;
    std::cout << "  Mass  = " << mass_comp.provideMass() << " kg" << std::endl;
    std::cout << std::endl;

    // Manual computation (will be automatic in Phase 3!)
    std::cout << "Manual computation of one time step:" << std::endl;
    double F = system.getComponent<4>().provideForce();
    double m = system.getComponent<1>().provideMass();
    double a = system.getComponent<2>().computeAcceleration(F, m);
    std::cout << "  Acceleration = F/m = " << F << "/" << m << " = " << a << " m/s²" << std::endl;

    double v = vel_state;
    double dv_dt = system.getComponent<0>().computeDerivative(0.0, v, a);
    std::cout << "  dv/dt = " << dv_dt << " m/s²" << std::endl;

    double x = pos_state;
    double dx_dt = system.getComponent<3>().computeDerivative(0.0, x, v);
    std::cout << "  dx/dt = " << dx_dt << " m/s" << std::endl;
    std::cout << std::endl;

    // Simulate
    double dt = 0.1;
    double new_v = v + dv_dt * dt;
    double new_x = x + dx_dt * dt;
    std::cout << "After dt = " << dt << " s:" << std::endl;
    std::cout << "  velocity = " << new_v << " m/s" << std::endl;
    std::cout << "  position = " << new_x << " m" << std::endl;
    std::cout << std::endl;

    std::cout << "========================================" << std::endl;
    std::cout << "Key Achievements" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "✓ Components composed in ARBITRARY ORDER" << std::endl;
    std::cout << "✓ Type-safe dependency checking at compile-time" << std::endl;
    std::cout << "✓ State management automatic (offsets computed at compile-time)" << std::endl;
    std::cout << "✓ Component access by index" << std::endl;
    std::cout << std::endl;
    std::cout << "Next: Phase 3 - Automatic computeDerivatives() generation!" << std::endl;
    std::cout << "  - Topological sort to determine execution order" << std::endl;
    std::cout << "  - Automatic dependency injection" << std::endl;
    std::cout << "  - Provision caching" << std::endl;
    std::cout << "  - Single function call: derivs = system.computeDerivatives(t, state)" << std::endl;

    return 0;
}
