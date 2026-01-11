#include "../physics/coupled_oscillator/coupled_system.hpp"
#include "../core/solver.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot;
using namespace sopot::physics::coupled;

#define ASSERT_NEAR(a, b, tol) \
    do { \
        double _a = (a), _b = (b); \
        if (std::abs(_a - _b) > (tol)) { \
            std::cerr << "FAIL at line " << __LINE__ << ": " \
                      << #a << " = " << _a << " != " << #b << " = " << _b << std::endl; \
            std::abort(); \
        } \
    } while(0)

//=============================================================================
// Test 1: Component Creation
//=============================================================================
void testComponentCreation() {
    std::cout << "Test 1: Component creation..." << std::endl;

    // Create individual components
    Mass1<double> m1(2.0, 0.0, 1.0, "left_mass");   // m=2, x0=0, v0=1
    Mass2<double> m2(3.0, 2.0, 0.0, "right_mass");  // m=3, x0=2, v0=0
    Spring12<double> spring(100.0, 1.5, 0.1);       // k=100, L0=1.5, c=0.1

    ASSERT_NEAR(m1.getMass(), 2.0, 1e-10);
    ASSERT_NEAR(m2.getMass(), 3.0, 1e-10);
    ASSERT_NEAR(spring.getStiffness(), 100.0, 1e-10);
    ASSERT_NEAR(spring.getRestLength(), 1.5, 1e-10);

    std::cout << "   Created Mass1: m=" << m1.getMass() << " kg" << std::endl;
    std::cout << "   Created Mass2: m=" << m2.getMass() << " kg" << std::endl;
    std::cout << "   Created Spring: k=" << spring.getStiffness() << " N/m, L0="
              << spring.getRestLength() << " m" << std::endl;
    std::cout << "   [OK] Component creation passed" << std::endl;
}

//=============================================================================
// Test 2: System Assembly
//=============================================================================
void testSystemAssembly() {
    std::cout << "\nTest 2: System assembly..." << std::endl;

    // Assemble system from components
    auto system = makeTypedODESystem<double>(
        createMass1<double>(1.0, 0.0, 0.0),
        createMass2<double>(1.0, 2.0, 0.0),
        createSpring<double>(4.0, 1.0),
        createEnergyMonitor<double>()
    );

    // Check system properties
    std::cout << "   State dimension: " << system.getStateDimension() << std::endl;
    std::cout << "   Component count: " << system.getComponentCount() << std::endl;

    ASSERT_NEAR(system.getStateDimension(), 4, 1e-10);  // 2 states per mass
    ASSERT_NEAR(system.getComponentCount(), 4, 1e-10);  // 2 masses + spring + monitor

    // Verify compile-time function availability
    static_assert(decltype(system)::hasFunction<mass1::Position>());
    static_assert(decltype(system)::hasFunction<mass2::Position>());
    static_assert(decltype(system)::hasFunction<mass1::Velocity>());
    static_assert(decltype(system)::hasFunction<mass2::Velocity>());
    static_assert(decltype(system)::hasFunction<mass1::Force>());
    static_assert(decltype(system)::hasFunction<mass2::Force>());
    static_assert(decltype(system)::hasFunction<spring::Extension>());
    static_assert(decltype(system)::hasFunction<system::TotalEnergy>());

    std::cout << "   All state functions available at compile time" << std::endl;
    std::cout << "   [OK] System assembly passed" << std::endl;
}

//=============================================================================
// Test 3: State Function Queries
//=============================================================================
void testStateFunctions() {
    std::cout << "\nTest 3: State function queries..." << std::endl;

    // Create system: two unit masses, spring k=4, rest length=1
    // Initial: x1=0, x2=2, v1=v2=0 => extension = 0-2-1 = -3 (compressed)
    auto system = makeTypedODESystem<double>(
        createMass1<double>(1.0, 0.0, 0.0),
        createMass2<double>(1.0, 3.0, 0.0),  // x2=3, so extension = 0-3-1 = -4
        createSpring<double>(4.0, 1.0),
        createEnergyMonitor<double>()
    );

    auto state = system.getInitialState();

    // Query positions
    double x1 = system.computeStateFunction<mass1::Position>(state);
    double x2 = system.computeStateFunction<mass2::Position>(state);
    std::cout << "   Position x1 = " << x1 << " m" << std::endl;
    std::cout << "   Position x2 = " << x2 << " m" << std::endl;
    ASSERT_NEAR(x1, 0.0, 1e-10);
    ASSERT_NEAR(x2, 3.0, 1e-10);

    // Query extension
    double ext = system.computeStateFunction<spring::Extension>(state);
    std::cout << "   Extension = " << ext << " m (x1 - x2 - L0)" << std::endl;
    ASSERT_NEAR(ext, 0.0 - 3.0 - 1.0, 1e-10);  // -4

    // Query forces (F = -k * extension)
    double F1 = system.computeStateFunction<mass1::Force>(state);
    double F2 = system.computeStateFunction<mass2::Force>(state);
    std::cout << "   Force on mass1 = " << F1 << " N" << std::endl;
    std::cout << "   Force on mass2 = " << F2 << " N" << std::endl;
    ASSERT_NEAR(F1, -4.0 * (-4.0), 1e-10);  // +16 N (pushed right)
    ASSERT_NEAR(F2, -F1, 1e-10);             // Newton's 3rd law

    // Query energy
    double E = system.computeStateFunction<system::TotalEnergy>(state);
    std::cout << "   Total energy = " << E << " J" << std::endl;
    ASSERT_NEAR(E, 0.5 * 4.0 * 16.0, 1e-10);  // 0.5 * k * ext^2 = 32 J

    std::cout << "   [OK] State function queries passed" << std::endl;
}

//=============================================================================
// Test 4: Integration and Energy Conservation
//=============================================================================
void testEnergyConservation() {
    std::cout << "\nTest 4: Energy conservation (undamped)..." << std::endl;

    // Equal masses, symmetric initial conditions
    auto system = createCoupledOscillator<double>(
        1.0, 0.0, 0.0,    // m1=1, x1_0=0, v1_0=0
        1.0, 2.0, 0.0,    // m2=1, x2_0=2, v2_0=0
        4.0, 1.0, 0.0     // k=4, L0=1, no damping
    );

    auto state = system.getInitialState();
    double E0 = system.computeStateFunction<system::TotalEnergy>(state);
    double p0 = system.computeStateFunction<system::Momentum>(state);

    std::cout << "   Initial energy: " << E0 << " J" << std::endl;
    std::cout << "   Initial momentum: " << p0 << " kg·m/s" << std::endl;

    // Integrate for several periods
    double dt = 0.001;
    double t_end = 5.0;
    double t = 0.0;

    double E_max_error = 0.0;
    double p_max_error = 0.0;

    while (t < t_end) {
        // RK4 step
        auto k1 = system.computeDerivatives(t, state);
        std::vector<double> s2(4), s3(4), s4(4);
        for (int i = 0; i < 4; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
        auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
        for (int i = 0; i < 4; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
        auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
        for (int i = 0; i < 4; ++i) s4[i] = state[i] + dt * k3[i];
        auto k4 = system.computeDerivatives(t + dt, s4);
        for (int i = 0; i < 4; ++i)
            state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        t += dt;

        double E = system.computeStateFunction<system::TotalEnergy>(state);
        double p = system.computeStateFunction<system::Momentum>(state);
        E_max_error = std::max(E_max_error, std::abs(E - E0) / E0);
        p_max_error = std::max(p_max_error, std::abs(p - p0));
    }

    std::cout << "   Final energy: " << system.computeStateFunction<system::TotalEnergy>(state) << " J" << std::endl;
    std::cout << "   Max energy error: " << E_max_error * 100 << "%" << std::endl;
    std::cout << "   Max momentum error: " << p_max_error << " kg·m/s" << std::endl;

    // RK4 should conserve energy very well
    if (E_max_error > 0.001) {
        std::cerr << "   [FAIL] Energy drift too large!" << std::endl;
        std::abort();
    }
    if (p_max_error > 1e-10) {
        std::cerr << "   [FAIL] Momentum not conserved!" << std::endl;
        std::abort();
    }

    std::cout << "   [OK] Energy conservation passed" << std::endl;
}

//=============================================================================
// Test 5: Analytical Solution Validation
//=============================================================================
void testAnalyticalSolution() {
    std::cout << "\nTest 5: Analytical solution validation..." << std::endl;

    double m = 1.0, k = 8.0, L0 = 1.0;
    double x1_0 = 0.0, x2_0 = 2.0;

    AnalyticalSolution analytical(m, k, L0, x1_0, x2_0);

    std::cout << "   Angular frequency: " << analytical.omega << " rad/s" << std::endl;
    std::cout << "   Period: " << analytical.period() << " s" << std::endl;
    std::cout << "   Center of mass: " << analytical.x_cm << " m" << std::endl;

    auto system = createCoupledOscillator<double>(
        m, x1_0, 0.0,
        m, x2_0, 0.0,
        k, L0, 0.0
    );

    auto state = system.getInitialState();
    double dt = 0.0001;  // Small timestep for accuracy
    double t = 0.0;
    double max_error = 0.0;

    // Integrate for one full period
    while (t < analytical.period()) {
        // RK4 step
        auto k1 = system.computeDerivatives(t, state);
        std::vector<double> s2(4), s3(4), s4(4);
        for (int i = 0; i < 4; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
        auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
        for (int i = 0; i < 4; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
        auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
        for (int i = 0; i < 4; ++i) s4[i] = state[i] + dt * k3[i];
        auto k4 = system.computeDerivatives(t + dt, s4);
        for (int i = 0; i < 4; ++i)
            state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        t += dt;

        auto [x1_exact, x2_exact] = analytical.positions(t);
        double x1_num = system.computeStateFunction<mass1::Position>(state);
        double x2_num = system.computeStateFunction<mass2::Position>(state);

        max_error = std::max(max_error, std::abs(x1_num - x1_exact));
        max_error = std::max(max_error, std::abs(x2_num - x2_exact));
    }

    std::cout << "   Max position error: " << max_error << " m" << std::endl;

    if (max_error > 1e-6) {
        std::cerr << "   [FAIL] Error too large!" << std::endl;
        std::abort();
    }

    std::cout << "   [OK] Analytical solution validation passed" << std::endl;
}

//=============================================================================
// Test 6: Damped System
//=============================================================================
void testDampedSystem() {
    std::cout << "\nTest 6: Damped system (energy decay)..." << std::endl;

    auto system = createCoupledOscillator<double>(
        1.0, 0.0, 0.0,    // m1=1
        1.0, 2.0, 0.0,    // m2=1
        4.0, 1.0, 0.5     // k=4, L0=1, damping=0.5
    );

    auto state = system.getInitialState();
    double E0 = system.computeStateFunction<system::TotalEnergy>(state);

    double dt = 0.001;
    double t = 0.0;

    while (t < 10.0) {
        auto k1 = system.computeDerivatives(t, state);
        std::vector<double> s2(4), s3(4), s4(4);
        for (int i = 0; i < 4; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
        auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
        for (int i = 0; i < 4; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
        auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
        for (int i = 0; i < 4; ++i) s4[i] = state[i] + dt * k3[i];
        auto k4 = system.computeDerivatives(t + dt, s4);
        for (int i = 0; i < 4; ++i)
            state[i] += dt / 6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        t += dt;
    }

    double E_final = system.computeStateFunction<system::TotalEnergy>(state);
    double decay_ratio = E_final / E0;

    std::cout << "   Initial energy: " << E0 << " J" << std::endl;
    std::cout << "   Final energy: " << E_final << " J" << std::endl;
    std::cout << "   Energy decay: " << (1.0 - decay_ratio) * 100 << "%" << std::endl;

    if (decay_ratio > 0.5) {
        std::cerr << "   [FAIL] Energy should decay significantly with damping!" << std::endl;
        std::abort();
    }

    std::cout << "   [OK] Damped system passed" << std::endl;
}

//=============================================================================
// Main
//=============================================================================
int main() {
    std::cout << "=== Coupled Oscillator Test Suite ===" << std::endl;
    std::cout << "Two masses connected by a spring - modular component design\n" << std::endl;

    testComponentCreation();
    testSystemAssembly();
    testStateFunctions();
    testEnergyConservation();
    testAnalyticalSolution();
    testDampedSystem();

    std::cout << "\n=== ALL TESTS PASSED ===" << std::endl;
    std::cout << "Demonstrated:" << std::endl;
    std::cout << "  - Fully separated components (Mass1, Mass2, Spring, EnergyMonitor)" << std::endl;
    std::cout << "  - Cross-component communication via typed state functions" << std::endl;
    std::cout << "  - Compile-time dispatch with zero runtime overhead" << std::endl;
    std::cout << "  - Energy and momentum conservation" << std::endl;
    std::cout << "  - Analytical solution validation" << std::endl;

    return 0;
}
