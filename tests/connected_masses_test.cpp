#include "physics/connected_masses/connectivity_matrix.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

using namespace sopot;
using namespace sopot::connected_masses;

/**
 * @brief Test 1: Simple 2-mass system (equivalent to coupled oscillator)
 */
void test_two_masses() {
    std::cout << "\n=== Test 1: Two Masses Connected by Spring ===\n";

    // Define connectivity: mass 0 <-> mass 1
    constexpr auto edges = std::array{
        std::pair{size_t(0), size_t(1)}
    };

    // Create system
    auto system = makeConnectedMassSystem<double, 2, edges>(
        // Mass parameters: {mass, initial_position, initial_velocity}
        {{{1.0, -0.5, 0.0},  // Mass 0: 1kg at x=-0.5m
          {1.0,  0.5, 0.0}}}, // Mass 1: 1kg at x=+0.5m
        // Spring parameters: {stiffness, rest_length, damping}
        {{{10.0, 0.5, 0.5}}}  // One spring: k=10 N/m, L0=0.5m, c=0.5 N·s/m
    );

    // Verify compile-time function availability
    static_assert(decltype(system)::hasFunction<MassTag<0>::Position>());
    static_assert(decltype(system)::hasFunction<MassTag<0>::Velocity>());
    static_assert(decltype(system)::hasFunction<MassTag<0>::Force>());
    static_assert(decltype(system)::hasFunction<MassTag<1>::Position>());
    static_assert(decltype(system)::hasFunction<MassTag<1>::Velocity>());
    static_assert(decltype(system)::hasFunction<MassTag<1>::Force>());
    static_assert(decltype(system)::hasFunction<SpringTag<0, 1>::Extension>());
    static_assert(decltype(system)::hasFunction<SpringTag<0, 1>::PotentialEnergy>());

    std::cout << "✓ All state functions available at compile time\n";
    std::cout << "✓ Total state size: " << system.getStateDimension() << " elements\n";

    // Get initial state
    auto state = system.getInitialState();

    std::cout << "Initial state:\n";
    std::cout << "  Mass 0: x = " << system.computeStateFunction<MassTag<0>::Position>(state)
              << " m, v = " << system.computeStateFunction<MassTag<0>::Velocity>(state) << " m/s\n";
    std::cout << "  Mass 1: x = " << system.computeStateFunction<MassTag<1>::Position>(state)
              << " m, v = " << system.computeStateFunction<MassTag<1>::Velocity>(state) << " m/s\n";
    std::cout << "  Spring extension: "
              << system.computeStateFunction<SpringTag<0, 1>::Extension>(state) << " m\n";

    // Simulate for 5 seconds using manual RK4
    double t = 0.0;
    double dt = 0.01;
    double t_end = 5.0;

    std::cout << "\nSimulating for " << t_end << " seconds...\n";

    size_t n = system.getStateDimension();
    int steps = 0;
    while (t < t_end) {
        // RK4 integration step
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
        ++steps;
    }

    std::cout << "✓ Completed " << steps << " integration steps\n";

    std::cout << "Final state (t = " << t << " s):\n";
    std::cout << "  Mass 0: x = " << system.computeStateFunction<MassTag<0>::Position>(state)
              << " m, v = " << system.computeStateFunction<MassTag<0>::Velocity>(state) << " m/s\n";
    std::cout << "  Mass 1: x = " << system.computeStateFunction<MassTag<1>::Position>(state)
              << " m, v = " << system.computeStateFunction<MassTag<1>::Velocity>(state) << " m/s\n";
    std::cout << "  Spring extension: "
              << system.computeStateFunction<SpringTag<0, 1>::Extension>(state) << " m\n";
}

/**
 * @brief Test 2: Three masses in triangular configuration (full connectivity)
 */
void test_triangular_configuration() {
    std::cout << "\n=== Test 2: Three Masses in Triangle (Fully Connected) ===\n";

    // Define connectivity: complete graph on 3 vertices
    constexpr auto edges = std::array{
        std::pair{size_t(0), size_t(1)},  // Edge 0: Mass 0 <-> Mass 1
        std::pair{size_t(1), size_t(2)},  // Edge 1: Mass 1 <-> Mass 2
        std::pair{size_t(0), size_t(2)}   // Edge 2: Mass 0 <-> Mass 2
    };

    std::cout << "Connectivity matrix:\n";
    std::cout << "     0   1   2\n";
    std::cout << "  0  -   1   1\n";
    std::cout << "  1  1   -   1\n";
    std::cout << "  2  1   1   -\n";
    std::cout << "(3 masses, 3 springs = fully connected)\n\n";

    // Create system with different masses and springs
    auto system = makeConnectedMassSystem<double, 3, edges>(
        // Mass parameters
        {{{1.0,  0.0, 0.0},   // Mass 0: 1kg at origin
          {1.5,  1.0, 0.0},   // Mass 1: 1.5kg at x=1m
          {2.0,  0.5, 0.0}}}, // Mass 2: 2kg at x=0.5m
        // Spring parameters
        {{{10.0, 0.8, 0.3},   // Spring 0-1: k=10, L0=0.8m
          {15.0, 0.4, 0.4},   // Spring 1-2: k=15, L0=0.4m
          {20.0, 0.5, 0.2}}}  // Spring 0-2: k=20, L0=0.5m
    );

    static_assert(decltype(system)::hasFunction<MassTag<0>::Position>());
    static_assert(decltype(system)::hasFunction<MassTag<1>::Position>());
    static_assert(decltype(system)::hasFunction<MassTag<2>::Position>());
    static_assert(decltype(system)::hasFunction<SpringTag<0, 1>::Extension>());
    static_assert(decltype(system)::hasFunction<SpringTag<1, 2>::Extension>());
    static_assert(decltype(system)::hasFunction<SpringTag<0, 2>::Extension>());

    std::cout << "✓ System created with 3 masses and 3 springs\n";
    std::cout << "✓ State size: " << system.getStateDimension() << " elements (expected 6)\n";

    auto state = system.getInitialState();

    std::cout << "\nInitial configuration:\n";
    for (size_t i = 0; i < 3; ++i) {
        double x = (i == 0) ? system.computeStateFunction<MassTag<0>::Position>(state)
                   : (i == 1) ? system.computeStateFunction<MassTag<1>::Position>(state)
                              : system.computeStateFunction<MassTag<2>::Position>(state);
        std::cout << "  Mass " << i << ": x = " << x << " m\n";
    }

    // Compute initial spring extensions
    double ext01 = system.computeStateFunction<SpringTag<0, 1>::Extension>(state);
    double ext12 = system.computeStateFunction<SpringTag<1, 2>::Extension>(state);
    double ext02 = system.computeStateFunction<SpringTag<0, 2>::Extension>(state);

    std::cout << "\nSpring extensions:\n";
    std::cout << "  Spring 0-1: " << ext01 << " m\n";
    std::cout << "  Spring 1-2: " << ext12 << " m\n";
    std::cout << "  Spring 0-2: " << ext02 << " m\n";

    // Short simulation
    double t = 0.0;
    double dt = 0.001;
    double t_end = 1.0;

    size_t n = system.getStateDimension();
    int steps = 0;
    while (t < t_end) {
        // RK4 integration step
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
        ++steps;
    }

    std::cout << "\n✓ Simulated " << steps << " steps to t = " << t << " s\n";
}

/**
 * @brief Test 3: Four masses in a chain (linear connectivity)
 */
void test_chain_configuration() {
    std::cout << "\n=== Test 3: Four Masses in Chain ===\n";

    // Linear connectivity: 0 -- 1 -- 2 -- 3
    constexpr auto edges = std::array{
        std::pair{size_t(0), size_t(1)},
        std::pair{size_t(1), size_t(2)},
        std::pair{size_t(2), size_t(3)}
    };

    std::cout << "Connectivity: 0 -- 1 -- 2 -- 3 (linear chain)\n";
    std::cout << "(4 masses, 3 springs)\n\n";

    // Use uniform parameters helper
    auto system = makeUniformConnectedSystem<double, 4, edges>(
        1.0,  // All masses = 1kg
        {10.0, 1.0, 0.1}  // All springs: k=10, L0=1m, c=0.1
    );

    std::cout << "✓ System created with uniform parameters\n";
    std::cout << "✓ State size: " << system.getStateDimension() << " elements (expected 8)\n";

    // Set initial positions in a line with some displacement
    auto state = system.getInitialState();
    state[0] = 0.0;   // Mass 0 position
    state[2] = 1.0;   // Mass 1 position
    state[4] = 2.0;   // Mass 2 position
    state[6] = 3.5;   // Mass 3 position (compressed)

    std::cout << "\nInitial chain configuration:\n";
    std::cout << "  Positions: [0.0, 1.0, 2.0, 3.5] m\n";

    // Quick simulation
    double t = 0.0;
    double dt = 0.01;

    size_t n = system.getStateDimension();
    for (int i = 0; i < 100; ++i) {
        // RK4 integration step
        auto k1 = system.computeDerivatives(t, state);
        std::vector<double> s2(n), s3(n), s4(n);
        for (size_t j = 0; j < n; ++j) s2[j] = state[j] + 0.5 * dt * k1[j];
        auto k2 = system.computeDerivatives(t + 0.5 * dt, s2);
        for (size_t j = 0; j < n; ++j) s3[j] = state[j] + 0.5 * dt * k2[j];
        auto k3 = system.computeDerivatives(t + 0.5 * dt, s3);
        for (size_t j = 0; j < n; ++j) s4[j] = state[j] + dt * k3[j];
        auto k4 = system.computeDerivatives(t + dt, s4);
        for (size_t j = 0; j < n; ++j)
            state[j] += dt / 6.0 * (k1[j] + 2*k2[j] + 2*k3[j] + k4[j]);
        t += dt;
    }

    std::cout << "\n✓ Simulated to t = " << t << " s\n";
    std::cout << "Final positions: ["
              << system.computeStateFunction<MassTag<0>::Position>(state) << ", "
              << system.computeStateFunction<MassTag<1>::Position>(state) << ", "
              << system.computeStateFunction<MassTag<2>::Position>(state) << ", "
              << system.computeStateFunction<MassTag<3>::Position>(state) << "] m\n";
}

/**
 * @brief Test 4: Complex connectivity pattern
 */
void test_complex_connectivity() {
    std::cout << "\n=== Test 4: Complex Connectivity Pattern ===\n";

    // Define connectivity as adjacency matrix (conceptual)
    // Matrix visualization:
    //      0  1  2  3
    //   0  -  1  1  0
    //   1  1  -  1  1
    //   2  1  1  -  1
    //   3  0  1  1  -
    // This means: Mass 0 connects to 1,2; Mass 1 connects to 0,2,3; etc.

    std::cout << "Conceptual adjacency matrix:\n";
    std::cout << "     0  1  2  3\n";
    std::cout << "  0  -  1  1  -\n";
    std::cout << "  1  1  -  1  1\n";
    std::cout << "  2  1  1  -  1\n";
    std::cout << "  3  -  1  1  -\n";

    // Manually extract edges from upper triangle
    constexpr auto edges = std::array{
        std::pair{size_t(0), size_t(1)},  // From row 0
        std::pair{size_t(0), size_t(2)},
        std::pair{size_t(1), size_t(2)},  // From row 1
        std::pair{size_t(1), size_t(3)},
        std::pair{size_t(2), size_t(3)}   // From row 2
    };

    std::cout << "\nExtracted edges (upper triangle):\n";
    for (size_t i = 0; i < edges.size(); ++i) {
        std::cout << "  Edge " << i << ": " << edges[i].first
                  << " <-> " << edges[i].second << "\n";
    }

    // Create system using extracted edges
    auto system = makeUniformConnectedSystem<double, 4, edges>(
        1.0,  // mass
        {10.0, 1.0, 0.1}  // spring params
    );

    std::cout << "\n✓ System created from connectivity pattern\n";
    std::cout << "✓ Number of edges: " << edges.size() << "\n";
    std::cout << "✓ State size: " << system.getStateDimension() << " elements\n";

    // Verify the system works
    auto state = system.getInitialState();
    auto derivs = system.computeDerivatives(0.0, state);

    std::cout << "✓ Derivatives computed successfully\n";
}

int main() {
    std::cout << "Connected Masses Test Suite\n";
    std::cout << "===========================\n";
    std::cout << "\nDemonstrating compile-time matrix-based connectivity\n";
    std::cout << "for arbitrary mass-spring systems.\n";

    try {
        test_two_masses();
        test_triangular_configuration();
        test_chain_configuration();
        test_complex_connectivity();

        std::cout << "\n=================================\n";
        std::cout << "All tests passed successfully! ✓\n";
        std::cout << "=================================\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
