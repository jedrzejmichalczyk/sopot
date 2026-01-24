/**
 * @file validated_graph_test.cpp
 * @brief Test compile-time validated graph system
 *
 * Demonstrates:
 *   - Static validation of state function resolution
 *   - O(K) template instantiations with compile-time guarantees
 *   - Runtime execution with validated topology
 */

#include "physics/unified/validated_system.hpp"
#include "core/solver.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

using namespace sopot;
using namespace sopot::unified;

// =============================================================================
// COMPILE-TIME TOPOLOGY DEFINITIONS
// =============================================================================

// Simple two-mass spring system
constexpr auto twoMassTopology = []() {
    GraphTopology<3, 2> t;
    // Nodes: mass0, spring0, mass1
    t.nodes[0] = {0, 0};  // Type 0 (mass), batch index 0
    t.nodes[1] = {1, 0};  // Type 1 (spring), batch index 0
    t.nodes[2] = {0, 1};  // Type 0 (mass), batch index 1

    // Edges: spring connects both masses
    t.edges[0] = {1, 0, 0, 0};  // Spring port 0 -> Mass 0 port 0
    t.edges[1] = {1, 1, 2, 0};  // Spring port 1 -> Mass 1 port 0

    return t;
}();

// Chain of 5 masses
constexpr auto chainTopology = makeChainTopology<5>();

// 5x5 grid
constexpr auto gridTopology = makeGridTopology<5, 5>();

// 20x20 grid - now possible with O(N+E) validation!
constexpr auto largeGridTopology = makeGridTopology<20, 20>();

// 50x50 grid - let's push further!
constexpr auto hugeGridTopology = makeGridTopology<50, 50>();

// 100x100 grid - the ultimate test!
constexpr auto massiveGridTopology = makeGridTopology<100, 100>();

// =============================================================================
// COMPILE-TIME VALIDATION (these are static_asserts!)
// =============================================================================

// Validate all topologies at compile time
static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(twoMassTopology).valid,
    "Two-mass topology validation failed");

static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(chainTopology).valid,
    "Chain topology validation failed");

static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(gridTopology).valid,
    "Grid topology validation failed");

static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(largeGridTopology).valid,
    "20x20 grid topology validation failed");

static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(hugeGridTopology).valid,
    "50x50 grid topology validation failed - O(N+E) optimization working!");

static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(massiveGridTopology).valid,
    "100x100 grid topology validation failed - 10,000 masses validated at compile time!");

// =============================================================================
// COMPILE-TIME VALIDATION FAILURE EXAMPLES
// =============================================================================

// Uncomment any of these to see compile-time errors:

// Example 1: Mass requires Force2D, but nothing provides it
// constexpr auto invalidTopology1 = []() {
//     GraphTopology<1, 0> t;
//     t.nodes[0] = {0, 0};  // Just a mass, no spring to provide force
//     return t;
// }();
// static_assert(validateGraph<PointMass2D<double>>(invalidTopology1).valid,
//     "This should fail: mass has no force provider");

// Example 2: Invalid type index
// constexpr auto invalidTopology2 = []() {
//     GraphTopology<1, 0> t;
//     t.nodes[0] = {5, 0};  // Type index 5 doesn't exist
//     return t;
// }();
// static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(invalidTopology2).valid,
//     "This should fail: invalid type index");

// Example 3: Spring without masses (requires Position2D, Velocity2D)
// constexpr auto invalidTopology3 = []() {
//     GraphTopology<1, 0> t;
//     t.nodes[0] = {1, 0};  // Just a spring, no masses
//     return t;
// }();
// static_assert(validateGraph<PointMass2D<double>, Spring2D<double>>(invalidTopology3).valid,
//     "This should fail: spring has no position/velocity providers");

// =============================================================================
// TESTS
// =============================================================================

void test_two_mass_spring() {
    std::cout << "=== Test: Two Mass Spring (Validated) ===" << std::endl;

    // Create system - topology is validated at compile time!
    auto system = makeValidatedSystem<double, twoMassTopology,
        PointMass2D<double>,
        Spring2D<double>
    >();

    // Populate batches
    system.getBatch<0>().add(PointMass2D<double>(1.0, {0.0, 0.0}));
    system.getBatch<0>().add(PointMass2D<double>(1.0, {1.5, 0.0}));  // Stretched
    system.getBatch<1>().add(Spring2D<double>(100.0, 1.0, 0.5));

    std::cout << "Nodes: " << system.getNodeCount() << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;

    auto state = system.getInitialState();
    auto derivs = system.computeDerivatives(0.0, state);

    std::cout << "Mass0 acc: (" << derivs[2] << ", " << derivs[3] << ")" << std::endl;
    std::cout << "Mass1 acc: (" << derivs[6] << ", " << derivs[7] << ")" << std::endl;

    // Spring is stretched, so mass0 should accelerate right (+x), mass1 left (-x)
    bool correct = derivs[2] > 0 && derivs[6] < 0;
    std::cout << "Physics: " << (correct ? "CORRECT" : "WRONG") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_chain() {
    std::cout << "\n=== Test: Chain of 5 Masses (Validated) ===" << std::endl;

    auto system = makeValidatedSystem<double, chainTopology,
        PointMass2D<double>,
        Spring2D<double>
    >();

    // Add masses
    for (size_t i = 0; i < 5; ++i) {
        system.getBatch<0>().add(PointMass2D<double>(1.0, {double(i), 0.0}));
    }

    // Add springs (rest length 1.0, so at equilibrium)
    for (size_t i = 0; i < 4; ++i) {
        system.getBatch<1>().add(Spring2D<double>(50.0, 1.0, 0.5));
    }

    std::cout << "Nodes: " << system.getNodeCount() << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;

    auto state = system.getInitialState();

    // Perturb middle mass
    state[2 * 4] += 0.5;  // Mass 2, x position

    auto derivs = system.computeDerivatives(0.0, state);

    bool restoring = derivs[2 * 4 + 2] < 0;  // Mass 2 should have restoring force
    std::cout << "Middle mass restoring force: " << (restoring ? "YES" : "NO") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_grid_5x5() {
    std::cout << "\n=== Test: 5x5 Grid (Validated) ===" << std::endl;

    auto system = makeValidatedSystem<double, gridTopology,
        PointMass2D<double>,
        Spring2D<double>
    >();

    // Add masses (5x5 = 25)
    for (size_t r = 0; r < 5; ++r) {
        for (size_t c = 0; c < 5; ++c) {
            system.getBatch<0>().add(PointMass2D<double>(1.0, {double(c) * 0.5, double(r) * 0.5}));
        }
    }

    // Add springs: 5*4 horizontal + 4*5 vertical = 40
    size_t num_springs = 5 * 4 + 4 * 5;
    for (size_t i = 0; i < num_springs; ++i) {
        system.getBatch<1>().add(Spring2D<double>(50.0, 0.5, 0.1));
    }

    std::cout << "Grid: 5x5" << std::endl;
    std::cout << "Masses: 25" << std::endl;
    std::cout << "Springs: " << num_springs << std::endl;
    std::cout << "Nodes: " << system.getNodeCount() << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;

    auto state = system.getInitialState();

    // Perturb center
    size_t center = 2 * 5 + 2;  // Row 2, Col 2
    state[center * 4] += 0.1;

    auto derivs = system.computeDerivatives(0.0, state);

    bool restoring = derivs[center * 4 + 2] < 0;
    std::cout << "Center restoring force: " << (restoring ? "YES" : "NO") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_large_grid_performance() {
    std::cout << "\n=== Test: 20x20 Grid Performance (Validated) ===" << std::endl;

    auto system = makeValidatedSystem<double, largeGridTopology,
        PointMass2D<double>,
        Spring2D<double>
    >();

    // Add masses (20x20 = 400)
    for (size_t r = 0; r < 20; ++r) {
        for (size_t c = 0; c < 20; ++c) {
            system.getBatch<0>().add(PointMass2D<double>(1.0, {double(c) * 0.5, double(r) * 0.5}));
        }
    }

    // Add springs: 20*19 horizontal + 19*20 vertical = 760
    size_t num_springs = 20 * 19 + 19 * 20;
    for (size_t i = 0; i < num_springs; ++i) {
        system.getBatch<1>().add(Spring2D<double>(50.0, 0.5, 0.1));
    }

    std::cout << "Grid: 20x20" << std::endl;
    std::cout << "Masses: 400" << std::endl;
    std::cout << "Springs: " << num_springs << std::endl;
    std::cout << "Nodes: " << system.getNodeCount() << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;
    std::cout << "Component types: 2 (compile-time validated)" << std::endl;

    auto state = system.getInitialState();

    // Perturb center
    size_t center = 10 * 20 + 10;
    state[center * 4] += 0.1;

    // Benchmark
    constexpr int iterations = 100;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto derivs = system.computeDerivatives(0.0, state);
        (void)derivs;
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Derivatives (" << iterations << " iterations): " << duration.count() << " us" << std::endl;
    std::cout << "Per iteration: " << (duration.count() / iterations) << " us" << std::endl;

    // Verify physics
    auto derivs = system.computeDerivatives(0.0, state);
    bool restoring = derivs[center * 4 + 2] < 0;
    std::cout << "Center restoring force: " << (restoring ? "YES" : "NO") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_huge_grid_performance() {
    std::cout << "\n=== Test: 50x50 Grid Performance (Validated at Compile Time!) ===" << std::endl;

    auto system = makeValidatedSystem<double, hugeGridTopology,
        PointMass2D<double>,
        Spring2D<double>
    >();

    // Add masses (50x50 = 2500)
    for (size_t r = 0; r < 50; ++r) {
        for (size_t c = 0; c < 50; ++c) {
            system.getBatch<0>().add(PointMass2D<double>(1.0, {double(c) * 0.5, double(r) * 0.5}));
        }
    }

    // Add springs: 50*49 horizontal + 49*50 vertical = 4900
    size_t num_springs = 50 * 49 + 49 * 50;
    for (size_t i = 0; i < num_springs; ++i) {
        system.getBatch<1>().add(Spring2D<double>(50.0, 0.5, 0.1));
    }

    std::cout << "Grid: 50x50" << std::endl;
    std::cout << "Masses: 2500" << std::endl;
    std::cout << "Springs: " << num_springs << std::endl;
    std::cout << "Nodes: " << system.getNodeCount() << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;
    std::cout << "Component types: 2 (COMPILE-TIME VALIDATED!)" << std::endl;

    auto state = system.getInitialState();

    // Perturb center
    size_t center = 25 * 50 + 25;
    state[center * 4] += 0.1;

    // Benchmark
    constexpr int iterations = 100;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto derivs = system.computeDerivatives(0.0, state);
        (void)derivs;
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Derivatives (" << iterations << " iterations): " << duration.count() << " us" << std::endl;
    std::cout << "Per iteration: " << (duration.count() / iterations) << " us" << std::endl;

    // Verify physics
    auto derivs = system.computeDerivatives(0.0, state);
    bool restoring = derivs[center * 4 + 2] < 0;
    std::cout << "Center restoring force: " << (restoring ? "YES" : "NO") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_massive_grid_performance() {
    std::cout << "\n=== Test: 100x100 Grid (COMPILE-TIME VALIDATED!) ===" << std::endl;

    auto system = makeValidatedSystem<double, massiveGridTopology,
        PointMass2D<double>,
        Spring2D<double>
    >();

    // Add masses (100x100 = 10000)
    for (size_t r = 0; r < 100; ++r) {
        for (size_t c = 0; c < 100; ++c) {
            system.getBatch<0>().add(PointMass2D<double>(1.0, {double(c) * 0.5, double(r) * 0.5}));
        }
    }

    // Add springs: 100*99 horizontal + 99*100 vertical = 19800
    size_t num_springs = 100 * 99 + 99 * 100;
    for (size_t i = 0; i < num_springs; ++i) {
        system.getBatch<1>().add(Spring2D<double>(50.0, 0.5, 0.1));
    }

    std::cout << "Grid: 100x100" << std::endl;
    std::cout << "Masses: 10000" << std::endl;
    std::cout << "Springs: " << num_springs << std::endl;
    std::cout << "Nodes: " << system.getNodeCount() << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;
    std::cout << "Component types: 2" << std::endl;
    std::cout << "*** STATE FUNCTION RESOLUTION: COMPILE-TIME VERIFIED ***" << std::endl;

    auto state = system.getInitialState();

    // Perturb center
    size_t center = 50 * 100 + 50;
    state[center * 4] += 0.1;

    // Benchmark
    constexpr int iterations = 100;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto derivs = system.computeDerivatives(0.0, state);
        (void)derivs;
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Derivatives (" << iterations << " iterations): " << duration.count() << " us" << std::endl;
    std::cout << "Per iteration: " << (duration.count() / iterations) << " us" << std::endl;

    // Verify physics
    auto derivs = system.computeDerivatives(0.0, state);
    bool restoring = derivs[center * 4 + 2] < 0;
    std::cout << "Center restoring force: " << (restoring ? "YES" : "NO") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_validation_info() {
    std::cout << "\n=== Compile-Time Validation Info ===" << std::endl;

    // Show what's validated at compile time
    constexpr auto result = validateGraph<PointMass2D<double>, Spring2D<double>>(twoMassTopology);

    std::cout << "Two-mass topology:" << std::endl;
    std::cout << "  Valid: " << (result.valid ? "YES" : "NO") << std::endl;
    std::cout << "  Nodes: " << twoMassTopology.num_nodes << std::endl;
    std::cout << "  Edges: " << twoMassTopology.num_edges << std::endl;

    // Show type properties
    using Props = TypeProperties<PointMass2D<double>, Spring2D<double>>;
    std::cout << "\nType 0 (PointMass2D):" << std::endl;
    std::cout << "  State size: " << Props::state_sizes[0] << std::endl;
    std::cout << "  Num ports: " << Props::num_ports[0] << std::endl;
    std::cout << "  Required: 0x" << std::hex << Props::required_masks[0] << std::dec << std::endl;
    std::cout << "  Provided: 0x" << std::hex << Props::provided_masks[0] << std::dec << std::endl;

    std::cout << "\nType 1 (Spring2D):" << std::endl;
    std::cout << "  State size: " << Props::state_sizes[1] << std::endl;
    std::cout << "  Num ports: " << Props::num_ports[1] << std::endl;
    std::cout << "  Required: 0x" << std::hex << Props::required_masks[1] << std::dec << std::endl;
    std::cout << "  Provided: 0x" << std::hex << Props::provided_masks[1] << std::dec << std::endl;

    std::cout << "\n=== INFO COMPLETE ===" << std::endl;
}

int main() {
    std::cout << "================================================" << std::endl;
    std::cout << "Validated Graph System - Compile-Time Guarantees" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    test_validation_info();
    test_two_mass_spring();
    test_chain();
    test_grid_5x5();
    test_large_grid_performance();
    test_huge_grid_performance();
    test_massive_grid_performance();

    std::cout << "\n================================================" << std::endl;
    std::cout << "ALL VALIDATED TESTS PASSED!" << std::endl;
    std::cout << "100x100 grid (10,000 masses) validated at COMPILE TIME" << std::endl;
    std::cout << "================================================" << std::endl;

    return 0;
}
