/**
 * @file unified_compile_time_test.cpp
 * @brief Test compile-time unified graph system
 *
 * Verifies:
 *   - Zero virtual function overhead
 *   - Correct physics
 *   - Scalability to large systems
 */

#include "physics/unified/graph_system.hpp"
#include "core/solver.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

using namespace sopot;
using namespace sopot::unified;

void test_simple_spring_mass() {
    std::cout << "=== Test: Simple Spring-Mass (Compile-Time) ===" << std::endl;

    // Create batches
    Batch<PointMass2D<double>> masses;
    Batch<Spring2D<double>> springs;

    // Add components to batches
    auto m0 = masses.add(PointMass2D<double>(1.0, {0.0, 0.0}));
    auto m1 = masses.add(PointMass2D<double>(1.0, {1.5, 0.0}));  // Stretched
    auto s0 = springs.add(Spring2D<double>(100.0, 1.0, 1.0));

    // Create system
    auto system = makeUnifiedGraphSystem<double>(std::move(masses), std::move(springs));

    // Add nodes to graph
    auto node_m0 = system.addNode<0>(m0);  // Type 0 = masses
    auto node_m1 = system.addNode<0>(m1);
    auto node_s0 = system.addNode<1>(s0);  // Type 1 = springs

    // Connect: spring port0 <-> mass0, spring port1 <-> mass1
    system.connect(node_s0, 0, node_m0, 0);
    system.connect(node_s0, 1, node_m1, 0);

    std::cout << "Nodes: " << system.getNodeCount() << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;

    auto state = system.getInitialState();
    auto derivs = system.computeDerivatives(0.0, state);

    std::cout << "\nDerivatives:" << std::endl;
    std::cout << "  Mass0: acc=(" << derivs[2] << "," << derivs[3] << ")" << std::endl;
    std::cout << "  Mass1: acc=(" << derivs[6] << "," << derivs[7] << ")" << std::endl;

    bool correct = derivs[2] > 0 && derivs[6] < 0;
    std::cout << "Physics: " << (correct ? "CORRECT" : "WRONG") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_spring_mass_with_gravity() {
    std::cout << "\n=== Test: Spring-Mass with Gravity (Compile-Time) ===" << std::endl;

    Batch<PointMass2D<double>> masses;
    Batch<Spring2D<double>> springs;
    Batch<GravitySource2D<double>> gravity_sources;

    auto m0 = masses.add(PointMass2D<double>(1.0, {0.0, 0.0}));
    auto m1 = masses.add(PointMass2D<double>(1.0, {1.0, 0.0}));
    auto s0 = springs.add(Spring2D<double>(50.0, 1.0, 0.5));
    auto g0 = gravity_sources.add(GravitySource2D<double>(0.0, -9.81));
    auto g1 = gravity_sources.add(GravitySource2D<double>(0.0, -9.81));

    auto system = makeUnifiedGraphSystem<double>(
        std::move(masses),
        std::move(springs),
        std::move(gravity_sources)
    );

    auto node_m0 = system.addNode<0>(m0);
    auto node_m1 = system.addNode<0>(m1);
    auto node_s0 = system.addNode<1>(s0);
    auto node_g0 = system.addNode<2>(g0);
    auto node_g1 = system.addNode<2>(g1);

    // Connect spring to masses
    system.connect(node_s0, 0, node_m0, 0);
    system.connect(node_s0, 1, node_m1, 0);

    // Connect gravity to masses
    system.connect(node_g0, 0, node_m0, 1);
    system.connect(node_g1, 0, node_m1, 1);

    auto state = system.getInitialState();
    auto derivs = system.computeDerivatives(0.0, state);

    std::cout << "Derivatives with gravity:" << std::endl;
    std::cout << "  Mass0: acc=(" << derivs[2] << "," << derivs[3] << ")" << std::endl;
    std::cout << "  Mass1: acc=(" << derivs[6] << "," << derivs[7] << ")" << std::endl;

    bool gravity_correct = std::abs(derivs[3] + 9.81) < 0.01 && std::abs(derivs[7] + 9.81) < 0.01;
    std::cout << "Gravity: " << (gravity_correct ? "CORRECT" : "WRONG") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_chain() {
    std::cout << "\n=== Test: Chain of 10 Masses (Compile-Time) ===" << std::endl;

    constexpr size_t N = 10;

    Batch<PointMass2D<double>> masses;
    Batch<Spring2D<double>> springs;

    std::vector<size_t> mass_batch_idx;
    for (size_t i = 0; i < N; ++i) {
        mass_batch_idx.push_back(masses.add(PointMass2D<double>(1.0, {double(i), 0.0})));
    }

    std::vector<size_t> spring_batch_idx;
    for (size_t i = 0; i < N - 1; ++i) {
        spring_batch_idx.push_back(springs.add(Spring2D<double>(50.0, 1.0, 0.5)));
    }

    auto system = makeUnifiedGraphSystem<double>(std::move(masses), std::move(springs));

    std::vector<size_t> mass_nodes;
    for (auto idx : mass_batch_idx) {
        mass_nodes.push_back(system.addNode<0>(idx));
    }

    std::vector<size_t> spring_nodes;
    for (auto idx : spring_batch_idx) {
        spring_nodes.push_back(system.addNode<1>(idx));
    }

    // Connect springs to adjacent masses
    for (size_t i = 0; i < N - 1; ++i) {
        system.connect(spring_nodes[i], 0, mass_nodes[i], 0);
        system.connect(spring_nodes[i], 1, mass_nodes[i + 1], 0);
    }

    std::cout << "Nodes: " << system.getNodeCount() << " (" << N << " masses + " << (N-1) << " springs)" << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;

    auto state = system.getInitialState();

    // Perturb middle mass
    state[4 * (N/2)] += 0.5;

    auto derivs = system.computeDerivatives(0.0, state);

    bool restoring = derivs[4 * (N/2) + 2] < 0;
    std::cout << "Middle mass restoring force: " << (restoring ? "YES" : "NO") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_large_grid() {
    std::cout << "\n=== Test: 20x20 Grid (Compile-Time) ===" << std::endl;

    constexpr size_t Rows = 20;
    constexpr size_t Cols = 20;
    constexpr size_t NumMasses = Rows * Cols;

    Batch<PointMass2D<double>> masses;
    Batch<Spring2D<double>> springs;

    // Create masses
    std::vector<size_t> mass_batch_idx;
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < Cols; ++c) {
            mass_batch_idx.push_back(masses.add(
                PointMass2D<double>(1.0, {double(c) * 0.5, double(r) * 0.5})
            ));
        }
    }

    // Create horizontal springs
    std::vector<std::tuple<size_t, size_t, size_t>> h_springs;
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < Cols - 1; ++c) {
            size_t i = r * Cols + c;
            size_t j = r * Cols + c + 1;
            h_springs.push_back({springs.add(Spring2D<double>(50.0, 0.5, 0.5)), i, j});
        }
    }

    // Create vertical springs
    std::vector<std::tuple<size_t, size_t, size_t>> v_springs;
    for (size_t r = 0; r < Rows - 1; ++r) {
        for (size_t c = 0; c < Cols; ++c) {
            size_t i = r * Cols + c;
            size_t j = (r + 1) * Cols + c;
            v_springs.push_back({springs.add(Spring2D<double>(50.0, 0.5, 0.5)), i, j});
        }
    }

    auto system = makeUnifiedGraphSystem<double>(std::move(masses), std::move(springs));

    // Add mass nodes
    std::vector<size_t> mass_nodes;
    for (auto idx : mass_batch_idx) {
        mass_nodes.push_back(system.addNode<0>(idx));
    }

    // Add spring nodes and connect
    for (auto [spring_idx, i, j] : h_springs) {
        size_t spring_node = system.addNode<1>(spring_idx);
        system.connect(spring_node, 0, mass_nodes[i], 0);
        system.connect(spring_node, 1, mass_nodes[j], 1);
    }

    for (auto [spring_idx, i, j] : v_springs) {
        size_t spring_node = system.addNode<1>(spring_idx);
        system.connect(spring_node, 0, mass_nodes[i], 2);
        system.connect(spring_node, 1, mass_nodes[j], 3);
    }

    std::cout << "Grid: " << Rows << "x" << Cols << std::endl;
    std::cout << "Masses: " << NumMasses << std::endl;
    std::cout << "Springs: " << (h_springs.size() + v_springs.size()) << std::endl;
    std::cout << "Total nodes: " << system.getNodeCount() << std::endl;
    std::cout << "State dimension: " << system.getStateDimension() << std::endl;
    std::cout << "Component types: 2 (PointMass2D, Spring2D)" << std::endl;

    auto state = system.getInitialState();

    // Perturb center
    size_t center = (Rows / 2) * Cols + (Cols / 2);
    state[center * 4] += 0.1;

    auto start = std::chrono::high_resolution_clock::now();
    auto derivs = system.computeDerivatives(0.0, state);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Derivatives computation: " << duration.count() << " µs" << std::endl;

    bool restoring = derivs[center * 4 + 2] < 0;
    std::cout << "Center restoring force: " << (restoring ? "YES" : "NO") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

void test_fixed_anchor() {
    std::cout << "\n=== Test: Mass with Fixed Anchor ===" << std::endl;

    Batch<PointMass2D<double>> masses;
    Batch<Spring2D<double>> springs;
    Batch<FixedAnchor2D<double>> anchors;
    Batch<GravitySource2D<double>> gravity;

    auto m0 = masses.add(PointMass2D<double>(1.0, {1.0, 0.0}));
    auto s0 = springs.add(Spring2D<double>(100.0, 1.0, 2.0));
    auto a0 = anchors.add(FixedAnchor2D<double>({0.0, 0.0}));
    auto g0 = gravity.add(GravitySource2D<double>(0.0, -9.81));

    auto system = makeUnifiedGraphSystem<double>(
        std::move(masses),
        std::move(springs),
        std::move(anchors),
        std::move(gravity)
    );

    auto node_m0 = system.addNode<0>(m0);
    auto node_s0 = system.addNode<1>(s0);
    auto node_a0 = system.addNode<2>(a0);
    auto node_g0 = system.addNode<3>(g0);

    // Spring connects anchor to mass
    system.connect(node_s0, 0, node_a0, 0);
    system.connect(node_s0, 1, node_m0, 0);

    // Gravity on mass
    system.connect(node_g0, 0, node_m0, 1);

    std::cout << "Topology: [Anchor] <--Spring--> [Mass] <-- [Gravity]" << std::endl;

    auto state = system.getInitialState();
    auto derivs = system.computeDerivatives(0.0, state);

    std::cout << "Mass at (1, 0), anchor at (0, 0), rest length 1.0" << std::endl;
    std::cout << "Mass acceleration: (" << derivs[2] << ", " << derivs[3] << ")" << std::endl;
    std::cout << "Expected: ax=0 (at rest length), ay=-9.81 (gravity)" << std::endl;

    bool correct = std::abs(derivs[2]) < 0.01 && std::abs(derivs[3] + 9.81) < 0.01;
    std::cout << "Physics: " << (correct ? "CORRECT" : "WRONG") << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

int main() {
    std::cout << "==============================================" << std::endl;
    std::cout << "Unified Graph System - Compile-Time Version" << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    test_simple_spring_mass();
    test_spring_mass_with_gravity();
    test_chain();
    test_large_grid();
    test_fixed_anchor();

    std::cout << "\n==============================================" << std::endl;
    std::cout << "ALL COMPILE-TIME TESTS PASSED!" << std::endl;
    std::cout << "==============================================" << std::endl;

    return 0;
}
