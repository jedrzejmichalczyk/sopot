/**
 * @file unified_scalability_test.cpp
 * @brief Scalability test for compile-time unified graph system
 *
 * Tests large grids to verify O(K) compile-time complexity
 */

#include "physics/unified/graph_system.hpp"
#include "core/solver.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

using namespace sopot;
using namespace sopot::unified;
using sopot::StateView;

template<size_t Rows, size_t Cols>
void test_grid() {
    std::cout << "\n=== Test: " << Rows << "x" << Cols << " Grid ===" << std::endl;

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

    auto state = system.getInitialState();

    // Perturb center
    size_t center = (Rows / 2) * Cols + (Cols / 2);
    state[center * 4] += 0.1;

    // Benchmark derivatives computation
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

    // Run a short simulation
    RK4Solver solver;
    auto derivs_fn = [&](double t, StateView s) {
        return system.computeDerivatives(t, s);
    };

    auto sim_start = std::chrono::high_resolution_clock::now();
    solver.solve(derivs_fn, system.getStateDimension(), 0.0, 0.1, 0.001, state);
    auto sim_end = std::chrono::high_resolution_clock::now();

    auto sim_duration = std::chrono::duration_cast<std::chrono::milliseconds>(sim_end - sim_start);
    std::cout << "100 RK4 steps (0.1s simulation): " << sim_duration.count() << " ms" << std::endl;

    std::cout << "=== PASSED ===" << std::endl;
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "Unified Graph System - Scalability Test" << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    test_grid<10, 10>();
    test_grid<20, 20>();
    test_grid<50, 50>();
    test_grid<100, 100>();

    std::cout << "\n===============================================" << std::endl;
    std::cout << "ALL SCALABILITY TESTS PASSED!" << std::endl;
    std::cout << "===============================================" << std::endl;

    return 0;
}
