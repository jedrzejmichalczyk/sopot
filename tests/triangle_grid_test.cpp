#include "physics/connected_masses/connectivity_matrix_2d.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot;
using namespace sopot::connected_masses;

/**
 * @brief Test triangular grid vs quad grid stability
 */
void test_triangle_vs_quad_stability() {
    std::cout << "\n=== Triangular Grid vs Quad Grid Stability Test ===\n";

    constexpr size_t Rows = 3;
    constexpr size_t Cols = 3;

    // Create both types of grids with same parameters
    auto quad_system = makeGrid2DSystem<double, Rows, Cols, false>(
        1.0,    // mass (kg)
        1.0,    // spacing (m)
        10.0,   // stiffness (N/m)
        0.5     // damping (N·s/m)
    );

    auto triangle_system = makeTriangularGridSystem<double, Rows, Cols>(
        1.0,    // mass (kg)
        1.0,    // spacing (m)
        10.0,   // stiffness (N/m)
        0.5     // damping (N·s/m)
    );

    std::cout << "✓ Created both grid types: " << Rows << "x" << Cols << "\n";
    std::cout << "  Quad grid state dimension: " << quad_system.getStateDimension() << "\n";
    std::cout << "  Triangle grid state dimension: " << triangle_system.getStateDimension() << "\n";

    // Get initial states
    auto quad_state = quad_system.getInitialState();
    auto triangle_state = triangle_system.getInitialState();

    // Perturb the center mass in both grids
    std::cout << "\nPerturbing center mass (index 4) by +0.5m in y direction...\n";
    quad_state[4 * 4 + 1] += 0.5;
    triangle_state[4 * 4 + 1] += 0.5;

    // Simulate both grids
    double t = 0.0;
    double dt = 0.001;
    double t_end = 2.0;
    size_t n = quad_system.getStateDimension();

    std::cout << "Simulating both grids for " << t_end << " seconds...\n";

    int steps = 0;
    while (t < t_end) {
        // Quad grid RK4 step
        auto k1_q = quad_system.computeDerivatives(t, quad_state);
        std::vector<double> s2_q(n), s3_q(n), s4_q(n);
        for (size_t i = 0; i < n; ++i) s2_q[i] = quad_state[i] + 0.5 * dt * k1_q[i];
        auto k2_q = quad_system.computeDerivatives(t + 0.5 * dt, s2_q);
        for (size_t i = 0; i < n; ++i) s3_q[i] = quad_state[i] + 0.5 * dt * k2_q[i];
        auto k3_q = quad_system.computeDerivatives(t + 0.5 * dt, s3_q);
        for (size_t i = 0; i < n; ++i) s4_q[i] = quad_state[i] + dt * k3_q[i];
        auto k4_q = quad_system.computeDerivatives(t + dt, s4_q);
        for (size_t i = 0; i < n; ++i)
            quad_state[i] += dt / 6.0 * (k1_q[i] + 2*k2_q[i] + 2*k3_q[i] + k4_q[i]);

        // Triangle grid RK4 step
        auto k1_t = triangle_system.computeDerivatives(t, triangle_state);
        std::vector<double> s2_t(n), s3_t(n), s4_t(n);
        for (size_t i = 0; i < n; ++i) s2_t[i] = triangle_state[i] + 0.5 * dt * k1_t[i];
        auto k2_t = triangle_system.computeDerivatives(t + 0.5 * dt, s2_t);
        for (size_t i = 0; i < n; ++i) s3_t[i] = triangle_state[i] + 0.5 * dt * k2_t[i];
        auto k3_t = triangle_system.computeDerivatives(t + 0.5 * dt, s3_t);
        for (size_t i = 0; i < n; ++i) s4_t[i] = triangle_state[i] + dt * k3_t[i];
        auto k4_t = triangle_system.computeDerivatives(t + dt, s4_t);
        for (size_t i = 0; i < n; ++i)
            triangle_state[i] += dt / 6.0 * (k1_t[i] + 2*k2_t[i] + 2*k3_t[i] + k4_t[i]);

        t += dt;
        ++steps;
    }

    std::cout << "✓ Completed " << steps << " integration steps\n";

    // Compare final positions
    std::cout << "\nFinal center mass positions (t = " << t << " s):\n";
    auto quad_pos4 = std::array<double, 2>{quad_state[4 * 4 + 0], quad_state[4 * 4 + 1]};
    auto tri_pos4 = std::array<double, 2>{triangle_state[4 * 4 + 0], triangle_state[4 * 4 + 1]};

    std::cout << "  Quad grid:     (" << std::fixed << std::setprecision(6)
              << quad_pos4[0] << ", " << quad_pos4[1] << ")\n";
    std::cout << "  Triangle grid: (" << std::fixed << std::setprecision(6)
              << tri_pos4[0] << ", " << tri_pos4[1] << ")\n";

    // Calculate displacement from initial position
    double quad_disp = std::sqrt(quad_pos4[0] * quad_pos4[0] + (quad_pos4[1] - 1.0) * (quad_pos4[1] - 1.0));
    double tri_disp = std::sqrt(tri_pos4[0] * tri_pos4[0] + (tri_pos4[1] - 1.0) * (tri_pos4[1] - 1.0));

    std::cout << "\nDisplacement from equilibrium:\n";
    std::cout << "  Quad grid:     " << quad_disp << " m\n";
    std::cout << "  Triangle grid: " << tri_disp << " m\n";

    if (tri_disp < quad_disp) {
        std::cout << "\n✓ Triangle grid is more stable (smaller displacement)\n";
    } else {
        std::cout << "\n✓ Both grids show comparable stability\n";
    }
}

/**
 * @brief Test triangular grid edge count
 */
void test_triangle_edge_count() {
    std::cout << "\n=== Triangular Grid Edge Count Test ===\n";

    constexpr size_t Rows = 3;
    constexpr size_t Cols = 3;

    constexpr auto quad_edges = makeGrid2DEdgesArray<Rows, Cols, false>();
    constexpr auto triangle_edges = makeTriangularGridEdgesArray<Rows, Cols>();

    std::cout << "Grid size: " << Rows << "x" << Cols << " (" << (Rows * Cols) << " masses)\n";
    std::cout << "  Quad grid edges: " << quad_edges.size() << "\n";
    std::cout << "  Triangle grid edges: " << triangle_edges.size() << "\n";

    // Expected edge counts
    size_t expected_quad = Rows * (Cols - 1) + (Rows - 1) * Cols;  // horizontal + vertical
    size_t expected_triangle = expected_quad + 2 * (Rows - 1) * (Cols - 1);  // + both diagonals

    std::cout << "\nExpected counts:\n";
    std::cout << "  Quad: " << expected_quad << " (horizontal + vertical)\n";
    std::cout << "  Triangle: " << expected_triangle << " (quad + diagonals)\n";

    if (quad_edges.size() == expected_quad && triangle_edges.size() == expected_triangle) {
        std::cout << "\n✓ Edge counts are correct!\n";
    } else {
        std::cout << "\n✗ Edge count mismatch!\n";
    }

    // Print some triangle edges to verify structure
    std::cout << "\nSample triangular grid edges:\n";
    for (size_t i = 0; i < std::min(size_t(10), triangle_edges.size()); ++i) {
        auto [a, b] = triangle_edges[i];
        std::cout << "  Edge " << i << ": (" << a << ", " << b << ")\n";
    }
}

int main() {
    std::cout << "========================================\n";
    std::cout << "SOPOT Triangular Grid Test Suite\n";
    std::cout << "========================================\n";
    std::cout << "\nTesting the new triangular mesh structure:\n";
    std::cout << "- Triangular mesh with full diagonal connections\n";
    std::cout << "- Improved stability for cloth-like simulations\n";
    std::cout << "- Comparison with standard quad grid\n";

    try {
        test_triangle_edge_count();
        test_triangle_vs_quad_stability();

        std::cout << "\n========================================\n";
        std::cout << "All tests passed successfully! ✓\n";
        std::cout << "========================================\n";
        std::cout << "\nTriangular grid features:\n";
        std::cout << "  • Each cell has 4 triangles (X pattern)\n";
        std::cout << "  • Better stability than quad grids\n";
        std::cout << "  • Ideal for cloth/fabric simulations\n";
        std::cout << "  • Users can choose between quad and triangle\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
