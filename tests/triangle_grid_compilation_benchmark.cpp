#include "physics/connected_masses/connectivity_matrix_2d.hpp"
#include <iostream>

using namespace sopot;
using namespace sopot::connected_masses;

// Test different grid sizes to measure compilation time
// Change GRID_SIZE macro to test different sizes

#ifndef GRID_ROWS
#define GRID_ROWS 5
#endif

#ifndef GRID_COLS
#define GRID_COLS 5
#endif

int main() {
    constexpr size_t Rows = GRID_ROWS;
    constexpr size_t Cols = GRID_COLS;

    std::cout << "Testing triangular grid compilation: " << Rows << "x" << Cols << "\n";
    std::cout << "Total masses: " << (Rows * Cols) << "\n";

    // Create the triangular grid system
    auto system = makeTriangularGridSystem<double, Rows, Cols>(
        1.0,    // mass (kg)
        1.0,    // spacing (m)
        10.0,   // stiffness (N/m)
        0.5     // damping (N·s/m)
    );

    std::cout << "State dimension: " << system.getStateDimension() << "\n";

    // Calculate edge count
    constexpr size_t horizontal_edges = Rows * (Cols - 1);
    constexpr size_t vertical_edges = (Rows - 1) * Cols;
    constexpr size_t diagonal_edges = 2 * (Rows - 1) * (Cols - 1);
    constexpr size_t total_edges = horizontal_edges + vertical_edges + diagonal_edges;

    std::cout << "Total edges (springs): " << total_edges << "\n";
    std::cout << "  Horizontal: " << horizontal_edges << "\n";
    std::cout << "  Vertical: " << vertical_edges << "\n";
    std::cout << "  Diagonal: " << diagonal_edges << "\n";

    // Quick simulation step to verify it works
    auto state = system.getInitialState();
    auto derivs = system.computeDerivatives(0.0, state);

    std::cout << "\n✓ Compilation and instantiation successful!\n";

    return 0;
}
