#pragma once

#include <array>
#include <vector>
#include <utility>
#include <stdexcept>

namespace sopot::connected_masses {

/**
 * @brief Generate edge list for a 2D rectangular grid with specified connectivity
 *
 * This function creates the connectivity pattern for a 2D grid of masses where:
 * - Each interior mass connects to its 4 neighbors (up, down, left, right)
 * - Edge masses connect to 3 neighbors
 * - Corner masses connect to 2 neighbors
 *
 * The grid is indexed row-major: index = row * cols + col
 *
 * Example 3x3 grid:
 * 0 -- 1 -- 2
 * |    |    |
 * 3 -- 4 -- 5
 * |    |    |
 * 6 -- 7 -- 8
 *
 * @param rows Number of rows in the grid (must be >= 2)
 * @param cols Number of columns in the grid (must be >= 2)
 * @param include_diagonals If true, also include diagonal connections
 * @return Vector of edge pairs (i, j) where i < j
 * @throws std::invalid_argument if rows or cols < 2
 */
[[nodiscard]] inline std::vector<std::pair<size_t, size_t>>
makeGrid2DEdges(size_t rows, size_t cols, bool include_diagonals = false) {
    if (rows < 2) {
        throw std::invalid_argument("Grid must have at least 2 rows (got " +
                                    std::to_string(rows) + ")");
    }
    if (cols < 2) {
        throw std::invalid_argument("Grid must have at least 2 columns (got " +
                                    std::to_string(cols) + ")");
    }

    std::vector<std::pair<size_t, size_t>> edges;

    auto index = [cols](size_t row, size_t col) -> size_t {
        return row * cols + col;
    };

    // Horizontal edges (connect adjacent columns in same row)
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols - 1; ++c) {
            edges.push_back({index(r, c), index(r, c + 1)});
        }
    }

    // Vertical edges (connect adjacent rows in same column)
    for (size_t r = 0; r < rows - 1; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            edges.push_back({index(r, c), index(r + 1, c)});
        }
    }

    // Diagonal edges (if requested)
    if (include_diagonals) {
        // Main diagonals (top-left to bottom-right)
        for (size_t r = 0; r < rows - 1; ++r) {
            for (size_t c = 0; c < cols - 1; ++c) {
                edges.push_back({index(r, c), index(r + 1, c + 1)});
            }
        }

        // Anti-diagonals (top-right to bottom-left)
        for (size_t r = 0; r < rows - 1; ++r) {
            for (size_t c = 1; c < cols; ++c) {
                edges.push_back({index(r, c), index(r + 1, c - 1)});
            }
        }
    }

    return edges;
}

/**
 * @brief Compile-time version for creating constexpr edge arrays
 *
 * Usage:
 * @code
 * constexpr auto edges = makeGrid2DEdgesArray<3, 3>();
 * @endcode
 */
template<size_t Rows, size_t Cols, bool IncludeDiagonals = false>
constexpr auto makeGrid2DEdgesArray() {
    static_assert(Rows >= 2, "Grid must have at least 2 rows");
    static_assert(Cols >= 2, "Grid must have at least 2 columns");

    constexpr auto index = [](size_t row, size_t col) -> size_t {
        return row * Cols + col;
    };

    // Count edges
    constexpr size_t horizontal_edges = Rows * (Cols - 1);
    constexpr size_t vertical_edges = (Rows - 1) * Cols;
    constexpr size_t diagonal_edges = IncludeDiagonals ?
        (2 * (Rows - 1) * (Cols - 1)) : 0;
    constexpr size_t total_edges = horizontal_edges + vertical_edges + diagonal_edges;

    std::array<std::pair<size_t, size_t>, total_edges> edges{};
    size_t edge_idx = 0;

    // Horizontal edges
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < Cols - 1; ++c) {
            edges[edge_idx++] = {index(r, c), index(r, c + 1)};
        }
    }

    // Vertical edges
    for (size_t r = 0; r < Rows - 1; ++r) {
        for (size_t c = 0; c < Cols; ++c) {
            edges[edge_idx++] = {index(r, c), index(r + 1, c)};
        }
    }

    // Diagonal edges
    if constexpr (IncludeDiagonals) {
        // Main diagonals
        for (size_t r = 0; r < Rows - 1; ++r) {
            for (size_t c = 0; c < Cols - 1; ++c) {
                edges[edge_idx++] = {index(r, c), index(r + 1, c + 1)};
            }
        }

        // Anti-diagonals
        for (size_t r = 0; r < Rows - 1; ++r) {
            for (size_t c = 1; c < Cols; ++c) {
                edges[edge_idx++] = {index(r, c), index(r + 1, c - 1)};
            }
        }
    }

    return edges;
}

/**
 * @brief Helper to convert grid (row, col) to linear index
 */
[[nodiscard]] constexpr size_t gridIndex(size_t row, size_t col, size_t cols) {
    return row * cols + col;
}

/**
 * @brief Helper to convert linear index to grid (row, col)
 */
[[nodiscard]] constexpr std::pair<size_t, size_t> gridCoords(size_t index, size_t cols) {
    return {index / cols, index % cols};
}

} // namespace sopot::connected_masses
