#pragma once

#include "indexed_point_mass.hpp"
#include "indexed_spring.hpp"
#include "core/typed_component.hpp"
#include <array>
#include <tuple>
#include <utility>
#include <stdexcept>
#include <set>

namespace sopot::connected_masses {

/**
 * @brief Parameters for a spring connection
 */
struct SpringParams {
    double stiffness;     // k (N/m)
    double rest_length;   // L0 (m)
    double damping;       // c (N·s/m)
};

/**
 * @brief Parameters for a mass
 */
struct MassParams {
    double mass;              // m (kg)
    double initial_position;  // x0 (m)
    double initial_velocity;  // v0 (m/s)
};

namespace detail {

/**
 * @brief Validate edge list for correctness
 *
 * Checks for:
 * - Self-loops (edges connecting a node to itself)
 * - Out-of-range indices (indices >= NumMasses)
 * - Duplicate edges
 *
 * @tparam NumMasses Number of masses in the system
 * @tparam NumEdges Number of edges
 * @param edges Edge list to validate
 * @throws std::invalid_argument if validation fails
 */
template<size_t NumMasses, size_t NumEdges>
constexpr void validateEdges(const std::array<std::pair<size_t, size_t>, NumEdges>& edges) {
    for (size_t k = 0; k < NumEdges; ++k) {
        const auto& edge = edges[k];
        size_t i = edge.first;
        size_t j = edge.second;

        // Check for self-loops
        if (i == j) {
            throw std::invalid_argument(
                "Self-loop detected: edge " + std::to_string(k) + " connects node " +
                std::to_string(i) + " to itself"
            );
        }

        // Check for out-of-range indices
        if (i >= NumMasses) {
            throw std::invalid_argument(
                "Edge " + std::to_string(k) + " has out-of-range first index: " +
                std::to_string(i) + " (must be < " + std::to_string(NumMasses) + ")"
            );
        }
        if (j >= NumMasses) {
            throw std::invalid_argument(
                "Edge " + std::to_string(k) + " has out-of-range second index: " +
                std::to_string(j) + " (must be < " + std::to_string(NumMasses) + ")"
            );
        }

        // Check for duplicate edges (both (i,j) and (j,i) representations)
        for (size_t m = k + 1; m < NumEdges; ++m) {
            const auto& other_edge = edges[m];
            size_t mi = other_edge.first;
            size_t mj = other_edge.second;

            if ((i == mi && j == mj) || (i == mj && j == mi)) {
                throw std::invalid_argument(
                    "Duplicate edge detected: edges " + std::to_string(k) + " and " +
                    std::to_string(m) + " both connect nodes " + std::to_string(i) +
                    " and " + std::to_string(j)
                );
            }
        }
    }
}

// Helper: Create a mass with given index
template<size_t Index, typename T>
auto makeMass(const MassParams& params) {
    return IndexedPointMass<Index, T>(
        params.mass,
        params.initial_position,
        params.initial_velocity
    );
}

// Helper: Create a spring connecting two specific indices
template<size_t I, size_t J, typename T>
auto makeSpring(const SpringParams& params) {
    return IndexedSpring<I, J, T>(
        params.stiffness,
        params.rest_length,
        params.damping
    );
}

// Generate tuple of masses for indices 0, 1, 2, ..., NumMasses-1
template<typename T, size_t... Indices>
auto makeMassTuple(
    const std::array<MassParams, sizeof...(Indices)>& mass_params,
    std::index_sequence<Indices...>
) {
    return std::make_tuple(makeMass<Indices, T>(mass_params[Indices])...);
}

// This is the tricky part: we need to create springs with compile-time indices
// extracted from runtime edge array. We use a constexpr context to make this work.

// Helper to get Kth edge's first index at compile time
template<size_t K, size_t NumEdges>
struct EdgeIndexExtractor {
    static constexpr size_t getFirst(const std::array<std::pair<size_t, size_t>, NumEdges>& edges) {
        return edges[K].first;
    }

    static constexpr size_t getSecond(const std::array<std::pair<size_t, size_t>, NumEdges>& edges) {
        return edges[K].second;
    }
};

// Generate springs from edge template parameter
template<typename T, auto Edges>
struct SpringTupleMaker {
    static constexpr size_t NumEdges = Edges.size();

    // Create spring for Kth edge with runtime parameters
    template<size_t K>
    static auto makeSpringForEdge(const std::array<SpringParams, NumEdges>& spring_params) {
        constexpr size_t I = Edges[K].first;
        constexpr size_t J = Edges[K].second;
        return IndexedSpring<I, J, T>(
            spring_params[K].stiffness,
            spring_params[K].rest_length,
            spring_params[K].damping
        );
    }

    // Generate tuple of springs
    template<size_t... Ks>
    static auto makeTuple(
        const std::array<SpringParams, NumEdges>& spring_params,
        std::index_sequence<Ks...>
    ) {
        return std::make_tuple(makeSpringForEdge<Ks>(spring_params)...);
    }
};

} // namespace detail

/**
 * @brief Create a connected mass-spring system from edge list
 *
 * This function generates all necessary components at compile time based on
 * the specified connectivity.
 *
 * Example:
 * @code
 * // Three masses in a triangular configuration
 * constexpr auto edges = std::array{
 *     std::pair{size_t(0), size_t(1)},  // Mass 0 <-> Mass 1
 *     std::pair{size_t(1), size_t(2)},  // Mass 1 <-> Mass 2
 *     std::pair{size_t(0), size_t(2)}   // Mass 0 <-> Mass 2
 * };
 *
 * auto system = makeConnectedMassSystem<double, 3, edges>(
 *     // Mass parameters: {{mass, initial_position, initial_velocity}, ...}
 *     {{{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 0.5, 0.0}}},
 *     // Spring parameters: {{stiffness, rest_length, damping}, ...}
 *     {{{10.0, 1.0, 0.5}, {10.0, 1.0, 0.5}, {10.0, 1.0, 0.5}}}
 * );
 * @endcode
 *
 * @tparam T Scalar type (double or Dual)
 * @tparam NumMasses Number of masses
 * @tparam Edges Compile-time edge list (array of pairs)
 * @param mass_params Parameters for each mass
 * @param spring_params Parameters for each spring (same order as edges)
 */
template<
    typename T,
    size_t NumMasses,
    auto Edges  // C++20: non-type template parameter of structural type
>
auto makeConnectedMassSystem(
    const std::array<MassParams, NumMasses>& mass_params,
    const std::array<SpringParams, Edges.size()>& spring_params
) {
    // Validate edge list at compile time
    static_assert(Edges.size() > 0, "Must have at least one edge");

    // Validate edges for self-loops, out-of-range indices, and duplicates
    detail::validateEdges<NumMasses>(Edges);

    // Generate mass components
    auto mass_tuple = detail::makeMassTuple<T>(
        mass_params,
        std::make_index_sequence<NumMasses>{}
    );

    // Generate spring components using the edge template parameter
    auto spring_tuple = detail::SpringTupleMaker<T, Edges>::makeTuple(
        spring_params,
        std::make_index_sequence<Edges.size()>{}
    );

    // Combine into single tuple
    auto all_components = std::tuple_cat(std::move(mass_tuple), std::move(spring_tuple));

    // Create TypedODESystem from tuple
    return std::apply([](auto&&... comps) {
        return makeTypedODESystem<T>(std::move(comps)...);
    }, std::move(all_components));
}

/**
 * @brief Simplified version with uniform mass and spring parameters
 *
 * Useful for quick prototyping.
 *
 * @param uniform_mass Mass value for all masses
 * @param uniform_spring Spring parameters for all connections
 */
template<typename T, size_t NumMasses, auto Edges>
auto makeUniformConnectedSystem(
    double uniform_mass,
    const SpringParams& uniform_spring
) {
    std::array<MassParams, NumMasses> masses;
    for (size_t i = 0; i < NumMasses; ++i) {
        masses[i] = {uniform_mass, 0.0, 0.0};
    }

    std::array<SpringParams, Edges.size()> springs;
    for (size_t i = 0; i < Edges.size(); ++i) {
        springs[i] = uniform_spring;
    }

    return makeConnectedMassSystem<T, NumMasses, Edges>(masses, springs);
}

/**
 * @brief Convert adjacency matrix to edge list (runtime helper)
 *
 * Takes a symmetric adjacency matrix and extracts the upper triangular edges.
 * For undirected graphs, the matrix must be symmetric: matrix[i][j] == matrix[j][i].
 * This function validates symmetry and throws if the matrix is asymmetric.
 *
 * @tparam N Matrix dimension
 * @param matrix Adjacency matrix (true = connected) - must be symmetric for undirected graphs
 * @return std::vector of edges from upper triangle
 * @throws std::invalid_argument if matrix is asymmetric (has diagonal elements set)
 */
template<size_t N>
inline auto matrixToEdges(const bool (&matrix)[N][N]) {
    // Validate matrix symmetry (for undirected graphs)
    for (size_t i = 0; i < N; ++i) {
        // Check diagonal (self-loops not allowed)
        if (matrix[i][i]) {
            throw std::invalid_argument(
                "Self-loop detected in adjacency matrix at diagonal element [" +
                std::to_string(i) + "][" + std::to_string(i) + "]"
            );
        }

        // Check symmetry
        for (size_t j = i + 1; j < N; ++j) {
            if (matrix[i][j] != matrix[j][i]) {
                throw std::invalid_argument(
                    "Adjacency matrix is not symmetric: matrix[" + std::to_string(i) +
                    "][" + std::to_string(j) + "] != matrix[" + std::to_string(j) +
                    "][" + std::to_string(i) + "]. For undirected graphs, the matrix must be symmetric."
                );
            }
        }
    }

    // Count edges in upper triangle
    size_t count = 0;
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
            if (matrix[i][j]) {
                ++count;
            }
        }
    }

    // Extract edges
    std::vector<std::pair<size_t, size_t>> edges;
    edges.reserve(count);
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
            if (matrix[i][j]) {
                edges.push_back({i, j});
            }
        }
    }

    return edges;
}

} // namespace sopot::connected_masses
