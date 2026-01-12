#pragma once

#include "indexed_point_mass.hpp"
#include "indexed_spring.hpp"
#include "core/typed_component.hpp"
#include <array>
#include <tuple>
#include <utility>

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
 *     std::pair{0uz, 1uz},  // Mass 0 <-> Mass 1
 *     std::pair{1uz, 2uz},  // Mass 1 <-> Mass 2
 *     std::pair{0uz, 2uz}   // Mass 0 <-> Mass 2
 * };
 *
 * auto system = makeConnectedMassSystem<double, 3, edges>(
 *     {{{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 0.5, 0.0}}},  // masses
 *     {{{10.0, 1.0, 0.5}, {10.0, 1.0, 0.5}, {10.0, 1.0, 0.5}}} // springs
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
 * This version returns a runtime array since we can't know the size at compile time
 * without evaluating the matrix content.
 *
 * @tparam N Matrix dimension
 * @tparam MaxEdges Maximum possible edges (N*(N-1)/2)
 * @param matrix Symmetric adjacency matrix (true = connected)
 * @return std::vector of edges
 */
template<size_t N>
inline auto matrixToEdges(const bool (&matrix)[N][N]) {
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
