#pragma once

#include "indexed_point_mass_2d.hpp"
#include "indexed_spring_2d.hpp"
#include "force_aggregator_2d.hpp"
#include "grid_2d.hpp"
#include "core/typed_component.hpp"
#include <array>
#include <tuple>
#include <utility>

namespace sopot::connected_masses {

/**
 * @brief Parameters for a 2D spring connection
 */
struct SpringParams2D {
    double stiffness;     // k (N/m)
    double rest_length;   // L0 (m)
    double damping;       // c (N·s/m)
};

/**
 * @brief Parameters for a 2D mass
 */
struct MassParams2D {
    double mass;          // m (kg)
    double initial_x;     // x0 (m)
    double initial_y;     // y0 (m)
    double initial_vx;    // vx0 (m/s)
    double initial_vy;    // vy0 (m/s)
};

namespace detail {

// Helper: Create a 2D mass with given index
template<size_t Index, typename T>
auto makeMass2D(const MassParams2D& params) {
    return IndexedPointMass2D<Index, T>(
        params.mass,
        params.initial_x,
        params.initial_y,
        params.initial_vx,
        params.initial_vy
    );
}

// Helper: Create a 2D spring connecting two specific indices
template<size_t I, size_t J, typename T>
auto makeSpring2D(const SpringParams2D& params) {
    return IndexedSpring2D<I, J, T>(
        params.stiffness,
        params.rest_length,
        params.damping
    );
}

// Generate tuple of 2D masses
template<typename T, size_t... Indices>
auto makeMassTuple2D(
    const std::array<MassParams2D, sizeof...(Indices)>& mass_params,
    std::index_sequence<Indices...>
) {
    return std::make_tuple(makeMass2D<Indices, T>(mass_params[Indices])...);
}

// Generate springs from edge template parameter
template<typename T, auto Edges>
struct SpringTupleMaker2D {
    static constexpr size_t NumEdges = Edges.size();

    template<size_t K>
    static auto makeSpringForEdge(const std::array<SpringParams2D, NumEdges>& spring_params) {
        constexpr size_t I = Edges[K].first;
        constexpr size_t J = Edges[K].second;
        return IndexedSpring2D<I, J, T>(
            spring_params[K].stiffness,
            spring_params[K].rest_length,
            spring_params[K].damping
        );
    }

    template<size_t... Ks>
    static auto makeTuple(
        const std::array<SpringParams2D, NumEdges>& spring_params,
        std::index_sequence<Ks...>
    ) {
        return std::make_tuple(makeSpringForEdge<Ks>(spring_params)...);
    }
};

} // namespace detail

/**
 * @brief Create a 2D connected mass-spring system from edge list
 *
 * Similar to makeConnectedMassSystem but for 2D masses and springs.
 *
 * @tparam T Scalar type (double or Dual)
 * @tparam NumMasses Number of masses
 * @tparam Edges Compile-time edge list (array of pairs)
 * @param mass_params Parameters for each mass
 * @param spring_params Parameters for each spring (same order as edges)
 */
template<typename T, size_t NumMasses, auto Edges>
auto makeConnectedMassSystem2D(
    const std::array<MassParams2D, NumMasses>& mass_params,
    const std::array<SpringParams2D, Edges.size()>& spring_params
) {
    static_assert(Edges.size() > 0, "Must have at least one edge");

    // Generate mass components
    auto mass_tuple = detail::makeMassTuple2D<T>(
        mass_params,
        std::make_index_sequence<NumMasses>{}
    );

    // Generate spring components
    auto spring_tuple = detail::SpringTupleMaker2D<T, Edges>::makeTuple(
        spring_params,
        std::make_index_sequence<Edges.size()>{}
    );

    // Generate force aggregators for each mass
    auto force_aggregator_tuple = detail::makeForceAggregatorsTuple<T, Edges>(
        std::make_index_sequence<NumMasses>{}
    );

    // Combine into single tuple
    auto all_components = std::tuple_cat(
        std::move(mass_tuple),
        std::move(spring_tuple),
        std::move(force_aggregator_tuple)
    );

    // Create TypedODESystem from tuple
    return std::apply([](auto&&... comps) {
        return makeTypedODESystem<T>(std::move(comps)...);
    }, std::move(all_components));
}

/**
 * @brief Create a 2D rectangular grid of masses and springs
 *
 * Convenience function to create a regular grid with uniform properties.
 *
 * @tparam T Scalar type
 * @tparam Rows Number of rows in grid
 * @tparam Cols Number of columns in grid
 * @tparam IncludeDiagonals Whether to include diagonal springs
 * @param mass Mass value for all masses (kg)
 * @param spacing Grid spacing (m) - used for both rest length and initial positions
 * @param stiffness Spring constant (N/m)
 * @param damping Damping coefficient (N·s/m)
 */
template<typename T, size_t Rows, size_t Cols, bool IncludeDiagonals = false>
auto makeGrid2DSystem(
    double mass,
    double spacing,
    double stiffness,
    double damping = 0.0
) {
    static_assert(Rows >= 2, "Grid must have at least 2 rows");
    static_assert(Cols >= 2, "Grid must have at least 2 columns");

    constexpr size_t NumMasses = Rows * Cols;
    constexpr auto edges = makeGrid2DEdgesArray<Rows, Cols, IncludeDiagonals>();

    // Create mass parameters with grid positions
    std::array<MassParams2D, NumMasses> mass_params;
    for (size_t i = 0; i < NumMasses; ++i) {
        auto [row, col] = gridCoords(i, Cols);
        mass_params[i] = {
            mass,
            static_cast<double>(col) * spacing,  // x position
            static_cast<double>(row) * spacing,  // y position
            0.0,  // vx
            0.0   // vy
        };
    }

    // Create spring parameters
    std::array<SpringParams2D, edges.size()> spring_params;
    for (size_t k = 0; k < edges.size(); ++k) {
        auto [i, j] = edges[k];
        auto [row_i, col_i] = gridCoords(i, Cols);
        auto [row_j, col_j] = gridCoords(j, Cols);

        // Calculate rest length based on connection type
        double dx = static_cast<double>(col_j) - static_cast<double>(col_i);
        double dy = static_cast<double>(row_j) - static_cast<double>(row_i);
        double rest_length = spacing * std::sqrt(dx * dx + dy * dy);

        spring_params[k] = {stiffness, rest_length, damping};
    }

    return makeConnectedMassSystem2D<T, NumMasses, edges>(mass_params, spring_params);
}

} // namespace sopot::connected_masses
