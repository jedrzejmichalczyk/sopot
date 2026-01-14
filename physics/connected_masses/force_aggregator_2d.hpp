#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "indexed_tags_2d.hpp"
#include <array>
#include <utility>

namespace sopot::connected_masses {

/**
 * @brief Aggregate all spring forces acting on a specific 2D mass
 *
 * This component sums up all forces from springs connected to mass Index.
 * It queries all springs that connect to this mass using the compile-time
 * edge list.
 *
 * @tparam Index The mass index to aggregate forces for
 * @tparam Edges Compile-time edge list
 * @tparam T Scalar type
 */
template<size_t Index, auto Edges, Scalar T = double>
class ForceAggregator2D final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using MassTags = MassTag2D<Index>;

private:
    std::string m_name;

public:
    explicit ForceAggregator2D()
        : m_name("ForceAggregator2D_" + std::to_string(Index))
    {}

    // Required: No state
    LocalState getInitialLocalState() const {
        return {};
    }

    // Required: Component identification
    std::string_view getComponentType() const {
        return "ForceAggregator2D";
    }

    std::string_view getComponentName() const {
        return m_name;
    }

    /**
     * @brief Compute total force on this mass from all connected springs
     *
     * Iterates through all edges in the edge list and sums forces from
     * springs connected to this mass.
     */
    template<typename Registry>
    std::array<T, 2> compute(typename MassTags::Force, std::span<const T> state, const Registry& registry) const {
        std::array<T, 2> total_force = {T(0.0), T(0.0)};

        // Iterate through all edges at compile time
        [&]<size_t... EdgeIndices>(std::index_sequence<EdgeIndices...>) {
            (
                [&] {
                    constexpr auto edge = Edges[EdgeIndices];
                    constexpr size_t I = edge.first;
                    constexpr size_t J = edge.second;

                    if constexpr (I == Index) {
                        // This spring connects to our mass as the first endpoint
                        // Force on mass I from spring (I,J)
                        auto force = registry.template computeFunction<typename SpringTag2D<I, J>::Force>(state);
                        total_force[0] += force[0];
                        total_force[1] += force[1];
                    } else if constexpr (J == Index) {
                        // This spring connects to our mass as the second endpoint
                        // Force on mass J is opposite of force on mass I (Newton's 3rd law)
                        auto force = registry.template computeFunction<typename SpringTag2D<I, J>::Force>(state);
                        total_force[0] -= force[0];
                        total_force[1] -= force[1];
                    }
                }(),
                ...
            );
        }(std::make_index_sequence<Edges.size()>{});

        return total_force;
    }
};

/**
 * @brief Generate tuple of force aggregators for all masses
 *
 * Helper to create ForceAggregator2D for each mass index.
 */
namespace detail {

template<typename T, auto Edges, size_t... MassIndices>
auto makeForceAggregatorsTuple(std::index_sequence<MassIndices...>) {
    return std::make_tuple(ForceAggregator2D<MassIndices, Edges, T>()...);
}

} // namespace detail

} // namespace sopot::connected_masses
