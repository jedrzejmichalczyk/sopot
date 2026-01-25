#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "indexed_tags.hpp"
#include <string>
#include <stdexcept>
#include <cmath>

namespace sopot::connected_masses {

/**
 * @brief Generic spring connecting two indexed masses
 *
 * Spring provides forces to both connected masses based on Hooke's law
 * with optional damping: F = -k*(x1-x2-L0) - c*(v1-v2)
 *
 * @tparam Index1 First mass index
 * @tparam Index2 Second mass index
 * @tparam T Scalar type
 */
template<size_t Index1, size_t Index2, Scalar T = double>
class IndexedSpring final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using TagSet1 = MassTag<Index1>;
    using TagSet2 = MassTag<Index2>;
    using SpringTagSet = SpringTag<Index1, Index2>;

private:
    double m_stiffness;      // Spring constant k (N/m)
    double m_rest_length;    // Rest length L0 (m)
    double m_damping;        // Damping coefficient c (N·s/m)
    double m_min_distance;   // Collision radius (m) - steep repulsion below this
    double m_repulsion_stiffness;  // Repulsion strength (N/m)
    std::string m_name;

public:
    /**
     * @brief Construct spring with specified properties
     *
     * @param stiffness Spring constant k (N/m) - must be positive
     * @param rest_length Natural length L0 (m) - must be non-negative
     * @param damping Damping coefficient c (N·s/m), default 0 - must be non-negative
     * @param min_distance Collision radius (m) - steep repulsion below this distance
     *                     Default: 10% of rest_length (or 0.01 if rest_length is 0)
     * @param repulsion_stiffness Strength of repulsion force (N/m)
     *                            Default: 10x stiffness
     * @throws std::invalid_argument if parameters are invalid
     */
    explicit IndexedSpring(
        double stiffness,
        double rest_length,
        double damping = 0.0,
        double min_distance = -1.0,
        double repulsion_stiffness = -1.0
    )
        : m_stiffness(stiffness)
        , m_rest_length(rest_length)
        , m_damping(damping)
        , m_min_distance(min_distance)  // Default 0 = no repulsion
        , m_repulsion_stiffness(repulsion_stiffness > 0.0 ? repulsion_stiffness : 10.0 * stiffness)
        , m_name("Spring" + std::to_string(Index1) + "_" + std::to_string(Index2))
    {
        if (stiffness <= 0.0) {
            throw std::invalid_argument(
                "Spring stiffness must be positive (got " + std::to_string(stiffness) +
                " for Spring" + std::to_string(Index1) + "_" + std::to_string(Index2) + ")"
            );
        }
        if (rest_length < 0.0) {
            throw std::invalid_argument(
                "Spring rest length must be non-negative (got " + std::to_string(rest_length) +
                " for Spring" + std::to_string(Index1) + "_" + std::to_string(Index2) + ")"
            );
        }
        if (damping < 0.0) {
            throw std::invalid_argument(
                "Spring damping must be non-negative (got " + std::to_string(damping) +
                " for Spring" + std::to_string(Index1) + "_" + std::to_string(Index2) + ")"
            );
        }
    }

    // Required: No internal state
    LocalState getInitialLocalState() const {
        return {};
    }

    // Required: Component identification
    std::string_view getComponentType() const {
        return "IndexedSpring";
    }

    std::string_view getComponentName() const {
        return m_name;
    }

    /**
     * @brief Compute force exerted by this spring on mass Index1 (registry-aware)
     *
     * Force pulls mass Index1 toward mass Index2 when spring is stretched.
     * Uses proper signed distance convention matching 2D spring behavior.
     *
     * Note: This provides SpringTag::Force, not MassTag::Force.
     * The ForceAggregator sums all spring forces to provide MassTag::Force.
     */
    template<typename Registry>
    T compute(
        typename SpringTagSet::Force,
        std::span<const T> state,
        const Registry& registry
    ) const {
        using std::sqrt;

        // Query positions of both masses
        T x1 = registry.template computeFunction<typename TagSet1::Position>(state);
        T x2 = registry.template computeFunction<typename TagSet2::Position>(state);

        // Vector from Index1 to Index2 (signed distance)
        T dx = x2 - x1;

        // Distance (always positive)
        T distance = sqrt(dx * dx);
        T distance_safe = distance < T(1e-10) ? T(1e-10) : distance;

        // Unit direction from Index1 toward Index2
        T direction = dx / distance_safe;

        // Extension (positive = stretched, negative = compressed)
        T extension = distance - T(m_rest_length);

        // Spring force magnitude: positive when stretched (pulls masses together)
        T force_magnitude = T(m_stiffness) * extension;

        // Add damping force if present
        // Damping opposes relative velocity along spring direction
        if (m_damping > 0.0) {
            T v1 = registry.template computeFunction<typename TagSet1::Velocity>(state);
            T v2 = registry.template computeFunction<typename TagSet2::Velocity>(state);
            // Relative velocity of Index2 w.r.t. Index1, projected onto spring direction
            T relative_velocity = (v2 - v1) * direction;
            force_magnitude += T(m_damping) * relative_velocity;
        }

        // Steep repulsion when masses get too close (only if min_distance > 0)
        if (m_min_distance > 0.0 && value_of(distance) < m_min_distance) {
            // Repulsion pushes masses apart (negative force_magnitude contribution)
            T repulsion = -T(m_repulsion_stiffness) * (T(m_min_distance) / distance_safe - T(1.0));
            force_magnitude += repulsion;
        }

        // Force on mass Index1 points toward Index2 when force_magnitude > 0
        return force_magnitude * direction;
    }

    /**
     * @brief Compute spring extension (registry-aware)
     *
     * Extension = distance - rest_length
     * Positive = stretched, Negative = compressed
     */
    template<typename Registry>
    T compute(
        typename SpringTagSet::Extension,
        std::span<const T> state,
        const Registry& registry
    ) const {
        using std::sqrt;
        T x1 = registry.template computeFunction<typename TagSet1::Position>(state);
        T x2 = registry.template computeFunction<typename TagSet2::Position>(state);
        T dx = x2 - x1;
        T distance = sqrt(dx * dx);
        return distance - T(m_rest_length);
    }

    /**
     * @brief Compute potential energy stored in spring (registry-aware)
     *
     * PE = 0.5 * k * extension^2
     */
    template<typename Registry>
    T compute(
        typename SpringTagSet::PotentialEnergy,
        std::span<const T> state,
        const Registry& registry
    ) const {
        T extension = compute(typename SpringTagSet::Extension{}, state, registry);
        return T(0.5) * T(m_stiffness) * extension * extension;
    }

};

} // namespace sopot::connected_masses
