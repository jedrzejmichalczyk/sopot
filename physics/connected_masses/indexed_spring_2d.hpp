#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "indexed_tags_2d.hpp"
#include <string>
#include <cmath>
#include <array>
#include <algorithm>

namespace sopot::connected_masses {

/**
 * @brief 2D spring connecting two point masses
 *
 * The spring exerts forces on both connected masses according to Hooke's law
 * in 2D space with optional damping.
 *
 * Force on mass I: F_I = -k * extension * direction - c * relative_velocity
 * where:
 *   extension = current_length - rest_length
 *   direction = (pos_j - pos_i) / current_length  (unit vector from I to J)
 *   relative_velocity = projection of (vel_j - vel_i) onto direction
 *
 * @tparam I Index of first mass
 * @tparam J Index of second mass
 * @tparam T Scalar type (double or Dual for autodiff)
 */
template<size_t I, size_t J, Scalar T = double>
class IndexedSpring2D final : public TypedComponent<0, T> {
    static_assert(I != J, "Spring cannot connect a mass to itself");

public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using SpringTags = SpringTag2D<I, J>;
    using MassITags = MassTag2D<I>;
    using MassJTags = MassTag2D<J>;

private:
    double m_stiffness;     // k (N/m)
    double m_rest_length;   // L0 (m)
    double m_damping;       // c (N·s/m)
    std::string m_name;

public:
    /**
     * @brief Construct 2D spring with specified properties
     *
     * @param stiffness Spring constant k (N/m) - must be non-negative
     * @param rest_length Natural length L0 (m) - must be positive
     * @param damping Damping coefficient c (N·s/m) - must be non-negative
     * @throws std::invalid_argument if parameters are invalid
     */
    explicit IndexedSpring2D(
        double stiffness,
        double rest_length,
        double damping = 0.0
    )
        : m_stiffness(stiffness)
        , m_rest_length(rest_length)
        , m_damping(damping)
        , m_name("Spring2D_" + std::to_string(I) + "_" + std::to_string(J))
    {
        if (stiffness < 0.0) {
            throw std::invalid_argument("Spring stiffness must be non-negative");
        }
        if (rest_length <= 0.0) {
            throw std::invalid_argument("Spring rest length must be positive");
        }
        if (damping < 0.0) {
            throw std::invalid_argument("Spring damping must be non-negative");
        }
    }

    // Required: No state (stateless component)
    LocalState getInitialLocalState() const {
        return {};
    }

    // Required: Component identification
    std::string_view getComponentType() const {
        return "IndexedSpring2D";
    }

    std::string_view getComponentName() const {
        return m_name;
    }

    /**
     * @brief Compute spring geometry and forces
     */
    template<typename Registry>
    auto computeSpringState(std::span<const T> state, const Registry& registry) const {
        using std::sqrt;  // Enable ADL for both double and Dual types

        // Get positions and velocities of both masses
        auto pos_i = registry.template computeFunction<typename MassITags::Position>(state);
        auto pos_j = registry.template computeFunction<typename MassJTags::Position>(state);
        auto vel_i = registry.template computeFunction<typename MassITags::Velocity>(state);
        auto vel_j = registry.template computeFunction<typename MassJTags::Velocity>(state);

        // Vector from I to J
        T dx = pos_j[0] - pos_i[0];
        T dy = pos_j[1] - pos_i[1];

        // Current length
        T length = sqrt(dx * dx + dy * dy);

        // Avoid division by zero
        T length_safe = length < T(1e-10) ? T(1e-10) : length;

        // Unit vector from I to J
        T ux = dx / length_safe;
        T uy = dy / length_safe;

        // Extension (positive = stretched, negative = compressed)
        T extension = length - T(m_rest_length);

        // Relative velocity along spring direction
        T dvx = vel_j[0] - vel_i[0];
        T dvy = vel_j[1] - vel_i[1];
        T relative_velocity = dvx * ux + dvy * uy;

        // Spring force magnitude (Hooke's law + damping)
        T force_magnitude = T(m_stiffness) * extension + T(m_damping) * relative_velocity;

        // Force vector on mass I (points from I toward J when spring is stretched)
        std::array<T, 2> force_on_i = {
            force_magnitude * ux,
            force_magnitude * uy
        };

        struct Result {
            T length;
            T extension;
            std::array<T, 2> force_on_i;
        };

        return Result{length, extension, force_on_i};
    }

    // State function: Spring extension
    template<typename Registry>
    T compute(typename SpringTags::Extension, std::span<const T> state, const Registry& registry) const {
        return computeSpringState(state, registry).extension;
    }

    // State function: Current length
    template<typename Registry>
    T compute(typename SpringTags::Length, std::span<const T> state, const Registry& registry) const {
        return computeSpringState(state, registry).length;
    }

    // State function: Force on mass I
    template<typename Registry>
    std::array<T, 2> compute(typename SpringTags::Force, std::span<const T> state, const Registry& registry) const {
        return computeSpringState(state, registry).force_on_i;
    }

    // State function: Potential energy
    template<typename Registry>
    T compute(typename SpringTags::PotentialEnergy, std::span<const T> state, const Registry& registry) const {
        T ext = computeSpringState(state, registry).extension;
        return T(0.5) * T(m_stiffness) * ext * ext;
    }
};

} // namespace sopot::connected_masses
