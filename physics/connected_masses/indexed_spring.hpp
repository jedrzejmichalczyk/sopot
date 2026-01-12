#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "indexed_tags.hpp"
#include <string>
#include <stdexcept>

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
    std::string m_name;

public:
    /**
     * @brief Construct spring with specified properties
     *
     * @param stiffness Spring constant k (N/m) - must be positive
     * @param rest_length Natural length L0 (m) - must be non-negative
     * @param damping Damping coefficient c (N·s/m), default 0 - must be non-negative
     * @throws std::invalid_argument if parameters are invalid
     */
    explicit IndexedSpring(
        double stiffness,
        double rest_length,
        double damping = 0.0
    )
        : m_stiffness(stiffness)
        , m_rest_length(rest_length)
        , m_damping(damping)
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
     * @brief Compute force on mass 1 (registry-aware)
     *
     * Force pulls mass 1 toward mass 2 when extended.
     */
    template<typename Registry>
    T compute(
        typename TagSet1::Force,
        std::span<const T> state,
        const Registry& registry
    ) const {
        // Query positions of both masses
        T x1 = registry.template computeFunction<typename TagSet1::Position>(state);
        T x2 = registry.template computeFunction<typename TagSet2::Position>(state);

        // Extension = (x1 - x2) - rest_length
        // Positive extension means stretched
        T extension = x1 - x2 - T(m_rest_length);

        // Spring force: F = -k * extension (restoring force)
        T spring_force = -T(m_stiffness) * extension;

        // Add damping force if present: F_damp = -c * (v1 - v2)
        if (m_damping > 0.0) {
            T v1 = registry.template computeFunction<typename TagSet1::Velocity>(state);
            T v2 = registry.template computeFunction<typename TagSet2::Velocity>(state);
            T relative_velocity = v1 - v2;
            T damping_force = -T(m_damping) * relative_velocity;
            spring_force += damping_force;
        }

        return spring_force;
    }

    /**
     * @brief Compute force on mass 2 (registry-aware)
     *
     * By Newton's third law: F2 = -F1
     */
    template<typename Registry>
    T compute(
        typename TagSet2::Force,
        std::span<const T> state,
        const Registry& registry
    ) const {
        // Force on mass 2 is opposite of force on mass 1
        T force1 = compute(typename TagSet1::Force{}, state, registry);
        return -force1;
    }

    /**
     * @brief Compute spring extension (registry-aware)
     */
    template<typename Registry>
    T compute(
        typename SpringTagSet::Extension,
        std::span<const T> state,
        const Registry& registry
    ) const {
        T x1 = registry.template computeFunction<typename TagSet1::Position>(state);
        T x2 = registry.template computeFunction<typename TagSet2::Position>(state);
        return x1 - x2 - T(m_rest_length);
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

    // Fallback compute methods (throw error when registry not available)
    // These should never be called - the registry-aware versions should always be used
    T compute(typename TagSet1::Force, std::span<const T> /*state*/) const {
        throw std::logic_error(
            "Spring force computation requires registry access. "
            "This fallback should never be called."
        );
    }

    T compute(typename TagSet2::Force, std::span<const T> /*state*/) const {
        throw std::logic_error(
            "Spring force computation requires registry access. "
            "This fallback should never be called."
        );
    }
};

} // namespace sopot::connected_masses
