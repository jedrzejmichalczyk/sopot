#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "indexed_tags.hpp"
#include <string>
#include <stdexcept>

namespace sopot::connected_masses {

/**
 * @brief Generic point mass component that works with any index
 *
 * This is essentially the same as coupled_oscillator::PointMass,
 * but uses compile-time index-based tags instead of named tag types.
 *
 * @tparam Index Compile-time mass index (0, 1, 2, ...)
 * @tparam T Scalar type (double or Dual for autodiff)
 */
template<size_t Index, Scalar T = double>
class IndexedPointMass final : public TypedComponent<2, T> {
public:
    using Base = TypedComponent<2, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using TagSet = MassTag<Index>;

private:
    double m_mass;
    double m_initial_position;
    double m_initial_velocity;
    std::string m_name;

public:
    /**
     * @brief Construct point mass with specified properties
     *
     * @param mass Mass value (kg) - must be positive
     * @param initial_position Initial position (m)
     * @param initial_velocity Initial velocity (m/s)
     * @throws std::invalid_argument if mass is not positive
     */
    explicit IndexedPointMass(
        double mass,
        double initial_position = 0.0,
        double initial_velocity = 0.0
    )
        : m_mass(mass)
        , m_initial_position(initial_position)
        , m_initial_velocity(initial_velocity)
        , m_name("Mass" + std::to_string(Index))
    {
        if (mass <= 0.0) {
            throw std::invalid_argument(
                "Mass must be positive (got " + std::to_string(mass) +
                " for Mass" + std::to_string(Index) + ")"
            );
        }
    }

    // Required: Initial state
    LocalState getInitialLocalState() const {
        return {T(m_initial_position), T(m_initial_velocity)};
    }

    // Required: Component identification
    std::string_view getComponentType() const {
        return "IndexedPointMass";
    }

    std::string_view getComponentName() const {
        return m_name;
    }

    /**
     * @brief Compute derivatives using registry to query force
     *
     * Dynamics: dv/dt = F/m, dx/dt = v
     */
    template<typename Registry>
    LocalDerivative derivatives(
        T /*t*/,
        std::span<const T> local,
        std::span<const T> global,
        const Registry& registry
    ) const {
        // Query total force from registry (provided by springs and other force sources)
        T force = registry.template computeFunction<typename TagSet::Force>(global);

        // F = ma => a = F/m
        T acceleration = force / T(m_mass);

        T velocity = local[1];

        // dx/dt = v, dv/dt = a
        return {velocity, acceleration};
    }


    // State function: Provide position
    T compute(typename TagSet::Position, std::span<const T> state) const {
        return this->getGlobalState(state, 0);  // First state element
    }

    // State function: Provide velocity
    T compute(typename TagSet::Velocity, std::span<const T> state) const {
        return this->getGlobalState(state, 1);  // Second state element
    }

    // State function: Provide mass (constant)
    T compute(typename TagSet::Mass, std::span<const T> /*state*/) const {
        return T(m_mass);
    }
};

} // namespace sopot::connected_masses
