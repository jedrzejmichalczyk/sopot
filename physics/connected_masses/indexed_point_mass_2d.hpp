#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "indexed_tags_2d.hpp"
#include <string>
#include <stdexcept>
#include <array>

namespace sopot::connected_masses {

/**
 * @brief 2D point mass component with position and velocity in x,y
 *
 * State vector: [x, y, vx, vy]
 *
 * @tparam Index Compile-time mass index (0, 1, 2, ...)
 * @tparam T Scalar type (double or Dual for autodiff)
 */
template<size_t Index, Scalar T = double>
class IndexedPointMass2D final : public TypedComponent<4, T> {
public:
    using Base = TypedComponent<4, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using TagSet = MassTag2D<Index>;

private:
    double m_mass;
    std::array<double, 2> m_initial_position;  // [x, y]
    std::array<double, 2> m_initial_velocity;  // [vx, vy]
    std::string m_name;

public:
    /**
     * @brief Construct 2D point mass with specified properties
     *
     * @param mass Mass value (kg) - must be positive
     * @param initial_x Initial x position (m)
     * @param initial_y Initial y position (m)
     * @param initial_vx Initial x velocity (m/s)
     * @param initial_vy Initial y velocity (m/s)
     * @throws std::invalid_argument if mass is not positive
     */
    explicit IndexedPointMass2D(
        double mass,
        double initial_x = 0.0,
        double initial_y = 0.0,
        double initial_vx = 0.0,
        double initial_vy = 0.0
    )
        : m_mass(mass)
        , m_initial_position{initial_x, initial_y}
        , m_initial_velocity{initial_vx, initial_vy}
        , m_name("Mass2D_" + std::to_string(Index))
    {
        if (mass <= 0.0) {
            throw std::invalid_argument(
                "Mass must be positive (got " + std::to_string(mass) +
                " for Mass2D_" + std::to_string(Index) + ")"
            );
        }
    }

    // Required: Initial state [x, y, vx, vy]
    LocalState getInitialLocalState() const {
        return {
            T(m_initial_position[0]),
            T(m_initial_position[1]),
            T(m_initial_velocity[0]),
            T(m_initial_velocity[1])
        };
    }

    // Required: Component identification
    std::string_view getComponentType() const {
        return "IndexedPointMass2D";
    }

    std::string_view getComponentName() const {
        return m_name;
    }

    /**
     * @brief Compute derivatives using registry to query forces
     *
     * Dynamics: d/dt[x, y, vx, vy] = [vx, vy, Fx/m, Fy/m]
     */
    template<typename Registry>
    LocalDerivative derivatives(
        T /*t*/,
        std::span<const T> local,
        std::span<const T> global,
        const Registry& registry
    ) const {
        // Query total force from registry (provided by springs and other force sources)
        auto force = registry.template computeFunction<typename TagSet::Force>(global);

        // F = ma => a = F/m
        T ax = force[0] / T(m_mass);
        T ay = force[1] / T(m_mass);

        T vx = local[2];
        T vy = local[3];

        // dx/dt = vx, dy/dt = vy, dvx/dt = ax, dvy/dt = ay
        return {vx, vy, ax, ay};
    }

    // State function: Provide position as [x, y]
    std::array<T, 2> compute(typename TagSet::Position, std::span<const T> state) const {
        return {
            this->getGlobalState(state, 0),  // x
            this->getGlobalState(state, 1)   // y
        };
    }

    // State function: Provide velocity as [vx, vy]
    std::array<T, 2> compute(typename TagSet::Velocity, std::span<const T> state) const {
        return {
            this->getGlobalState(state, 2),  // vx
            this->getGlobalState(state, 3)   // vy
        };
    }

    // State function: Provide mass (constant)
    T compute(typename TagSet::Mass, std::span<const T> /*state*/) const {
        return T(m_mass);
    }
};

} // namespace sopot::connected_masses
