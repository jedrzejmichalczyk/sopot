#pragma once

/**
 * @file components.hpp
 * @brief Compile-time component definitions for the unified graph
 *
 * Each component is a struct with constexpr properties and template methods.
 * No virtual functions - everything resolved at compile time.
 */

#include "state_functions.hpp"
#include "core/scalar.hpp"
#include <array>
#include <cmath>
#include <span>

namespace sopot::unified {

// =============================================================================
// COMPONENT TRAITS - Compile-time properties
// =============================================================================

/**
 * @brief Compile-time traits for a component type
 *
 * Specialize this for each component type to define:
 *   - state_size: Number of state variables
 *   - num_ports: Number of connection ports
 *   - required: Bitmask of required state functions
 *   - provided: Bitmask of provided state functions
 */
template<typename Component>
struct ComponentTraits;

// =============================================================================
// POINT MASS 2D
// =============================================================================

/**
 * @brief 2D Point Mass - a node with position and velocity state
 *
 * State: [x, y, vx, vy]
 * Requires: Force2D
 * Provides: Position2D, Velocity2D, MassValue
 */
template<Scalar T>
struct PointMass2D {
    double mass;
    std::array<double, 2> initial_pos;
    std::array<double, 2> initial_vel;

    static constexpr size_t state_size = 4;
    static constexpr size_t num_ports = 4;  // Can connect to up to 4 neighbors
    static constexpr uint32_t required = FunctionMask<Force2D>;
    static constexpr uint32_t provided = FunctionMask<Position2D, Velocity2D, MassValue>;

    constexpr PointMass2D(double m, std::array<double, 2> pos = {}, std::array<double, 2> vel = {})
        : mass(m), initial_pos(pos), initial_vel(vel) {}

    void initState(std::span<T> state) const {
        state[0] = T(initial_pos[0]);
        state[1] = T(initial_pos[1]);
        state[2] = T(initial_vel[0]);
        state[3] = T(initial_vel[1]);
    }

    std::array<T, 2> getPosition(std::span<const T> state) const {
        return {state[0], state[1]};
    }

    std::array<T, 2> getVelocity(std::span<const T> state) const {
        return {state[2], state[3]};
    }

    T getMass() const { return T(mass); }

    // Export provided values to NodeData
    void exportValues(std::span<const T> state, NodeData<T>& data) const {
        data.position = getPosition(state);
        data.velocity = getVelocity(state);
        data.mass = getMass();
        data.has_position = true;
        data.has_velocity = true;
        data.has_mass = true;
    }

    // Compute derivatives given accumulated inputs
    std::array<T, 4> computeDerivatives(std::span<const T> state, const NodeData<T>& inputs) const {
        return {
            state[2],                       // dx/dt = vx
            state[3],                       // dy/dt = vy
            inputs.force[0] / T(mass),      // dvx/dt = Fx/m
            inputs.force[1] / T(mass)       // dvy/dt = Fy/m
        };
    }
};

template<Scalar T>
struct ComponentTraits<PointMass2D<T>> {
    static constexpr size_t state_size = PointMass2D<T>::state_size;
    static constexpr size_t num_ports = PointMass2D<T>::num_ports;
    static constexpr uint32_t required = PointMass2D<T>::required;
    static constexpr uint32_t provided = PointMass2D<T>::provided;
};

// =============================================================================
// SPRING 2D
// =============================================================================

/**
 * @brief 2D Spring - stateless node connecting two neighbors
 *
 * State: none
 * Requires: Position2D, Velocity2D (from both ports)
 * Provides: Force2D (to both ports)
 *
 * Optional collision avoidance via steep repulsion when masses get too close.
 * Uses inverse formula: F_repulsion = k_rep * (r_min/r - 1) when r < r_min
 */
template<Scalar T>
struct Spring2D {
    double stiffness;
    double rest_length;
    double damping;
    double min_distance;          // Collision radius (m) - repulsion below this
    double repulsion_stiffness;   // Repulsion strength (N/m)

    static constexpr size_t state_size = 0;
    static constexpr size_t num_ports = 2;
    static constexpr uint32_t required = FunctionMask<Position2D, Velocity2D>;
    static constexpr uint32_t provided = FunctionMask<Force2D>;

    constexpr Spring2D(double k, double L0, double c = 0.0,
                       double min_dist = 0.0, double rep_stiff = -1.0)
        : stiffness(k), rest_length(L0), damping(c),
          min_distance(min_dist),
          repulsion_stiffness(rep_stiff > 0.0 ? rep_stiff : 10.0 * k) {}

    void initState(std::span<T> /*state*/) const {}

    void exportValues(std::span<const T> /*state*/, NodeData<T>& /*data*/) const {}

    // Compute force outputs given neighbor data
    // Returns: [force_on_port0, force_on_port1]
    std::array<std::array<T, 2>, 2> computeForces(
        const NodeData<T>& neighbor0,
        const NodeData<T>& neighbor1
    ) const {
        using std::sqrt;

        auto& pos0 = neighbor0.position;
        auto& pos1 = neighbor1.position;
        auto& vel0 = neighbor0.velocity;
        auto& vel1 = neighbor1.velocity;

        T dx = pos1[0] - pos0[0];
        T dy = pos1[1] - pos0[1];
        T len = sqrt(dx * dx + dy * dy);
        T len_safe = len < T(1e-10) ? T(1e-10) : len;

        T ux = dx / len_safe;
        T uy = dy / len_safe;
        T extension = len - T(rest_length);

        T dvx = vel1[0] - vel0[0];
        T dvy = vel1[1] - vel0[1];
        T rel_vel = dvx * ux + dvy * uy;

        T F = T(stiffness) * extension + T(damping) * rel_vel;

        // Steep repulsion when masses get too close (only if min_distance > 0)
        // Uses inverse formula: F_repulsion = k_rep * (r_min/r - 1) when r < r_min
        if (min_distance > 0.0 && value_of(len) < min_distance) {
            T repulsion = -T(repulsion_stiffness) * (T(min_distance) / len_safe - T(1.0));
            F += repulsion;
        }

        std::array<T, 2> force_on_0 = {F * ux, F * uy};
        std::array<T, 2> force_on_1 = {-F * ux, -F * uy};

        return {force_on_0, force_on_1};
    }
};

template<Scalar T>
struct ComponentTraits<Spring2D<T>> {
    static constexpr size_t state_size = Spring2D<T>::state_size;
    static constexpr size_t num_ports = Spring2D<T>::num_ports;
    static constexpr uint32_t required = Spring2D<T>::required;
    static constexpr uint32_t provided = Spring2D<T>::provided;
};

// =============================================================================
// DAMPER 2D
// =============================================================================

/**
 * @brief 2D Damper - velocity-dependent force between two neighbors
 */
template<Scalar T>
struct Damper2D {
    double damping;

    static constexpr size_t state_size = 0;
    static constexpr size_t num_ports = 2;
    static constexpr uint32_t required = FunctionMask<Position2D, Velocity2D>;
    static constexpr uint32_t provided = FunctionMask<Force2D>;

    constexpr Damper2D(double c) : damping(c) {}

    void initState(std::span<T> /*state*/) const {}
    void exportValues(std::span<const T> /*state*/, NodeData<T>& /*data*/) const {}

    std::array<std::array<T, 2>, 2> computeForces(
        const NodeData<T>& neighbor0,
        const NodeData<T>& neighbor1
    ) const {
        using std::sqrt;

        auto& pos0 = neighbor0.position;
        auto& pos1 = neighbor1.position;
        auto& vel0 = neighbor0.velocity;
        auto& vel1 = neighbor1.velocity;

        T dx = pos1[0] - pos0[0];
        T dy = pos1[1] - pos0[1];
        T len = sqrt(dx * dx + dy * dy);
        T len_safe = len < T(1e-10) ? T(1e-10) : len;

        T ux = dx / len_safe;
        T uy = dy / len_safe;

        T dvx = vel1[0] - vel0[0];
        T dvy = vel1[1] - vel0[1];
        T rel_vel = dvx * ux + dvy * uy;

        T F = T(damping) * rel_vel;

        return {std::array<T, 2>{F * ux, F * uy}, std::array<T, 2>{-F * ux, -F * uy}};
    }
};

template<Scalar T>
struct ComponentTraits<Damper2D<T>> {
    static constexpr size_t state_size = Damper2D<T>::state_size;
    static constexpr size_t num_ports = Damper2D<T>::num_ports;
    static constexpr uint32_t required = Damper2D<T>::required;
    static constexpr uint32_t provided = Damper2D<T>::provided;
};

// =============================================================================
// GRAVITY SOURCE 2D
// =============================================================================

/**
 * @brief Gravity source - applies gravitational force to one neighbor
 */
template<Scalar T>
struct GravitySource2D {
    std::array<double, 2> gravity;

    static constexpr size_t state_size = 0;
    static constexpr size_t num_ports = 1;
    static constexpr uint32_t required = FunctionMask<MassValue>;
    static constexpr uint32_t provided = FunctionMask<Force2D>;

    constexpr GravitySource2D(double gx = 0.0, double gy = -9.81) : gravity{gx, gy} {}

    void initState(std::span<T> /*state*/) const {}
    void exportValues(std::span<const T> /*state*/, NodeData<T>& /*data*/) const {}

    std::array<T, 2> computeForce(const NodeData<T>& neighbor) const {
        T m = neighbor.mass;
        return {T(gravity[0]) * m, T(gravity[1]) * m};
    }
};

template<Scalar T>
struct ComponentTraits<GravitySource2D<T>> {
    static constexpr size_t state_size = GravitySource2D<T>::state_size;
    static constexpr size_t num_ports = GravitySource2D<T>::num_ports;
    static constexpr uint32_t required = GravitySource2D<T>::required;
    static constexpr uint32_t provided = GravitySource2D<T>::provided;
};

// =============================================================================
// FIXED ANCHOR 2D
// =============================================================================

/**
 * @brief Fixed anchor - provides fixed position, useful for boundary conditions
 */
template<Scalar T>
struct FixedAnchor2D {
    std::array<double, 2> position;

    static constexpr size_t state_size = 0;
    static constexpr size_t num_ports = 4;
    static constexpr uint32_t required = 0;  // Requires nothing
    static constexpr uint32_t provided = FunctionMask<Position2D, Velocity2D>;

    constexpr FixedAnchor2D(std::array<double, 2> pos) : position(pos) {}

    void initState(std::span<T> /*state*/) const {}

    void exportValues(std::span<const T> /*state*/, NodeData<T>& data) const {
        data.position = {T(position[0]), T(position[1])};
        data.velocity = {T(0), T(0)};
        data.has_position = true;
        data.has_velocity = true;
    }
};

template<Scalar T>
struct ComponentTraits<FixedAnchor2D<T>> {
    static constexpr size_t state_size = FixedAnchor2D<T>::state_size;
    static constexpr size_t num_ports = FixedAnchor2D<T>::num_ports;
    static constexpr uint32_t required = FixedAnchor2D<T>::required;
    static constexpr uint32_t provided = FixedAnchor2D<T>::provided;
};

} // namespace sopot::unified
