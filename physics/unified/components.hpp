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
 * @brief 2D Point Mass with rotation - a node with position, velocity, angle, and angular velocity
 *
 * State: [x, y, vx, vy, θ, ω]
 * Requires: Force2D, Torque
 * Provides: Position2D, Velocity2D, MassValue, Radius, Angle, AngularVel
 *
 * Rotational dynamics model:
 * - Moment of inertia: I = ½mr² (uniform disk approximation)
 * - Angular acceleration: dω/dt = τ/I
 * - Springs attach at the edge of the mass (at radius r from center)
 */
template<Scalar T>
struct PointMass2D {
    double mass;
    double radius;  // Radius for moment of inertia and spring attachment points
    std::array<double, 2> initial_pos;
    std::array<double, 2> initial_vel;
    double initial_angle;
    double initial_angular_vel;

    static constexpr size_t state_size = 6;  // [x, y, vx, vy, θ, ω]
    static constexpr size_t num_ports = 4;   // Can connect to up to 4 neighbors
    static constexpr uint32_t required = FunctionMask<Force2D, Torque>;
    static constexpr uint32_t provided = FunctionMask<Position2D, Velocity2D, MassValue, Radius, Angle, AngularVel, KineticEnergy>;

    constexpr PointMass2D(double m, std::array<double, 2> pos = {}, std::array<double, 2> vel = {},
                          double r = 0.05, double angle = 0.0, double ang_vel = 0.0)
        : mass(m), radius(r), initial_pos(pos), initial_vel(vel),
          initial_angle(angle), initial_angular_vel(ang_vel) {}

    void initState(std::span<T> state) const {
        state[0] = T(initial_pos[0]);
        state[1] = T(initial_pos[1]);
        state[2] = T(initial_vel[0]);
        state[3] = T(initial_vel[1]);
        state[4] = T(initial_angle);
        state[5] = T(initial_angular_vel);
    }

    std::array<T, 2> getPosition(std::span<const T> state) const {
        return {state[0], state[1]};
    }

    std::array<T, 2> getVelocity(std::span<const T> state) const {
        return {state[2], state[3]};
    }

    T getAngle(std::span<const T> state) const {
        return state[4];
    }

    T getAngularVel(std::span<const T> state) const {
        return state[5];
    }

    T getMass() const { return T(mass); }
    T getRadius() const { return T(radius); }

    // Moment of inertia for uniform disk: I = ½mr²
    T getMomentOfInertia() const { return T(0.5) * T(mass) * T(radius) * T(radius); }

    // Kinetic energy: KE = ½mv² + ½Iω²
    T getKineticEnergy(std::span<const T> state) const {
        T vx = state[2];
        T vy = state[3];
        T omega = state[5];
        T I = getMomentOfInertia();
        return T(0.5) * T(mass) * (vx*vx + vy*vy) + T(0.5) * I * omega * omega;
    }

    // Export provided values to NodeData
    void exportValues(std::span<const T> state, NodeData<T>& data) const {
        data.position = getPosition(state);
        data.velocity = getVelocity(state);
        data.mass = getMass();
        data.radius = getRadius();
        data.angle = getAngle(state);
        data.angular_vel = getAngularVel(state);
        data.kinetic_energy = getKineticEnergy(state);
        data.has_position = true;
        data.has_velocity = true;
        data.has_mass = true;
        data.has_radius = true;
        data.has_angle = true;
        data.has_angular_vel = true;
    }

    // Compute derivatives given accumulated inputs
    std::array<T, 6> computeDerivatives(std::span<const T> state, const NodeData<T>& inputs) const {
        T I = getMomentOfInertia();
        return {
            state[2],                       // dx/dt = vx
            state[3],                       // dy/dt = vy
            inputs.force[0] / T(mass),      // dvx/dt = Fx/m
            inputs.force[1] / T(mass),      // dvy/dt = Fy/m
            state[5],                       // dθ/dt = ω
            inputs.torque / I               // dω/dt = τ/I
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
 * @brief 2D Spring with rotational coupling - stateless node connecting two neighbors
 *
 * State: none
 * Requires: Position2D, Velocity2D, Angle, AngularVel, Radius (from both ports)
 * Provides: Force2D, Torque (to both ports)
 *
 * Spring attachment model:
 * - Springs attach at the edge of each mass (at radius r from center)
 * - Attachment points rotate with the mass
 * - Off-center forces create torques: τ = r × F
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
    double attachment_angle_0;    // Angle of attachment on mass 0 (relative to mass orientation)
    double attachment_angle_1;    // Angle of attachment on mass 1 (relative to mass orientation)

    static constexpr size_t state_size = 0;
    static constexpr size_t num_ports = 2;
    static constexpr uint32_t required = FunctionMask<Position2D, Velocity2D, Angle, AngularVel, Radius>;
    static constexpr uint32_t provided = FunctionMask<Force2D, Torque, PotentialEnergy>;

    constexpr Spring2D(double k, double L0, double c = 0.0,
                       double min_dist = 0.0, double rep_stiff = -1.0,
                       double attach_angle_0 = 0.0, double attach_angle_1 = 3.14159265358979323846)
        : stiffness(k), rest_length(L0), damping(c),
          min_distance(min_dist),
          repulsion_stiffness(rep_stiff > 0.0 ? rep_stiff : 10.0 * k),
          attachment_angle_0(attach_angle_0),
          attachment_angle_1(attach_angle_1) {}

    void initState(std::span<T> /*state*/) const {}

    void exportValues(std::span<const T> /*state*/, NodeData<T>& /*data*/) const {}

    // Result structure for forces, torques, and potential energy
    struct ForcesAndTorques {
        std::array<T, 2> force_on_0;
        std::array<T, 2> force_on_1;
        T torque_on_0;
        T torque_on_1;
        T potential_energy;  // Elastic PE = ½k(L-L0)²
    };

    // Compute force and torque outputs given neighbor data
    ForcesAndTorques computeForcesAndTorques(
        const NodeData<T>& neighbor0,
        const NodeData<T>& neighbor1
    ) const {
        using std::sqrt;
        using std::cos;
        using std::sin;

        // Get mass centers and orientations
        auto& center0 = neighbor0.position;
        auto& center1 = neighbor1.position;
        auto& vel0 = neighbor0.velocity;
        auto& vel1 = neighbor1.velocity;
        T theta0 = neighbor0.angle;
        T theta1 = neighbor1.angle;
        T omega0 = neighbor0.angular_vel;
        T omega1 = neighbor1.angular_vel;
        T r0 = neighbor0.radius;
        T r1 = neighbor1.radius;

        // Compute attachment point offsets in world coordinates
        // Attachment point rotates with the mass: world_angle = theta + attachment_angle
        T world_angle_0 = theta0 + T(attachment_angle_0);
        T world_angle_1 = theta1 + T(attachment_angle_1);

        // Offset vectors from mass center to attachment point
        T offset0_x = r0 * cos(world_angle_0);
        T offset0_y = r0 * sin(world_angle_0);
        T offset1_x = r1 * cos(world_angle_1);
        T offset1_y = r1 * sin(world_angle_1);

        // Actual attachment point positions
        T attach0_x = center0[0] + offset0_x;
        T attach0_y = center0[1] + offset0_y;
        T attach1_x = center1[0] + offset1_x;
        T attach1_y = center1[1] + offset1_y;

        // Velocity of attachment points (includes rotational component)
        // v_attach = v_center + omega × r (in 2D: omega × r = omega * [-r_y, r_x])
        T vel_attach0_x = vel0[0] - omega0 * offset0_y;
        T vel_attach0_y = vel0[1] + omega0 * offset0_x;
        T vel_attach1_x = vel1[0] - omega1 * offset1_y;
        T vel_attach1_y = vel1[1] + omega1 * offset1_x;

        // Spring vector (from attachment 0 to attachment 1)
        T dx = attach1_x - attach0_x;
        T dy = attach1_y - attach0_y;
        T len = sqrt(dx * dx + dy * dy);
        T len_safe = len < T(1e-10) ? T(1e-10) : len;

        // Unit vector along spring
        T ux = dx / len_safe;
        T uy = dy / len_safe;

        // Spring extension
        T extension = len - T(rest_length);

        // Relative velocity of attachment points along spring direction
        T dvx = vel_attach1_x - vel_attach0_x;
        T dvy = vel_attach1_y - vel_attach0_y;
        T rel_vel = dvx * ux + dvy * uy;

        // Spring force magnitude (positive = tension)
        T F = T(stiffness) * extension + T(damping) * rel_vel;

        // Steep repulsion when attachment points get too close
        if (min_distance > 0.0 && value_of(len) < min_distance) {
            T repulsion = -T(repulsion_stiffness) * (T(min_distance) / len_safe - T(1.0));
            F += repulsion;
        }

        // Force vectors (F on mass 0 points toward attachment 1)
        std::array<T, 2> force_on_0 = {F * ux, F * uy};
        std::array<T, 2> force_on_1 = {-F * ux, -F * uy};

        // Torque = r × F (2D cross product: r_x * F_y - r_y * F_x)
        // Torque on mass 0 from force at offset0
        T torque_on_0 = offset0_x * force_on_0[1] - offset0_y * force_on_0[0];
        // Torque on mass 1 from force at offset1
        T torque_on_1 = offset1_x * force_on_1[1] - offset1_y * force_on_1[0];

        // Elastic potential energy: PE = ½k(L-L0)²
        T pe = T(0.5) * T(stiffness) * extension * extension;

        return {force_on_0, force_on_1, torque_on_0, torque_on_1, pe};
    }

    // Legacy computeForces for backward compatibility (ignores rotation)
    std::array<std::array<T, 2>, 2> computeForces(
        const NodeData<T>& neighbor0,
        const NodeData<T>& neighbor1
    ) const {
        auto result = computeForcesAndTorques(neighbor0, neighbor1);
        return {result.force_on_0, result.force_on_1};
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
