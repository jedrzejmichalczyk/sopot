#pragma once

/**
 * @file state_functions.hpp
 * @brief State function tags for the unified graph architecture
 *
 * State functions are the "currency" that flows between nodes.
 * Each tag identifies a type of data that can be required or provided.
 */

#include <array>
#include <cstdint>

namespace sopot::unified {

// =============================================================================
// STATE FUNCTION TAGS
// =============================================================================

struct Position2D   { static constexpr uint32_t id = 0; };
struct Velocity2D   { static constexpr uint32_t id = 1; };
struct Force2D      { static constexpr uint32_t id = 2; };
struct MassValue    { static constexpr uint32_t id = 3; };
struct Angle        { static constexpr uint32_t id = 4; };
struct AngularVel   { static constexpr uint32_t id = 5; };
struct Torque       { static constexpr uint32_t id = 6; };

constexpr uint32_t MAX_FUNCTIONS = 16;

// =============================================================================
// FUNCTION MASK UTILITIES
// =============================================================================

template<typename... Tags>
constexpr uint32_t FunctionMask = ((1u << Tags::id) | ...);

template<typename Tag>
constexpr bool hasFunction(uint32_t mask) {
    return (mask & (1u << Tag::id)) != 0;
}

// =============================================================================
// NODE VALUES - Data exchanged between nodes
// =============================================================================

template<typename T>
struct NodeData {
    std::array<T, 2> position = {T(0), T(0)};
    std::array<T, 2> velocity = {T(0), T(0)};
    std::array<T, 2> force = {T(0), T(0)};
    T mass = T(0);
    T angle = T(0);
    T angular_vel = T(0);
    T torque = T(0);

    bool has_position = false;
    bool has_velocity = false;
    bool has_mass = false;
    bool has_angle = false;
    bool has_angular_vel = false;

    void addForce(const std::array<T, 2>& f) {
        force[0] += f[0];
        force[1] += f[1];
    }

    void addTorque(T t) {
        torque += t;
    }
};

} // namespace sopot::unified
