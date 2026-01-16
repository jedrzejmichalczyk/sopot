#pragma once

#include "core/state_function_tags.hpp"
#include <array>

namespace sopot::control {

/**
 * @brief State function tags for control systems
 *
 * These tags are used by controlled mechanical systems like the
 * inverted pendulum on a cart. They enable modular separation of
 * plant dynamics from controller logic.
 */

// Cart-related state functions
namespace cart {
    struct Position : StateFunction {
        static constexpr auto description = "Cart position along track (m)";
        using return_type = double;
    };

    struct Velocity : StateFunction {
        static constexpr auto description = "Cart velocity (m/s)";
        using return_type = double;
    };

    struct Mass : StateFunction {
        static constexpr auto description = "Cart mass (kg)";
        using return_type = double;
    };
}

// Pendulum link 1 (attached to cart)
namespace link1 {
    struct Angle : StateFunction {
        static constexpr auto description = "Angle of link 1 from vertical (rad), 0 = upright";
        using return_type = double;
    };

    struct AngularVelocity : StateFunction {
        static constexpr auto description = "Angular velocity of link 1 (rad/s)";
        using return_type = double;
    };

    struct Mass : StateFunction {
        static constexpr auto description = "Mass of link 1 (kg)";
        using return_type = double;
    };

    struct Length : StateFunction {
        static constexpr auto description = "Length of link 1 (m)";
        using return_type = double;
    };

    struct TipPosition : StateFunction {
        static constexpr auto description = "Cartesian position [x, y] of link 1 tip (m)";
        using return_type = std::array<double, 2>;
    };
}

// Pendulum link 2 (attached to link 1)
namespace link2 {
    struct Angle : StateFunction {
        static constexpr auto description = "Angle of link 2 from vertical (rad), 0 = upright";
        using return_type = double;
    };

    struct AngularVelocity : StateFunction {
        static constexpr auto description = "Angular velocity of link 2 (rad/s)";
        using return_type = double;
    };

    struct Mass : StateFunction {
        static constexpr auto description = "Mass of link 2 (kg)";
        using return_type = double;
    };

    struct Length : StateFunction {
        static constexpr auto description = "Length of link 2 (m)";
        using return_type = double;
    };

    struct TipPosition : StateFunction {
        static constexpr auto description = "Cartesian position [x, y] of link 2 tip (m)";
        using return_type = std::array<double, 2>;
    };
}

// Control system tags
namespace controller {
    struct ControlInput : StateFunction {
        static constexpr auto description = "Control force applied to cart (N)";
        using return_type = double;
    };

    struct StateError : StateFunction {
        static constexpr auto description = "Error from reference state";
        using return_type = std::array<double, 6>;  // [x, θ1, θ2, ẋ, ω1, ω2] error
    };

    struct ReferenceState : StateFunction {
        static constexpr auto description = "Reference/target state";
        using return_type = std::array<double, 6>;
    };
}

// System-level tags
namespace system {
    struct FullState : StateFunction {
        static constexpr auto description = "Full state vector [x, θ1, θ2, ẋ, ω1, ω2]";
        using return_type = std::array<double, 6>;
    };

    struct TotalEnergy : StateFunction {
        static constexpr auto description = "Total mechanical energy (J)";
        using return_type = double;
    };

    struct KineticEnergy : StateFunction {
        static constexpr auto description = "Total kinetic energy (J)";
        using return_type = double;
    };

    struct PotentialEnergy : StateFunction {
        static constexpr auto description = "Total potential energy (J)";
        using return_type = double;
    };
}

} // namespace sopot::control
