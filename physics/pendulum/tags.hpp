#pragma once

#include "core/state_function_tags.hpp"
#include <array>

namespace sopot::pendulum {

/**
 * @brief State function tags for double pendulum systems
 *
 * The double pendulum has two masses connected by rigid rods.
 * Mass 1 is connected to the pivot, Mass 2 is connected to Mass 1.
 *
 * Generalized coordinates: theta1, theta2 (angles from vertical)
 * Generalized velocities: omega1, omega2 (angular velocities)
 */

// Tags for Mass 1 (upper mass, connected to pivot)
namespace mass1 {
    struct Angle : StateFunction {
        static constexpr auto description = "Angle of rod 1 from vertical (rad)";
        using return_type = double;
    };

    struct AngularVelocity : StateFunction {
        static constexpr auto description = "Angular velocity of rod 1 (rad/s)";
        using return_type = double;
    };

    struct CartesianPosition : StateFunction {
        static constexpr auto description = "Cartesian position [x, y] of mass 1 (m)";
        using return_type = std::array<double, 2>;
    };

    struct CartesianVelocity : StateFunction {
        static constexpr auto description = "Cartesian velocity [vx, vy] of mass 1 (m/s)";
        using return_type = std::array<double, 2>;
    };

    struct Mass : StateFunction {
        static constexpr auto description = "Mass value of mass 1 (kg)";
        using return_type = double;
    };
}

// Tags for Mass 2 (lower mass, connected to mass 1)
namespace mass2 {
    struct Angle : StateFunction {
        static constexpr auto description = "Angle of rod 2 from vertical (rad)";
        using return_type = double;
    };

    struct AngularVelocity : StateFunction {
        static constexpr auto description = "Angular velocity of rod 2 (rad/s)";
        using return_type = double;
    };

    struct CartesianPosition : StateFunction {
        static constexpr auto description = "Cartesian position [x, y] of mass 2 (m)";
        using return_type = std::array<double, 2>;
    };

    struct CartesianVelocity : StateFunction {
        static constexpr auto description = "Cartesian velocity [vx, vy] of mass 2 (m/s)";
        using return_type = std::array<double, 2>;
    };

    struct Mass : StateFunction {
        static constexpr auto description = "Mass value of mass 2 (kg)";
        using return_type = double;
    };
}

// System-level tags
namespace system {
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

    struct Length1 : StateFunction {
        static constexpr auto description = "Length of rod 1 (m)";
        using return_type = double;
    };

    struct Length2 : StateFunction {
        static constexpr auto description = "Length of rod 2 (m)";
        using return_type = double;
    };
}

} // namespace sopot::pendulum
