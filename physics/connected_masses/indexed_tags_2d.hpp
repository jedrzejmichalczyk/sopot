#pragma once

#include "core/state_function_tags.hpp"
#include <array>

namespace sopot::connected_masses {

/**
 * @brief State function tags for 2D indexed masses
 *
 * Each mass with index I provides:
 * - MassTag2D<I>::Position - returns std::array<T, 2> for [x, y]
 * - MassTag2D<I>::Velocity - returns std::array<T, 2> for [vx, vy]
 * - MassTag2D<I>::Force - returns std::array<T, 2> for [Fx, Fy]
 * - MassTag2D<I>::Mass - returns T for mass value
 */
template<size_t Index>
struct MassTag2D {
    struct Position : StateFunction {
        static constexpr auto description = "Position (x,y) of mass";
        using return_type = std::array<double, 2>;  // Will be templated at usage
    };

    struct Velocity : StateFunction {
        static constexpr auto description = "Velocity (vx,vy) of mass";
        using return_type = std::array<double, 2>;
    };

    struct Force : StateFunction {
        static constexpr auto description = "Total force (Fx,Fy) on mass";
        using return_type = std::array<double, 2>;
    };

    struct Mass : StateFunction {
        static constexpr auto description = "Mass value";
        using return_type = double;
    };
};

/**
 * @brief State function tags for 2D springs connecting masses I and J
 *
 * Each spring provides:
 * - SpringTag2D<I,J>::Extension - returns T (positive = stretched, negative = compressed)
 * - SpringTag2D<I,J>::Length - returns T (current length)
 * - SpringTag2D<I,J>::Force - returns std::array<T, 2> force on mass I (Newton's 3rd law applies to J)
 * - SpringTag2D<I,J>::PotentialEnergy - returns T (elastic potential energy)
 */
template<size_t I, size_t J>
struct SpringTag2D {
    static_assert(I != J, "Spring cannot connect mass to itself");

    struct Extension : StateFunction {
        static constexpr auto description = "Spring extension (current - rest length)";
        using return_type = double;
    };

    struct Length : StateFunction {
        static constexpr auto description = "Current spring length";
        using return_type = double;
    };

    struct Force : StateFunction {
        static constexpr auto description = "Spring force on mass I";
        using return_type = std::array<double, 2>;
    };

    struct PotentialEnergy : StateFunction {
        static constexpr auto description = "Spring elastic potential energy";
        using return_type = double;
    };
};

} // namespace sopot::connected_masses
