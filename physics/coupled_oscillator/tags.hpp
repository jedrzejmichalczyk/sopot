#pragma once

#include "../../core/state_function_tags.hpp"
#include <string_view>

namespace sopot::physics::coupled {

//=============================================================================
// State Function Tags for Coupled Oscillator System
//=============================================================================
// Each mass has its own TagSet struct containing Position, Velocity, Force tags.
// This allows the Spring component to distinguish between masses via templates.
//=============================================================================

// Tags for mass 1
struct mass1 {
    struct Position : categories::Kinematics {
        static constexpr std::string_view name() { return "mass1_position"; }
        static constexpr size_t type_id() { return 10001; }
    };

    struct Velocity : categories::Kinematics {
        static constexpr std::string_view name() { return "mass1_velocity"; }
        static constexpr size_t type_id() { return 10002; }
    };

    struct Force : categories::Dynamics {
        static constexpr std::string_view name() { return "mass1_force"; }
        static constexpr size_t type_id() { return 10003; }
    };

    struct Mass : categories::Dynamics {
        static constexpr std::string_view name() { return "mass1_mass"; }
        static constexpr size_t type_id() { return 10004; }
    };
};

// Tags for mass 2
struct mass2 {
    struct Position : categories::Kinematics {
        static constexpr std::string_view name() { return "mass2_position"; }
        static constexpr size_t type_id() { return 10011; }
    };

    struct Velocity : categories::Kinematics {
        static constexpr std::string_view name() { return "mass2_velocity"; }
        static constexpr size_t type_id() { return 10012; }
    };

    struct Force : categories::Dynamics {
        static constexpr std::string_view name() { return "mass2_force"; }
        static constexpr size_t type_id() { return 10013; }
    };

    struct Mass : categories::Dynamics {
        static constexpr std::string_view name() { return "mass2_mass"; }
        static constexpr size_t type_id() { return 10014; }
    };
};

// Tags for spring properties
namespace spring {
    struct Extension : categories::Analysis {
        static constexpr std::string_view name() { return "spring_extension"; }
        static constexpr size_t type_id() { return 10021; }
    };

    struct PotentialEnergy : categories::Analysis {
        static constexpr std::string_view name() { return "spring_potential_energy"; }
        static constexpr size_t type_id() { return 10022; }
    };

    struct Stiffness : categories::Analysis {
        static constexpr std::string_view name() { return "spring_stiffness"; }
        static constexpr size_t type_id() { return 10023; }
    };
}

// Tags for system-level quantities
namespace system {
    struct TotalEnergy : categories::Analysis {
        static constexpr std::string_view name() { return "total_energy"; }
        static constexpr size_t type_id() { return 10031; }
    };

    struct CenterOfMass : categories::Kinematics {
        static constexpr std::string_view name() { return "center_of_mass"; }
        static constexpr size_t type_id() { return 10032; }
    };

    struct Momentum : categories::Dynamics {
        static constexpr std::string_view name() { return "momentum"; }
        static constexpr size_t type_id() { return 10033; }
    };
}

} // namespace sopot::physics::coupled
