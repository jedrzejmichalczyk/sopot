#pragma once

#include "../../core/state_function_tags.hpp"
#include <string_view>

namespace sopot::physics::kink {

//=============================================================================
// State Function Tags for Kink Chain System
//=============================================================================
// Each segment has its own TagSet containing Angle, AngularVelocity, Torque tags.
// This allows components to distinguish between segments via templates.
//=============================================================================

// Helper template to generate tags for segment i
template<size_t Index>
struct segment {
    struct Angle : categories::Kinematics {
        static constexpr std::string_view name() { return "segment_angle"; }
        static constexpr size_t type_id() { return 20000 + Index * 10 + 0; }
    };

    struct AngularVelocity : categories::Kinematics {
        static constexpr std::string_view name() { return "segment_angular_velocity"; }
        static constexpr size_t type_id() { return 20000 + Index * 10 + 1; }
    };

    struct Torque : categories::Dynamics {
        static constexpr std::string_view name() { return "segment_torque"; }
        static constexpr size_t type_id() { return 20000 + Index * 10 + 2; }
    };

    struct MomentOfInertia : categories::Dynamics {
        static constexpr std::string_view name() { return "segment_inertia"; }
        static constexpr size_t type_id() { return 20000 + Index * 10 + 3; }
    };
};

// Tags for torsional spring properties
namespace spring {
    struct AngleDifference : categories::Analysis {
        static constexpr std::string_view name() { return "spring_angle_diff"; }
        static constexpr size_t type_id() { return 20100; }
    };

    struct TorsionalStiffness : categories::Analysis {
        static constexpr std::string_view name() { return "torsional_stiffness"; }
        static constexpr size_t type_id() { return 20101; }
    };

    struct PotentialEnergy : categories::Analysis {
        static constexpr std::string_view name() { return "spring_potential_energy"; }
        static constexpr size_t type_id() { return 20102; }
    };
}

// Tags for system-level quantities
namespace system {
    struct TotalEnergy : categories::Analysis {
        static constexpr std::string_view name() { return "total_energy"; }
        static constexpr size_t type_id() { return 20200; }
    };

    struct WindingNumber : categories::Analysis {
        static constexpr std::string_view name() { return "winding_number"; }
        static constexpr size_t type_id() { return 20201; }
    };

    struct TotalAngularMomentum : categories::Dynamics {
        static constexpr std::string_view name() { return "total_angular_momentum"; }
        static constexpr size_t type_id() { return 20202; }
    };
}

} // namespace sopot::physics::kink
