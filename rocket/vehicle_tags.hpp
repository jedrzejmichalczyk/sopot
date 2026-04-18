#pragma once

#include "../core/state_function_tags.hpp"
#include <string_view>

namespace sopot::rocket {

// Vehicle role markers. Every rocket-stack tag is scoped by one of these so
// two vehicles can coexist in a single registry without tag collisions.
struct Missile {
    static constexpr size_t id = 1;
    static constexpr std::string_view name() { return "missile"; }
};

struct Interceptor {
    static constexpr size_t id = 2;
    static constexpr std::string_view name() { return "interceptor"; }
};

template<typename V>
concept VehicleConcept = requires {
    { V::id } -> std::convertible_to<size_t>;
    { V::name() } -> std::convertible_to<std::string_view>;
};

namespace categories {
    struct RocketKinematics  : sopot::StateFunction { static constexpr std::string_view category() { return "rocket_kinematics"; } };
    struct RocketDynamics    : sopot::StateFunction { static constexpr std::string_view category() { return "rocket_dynamics"; } };
    struct RocketAero        : sopot::StateFunction { static constexpr std::string_view category() { return "rocket_aero"; } };
    struct RocketPropulsion  : sopot::StateFunction { static constexpr std::string_view category() { return "rocket_propulsion"; } };
    struct RocketEnvironment : sopot::StateFunction { static constexpr std::string_view category() { return "rocket_environment"; } };
}

// Simulation time is the one truly global tag — shared by every vehicle.
namespace sim {
    struct Time : sopot::StateFunction {
        static constexpr std::string_view category() { return "simulation"; }
        static constexpr std::string_view name() { return "time"; }
        static constexpr size_t type_id() { return 900; }
    };
}

// All vehicle-scoped tags live here. Distinct vehicle types produce distinct
// nested tag types (C++ type identity), so the registry sees them as different
// state functions even though their names are identical.
//
// Type-ID scheme: Vehicle::id * 1000 + local offset. Missile -> 1100..., Interceptor -> 2100....
template<VehicleConcept Vehicle>
struct VehicleTags {
    static constexpr size_t base = Vehicle::id * 1000;

    // --- Kinematics (offsets 100..) ---
    struct PositionENU : categories::RocketKinematics {
        static constexpr std::string_view name() { return "position_enu"; }
        static constexpr size_t type_id() { return base + 100; }
    };
    struct VelocityENU : categories::RocketKinematics {
        static constexpr std::string_view name() { return "velocity_enu"; }
        static constexpr size_t type_id() { return base + 101; }
    };
    struct VelocityBody : categories::RocketKinematics {
        static constexpr std::string_view name() { return "velocity_body"; }
        static constexpr size_t type_id() { return base + 102; }
    };
    struct AttitudeQuaternion : categories::RocketKinematics {
        static constexpr std::string_view name() { return "attitude_quaternion"; }
        static constexpr size_t type_id() { return base + 103; }
    };
    struct AngularVelocity : categories::RocketKinematics {
        static constexpr std::string_view name() { return "angular_velocity"; }
        static constexpr size_t type_id() { return base + 104; }
    };
    struct Altitude : categories::RocketKinematics {
        static constexpr std::string_view name() { return "altitude"; }
        static constexpr size_t type_id() { return base + 105; }
    };

    // --- Dynamics (offsets 200..) ---
    struct TotalForceENU : categories::RocketDynamics {
        static constexpr std::string_view name() { return "total_force_enu"; }
        static constexpr size_t type_id() { return base + 200; }
    };
    struct TotalTorqueBody : categories::RocketDynamics {
        static constexpr std::string_view name() { return "total_torque_body"; }
        static constexpr size_t type_id() { return base + 201; }
    };
    struct Mass : categories::RocketDynamics {
        static constexpr std::string_view name() { return "mass"; }
        static constexpr size_t type_id() { return base + 202; }
    };
    struct MomentOfInertia : categories::RocketDynamics {
        static constexpr std::string_view name() { return "moment_of_inertia"; }
        static constexpr size_t type_id() { return base + 203; }
    };
    struct CenterOfMass : categories::RocketDynamics {
        static constexpr std::string_view name() { return "center_of_mass"; }
        static constexpr size_t type_id() { return base + 204; }
    };
    struct GravityForce : categories::RocketDynamics {
        static constexpr std::string_view name() { return "gravity_force"; }
        static constexpr size_t type_id() { return base + 205; }
    };
    struct GravityAcceleration : categories::RocketDynamics {
        static constexpr std::string_view name() { return "gravity_acceleration"; }
        static constexpr size_t type_id() { return base + 206; }
    };
    // Lateral acceleration command from a guidance law, expressed in ENU [m/s^2].
    // Every vehicle provides this tag; ballistic vehicles return zero.
    struct GuidanceCommandENU : categories::RocketDynamics {
        static constexpr std::string_view name() { return "guidance_command_enu"; }
        static constexpr size_t type_id() { return base + 207; }
    };

    // --- Aero (offsets 300..) ---
    struct AeroForceBody : categories::RocketAero {
        static constexpr std::string_view name() { return "aero_force_body"; }
        static constexpr size_t type_id() { return base + 300; }
    };
    struct AeroForceENU : categories::RocketAero {
        static constexpr std::string_view name() { return "aero_force_enu"; }
        static constexpr size_t type_id() { return base + 301; }
    };
    struct AeroMomentBody : categories::RocketAero {
        static constexpr std::string_view name() { return "aero_moment_body"; }
        static constexpr size_t type_id() { return base + 302; }
    };
    struct AngleOfAttack : categories::RocketAero {
        static constexpr std::string_view name() { return "angle_of_attack"; }
        static constexpr size_t type_id() { return base + 303; }
    };
    struct AngleOfSideslip : categories::RocketAero {
        static constexpr std::string_view name() { return "angle_of_sideslip"; }
        static constexpr size_t type_id() { return base + 304; }
    };
    struct MachNumber : categories::RocketAero {
        static constexpr std::string_view name() { return "mach_number"; }
        static constexpr size_t type_id() { return base + 305; }
    };
    struct DynamicPressure : categories::RocketAero {
        static constexpr std::string_view name() { return "dynamic_pressure"; }
        static constexpr size_t type_id() { return base + 306; }
    };
    struct Airspeed : categories::RocketAero {
        static constexpr std::string_view name() { return "airspeed"; }
        static constexpr size_t type_id() { return base + 307; }
    };

    // --- Propulsion (offsets 400..) ---
    struct ThrustForceBody : categories::RocketPropulsion {
        static constexpr std::string_view name() { return "thrust_force_body"; }
        static constexpr size_t type_id() { return base + 400; }
    };
    struct ThrustForceENU : categories::RocketPropulsion {
        static constexpr std::string_view name() { return "thrust_force_enu"; }
        static constexpr size_t type_id() { return base + 401; }
    };
    struct MassFlowRate : categories::RocketPropulsion {
        static constexpr std::string_view name() { return "mass_flow_rate"; }
        static constexpr size_t type_id() { return base + 402; }
    };
    struct EngineActive : categories::RocketPropulsion {
        static constexpr std::string_view name() { return "engine_active"; }
        static constexpr size_t type_id() { return base + 403; }
    };

    // --- Environment (offsets 500..) ---
    // Atmosphere is queried at the vehicle's own altitude, so each vehicle has
    // its own atmosphere state functions even though the USSA76 physics is shared.
    struct AtmosphericPressure : categories::RocketEnvironment {
        static constexpr std::string_view name() { return "atmospheric_pressure"; }
        static constexpr size_t type_id() { return base + 500; }
    };
    struct AtmosphericTemperature : categories::RocketEnvironment {
        static constexpr std::string_view name() { return "atmospheric_temperature"; }
        static constexpr size_t type_id() { return base + 501; }
    };
    struct AtmosphericDensity : categories::RocketEnvironment {
        static constexpr std::string_view name() { return "atmospheric_density"; }
        static constexpr size_t type_id() { return base + 502; }
    };
    struct SpeedOfSound : categories::RocketEnvironment {
        static constexpr std::string_view name() { return "speed_of_sound"; }
        static constexpr size_t type_id() { return base + 503; }
    };
    struct WindVelocity : categories::RocketEnvironment {
        static constexpr std::string_view name() { return "wind_velocity"; }
        static constexpr size_t type_id() { return base + 504; }
    };
};

} // namespace sopot::rocket
