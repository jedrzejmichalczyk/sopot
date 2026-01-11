#pragma once

#include "../core/state_function_tags.hpp"
#include <string_view>

namespace sopot::rocket {

// State function tag categories for rocket simulation
// These inherit from sopot::StateFunction to satisfy StateTagConcept
namespace categories {
    struct RocketKinematics : sopot::StateFunction {
        static constexpr std::string_view category() { return "rocket_kinematics"; }
    };
    struct RocketDynamics : sopot::StateFunction {
        static constexpr std::string_view category() { return "rocket_dynamics"; }
    };
    struct RocketAero : sopot::StateFunction {
        static constexpr std::string_view category() { return "rocket_aero"; }
    };
    struct RocketPropulsion : sopot::StateFunction {
        static constexpr std::string_view category() { return "rocket_propulsion"; }
    };
    struct RocketEnvironment : sopot::StateFunction {
        static constexpr std::string_view category() { return "rocket_environment"; }
    };
}

// Kinematic state functions
namespace kinematics {

// Position in ENU frame [m]
struct PositionENU : categories::RocketKinematics {
    static constexpr std::string_view name() { return "position_enu"; }
    static constexpr size_t type_id() { return 100; }
};

// Velocity in ENU frame [m/s]
struct VelocityENU : categories::RocketKinematics {
    static constexpr std::string_view name() { return "velocity_enu"; }
    static constexpr size_t type_id() { return 101; }
};

// Velocity in body frame [m/s]
struct VelocityBody : categories::RocketKinematics {
    static constexpr std::string_view name() { return "velocity_body"; }
    static constexpr size_t type_id() { return 102; }
};

// Attitude quaternion (body to reference)
struct AttitudeQuaternion : categories::RocketKinematics {
    static constexpr std::string_view name() { return "attitude_quaternion"; }
    static constexpr size_t type_id() { return 103; }
};

// Angular velocity in body frame [rad/s]
struct AngularVelocity : categories::RocketKinematics {
    static constexpr std::string_view name() { return "angular_velocity"; }
    static constexpr size_t type_id() { return 104; }
};

// Altitude [m]
struct Altitude : categories::RocketKinematics {
    static constexpr std::string_view name() { return "altitude"; }
    static constexpr size_t type_id() { return 105; }
};

} // namespace kinematics

// Dynamic state functions
namespace dynamics {

// Total force in ENU frame [N]
struct TotalForceENU : categories::RocketDynamics {
    static constexpr std::string_view name() { return "total_force_enu"; }
    static constexpr size_t type_id() { return 200; }
};

// Total torque in body frame [N·m]
struct TotalTorqueBody : categories::RocketDynamics {
    static constexpr std::string_view name() { return "total_torque_body"; }
    static constexpr size_t type_id() { return 201; }
};

// Rocket mass [kg]
struct Mass : categories::RocketDynamics {
    static constexpr std::string_view name() { return "mass"; }
    static constexpr size_t type_id() { return 202; }
};

// Moments of inertia [kg·m²]
struct MomentOfInertia : categories::RocketDynamics {
    static constexpr std::string_view name() { return "moment_of_inertia"; }
    static constexpr size_t type_id() { return 203; }
};

// Center of mass position in body frame [m]
struct CenterOfMass : categories::RocketDynamics {
    static constexpr std::string_view name() { return "center_of_mass"; }
    static constexpr size_t type_id() { return 204; }
};

// Gravity force [N]
struct GravityForce : categories::RocketDynamics {
    static constexpr std::string_view name() { return "gravity_force"; }
    static constexpr size_t type_id() { return 205; }
};

// Gravitational acceleration magnitude [m/s²]
struct GravityAcceleration : categories::RocketDynamics {
    static constexpr std::string_view name() { return "gravity_acceleration"; }
    static constexpr size_t type_id() { return 206; }
};

} // namespace dynamics

// Aerodynamic state functions
namespace aero {

// Aerodynamic force in body frame [N]
struct AeroForceBody : categories::RocketAero {
    static constexpr std::string_view name() { return "aero_force_body"; }
    static constexpr size_t type_id() { return 300; }
};

// Aerodynamic force in ENU frame [N]
struct AeroForceENU : categories::RocketAero {
    static constexpr std::string_view name() { return "aero_force_enu"; }
    static constexpr size_t type_id() { return 301; }
};

// Aerodynamic moment in body frame [N·m]
struct AeroMomentBody : categories::RocketAero {
    static constexpr std::string_view name() { return "aero_moment_body"; }
    static constexpr size_t type_id() { return 302; }
};

// Angle of attack [rad]
struct AngleOfAttack : categories::RocketAero {
    static constexpr std::string_view name() { return "angle_of_attack"; }
    static constexpr size_t type_id() { return 303; }
};

// Angle of sideslip [rad]
struct AngleOfSideslip : categories::RocketAero {
    static constexpr std::string_view name() { return "angle_of_sideslip"; }
    static constexpr size_t type_id() { return 304; }
};

// Mach number
struct MachNumber : categories::RocketAero {
    static constexpr std::string_view name() { return "mach_number"; }
    static constexpr size_t type_id() { return 305; }
};

// Dynamic pressure [Pa]
struct DynamicPressure : categories::RocketAero {
    static constexpr std::string_view name() { return "dynamic_pressure"; }
    static constexpr size_t type_id() { return 306; }
};

// Airspeed magnitude [m/s]
struct Airspeed : categories::RocketAero {
    static constexpr std::string_view name() { return "airspeed"; }
    static constexpr size_t type_id() { return 307; }
};

} // namespace aero

// Propulsion state functions
namespace propulsion {

// Thrust force in body frame [N]
struct ThrustForceBody : categories::RocketPropulsion {
    static constexpr std::string_view name() { return "thrust_force_body"; }
    static constexpr size_t type_id() { return 400; }
};

// Thrust force in ENU frame [N]
struct ThrustForceENU : categories::RocketPropulsion {
    static constexpr std::string_view name() { return "thrust_force_enu"; }
    static constexpr size_t type_id() { return 401; }
};

// Mass flow rate [kg/s]
struct MassFlowRate : categories::RocketPropulsion {
    static constexpr std::string_view name() { return "mass_flow_rate"; }
    static constexpr size_t type_id() { return 402; }
};

// Engine on/off state
struct EngineActive : categories::RocketPropulsion {
    static constexpr std::string_view name() { return "engine_active"; }
    static constexpr size_t type_id() { return 403; }
};

// Simulation time [s]
struct Time : categories::RocketPropulsion {
    static constexpr std::string_view name() { return "time"; }
    static constexpr size_t type_id() { return 404; }
};

} // namespace propulsion

// Environment state functions
namespace environment {

// Atmospheric pressure [Pa]
struct AtmosphericPressure : categories::RocketEnvironment {
    static constexpr std::string_view name() { return "atmospheric_pressure"; }
    static constexpr size_t type_id() { return 500; }
};

// Atmospheric temperature [K]
struct AtmosphericTemperature : categories::RocketEnvironment {
    static constexpr std::string_view name() { return "atmospheric_temperature"; }
    static constexpr size_t type_id() { return 501; }
};

// Atmospheric density [kg/m³]
struct AtmosphericDensity : categories::RocketEnvironment {
    static constexpr std::string_view name() { return "atmospheric_density"; }
    static constexpr size_t type_id() { return 502; }
};

// Speed of sound [m/s]
struct SpeedOfSound : categories::RocketEnvironment {
    static constexpr std::string_view name() { return "speed_of_sound"; }
    static constexpr size_t type_id() { return 503; }
};

// Wind velocity in ENU frame [m/s]
struct WindVelocity : categories::RocketEnvironment {
    static constexpr std::string_view name() { return "wind_velocity"; }
    static constexpr size_t type_id() { return 504; }
};

} // namespace environment

} // namespace sopot::rocket
