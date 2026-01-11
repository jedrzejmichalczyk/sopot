#pragma once

#include "../core/typed_component.hpp"
#include "../io/csv_parser.hpp"
#include "../io/interpolation.hpp"
#include "rocket_tags.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"
#include <cmath>
#include <span>

namespace sopot::rocket {

/**
 * AxisymmetricAerodynamics: Aerodynamic forces for axisymmetric rocket
 *
 * Uses interpolated coefficients: Cd(AoA, Mach), Cl(AoA, Mach), Cs(AoSS, Mach)
 *
 * State (0 elements): No own state
 *
 * Provides: aero::AeroForceBody, aero::AeroForceENU, aero::AeroMomentBody,
 *           aero::AngleOfAttack, aero::AngleOfSideslip, aero::MachNumber,
 *           aero::DynamicPressure, aero::Airspeed
 * Requires: kinematics::VelocityENU, kinematics::AttitudeQuaternion,
 *           kinematics::AngularVelocity, kinematics::Altitude,
 *           environment::AtmosphericDensity, environment::SpeedOfSound
 */
template<Scalar T = double>
class AxisymmetricAerodynamics final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    // Aerodynamic coefficients (2D tables: AoA x Mach)
    io::BilinearInterpolator<T> m_cd_power_off;     // Drag coefficient
    io::BilinearInterpolator<T> m_cd_power_on;
    io::BilinearInterpolator<T> m_cl_power_off;     // Lift coefficient (normal force)
    io::BilinearInterpolator<T> m_cl_power_on;
    io::BilinearInterpolator<T> m_cs_power_off;     // Side force coefficient
    io::BilinearInterpolator<T> m_cs_power_on;
    io::BilinearInterpolator<T> m_cmq_x;            // Damping moment coefficient X
    io::BilinearInterpolator<T> m_cmq_yz;           // Damping moment coefficient YZ

    double m_reference_area{0.159};    // Reference area [m²] (default for 0.45m diameter)
    double m_reference_length{1.0};    // Reference length [m]
    bool m_engine_on{true};            // Use power-on coefficients
    std::string m_name{"aerodynamics"};

public:
    AxisymmetricAerodynamics(
        double diameter = 0.45,
        std::string_view name = "aerodynamics"
    ) : m_name(name) {
        m_reference_area = 3.14159265358979 * diameter * diameter / 4.0;
        m_reference_length = diameter;
    }

    void setOffset(size_t) const {} // No state

    std::string_view getComponentType() const { return "AxisymmetricAerodynamics"; }
    std::string_view getComponentName() const { return m_name; }

    void setReferenceArea(double area) { m_reference_area = area; }
    void setReferenceDiameter(double d) {
        m_reference_area = 3.14159265358979 * d * d / 4.0;
        m_reference_length = d;
    }
    void setEngineOn(bool on) { m_engine_on = on; }

    // Load coefficient tables from files
    void loadCdPowerOff(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        m_cd_power_off.setupFromTable(table.data, "cd_off");
    }

    void loadCdPowerOn(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        m_cd_power_on.setupFromTable(table.data, "cd_on");
    }

    void loadClPowerOff(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        m_cl_power_off.setupFromTable(table.data, "cl_off");
    }

    void loadClPowerOn(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        m_cl_power_on.setupFromTable(table.data, "cl_on");
    }

    void loadCsPowerOff(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        m_cs_power_off.setupFromTable(table.data, "cs_off");
    }

    void loadCsPowerOn(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        m_cs_power_on.setupFromTable(table.data, "cs_on");
    }

    void loadDampingX(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        m_cmq_x.setupFromTable(table.data, "cmq_x");
    }

    void loadDampingYZ(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        m_cmq_yz.setupFromTable(table.data, "cmq_yz");
    }

    LocalState getInitialLocalState() const { return {}; }

    // Compute angle of attack from body velocity [rad]
    T computeAoA(const Vector3<T>& velocity_body) const {
        using std::atan2;
        T vx = velocity_body.x;
        T vz = velocity_body.z;
        if (value_of(vx * vx + vz * vz) < 1e-10) return T(0);
        return atan2(-vz, vx);
    }

    // Compute angle of sideslip from body velocity [rad]
    T computeAoSS(const Vector3<T>& velocity_body) const {
        using std::atan2;
        T vx = velocity_body.x;
        T vy = velocity_body.y;
        if (value_of(vx * vx + vy * vy) < 1e-10) return T(0);
        return atan2(vy, vx);
    }

    // Get drag coefficient
    // Table format: rows = Mach, columns = AoA (degrees)
    T getCd(T aoa_deg, T mach) const {
        if (m_engine_on && !m_cd_power_on.empty()) {
            return m_cd_power_on.interpolate(mach, aoa_deg);
        }
        if (!m_cd_power_off.empty()) {
            return m_cd_power_off.interpolate(mach, aoa_deg);
        }
        return T(0.3);  // Default
    }

    // Get lift coefficient
    // Table format: rows = Mach, columns = AoA (degrees)
    T getCl(T aoa_deg, T mach) const {
        if (m_engine_on && !m_cl_power_on.empty()) {
            return m_cl_power_on.interpolate(mach, aoa_deg);
        }
        if (!m_cl_power_off.empty()) {
            return m_cl_power_off.interpolate(mach, aoa_deg);
        }
        return T(0);
    }

    // Get side force coefficient
    // Table format: rows = Mach, columns = AoSS (degrees)
    T getCs(T aoss_deg, T mach) const {
        if (m_engine_on && !m_cs_power_on.empty()) {
            return m_cs_power_on.interpolate(mach, aoss_deg);
        }
        if (!m_cs_power_off.empty()) {
            return m_cs_power_off.interpolate(mach, aoss_deg);
        }
        return T(0);
    }

    // Compute aerodynamic forces in body frame
    template<typename Registry>
    Vector3<T> computeAeroForceBody(std::span<const T> state, const Registry& registry) const {
        using std::abs;
        using std::sqrt;
        constexpr T deg2rad = T(3.14159265358979) / T(180);
        constexpr T rad2deg = T(180) / T(3.14159265358979);

        // Get velocity in ENU frame
        Vector3<T> vel_enu = registry.template computeFunction<kinematics::VelocityENU>(state);

        // Transform to body frame
        Quaternion<T> q = registry.template computeFunction<kinematics::AttitudeQuaternion>(state);
        Vector3<T> vel_body = q.rotate_reference_to_body(vel_enu);

        // Airspeed
        T airspeed = vel_body.norm();
        if (value_of(airspeed) < 1.0) {
            return Vector3<T>::zero();  // Too slow for meaningful aero
        }

        // Get atmospheric properties
        T altitude = registry.template computeFunction<kinematics::Altitude>(state);
        T density = registry.template computeFunction<environment::AtmosphericDensity>(state);
        T speed_of_sound = registry.template computeFunction<environment::SpeedOfSound>(state);

        // Mach number and dynamic pressure
        T mach = airspeed / speed_of_sound;
        T dynamic_pressure = T(0.5) * density * airspeed * airspeed;

        // Angles of attack and sideslip
        T aoa = computeAoA(vel_body);
        T aoss = computeAoSS(vel_body);
        T aoa_deg = abs(aoa * rad2deg);
        T aoss_deg = abs(aoss * rad2deg);

        // Get coefficients
        T cd = getCd(aoa_deg, mach);
        T cl = getCl(aoa_deg, mach);
        T cs = getCs(aoss_deg, mach);

        // Forces: F = q * S * C
        T qS = dynamic_pressure * T(m_reference_area);

        // Drag is opposite to velocity direction
        Vector3<T> vel_unit = vel_body.normalized();
        T drag = qS * cd;

        // Lift in vertical plane (perpendicular to velocity in XZ plane)
        // Side force in horizontal plane
        using std::sin;
        using std::cos;
        T lift = qS * cl;
        T side = qS * cs;

        // Force in body frame:
        // - Drag opposite to velocity
        // - Lift perpendicular to velocity in XZ plane
        // - Side force perpendicular to velocity in XY plane
        Vector3<T> F_drag = -vel_unit * drag;

        // Lift direction (perpendicular to velocity in pitch plane)
        Vector3<T> lift_dir = {-sin(aoa), T(0), cos(aoa)};
        if (value_of(aoa) < 0) lift_dir.z = -lift_dir.z;
        Vector3<T> F_lift = lift_dir * lift;

        // Side force direction
        Vector3<T> side_dir = {T(0), -sin(aoss), T(0)};
        if (value_of(aoss) < 0) side_dir.y = -side_dir.y;
        Vector3<T> F_side = side_dir * side;

        return F_drag + F_lift + F_side;
    }

    // Compute aerodynamic forces in ENU frame
    template<typename Registry>
    Vector3<T> computeAeroForceENU(std::span<const T> state, const Registry& registry) const {
        Vector3<T> F_body = computeAeroForceBody(state, registry);
        Quaternion<T> q = registry.template computeFunction<kinematics::AttitudeQuaternion>(state);
        return q.rotate_body_to_reference(F_body);
    }

    // Compute aerodynamic moments in body frame (simplified: damping only)
    template<typename Registry>
    Vector3<T> computeAeroMomentBody(std::span<const T> state, const Registry& registry) const {
        // Get angular velocity
        Vector3<T> omega = registry.template computeFunction<kinematics::AngularVelocity>(state);

        // Get atmospheric properties for dynamic pressure
        Vector3<T> vel_enu = registry.template computeFunction<kinematics::VelocityENU>(state);
        Quaternion<T> q = registry.template computeFunction<kinematics::AttitudeQuaternion>(state);
        Vector3<T> vel_body = q.rotate_reference_to_body(vel_enu);
        T airspeed = vel_body.norm();

        if (value_of(airspeed) < 1.0) {
            return Vector3<T>::zero();
        }

        T density = registry.template computeFunction<environment::AtmosphericDensity>(state);
        T dynamic_pressure = T(0.5) * density * airspeed * airspeed;
        T qSd = dynamic_pressure * T(m_reference_area) * T(m_reference_length);

        // Damping moments: M = q * S * d * Cmq * (omega * d / V)
        T damping_factor = T(m_reference_length) / airspeed;

        // Simplified damping (using default coefficient if tables not loaded)
        T cmq = T(-450.0);  // Default damping coefficient
        if (!m_cmq_yz.empty()) {
            T speed_of_sound = registry.template computeFunction<environment::SpeedOfSound>(state);
            T mach = airspeed / speed_of_sound;
            T aoa_deg = value_of(computeAoA(vel_body)) * T(180) / T(3.14159265358979);
            cmq = m_cmq_yz.interpolate(aoa_deg, mach);
        }

        return {
            cmq * qSd * damping_factor * omega.x,
            cmq * qSd * damping_factor * omega.y,
            cmq * qSd * damping_factor * omega.z
        };
    }

    //=========================================================================
    // STATE FUNCTIONS - For registry queries
    //=========================================================================

    // Fallback: zero force (without registry)
    Vector3<T> compute(aero::AeroForceBody, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    // Registry-aware: compute aerodynamic force in body frame
    template<typename Registry>
    Vector3<T> compute(aero::AeroForceBody, std::span<const T> state, const Registry& registry) const {
        return computeAeroForceBody(state, registry);
    }

    // Fallback: zero force ENU
    Vector3<T> compute(aero::AeroForceENU, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    // Registry-aware: compute aerodynamic force in ENU frame
    template<typename Registry>
    Vector3<T> compute(aero::AeroForceENU, std::span<const T> state, const Registry& registry) const {
        return computeAeroForceENU(state, registry);
    }

    // Fallback: zero moment
    Vector3<T> compute(aero::AeroMomentBody, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    // Registry-aware: compute aerodynamic moment in body frame
    template<typename Registry>
    Vector3<T> compute(aero::AeroMomentBody, std::span<const T> state, const Registry& registry) const {
        return computeAeroMomentBody(state, registry);
    }
};

// Factory function
template<Scalar T = double>
AxisymmetricAerodynamics<T> createAxisymmetricAerodynamics(
    double diameter = 0.45,
    std::string_view name = "aerodynamics"
) {
    return AxisymmetricAerodynamics<T>(diameter, name);
}

} // namespace sopot::rocket
