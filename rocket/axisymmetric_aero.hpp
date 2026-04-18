#pragma once

#include "../core/typed_component.hpp"
#include "../io/csv_parser.hpp"
#include "../io/interpolation.hpp"
#include "vehicle_tags.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"
#include <cmath>
#include <span>

namespace sopot::rocket {

/**
 * AxisymmetricAerodynamics: Aerodynamic forces for a single axisymmetric vehicle.
 *
 * Provides: VehicleTags<V>::AeroForceBody, AeroForceENU, AeroMomentBody, ...
 * Requires: VehicleTags<V>::VelocityENU, AttitudeQuaternion, AngularVelocity,
 *           Altitude, AtmosphericDensity, SpeedOfSound
 */
template<VehicleConcept Vehicle, Scalar T = double>
class AxisymmetricAerodynamics final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    io::BilinearInterpolator<T> m_cd_power_off;
    io::BilinearInterpolator<T> m_cd_power_on;
    io::BilinearInterpolator<T> m_cl_power_off;
    io::BilinearInterpolator<T> m_cl_power_on;
    io::BilinearInterpolator<T> m_cs_power_off;
    io::BilinearInterpolator<T> m_cs_power_on;
    io::BilinearInterpolator<T> m_cmq_x;
    io::BilinearInterpolator<T> m_cmq_yz;

    double m_reference_area{0.159};
    double m_reference_length{1.0};
    bool m_engine_on{true};
    std::string m_name{"aerodynamics"};

public:
    AxisymmetricAerodynamics(
        double diameter = 0.45,
        std::string_view name = "aerodynamics"
    ) : m_name(name) {
        m_reference_area = 3.14159265358979 * diameter * diameter / 4.0;
        m_reference_length = diameter;
    }

    void setOffset(size_t) const {}

    std::string_view getComponentType() const { return "AxisymmetricAerodynamics"; }
    std::string_view getComponentName() const { return m_name; }

    void setReferenceArea(double area) { m_reference_area = area; }
    void setReferenceDiameter(double d) {
        m_reference_area = 3.14159265358979 * d * d / 4.0;
        m_reference_length = d;
    }
    void setEngineOn(bool on) { m_engine_on = on; }

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

    T computeAoA(const Vector3<T>& velocity_body) const {
        using std::atan2;
        T vx = velocity_body.x;
        T vz = velocity_body.z;
        if (value_of(vx * vx + vz * vz) < 1e-10) return T(0);
        return atan2(-vz, vx);
    }

    T computeAoSS(const Vector3<T>& velocity_body) const {
        using std::atan2;
        T vx = velocity_body.x;
        T vy = velocity_body.y;
        if (value_of(vx * vx + vy * vy) < 1e-10) return T(0);
        return atan2(vy, vx);
    }

    T getCd(T aoa_deg, T mach) const {
        if (m_engine_on && !m_cd_power_on.empty()) {
            return m_cd_power_on.interpolate(mach, aoa_deg);
        }
        if (!m_cd_power_off.empty()) {
            return m_cd_power_off.interpolate(mach, aoa_deg);
        }
        return T(0.3);
    }

    T getCl(T aoa_deg, T mach) const {
        if (m_engine_on && !m_cl_power_on.empty()) {
            return m_cl_power_on.interpolate(mach, aoa_deg);
        }
        if (!m_cl_power_off.empty()) {
            return m_cl_power_off.interpolate(mach, aoa_deg);
        }
        return T(0);
    }

    T getCs(T aoss_deg, T mach) const {
        if (m_engine_on && !m_cs_power_on.empty()) {
            return m_cs_power_on.interpolate(mach, aoss_deg);
        }
        if (!m_cs_power_off.empty()) {
            return m_cs_power_off.interpolate(mach, aoss_deg);
        }
        return T(0);
    }

    template<typename Registry>
    Vector3<T> computeAeroForceBody(std::span<const T> state, const Registry& registry) const {
        using std::abs;
        using std::sqrt;
        constexpr T rad2deg = T(180) / T(3.14159265358979);

        Vector3<T> vel_enu = registry.template computeFunction<typename Tags::VelocityENU>(state);

        Quaternion<T> q = registry.template computeFunction<typename Tags::AttitudeQuaternion>(state);
        Vector3<T> vel_body = q.rotate_reference_to_body(vel_enu);

        T airspeed = vel_body.norm();
        if (value_of(airspeed) < 1.0) {
            return Vector3<T>::zero();
        }

        T density = registry.template computeFunction<typename Tags::AtmosphericDensity>(state);
        T speed_of_sound = registry.template computeFunction<typename Tags::SpeedOfSound>(state);

        T mach = airspeed / speed_of_sound;
        T dynamic_pressure = T(0.5) * density * airspeed * airspeed;

        T aoa = computeAoA(vel_body);
        T aoss = computeAoSS(vel_body);
        T aoa_deg = abs(aoa * rad2deg);
        T aoss_deg = abs(aoss * rad2deg);

        T cd = getCd(aoa_deg, mach);
        T cl = getCl(aoa_deg, mach);
        T cs = getCs(aoss_deg, mach);

        T qS = dynamic_pressure * T(m_reference_area);

        Vector3<T> vel_unit = vel_body.normalized();
        T drag = qS * cd;

        using std::sin;
        using std::cos;
        T lift = qS * cl;
        T side = qS * cs;

        Vector3<T> F_drag = -vel_unit * drag;

        Vector3<T> lift_dir = {-sin(aoa), T(0), cos(aoa)};
        if (value_of(aoa) < 0) lift_dir.z = -lift_dir.z;
        Vector3<T> F_lift = lift_dir * lift;

        Vector3<T> side_dir = {T(0), -sin(aoss), T(0)};
        if (value_of(aoss) < 0) side_dir.y = -side_dir.y;
        Vector3<T> F_side = side_dir * side;

        return F_drag + F_lift + F_side;
    }

    template<typename Registry>
    Vector3<T> computeAeroForceENU(std::span<const T> state, const Registry& registry) const {
        Vector3<T> F_body = computeAeroForceBody(state, registry);
        Quaternion<T> q = registry.template computeFunction<typename Tags::AttitudeQuaternion>(state);
        return q.rotate_body_to_reference(F_body);
    }

    template<typename Registry>
    Vector3<T> computeAeroMomentBody(std::span<const T> state, const Registry& registry) const {
        Vector3<T> omega = registry.template computeFunction<typename Tags::AngularVelocity>(state);

        Vector3<T> vel_enu = registry.template computeFunction<typename Tags::VelocityENU>(state);
        Quaternion<T> q = registry.template computeFunction<typename Tags::AttitudeQuaternion>(state);
        Vector3<T> vel_body = q.rotate_reference_to_body(vel_enu);
        T airspeed = vel_body.norm();

        if (value_of(airspeed) < 1.0) {
            return Vector3<T>::zero();
        }

        T density = registry.template computeFunction<typename Tags::AtmosphericDensity>(state);
        T dynamic_pressure = T(0.5) * density * airspeed * airspeed;
        T qSd = dynamic_pressure * T(m_reference_area) * T(m_reference_length);

        T damping_factor = T(m_reference_length) / airspeed;

        T cmq = T(-450.0);
        if (!m_cmq_yz.empty()) {
            T speed_of_sound = registry.template computeFunction<typename Tags::SpeedOfSound>(state);
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

    Vector3<T> compute(typename Tags::AeroForceBody, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    template<typename Registry>
    Vector3<T> compute(typename Tags::AeroForceBody, std::span<const T> state, const Registry& registry) const {
        return computeAeroForceBody(state, registry);
    }

    Vector3<T> compute(typename Tags::AeroForceENU, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    template<typename Registry>
    Vector3<T> compute(typename Tags::AeroForceENU, std::span<const T> state, const Registry& registry) const {
        return computeAeroForceENU(state, registry);
    }

    Vector3<T> compute(typename Tags::AeroMomentBody, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    template<typename Registry>
    Vector3<T> compute(typename Tags::AeroMomentBody, std::span<const T> state, const Registry& registry) const {
        return computeAeroMomentBody(state, registry);
    }
};

template<VehicleConcept Vehicle, Scalar T = double>
AxisymmetricAerodynamics<Vehicle, T> createAxisymmetricAerodynamics(
    double diameter = 0.45,
    std::string_view name = "aerodynamics"
) {
    return AxisymmetricAerodynamics<Vehicle, T>(diameter, name);
}

} // namespace sopot::rocket
