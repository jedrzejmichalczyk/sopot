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
 * InterpolatedEngine: Thrust model from interpolated data for a single vehicle.
 *
 * Provides: VehicleTags<V>::ThrustForceBody, ThrustForceENU, MassFlowRate
 * Requires: VehicleTags<V>::AttitudeQuaternion, AtmosphericPressure, sim::Time
 */
template<VehicleConcept Vehicle, Scalar T = double>
class InterpolatedEngine final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

    static constexpr double R = 8.3144598;
    static constexpr double PI = 3.14159265358979323846;

private:
    io::LinearInterpolator<T> m_throat_diameter;
    io::LinearInterpolator<T> m_exit_diameter;
    io::LinearInterpolator<T> m_combustion_pressure;
    io::LinearInterpolator<T> m_combustion_temp;
    io::LinearInterpolator<T> m_gamma;
    io::LinearInterpolator<T> m_mol_mass;
    io::LinearInterpolator<T> m_efficiency;

    io::LinearInterpolator<T> m_pressure_ratio;
    io::LinearInterpolator<T> m_mass_flow;
    io::LinearInterpolator<T> m_engine_force;

    double m_thrust_time{0.0};
    bool m_data_loaded{false};
    std::string m_name{"engine"};

public:
    InterpolatedEngine(std::string_view name = "engine") : m_name(name) {}

    void setOffset(size_t) const {}

    std::string_view getComponentType() const { return "InterpolatedEngine"; }
    std::string_view getComponentName() const { return m_name; }

    void loadFromFile(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);

        if (table.cols() < 8) {
            throw std::runtime_error("Engine CSV needs 8 columns: time, throat_d, exit_d, p, T, gamma, mol_mass, eff");
        }

        auto times = table.column(0);
        m_thrust_time = times.back();

        m_throat_diameter.setup(times, table.column(1), "throat_d");
        m_exit_diameter.setup(times, table.column(2), "exit_d");
        m_combustion_pressure.setup(times, table.column(3), "p_comb");
        m_combustion_temp.setup(times, table.column(4), "T_comb");
        m_gamma.setup(times, table.column(5), "gamma");
        m_mol_mass.setup(times, table.column(6), "mol_mass");
        m_efficiency.setup(times, table.column(7), "efficiency");

        precomputeInterpolants(times);
        m_data_loaded = true;
    }

    bool isLoaded() const { return m_data_loaded; }
    double getBurnTime() const { return m_thrust_time; }

    LocalState getInitialLocalState() const { return {}; }

    bool isActive(T time) const {
        return value_of(time) <= m_thrust_time && m_data_loaded;
    }

    T computeMassFlowRate(T time) const {
        if (!isActive(time)) return T(0);
        return m_mass_flow.interpolate(time);
    }

    T computeThrustMagnitude(T time, T atmospheric_pressure) const {
        if (!isActive(time)) return T(0);

        T d_exit = m_exit_diameter.interpolate(time);
        T p_comb = m_combustion_pressure.interpolate(time);
        T press_ratio = m_pressure_ratio.interpolate(time);

        T exit_area = T(PI) * d_exit * d_exit / T(4);
        T exit_pressure = p_comb * press_ratio;

        T engine_force = m_engine_force.interpolate(time);
        T pressure_force = exit_area * (exit_pressure - atmospheric_pressure);

        return engine_force + pressure_force;
    }

    Vector3<T> computeThrustBody(T time, T atmospheric_pressure) const {
        T thrust_mag = computeThrustMagnitude(time, atmospheric_pressure);
        return {thrust_mag, T(0), T(0)};
    }

    template<typename Registry>
    Vector3<T> computeThrustENU(std::span<const T> state, T time, const Registry& registry) const {
        T atm_pressure = registry.template computeFunction<typename Tags::AtmosphericPressure>(state);
        Vector3<T> thrust_body = computeThrustBody(time, atm_pressure);

        Quaternion<T> q = registry.template computeFunction<typename Tags::AttitudeQuaternion>(state);
        return q.rotate_body_to_reference(thrust_body);
    }

    Vector3<T> compute(typename Tags::ThrustForceBody, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    template<typename Registry>
    Vector3<T> compute(typename Tags::ThrustForceBody, std::span<const T> state, const Registry& registry) const {
        if (!m_data_loaded) {
            return Vector3<T>::zero();
        }

        T time = registry.template computeFunction<sim::Time>(state);
        if (!isActive(time)) {
            return Vector3<T>::zero();
        }

        T atm_pressure = registry.template computeFunction<typename Tags::AtmosphericPressure>(state);
        return computeThrustBody(time, atm_pressure);
    }

    T compute(typename Tags::MassFlowRate, [[maybe_unused]] std::span<const T>) const {
        return T(0);
    }

    template<typename Registry>
    T compute(typename Tags::MassFlowRate, std::span<const T> state, const Registry& registry) const {
        if (!m_data_loaded) {
            return T(0);
        }
        T time = registry.template computeFunction<sim::Time>(state);
        return computeMassFlowRate(time);
    }

private:
    void precomputeInterpolants(const std::vector<double>& times) {
        std::vector<double> ratios(times.size());
        std::vector<double> mass_flows(times.size());
        std::vector<double> forces(times.size());

        for (size_t i = 0; i < times.size(); ++i) {
            double t = times[i];

            double k = value_of(m_gamma.interpolate(T(t)));
            double d_throat = value_of(m_throat_diameter.interpolate(T(t)));
            double d_exit = value_of(m_exit_diameter.interpolate(T(t)));
            double p = value_of(m_combustion_pressure.interpolate(T(t)));
            double temp = value_of(m_combustion_temp.interpolate(T(t)));
            double mol_mass = value_of(m_mol_mass.interpolate(T(t)));
            double eff = value_of(m_efficiency.interpolate(T(t)));

            double throat_area = PI * d_throat * d_throat / 4.0;
            double exit_area = PI * d_exit * d_exit / 4.0;
            double area_ratio = exit_area / throat_area;

            ratios[i] = computePressureRatio(k, area_ratio);

            double throat_pressure = p * std::pow(2.0 / (k + 1.0), k / (k - 1.0));
            double throat_temp = temp * (2.0 / (k + 1.0));
            double R_specific = R / mol_mass;
            double throat_density = throat_pressure / (R_specific * throat_temp);
            double throat_velocity = std::sqrt(k * R_specific * throat_temp);

            mass_flows[i] = throat_density * throat_velocity * throat_area;
            if (mass_flows[i] < 0) mass_flows[i] = 0;

            double exit_velocity = eff * std::sqrt(
                2.0 * k / (k - 1.0) * R_specific * temp *
                (1.0 - std::pow(ratios[i], (k - 1.0) / k))
            );
            forces[i] = mass_flows[i] * exit_velocity;
        }

        m_pressure_ratio.setup(times, ratios, "press_ratio");
        m_mass_flow.setup(times, mass_flows, "mass_flow");
        m_engine_force.setup(times, forces, "engine_force");
    }

    static double computePressureRatio(double k, double area_ratio) {
        double pr = 0.01;
        for (int iter = 0; iter < 20; ++iter) {
            double M2 = (2.0 / (k - 1.0)) * (std::pow(1.0 / pr, (k - 1.0) / k) - 1.0);
            if (M2 < 0) M2 = 0;
            double M = std::sqrt(M2);

            double term = (2.0 / (k + 1.0)) * (1.0 + (k - 1.0) / 2.0 * M2);
            double exponent = (k + 1.0) / (2.0 * (k - 1.0));
            double area_calc = (1.0 / M) * std::pow(term, exponent);

            if (std::abs(area_calc - area_ratio) < 0.001 * area_ratio) break;

            if (area_calc > area_ratio) {
                pr *= 0.95;
            } else {
                pr *= 1.05;
            }
            if (pr < 1e-6) pr = 1e-6;
            if (pr > 1.0) pr = 1.0;
        }
        return pr;
    }
};

template<VehicleConcept Vehicle, Scalar T = double>
InterpolatedEngine<Vehicle, T> createInterpolatedEngine(std::string_view name = "engine") {
    return InterpolatedEngine<Vehicle, T>(name);
}

template<VehicleConcept Vehicle, Scalar T = double>
InterpolatedEngine<Vehicle, T> createInterpolatedEngine(const std::string& filename, std::string_view name = "engine") {
    InterpolatedEngine<Vehicle, T> engine(name);
    engine.loadFromFile(filename);
    return engine;
}

} // namespace sopot::rocket
