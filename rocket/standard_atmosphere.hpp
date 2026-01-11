#pragma once

#include "../core/typed_component.hpp"
#include "rocket_tags.hpp"
#include <cmath>
#include <array>
#include <span>

namespace sopot::rocket {

template<Scalar T = double>
class StandardAtmosphere final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

    static constexpr double R = 8.3144598, M = 0.0289644, Rs = R / M;
    static constexpr double gamma = 1.4, g0 = 9.80665;
    static constexpr double P0 = 101325.0, T0 = 288.15, rho0 = 1.225;

private:
    struct Layer { double h_top, P_base, T_base, lapse_rate, h_base; };
    static constexpr std::array<Layer, 7> layers = {{
        {11000.0, 101325.0, 288.15, -0.0065, 0.0},
        {20000.0, 22632.0, 216.65, 0.0, 11000.0},
        {32000.0, 5474.89, 216.65, 0.001, 20000.0},
        {47000.0, 868.02, 228.65, 0.0028, 32000.0},
        {51000.0, 110.91, 270.65, 0.0, 47000.0},
        {71000.0, 66.94, 270.65, -0.0028, 51000.0},
        {100000.0, 3.96, 214.65, -0.002, 71000.0}
    }};
    std::string m_name{"atmosphere"};
    mutable size_t m_altitude_offset{3}; // Position Z is at offset 3 (after time + pos_xy)

public:
    StandardAtmosphere(std::string_view name = "atmosphere") : m_name(name) {}
    void setOffset(size_t) const {} // No state
    void setAltitudeOffset(size_t off) const { m_altitude_offset = off; }

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "StandardAtmosphere"; }
    std::string_view getComponentName() const { return m_name; }

    T computeTemperature(T altitude) const {
        double h = value_of(altitude);
        if (h <= 0) return T(T0);
        if (h >= 100000.0) return T(186.95);
        for (const auto& l : layers)
            if (h <= l.h_top)
                return std::abs(l.lapse_rate) < 1e-10 ? T(l.T_base)
                    : T(l.T_base) + T(l.lapse_rate) * (altitude - T(l.h_base));
        return T(186.95);
    }

    T computePressure(T altitude) const {
        using std::pow; using std::exp;
        double h = value_of(altitude);
        if (h <= 0) return T(P0);
        if (h >= 100000.0) return T(1e-4);
        for (const auto& l : layers) {
            if (h <= l.h_top) {
                if (std::abs(l.lapse_rate) < 1e-10)
                    return T(l.P_base) * exp(-T(g0 * M) * (altitude - T(l.h_base)) / (T(R) * T(l.T_base)));
                T temp_ratio = T(l.T_base) / (T(l.T_base) + T(l.lapse_rate) * (altitude - T(l.h_base)));
                return T(l.P_base) * pow(temp_ratio, T(g0 * M / (R * l.lapse_rate)));
            }
        }
        return T(1e-4);
    }

    T computeDensity(T alt) const { return computePressure(alt) / (T(Rs) * computeTemperature(alt)); }
    T computeSpeedOfSound(T alt) const { return std::sqrt(T(gamma * Rs) * computeTemperature(alt)); }

    // State functions query altitude from registry
    T compute(environment::AtmosphericPressure, std::span<const T> state) const {
        return computePressure(state[m_altitude_offset]);
    }
    T compute(environment::AtmosphericTemperature, std::span<const T> state) const {
        return computeTemperature(state[m_altitude_offset]);
    }
    T compute(environment::AtmosphericDensity, std::span<const T> state) const {
        return computeDensity(state[m_altitude_offset]);
    }
    T compute(environment::SpeedOfSound, std::span<const T> state) const {
        return computeSpeedOfSound(state[m_altitude_offset]);
    }
};

} // namespace sopot::rocket
