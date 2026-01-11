#pragma once

#include "../core/typed_component.hpp"
#include "../io/csv_parser.hpp"
#include "../io/interpolation.hpp"
#include "rocket_tags.hpp"
#include "vector3.hpp"
#include <span>

namespace sopot::rocket {

/**
 * RocketBody: Time-varying mass properties
 *
 * Loads mass, center of gravity, and moments of inertia from CSV files
 *
 * State (0 elements): No own state (uses simulation time)
 *
 * Provides: dynamics::Mass, dynamics::MomentOfInertia, dynamics::CenterOfMass
 * Requires: propulsion::Time
 */
template<Scalar T = double>
class RocketBody final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    io::LinearInterpolator<T> m_mass;     // Mass vs time [kg]
    io::LinearInterpolator<T> m_cog;      // Center of gravity X vs time [m]
    io::LinearInterpolator<T> m_ix;       // Moment of inertia X vs time [kg·m²]
    io::LinearInterpolator<T> m_iyz;      // Moment of inertia Y=Z vs time [kg·m²]

    // Constant values (if not interpolated)
    double m_const_mass{100.0};
    double m_const_cog{1.5};
    double m_const_ix{10.0};
    double m_const_iyz{100.0};

    bool m_use_interpolation{false};
    std::string m_name{"rocket_body"};

public:
    RocketBody(std::string_view name = "rocket_body") : m_name(name) {}

    // Set constant values
    void setConstantMass(double mass) { m_const_mass = mass; }
    void setConstantCoG(double cog) { m_const_cog = cog; }
    void setConstantInertia(double ix, double iyz) {
        m_const_ix = ix;
        m_const_iyz = iyz;
    }

    // Load from files
    void loadMass(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        if (table.cols() >= 2) {
            m_mass.setup(table.column(0), table.column(1), "mass");
            m_use_interpolation = true;
        }
    }

    void loadCoG(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        if (table.cols() >= 2) {
            m_cog.setup(table.column(0), table.column(1), "cog");
            m_use_interpolation = true;
        }
    }

    void loadInertiaX(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        if (table.cols() >= 2) {
            m_ix.setup(table.column(0), table.column(1), "ix");
            m_use_interpolation = true;
        }
    }

    void loadInertiaYZ(const std::string& filename) {
        auto table = io::CsvParser::parseFile(filename);
        if (table.cols() >= 2) {
            m_iyz.setup(table.column(0), table.column(1), "iyz");
            m_use_interpolation = true;
        }
    }

    void setOffset(size_t) const {} // No state

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "RocketBody"; }
    std::string_view getComponentName() const { return m_name; }

    // Get mass at time t
    T getMass(T time) const {
        if (m_use_interpolation && !m_mass.empty()) {
            return m_mass.interpolate(time);
        }
        return T(m_const_mass);
    }

    // Direct interpolator access for benchmarking
    const io::LinearInterpolator<T>& getMassInterpolator() const { return m_mass; }
    bool usesInterpolation() const { return m_use_interpolation; }

    // Get center of gravity at time t
    T getCoG(T time) const {
        if (m_use_interpolation && !m_cog.empty()) {
            return m_cog.interpolate(time);
        }
        return T(m_const_cog);
    }

    // Get moment of inertia at time t
    Vector3<T> getMomentOfInertia(T time) const {
        T ix, iyz;
        if (m_use_interpolation && !m_ix.empty()) {
            ix = m_ix.interpolate(time);
        } else {
            ix = T(m_const_ix);
        }

        if (m_use_interpolation && !m_iyz.empty()) {
            iyz = m_iyz.interpolate(time);
        } else {
            iyz = T(m_const_iyz);
        }

        return {ix, iyz, iyz};  // Axisymmetric: Iy = Iz
    }

    // State function: Mass (fallback without registry - uses constant)
    T compute(dynamics::Mass, [[maybe_unused]] std::span<const T>) const {
        return T(m_const_mass);
    }

    // State function: Mass - registry-aware version queries Time state function
    template<typename Registry>
    T compute(dynamics::Mass, std::span<const T> state, const Registry& registry) const {
        if (m_use_interpolation && !m_mass.empty()) {
            T time = registry.template computeFunction<propulsion::Time>(state);
            return m_mass.interpolate(time);
        }
        return T(m_const_mass);
    }

    // State function: Moment of inertia (fallback without registry)
    Vector3<T> compute(dynamics::MomentOfInertia, [[maybe_unused]] std::span<const T>) const {
        return {T(m_const_ix), T(m_const_iyz), T(m_const_iyz)};
    }

    // State function: Moment of inertia - registry-aware version
    template<typename Registry>
    Vector3<T> compute(dynamics::MomentOfInertia, std::span<const T> state, const Registry& registry) const {
        if (m_use_interpolation) {
            T time = registry.template computeFunction<propulsion::Time>(state);
            return getMomentOfInertia(time);
        }
        return {T(m_const_ix), T(m_const_iyz), T(m_const_iyz)};
    }
};

// Factory function
template<Scalar T = double>
RocketBody<T> createRocketBody(std::string_view name = "rocket_body") {
    return RocketBody<T>(name);
}

template<Scalar T = double>
RocketBody<T> createRocketBody(
    double mass,
    double cog,
    double ix,
    double iyz,
    std::string_view name = "rocket_body"
) {
    RocketBody<T> body(name);
    body.setConstantMass(mass);
    body.setConstantCoG(cog);
    body.setConstantInertia(ix, iyz);
    return body;
}

} // namespace sopot::rocket
