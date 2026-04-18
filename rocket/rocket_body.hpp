#pragma once

#include "../core/typed_component.hpp"
#include "../io/csv_parser.hpp"
#include "../io/interpolation.hpp"
#include "vehicle_tags.hpp"
#include "vector3.hpp"
#include <span>

namespace sopot::rocket {

/**
 * RocketBody: Time-varying mass properties for a single vehicle.
 *
 * Provides: VehicleTags<V>::Mass, MomentOfInertia, CenterOfMass
 * Requires: sim::Time
 */
template<VehicleConcept Vehicle, Scalar T = double>
class RocketBody final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    io::LinearInterpolator<T> m_mass;
    io::LinearInterpolator<T> m_cog;
    io::LinearInterpolator<T> m_ix;
    io::LinearInterpolator<T> m_iyz;

    double m_const_mass{100.0};
    double m_const_cog{1.5};
    double m_const_ix{10.0};
    double m_const_iyz{100.0};

    bool m_use_interpolation{false};
    std::string m_name{"rocket_body"};

public:
    RocketBody(std::string_view name = "rocket_body") : m_name(name) {}

    void setConstantMass(double mass) { m_const_mass = mass; }
    void setConstantCoG(double cog) { m_const_cog = cog; }
    void setConstantInertia(double ix, double iyz) {
        m_const_ix = ix;
        m_const_iyz = iyz;
    }

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

    void setOffset(size_t) const {}

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "RocketBody"; }
    std::string_view getComponentName() const { return m_name; }

    T getMass(T time) const {
        if (m_use_interpolation && !m_mass.empty()) {
            return m_mass.interpolate(time);
        }
        return T(m_const_mass);
    }

    const io::LinearInterpolator<T>& getMassInterpolator() const { return m_mass; }
    bool usesInterpolation() const { return m_use_interpolation; }

    T getCoG(T time) const {
        if (m_use_interpolation && !m_cog.empty()) {
            return m_cog.interpolate(time);
        }
        return T(m_const_cog);
    }

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

        return {ix, iyz, iyz};
    }

    T compute(typename Tags::Mass, [[maybe_unused]] std::span<const T>) const {
        return T(m_const_mass);
    }

    template<typename Registry>
    T compute(typename Tags::Mass, std::span<const T> state, const Registry& registry) const {
        if (m_use_interpolation && !m_mass.empty()) {
            T time = registry.template computeFunction<sim::Time>(state);
            return m_mass.interpolate(time);
        }
        return T(m_const_mass);
    }

    Vector3<T> compute(typename Tags::MomentOfInertia, [[maybe_unused]] std::span<const T>) const {
        return {T(m_const_ix), T(m_const_iyz), T(m_const_iyz)};
    }

    template<typename Registry>
    Vector3<T> compute(typename Tags::MomentOfInertia, std::span<const T> state, const Registry& registry) const {
        if (m_use_interpolation) {
            T time = registry.template computeFunction<sim::Time>(state);
            return getMomentOfInertia(time);
        }
        return {T(m_const_ix), T(m_const_iyz), T(m_const_iyz)};
    }
};

template<VehicleConcept Vehicle, Scalar T = double>
RocketBody<Vehicle, T> createRocketBody(std::string_view name = "rocket_body") {
    return RocketBody<Vehicle, T>(name);
}

template<VehicleConcept Vehicle, Scalar T = double>
RocketBody<Vehicle, T> createRocketBody(
    double mass,
    double cog,
    double ix,
    double iyz,
    std::string_view name = "rocket_body"
) {
    RocketBody<Vehicle, T> body(name);
    body.setConstantMass(mass);
    body.setConstantCoG(cog);
    body.setConstantInertia(ix, iyz);
    return body;
}

} // namespace sopot::rocket
