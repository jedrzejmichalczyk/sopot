#pragma once

#include "../core/typed_component.hpp"
#include "vehicle_tags.hpp"
#include <span>

namespace sopot::rocket {

template<VehicleConcept Vehicle, Scalar T = double>
class Gravity final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;
    static constexpr double g0 = 9.80665;

private:
    T m_gravity{T(g0)};
    std::string m_name{"gravity"};

public:
    Gravity(T gravity = T(g0), std::string_view name = "gravity")
        : m_gravity(gravity), m_name(name) {}

    void setOffset(size_t) const {}
    void setConstantGravity(T g) { m_gravity = g; }

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "Gravity"; }
    std::string_view getComponentName() const { return m_name; }

    T compute(typename Tags::GravityAcceleration, std::span<const T>) const { return m_gravity; }
};

} // namespace sopot::rocket
