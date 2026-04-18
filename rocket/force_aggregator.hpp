#pragma once

#include "../core/typed_component.hpp"
#include "vehicle_tags.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"
#include <span>

namespace sopot::rocket {

/**
 * ForceAggregator: Aggregates forces/torques for one vehicle.
 *
 * Provides: VehicleTags<V>::TotalForceENU, TotalTorqueBody
 * Queries:  VehicleTags<V>::Mass, GravityAcceleration, ThrustForceBody,
 *           AttitudeQuaternion, AeroForceENU, AeroMomentBody
 */
template<VehicleConcept Vehicle, Scalar T = double>
class ForceAggregator final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    std::string m_name{"force_aggregator"};

public:
    ForceAggregator(std::string_view name = "force_aggregator") : m_name(name) {}

    void setOffset(size_t) const {}

    std::string_view getComponentType() const { return "ForceAggregator"; }
    std::string_view getComponentName() const { return m_name; }
    LocalState getInitialLocalState() const { return {}; }

    Vector3<T> compute(typename Tags::TotalForceENU, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    template<typename Registry>
    Vector3<T> compute(typename Tags::TotalForceENU, std::span<const T> state, const Registry& registry) const {
        T mass = registry.template computeFunction<typename Tags::Mass>(state);
        T g = registry.template computeFunction<typename Tags::GravityAcceleration>(state);
        Vector3<T> F_gravity{T(0), T(0), -g * mass};

        Vector3<T> thrust_body = registry.template computeFunction<typename Tags::ThrustForceBody>(state);
        Quaternion<T> quat = registry.template computeFunction<typename Tags::AttitudeQuaternion>(state);
        Vector3<T> F_thrust = quat.rotate_body_to_reference(thrust_body);

        Vector3<T> F_aero = registry.template computeFunction<typename Tags::AeroForceENU>(state);

        return F_gravity + F_thrust + F_aero;
    }

    Vector3<T> compute(typename Tags::TotalTorqueBody, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    template<typename Registry>
    Vector3<T> compute(typename Tags::TotalTorqueBody, std::span<const T> state, const Registry& registry) const {
        Vector3<T> M_aero = registry.template computeFunction<typename Tags::AeroMomentBody>(state);
        // TODO: add thrust moment when TVC is added
        // TODO: add control-surface moments when fins/canards are added
        return M_aero;
    }
};

template<VehicleConcept Vehicle, Scalar T = double>
ForceAggregator<Vehicle, T> createForceAggregator(std::string_view name = "force_aggregator") {
    return ForceAggregator<Vehicle, T>(name);
}

} // namespace sopot::rocket
