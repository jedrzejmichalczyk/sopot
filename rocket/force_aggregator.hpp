#pragma once

#include "../core/typed_component.hpp"
#include "rocket_tags.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"
#include <span>

namespace sopot::rocket {

/**
 * ForceAggregator: Aggregates all forces and torques for 6DOF simulation
 *
 * This component ONLY aggregates forces from other components - it contains
 * no physics computation logic itself. All forces come from dedicated components:
 *   - Gravity force from mass and gravity acceleration
 *   - Thrust from engine component
 *   - Aerodynamic forces from aerodynamics component
 *
 * State (0 elements): No own state
 *
 * Provides: dynamics::TotalForceENU, dynamics::TotalTorqueBody
 *
 * Queries from registry:
 *   - dynamics::Mass
 *   - dynamics::GravityAcceleration
 *   - propulsion::ThrustForceBody
 *   - kinematics::AttitudeQuaternion
 *   - aero::AeroForceENU (from aerodynamics component)
 *   - aero::AeroMomentBody (from aerodynamics component)
 */
template<Scalar T = double>
class ForceAggregator final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    std::string m_name{"force_aggregator"};

public:
    ForceAggregator(std::string_view name = "force_aggregator") : m_name(name) {}

    void setOffset(size_t) const {} // No state

    std::string_view getComponentType() const { return "ForceAggregator"; }
    std::string_view getComponentName() const { return m_name; }
    LocalState getInitialLocalState() const { return {}; }

    //=========================================================================
    // STATE FUNCTIONS - Aggregate forces from other components
    //=========================================================================

    // Fallback: zero force (shouldn't be used in real simulation)
    Vector3<T> compute(dynamics::TotalForceENU, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    // Registry-aware: aggregate total force from all contributors
    template<typename Registry>
    Vector3<T> compute(dynamics::TotalForceENU, std::span<const T> state, const Registry& registry) const {
        // 1. GRAVITY - query mass and gravity acceleration
        T mass = registry.template computeFunction<dynamics::Mass>(state);
        T g = registry.template computeFunction<dynamics::GravityAcceleration>(state);
        Vector3<T> F_gravity{T(0), T(0), -g * mass};

        // 2. THRUST - query thrust in body frame and transform to ENU
        Vector3<T> thrust_body = registry.template computeFunction<propulsion::ThrustForceBody>(state);
        Quaternion<T> quat = registry.template computeFunction<kinematics::AttitudeQuaternion>(state);
        Vector3<T> F_thrust = quat.rotate_body_to_reference(thrust_body);

        // 3. AERODYNAMIC FORCE - query from aerodynamics component (already in ENU)
        Vector3<T> F_aero = registry.template computeFunction<aero::AeroForceENU>(state);

        // Total force
        return F_gravity + F_thrust + F_aero;
    }

    // Fallback: zero torque
    Vector3<T> compute(dynamics::TotalTorqueBody, [[maybe_unused]] std::span<const T>) const {
        return Vector3<T>::zero();
    }

    // Registry-aware: aggregate total torque from all contributors
    template<typename Registry>
    Vector3<T> compute(dynamics::TotalTorqueBody, std::span<const T> state, const Registry& registry) const {
        // Aerodynamic moments from aerodynamics component
        Vector3<T> M_aero = registry.template computeFunction<aero::AeroMomentBody>(state);

        // TODO: Add thrust moment if TVC is active
        // TODO: Add control surface moments if applicable

        return M_aero;
    }
};

// Factory function
template<Scalar T = double>
ForceAggregator<T> createForceAggregator(std::string_view name = "force_aggregator") {
    return ForceAggregator<T>(name);
}

} // namespace sopot::rocket
