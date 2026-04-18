#pragma once

// Planar (2D, east-up) point-mass components for prototyping guidance laws
// against the vehicle-tag architecture. Uses VehicleTags<V> scoping so two
// vehicles coexist in a single TypedODESystem without tag collisions.
//
// Motion is confined to the East-Up plane (y = 0 in ENU). Position and
// velocity are still returned as Vector3<T> so the tags contract matches the
// full 6DOF components; the North (y) channel is simply unused.
//
// Intentionally simple: constant mass, constant thrust, no aero. A full-fidelity
// replacement for any of these is straightforward because each is a normal
// TypedComponent<N, T>.

#include "../core/typed_component.hpp"
#include "vehicle_tags.hpp"
#include "vector3.hpp"
#include <cmath>
#include <span>

namespace sopot::rocket::planar {

//=============================================================================
// Kinematics: position integrator in the East-Up plane.
// State (2): [east, up].   dPos/dt = velocity (from dynamics component).
//=============================================================================
template<VehicleConcept Vehicle, Scalar T = double>
class PointMassKinematics2D final : public TypedComponent<2, T> {
public:
    using Base = TypedComponent<2, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    T m_east0{T(0)};
    T m_up0{T(0)};
    std::string m_name{"planar_kinematics"};

public:
    PointMassKinematics2D(T east0 = T(0), T up0 = T(0),
                          std::string_view name = "planar_kinematics")
        : m_east0(east0), m_up0(up0), m_name(name) {}

    void setInitialPosition(T east, T up) { m_east0 = east; m_up0 = up; }

    LocalState getInitialLocalState() const { return {m_east0, m_up0}; }
    std::string_view getComponentType() const { return "PointMassKinematics2D"; }
    std::string_view getComponentName() const { return m_name; }

    template<typename Registry>
    LocalDerivative derivatives(T, std::span<const T>, std::span<const T> global,
                                const Registry& registry) const {
        Vector3<T> v = registry.template computeFunction<typename Tags::VelocityENU>(global);
        return {v.x, v.z};
    }

    Vector3<T> compute(typename Tags::PositionENU, std::span<const T> state) const {
        return { this->getGlobalState(state, 0), T(0), this->getGlobalState(state, 1) };
    }
    T compute(typename Tags::Altitude, std::span<const T> state) const {
        return this->getGlobalState(state, 1);
    }
};

//=============================================================================
// Dynamics: velocity integrator in the East-Up plane.
// State (2): [v_east, v_up].   dVel/dt = F/m, F from TotalForceENU, m from Mass.
//=============================================================================
template<VehicleConcept Vehicle, Scalar T = double>
class PointMassDynamics2D final : public TypedComponent<2, T> {
public:
    using Base = TypedComponent<2, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    T m_ve0{T(0)};
    T m_vu0{T(0)};
    std::string m_name{"planar_dynamics"};

public:
    PointMassDynamics2D(T ve0 = T(0), T vu0 = T(0),
                        std::string_view name = "planar_dynamics")
        : m_ve0(ve0), m_vu0(vu0), m_name(name) {}

    void setInitialVelocity(T ve, T vu) { m_ve0 = ve; m_vu0 = vu; }

    LocalState getInitialLocalState() const { return {m_ve0, m_vu0}; }
    std::string_view getComponentType() const { return "PointMassDynamics2D"; }
    std::string_view getComponentName() const { return m_name; }

    template<typename Registry>
    LocalDerivative derivatives(T, std::span<const T>, std::span<const T> global,
                                const Registry& registry) const {
        Vector3<T> F = registry.template computeFunction<typename Tags::TotalForceENU>(global);
        T m = registry.template computeFunction<typename Tags::Mass>(global);
        return {F.x / m, F.z / m};
    }

    Vector3<T> compute(typename Tags::VelocityENU, std::span<const T> state) const {
        return { this->getGlobalState(state, 0), T(0), this->getGlobalState(state, 1) };
    }
};

//=============================================================================
// Mass: constant, stateless.
//=============================================================================
template<VehicleConcept Vehicle, Scalar T = double>
class ConstantMass final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    T m_mass{T(100)};
    std::string m_name{"mass"};

public:
    ConstantMass(T mass = T(100), std::string_view name = "mass") : m_mass(mass), m_name(name) {}
    void setOffset(size_t) const {}

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "ConstantMass"; }
    std::string_view getComponentName() const { return m_name; }

    T compute(typename Tags::Mass, std::span<const T>) const { return m_mass; }
};

//=============================================================================
// Thrust: constant-magnitude along the velocity direction, zero when speed is
// too low to define a direction. Stateless. Models a sustainer rocket motor
// good enough for guidance prototyping; replace with InterpolatedEngine for
// fidelity.
//=============================================================================
template<VehicleConcept Vehicle, Scalar T = double>
class VelocityAlignedThrust2D final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    T m_thrust{T(0)};
    std::string m_name{"thrust"};

public:
    VelocityAlignedThrust2D(T thrust = T(0), std::string_view name = "thrust")
        : m_thrust(thrust), m_name(name) {}
    void setOffset(size_t) const {}

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "VelocityAlignedThrust2D"; }
    std::string_view getComponentName() const { return m_name; }

    Vector3<T> compute(typename Tags::ThrustForceENU, std::span<const T>) const {
        return Vector3<T>::zero();
    }
    template<typename Registry>
    Vector3<T> compute(typename Tags::ThrustForceENU, std::span<const T> state,
                       const Registry& registry) const {
        Vector3<T> v = registry.template computeFunction<typename Tags::VelocityENU>(state);
        T speed = v.norm();
        if (value_of(speed) < 1.0) return Vector3<T>::zero();
        return v * (m_thrust / speed);
    }
};

//=============================================================================
// Ballistic vehicle: no guidance. Provides zero GuidanceCommandENU so the
// force aggregator can always query the tag without branching.
//=============================================================================
template<VehicleConcept Vehicle, Scalar T = double>
class NoGuidance final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    std::string m_name{"no_guidance"};

public:
    NoGuidance(std::string_view name = "no_guidance") : m_name(name) {}
    void setOffset(size_t) const {}

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "NoGuidance"; }
    std::string_view getComponentName() const { return m_name; }

    Vector3<T> compute(typename Tags::GuidanceCommandENU, std::span<const T>) const {
        return Vector3<T>::zero();
    }
};

//=============================================================================
// Proportional Navigation guidance.
//   a_cmd = N * V_c * (omega_LOS x r_hat)
// where
//   r          = p_target - p_interceptor  (LOS vector)
//   dr         = v_target - v_interceptor  (LOS rate vector)
//   omega_LOS  = (r x dr) / |r|^2
//   V_c        = -dot(dr, r_hat)           (closing velocity, positive = closing)
//
// The command is an acceleration perpendicular to the line of sight. We return
// it in ENU so PointMassForce2D can add m * a_cmd directly. For planar motion
// the command stays in the East-Up plane (North component is zero by symmetry).
//=============================================================================
template<VehicleConcept Attacker, VehicleConcept Target, Scalar T = double>
class PNGuidance final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using AttackerTags = VehicleTags<Attacker>;
    using TargetTags   = VehicleTags<Target>;

private:
    T m_gain{T(3)};
    T m_a_max{T(1.0e9)};  // no saturation by default
    std::string m_name{"pn_guidance"};

public:
    PNGuidance(T gain = T(3), std::string_view name = "pn_guidance")
        : m_gain(gain), m_name(name) {}

    void setGain(T gain) { m_gain = gain; }
    void setAccelLimit(T a_max) { m_a_max = a_max; }
    void setOffset(size_t) const {}

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "PNGuidance"; }
    std::string_view getComponentName() const { return m_name; }

    Vector3<T> compute(typename AttackerTags::GuidanceCommandENU, std::span<const T>) const {
        return Vector3<T>::zero();
    }

    template<typename Registry>
    Vector3<T> compute(typename AttackerTags::GuidanceCommandENU,
                       std::span<const T> state, const Registry& registry) const {
        Vector3<T> p_T = registry.template computeFunction<typename TargetTags::PositionENU>(state);
        Vector3<T> v_T = registry.template computeFunction<typename TargetTags::VelocityENU>(state);
        Vector3<T> p_I = registry.template computeFunction<typename AttackerTags::PositionENU>(state);
        Vector3<T> v_I = registry.template computeFunction<typename AttackerTags::VelocityENU>(state);

        Vector3<T> r  = p_T - p_I;
        Vector3<T> dr = v_T - v_I;

        T r_mag2 = r.dot(r);
        if (value_of(r_mag2) < T(1e-6)) return Vector3<T>::zero();
        T r_mag = std::sqrt(r_mag2);
        Vector3<T> r_hat = r * (T(1) / r_mag);

        Vector3<T> omega_los = r.cross(dr) * (T(1) / r_mag2);
        T V_c = -dr.dot(r_hat);
        if (value_of(V_c) <= T(0)) return Vector3<T>::zero();  // opening geometry; PN undefined

        Vector3<T> a_cmd = omega_los.cross(r_hat) * (m_gain * V_c);

        T a_mag = a_cmd.norm();
        if (value_of(a_mag) > value_of(m_a_max)) {
            a_cmd = a_cmd * (m_a_max / a_mag);
        }
        return a_cmd;
    }
};

//=============================================================================
// Force aggregator for the planar point-mass stack:
//   F_total = F_gravity + F_thrust + m * a_guidance
// All forces are in ENU. North channel ignored by the 2D dynamics component.
//=============================================================================
template<VehicleConcept Vehicle, Scalar T = double>
class PointMassForce2D final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    std::string m_name{"planar_force"};

public:
    PointMassForce2D(std::string_view name = "planar_force") : m_name(name) {}
    void setOffset(size_t) const {}

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "PointMassForce2D"; }
    std::string_view getComponentName() const { return m_name; }

    Vector3<T> compute(typename Tags::TotalForceENU, std::span<const T>) const {
        return Vector3<T>::zero();
    }

    template<typename Registry>
    Vector3<T> compute(typename Tags::TotalForceENU, std::span<const T> state,
                       const Registry& registry) const {
        T m = registry.template computeFunction<typename Tags::Mass>(state);
        T g = registry.template computeFunction<typename Tags::GravityAcceleration>(state);
        Vector3<T> F_gravity{T(0), T(0), -g * m};

        Vector3<T> F_thrust = registry.template computeFunction<typename Tags::ThrustForceENU>(state);

        Vector3<T> a_guide = registry.template computeFunction<typename Tags::GuidanceCommandENU>(state);
        Vector3<T> F_guide = a_guide * m;

        return F_gravity + F_thrust + F_guide;
    }
};

} // namespace sopot::rocket::planar
