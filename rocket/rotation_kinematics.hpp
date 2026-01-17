#pragma once

#include "../core/typed_component.hpp"
#include "rocket_tags.hpp"
#include "quaternion.hpp"
#include <span>

namespace sopot::rocket {

template<Scalar T = double>
class RotationKinematics final : public TypedComponent<4, T> {
public:
    using Base = TypedComponent<4, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    Quaternion<T> m_initial_quaternion{T(0), T(0), T(0), T(1)};
    std::string m_name{"rotation_kinematics"};

public:
    RotationKinematics(Quaternion<T> initial_quaternion = Quaternion<T>::identity(),
        std::string_view name = "rotation_kinematics")
        : m_initial_quaternion(initial_quaternion), m_name(name) {}

    void setInitialQuaternion(const Quaternion<T>& q) { m_initial_quaternion = q; }
    void setInitialFromLauncherAngles(T elevation_deg, T azimuth_deg) {
        m_initial_quaternion = Quaternion<T>::from_launcher_angles(elevation_deg, azimuth_deg);
    }

    LocalState getInitialLocalState() const {
        return {m_initial_quaternion.q1, m_initial_quaternion.q2,
                m_initial_quaternion.q3, m_initial_quaternion.q4};
    }

    std::string_view getComponentType() const { return "RotationKinematics"; }
    std::string_view getComponentName() const { return m_name; }

    template<typename Registry>
    LocalDerivative derivatives(T, std::span<const T> local, std::span<const T> global,
        const Registry& registry) const {
        Quaternion<T> q{local[0], local[1], local[2], local[3]};
        Vector3<T> omega = registry.template computeFunction<kinematics::AngularVelocity>(global);
        Quaternion<T> dq = q.derivative(omega);
        return {dq.q1, dq.q2, dq.q3, dq.q4};
    }

    Quaternion<T> compute(kinematics::AttitudeQuaternion, std::span<const T> state) const {
        return {
            this->getGlobalState(state, 0),
            this->getGlobalState(state, 1),
            this->getGlobalState(state, 2),
            this->getGlobalState(state, 3)
        };
    }
};

} // namespace sopot::rocket
