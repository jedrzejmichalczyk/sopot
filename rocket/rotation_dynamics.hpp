#pragma once

#include "../core/typed_component.hpp"
#include "rocket_tags.hpp"
#include "vector3.hpp"
#include <span>

namespace sopot::rocket {

template<Scalar T = double>
class RotationDynamics final : public TypedComponent<3, T> {
public:
    using Base = TypedComponent<3, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    Vector3<T> m_initial_omega{T(0), T(0), T(0)};
    std::string m_name{"rotation_dynamics"};

public:
    RotationDynamics(Vector3<T> initial_omega = Vector3<T>::zero(),
        std::string_view name = "rotation_dynamics")
        : m_initial_omega(initial_omega), m_name(name) {}

    void setInitialAngularVelocity(const Vector3<T>& omega) { m_initial_omega = omega; }

    LocalState getInitialLocalState() const {
        return {m_initial_omega.x, m_initial_omega.y, m_initial_omega.z};
    }

    std::string_view getComponentType() const { return "RotationDynamics"; }
    std::string_view getComponentName() const { return m_name; }

    template<typename Registry>
    LocalDerivative derivatives(T, std::span<const T> local, std::span<const T> global,
        const Registry& registry) const {
        Vector3<T> omega{local[0], local[1], local[2]};
        Vector3<T> torque = registry.template computeFunction<dynamics::TotalTorqueBody>(global);
        Vector3<T> moi = registry.template computeFunction<dynamics::MomentOfInertia>(global);

        T dOmegaX = (torque.x - (moi.z - moi.y) * omega.y * omega.z) / moi.x;
        T dOmegaY = (torque.y - (moi.x - moi.z) * omega.x * omega.z) / moi.y;
        T dOmegaZ = (torque.z - (moi.y - moi.x) * omega.x * omega.y) / moi.z;

        return {dOmegaX, dOmegaY, dOmegaZ};
    }

    Vector3<T> compute(kinematics::AngularVelocity, std::span<const T> state) const {
        return {
            this->getGlobalState(state, 0),
            this->getGlobalState(state, 1),
            this->getGlobalState(state, 2)
        };
    }
};

} // namespace sopot::rocket
