#pragma once

#include "../core/typed_component.hpp"
#include "rocket_tags.hpp"
#include "vector3.hpp"
#include <span>

namespace sopot::rocket {

template<Scalar T = double>
class TranslationDynamics final : public TypedComponent<3, T> {
public:
    using Base = TypedComponent<3, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    Vector3<T> m_initial_velocity{T(0), T(0), T(0)};
    std::string m_name{"translation_dynamics"};

public:
    TranslationDynamics(Vector3<T> initial_velocity = Vector3<T>::zero(),
        std::string_view name = "translation_dynamics")
        : m_initial_velocity(initial_velocity), m_name(name) {}

    void setInitialVelocity(const Vector3<T>& vel) { m_initial_velocity = vel; }

    LocalState getInitialLocalState() const {
        return {m_initial_velocity.x, m_initial_velocity.y, m_initial_velocity.z};
    }

    std::string_view getComponentType() const { return "TranslationDynamics"; }
    std::string_view getComponentName() const { return m_name; }

    template<typename Registry>
    LocalDerivative derivatives(T, std::span<const T>, std::span<const T> global,
        const Registry& registry) const {
        Vector3<T> force = registry.template computeFunction<dynamics::TotalForceENU>(global);
        T mass = registry.template computeFunction<dynamics::Mass>(global);
        Vector3<T> acc = force / mass;
        return {acc.x, acc.y, acc.z};
    }

    Vector3<T> compute(kinematics::VelocityENU, std::span<const T> state) const {
        return {
            this->getGlobalState(state, 0),
            this->getGlobalState(state, 1),
            this->getGlobalState(state, 2)
        };
    }
};

} // namespace sopot::rocket
