#pragma once

#include "../core/typed_component.hpp"
#include "vehicle_tags.hpp"
#include "vector3.hpp"
#include <span>

namespace sopot::rocket {

template<VehicleConcept Vehicle, Scalar T = double>
class TranslationKinematics final : public TypedComponent<3, T> {
public:
    using Base = TypedComponent<3, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;
    using Tags = VehicleTags<Vehicle>;

private:
    Vector3<T> m_initial_position{T(0), T(0), T(0)};
    std::string m_name{"translation_kinematics"};

public:
    TranslationKinematics(
        Vector3<T> initial_position = Vector3<T>::zero(),
        std::string_view name = "translation_kinematics"
    ) : m_initial_position(initial_position), m_name(name) {}

    void setInitialPosition(const Vector3<T>& pos) { m_initial_position = pos; }

    LocalState getInitialLocalState() const {
        return {m_initial_position.x, m_initial_position.y, m_initial_position.z};
    }

    std::string_view getComponentType() const { return "TranslationKinematics"; }
    std::string_view getComponentName() const { return m_name; }

    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        [[maybe_unused]] std::span<const T> local,
        std::span<const T> global,
        const Registry& registry
    ) const {
        Vector3<T> velocity = registry.template computeFunction<typename Tags::VelocityENU>(global);
        return {velocity.x, velocity.y, velocity.z};
    }

    Vector3<T> compute(typename Tags::PositionENU, std::span<const T> state) const {
        return {
            this->getGlobalState(state, 0),
            this->getGlobalState(state, 1),
            this->getGlobalState(state, 2)
        };
    }

    T compute(typename Tags::Altitude, std::span<const T> state) const {
        return this->getGlobalState(state, 2);
    }
};

template<VehicleConcept Vehicle, Scalar T = double>
TranslationKinematics<Vehicle, T> createTranslationKinematics(
    Vector3<T> initial_position = Vector3<T>::zero(),
    std::string_view name = "translation_kinematics"
) {
    return TranslationKinematics<Vehicle, T>(initial_position, name);
}

} // namespace sopot::rocket
