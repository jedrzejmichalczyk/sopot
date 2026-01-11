#pragma once

#include "../core/typed_component.hpp"
#include "rocket_tags.hpp"
#include "vector3.hpp"
#include <span>

namespace sopot::rocket {

/**
 * TranslationKinematics: Position integration in ENU frame
 *
 * State (3 elements): [posE, posN, posU]
 * Derivative: dPos/dt = velocity (from TranslationDynamics via registry)
 *
 * Provides: kinematics::PositionENU, kinematics::Altitude
 * Requires: kinematics::VelocityENU
 */
template<Scalar T = double>
class TranslationKinematics final : public TypedComponent<3, T> {
public:
    using Base = TypedComponent<3, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    Vector3<T> m_initial_position{T(0), T(0), T(0)};
    std::string m_name{"translation_kinematics"};
    mutable size_t m_offset{0};  // Set by system, used for state function access

public:
    TranslationKinematics(
        Vector3<T> initial_position = Vector3<T>::zero(),
        std::string_view name = "translation_kinematics"
    ) : m_initial_position(initial_position), m_name(name) {}

    void setInitialPosition(const Vector3<T>& pos) { m_initial_position = pos; }
    void setOffset(size_t off) const { m_offset = off; }

    LocalState getInitialLocalState() const override {
        return {m_initial_position.x, m_initial_position.y, m_initial_position.z};
    }

    std::string_view getComponentType() const override { return "TranslationKinematics"; }
    std::string_view getComponentName() const override { return m_name; }

    // Non-virtual derivatives - called directly by system
    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        [[maybe_unused]] std::span<const T> local,
        std::span<const T> global,
        const Registry& registry
    ) const {
        Vector3<T> velocity = registry.template computeFunction<kinematics::VelocityENU>(global);
        return {velocity.x, velocity.y, velocity.z};
    }

    // State function: Position ENU
    Vector3<T> compute(kinematics::PositionENU, std::span<const T> state) const {
        return {state[m_offset], state[m_offset + 1], state[m_offset + 2]};
    }

    // State function: Altitude (Up component)
    T compute(kinematics::Altitude, std::span<const T> state) const {
        return state[m_offset + 2];
    }
};

template<Scalar T = double>
TranslationKinematics<T> createTranslationKinematics(
    Vector3<T> initial_position = Vector3<T>::zero(),
    std::string_view name = "translation_kinematics"
) {
    return TranslationKinematics<T>(initial_position, name);
}

} // namespace sopot::rocket
