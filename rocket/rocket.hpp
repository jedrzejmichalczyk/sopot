#pragma once

#include "../core/typed_component.hpp"
#include "../core/solver.hpp"
#include "vehicle_tags.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"
#include "simulation_time.hpp"
#include "translation_kinematics.hpp"
#include "translation_dynamics.hpp"
#include "rotation_kinematics.hpp"
#include "rotation_dynamics.hpp"
#include "standard_atmosphere.hpp"
#include "gravity.hpp"
#include "rocket_body.hpp"
#include "interpolated_engine.hpp"
#include "axisymmetric_aero.hpp"
#include "force_aggregator.hpp"
#include "simulation_result.hpp"

#include <memory>
#include <iostream>

namespace sopot::rocket {

/**
 * Rocket: Assembles one vehicle's worth of components using TypedODESystem.
 *
 * Templated on Vehicle (role tag — Missile, Interceptor, ...) and on Scalar T.
 * Multiple Rocket<Vehicle, T> instances can later be merged into a single
 * multi-vehicle TypedODESystem without tag collisions because every state
 * function is scoped by VehicleTags<Vehicle>.
 *
 * State layout (14 states total for a single vehicle):
 *   [0]     - Simulation time (global, sim::Time)
 *   [1-3]   - Position ENU
 *   [4-6]   - Velocity ENU
 *   [7-10]  - Quaternion
 *   [11-13] - Angular velocity
 */
template<VehicleConcept Vehicle, Scalar T = double>
class Rocket {
public:
    using scalar_type = T;
    using vehicle_type = Vehicle;
    using Tags = VehicleTags<Vehicle>;

    using TimeComponent   = SimulationTime<T>;
    using PosComponent    = TranslationKinematics<Vehicle, T>;
    using VelComponent    = TranslationDynamics<Vehicle, T>;
    using QuatComponent   = RotationKinematics<Vehicle, T>;
    using OmegaComponent  = RotationDynamics<Vehicle, T>;
    using AtmoComponent   = StandardAtmosphere<Vehicle, T>;
    using GravComponent   = Gravity<Vehicle, T>;
    using BodyComponent   = RocketBody<Vehicle, T>;
    using EngineComponent = InterpolatedEngine<Vehicle, T>;
    using AeroComponent   = AxisymmetricAerodynamics<Vehicle, T>;
    using ForceComponent  = ForceAggregator<Vehicle, T>;

    using SystemType = TypedODESystem<T,
        TimeComponent,
        PosComponent,
        VelComponent,
        QuatComponent,
        OmegaComponent,
        AtmoComponent,
        GravComponent,
        BodyComponent,
        EngineComponent,
        AeroComponent,
        ForceComponent
    >;

private:
    TimeComponent m_time;
    PosComponent m_position;
    VelComponent m_velocity;
    QuatComponent m_attitude;
    OmegaComponent m_angular_velocity;
    AtmoComponent m_atmosphere;
    GravComponent m_gravity;
    BodyComponent m_body;
    EngineComponent m_engine;
    AeroComponent m_aero;
    ForceComponent m_forces;

    std::unique_ptr<SystemType> m_system;

    T m_elevation_deg{90};
    T m_azimuth_deg{0};

public:
    Rocket() = default;

    //=========================================================================
    // CONFIGURATION (call before setupBeforeSimulation)
    //=========================================================================

    void setLauncher(T elevation_deg, T azimuth_deg) {
        m_elevation_deg = elevation_deg;
        m_azimuth_deg = azimuth_deg;
    }

    void setDiameter(double diameter) {
        m_aero.setReferenceDiameter(diameter);
    }

    void loadMassData(const std::string& path) {
        m_body.loadMass(path + "sim_mass.csv");
        m_body.loadCoG(path + "sim_cog.csv");
        m_body.loadInertiaX(path + "sim_ix.csv");
        m_body.loadInertiaYZ(path + "sim_iyz.csv");
    }

    void loadEngineData(const std::string& path) {
        m_engine.loadFromFile(path + "sim_engine.csv");
    }

    void loadAeroData(const std::string& path) {
        m_aero.loadCdPowerOff(path + "cd_power_off_rocket.csv");
        m_aero.loadCdPowerOn(path + "cd_power_on_rocket.csv");
        m_aero.loadClPowerOff(path + "cl_power_off_rocket.csv");
        m_aero.loadClPowerOn(path + "cl_power_on_rocket.csv");
        m_aero.loadCsPowerOff(path + "cs_power_off_rocket.csv");
        m_aero.loadCsPowerOn(path + "cs_power_on_rocket.csv");
    }

    void loadDampingData(const std::string& path) {
        m_aero.loadDampingX(path + "cmq_x_power_on_rocket.csv");
        m_aero.loadDampingYZ(path + "cmq_yz_power_on_rocket.csv");
    }

    double getBurnTime() const { return m_engine.getBurnTime(); }

    AeroComponent& aero() { return m_aero; }
    const AeroComponent& aero() const { return m_aero; }

    //=========================================================================
    // SETUP - Creates the TypedODESystem
    //=========================================================================

    void setupBeforeSimulation() {
        m_attitude.setInitialFromLauncherAngles(m_elevation_deg, m_azimuth_deg);

        m_system = std::make_unique<SystemType>(
            m_time,
            m_position,
            m_velocity,
            m_attitude,
            m_angular_velocity,
            m_atmosphere,
            m_gravity,
            m_body,
            m_engine,
            m_aero,
            m_forces
        );

        static_assert(SystemType::template hasFunction<sim::Time>(),
            "System must provide sim::Time state function");
        static_assert(SystemType::template hasFunction<typename Tags::Mass>(),
            "System must provide VehicleTags::Mass state function");
        static_assert(SystemType::template hasFunction<typename Tags::AeroForceENU>(),
            "System must provide VehicleTags::AeroForceENU state function");
        static_assert(SystemType::template hasFunction<typename Tags::TotalForceENU>(),
            "System must provide VehicleTags::TotalForceENU state function");
    }

    //=========================================================================
    // INTEGRATION INTERFACE
    //=========================================================================

    std::vector<T> getInitialState() const {
        if (!m_system) {
            throw std::runtime_error("Call setupBeforeSimulation() first");
        }
        return m_system->getInitialState();
    }

    std::vector<T> computeDerivatives(T t, const std::vector<T>& state) const {
        if (!m_system) {
            throw std::runtime_error("Call setupBeforeSimulation() first");
        }
        return m_system->computeDerivatives(t, state);
    }

    size_t getStateDimension() const {
        if (!m_system) {
            throw std::runtime_error("Call setupBeforeSimulation() first");
        }
        return m_system->getStateDimension();
    }

    //=========================================================================
    // STATE FUNCTION QUERIES (for analysis/output)
    //=========================================================================

    template<StateTagConcept Tag>
    auto queryStateFunction(const std::vector<T>& state) const {
        if (!m_system) {
            throw std::runtime_error("Call setupBeforeSimulation() first");
        }
        return m_system->template computeStateFunction<Tag>(state);
    }

    T getTime(const std::vector<T>& state) const {
        return queryStateFunction<sim::Time>(state);
    }

    Vector3<T> getPosition(const std::vector<T>& state) const {
        return queryStateFunction<typename Tags::PositionENU>(state);
    }

    Vector3<T> getVelocity(const std::vector<T>& state) const {
        return queryStateFunction<typename Tags::VelocityENU>(state);
    }

    Quaternion<T> getQuaternion(const std::vector<T>& state) const {
        return queryStateFunction<typename Tags::AttitudeQuaternion>(state);
    }

    T getAltitude(const std::vector<T>& state) const {
        return queryStateFunction<typename Tags::Altitude>(state);
    }

    T getMass(const std::vector<T>& state) const {
        return queryStateFunction<typename Tags::Mass>(state);
    }

    T getSpeed(const std::vector<T>& state) const {
        return getVelocity(state).norm();
    }

    // For benchmarking: direct access to body component's interpolator
    const BodyComponent& getBodyComponent() const {
        return m_system->template getComponent<7>();  // BodyComponent is at index 7
    }
};

/**
 * Simulate a single rocket (returns raw SolutionResult)
 */
template<VehicleConcept Vehicle, Scalar T = double>
SolutionResult simulateRocket(
    Rocket<Vehicle, T>& rocket,
    double t_end,
    double dt = 0.01
) {
    rocket.setupBeforeSimulation();

    auto initial_state = rocket.getInitialState();
    size_t state_dim = rocket.getStateDimension();

    auto derivs_func = [&rocket](double t, StateView state_view) -> StateDerivative {
        std::vector<T> state(state_view.begin(), state_view.end());
        auto derivs = rocket.computeDerivatives(T(t), state);

        StateDerivative result(derivs.size());
        for (size_t i = 0; i < derivs.size(); ++i) {
            result[i] = value_of(derivs[i]);
        }
        return result;
    };

    StateVector init_state(initial_state.size());
    for (size_t i = 0; i < initial_state.size(); ++i) {
        init_state[i] = value_of(initial_state[i]);
    }

    auto solver = createRK4Solver();
    return solver.solve(derivs_func, state_dim, 0.0, t_end, dt, init_state);
}

/**
 * Simulate and return SimulationResult with state function interpolation
 */
template<VehicleConcept Vehicle, Scalar T = double>
SimulationResult<Rocket<Vehicle, T>> simulate(
    Rocket<Vehicle, T>& rocket,
    double t_end,
    double dt = 0.01
) {
    auto solution = simulateRocket(rocket, t_end, dt);
    auto result = SimulationResult<Rocket<Vehicle, T>>(rocket, std::move(solution));
    result.computeStatistics(rocket.getBurnTime());
    return result;
}

} // namespace sopot::rocket
