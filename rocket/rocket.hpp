#pragma once

#include "../core/typed_component.hpp"
#include "../core/solver.hpp"
#include "rocket_tags.hpp"
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
 * Rocket: Assembles components using TypedODESystem
 *
 * This class demonstrates the proper use of the framework:
 * 1. Components are created and configured
 * 2. makeTypedODESystem composes them with compile-time state function dispatch
 * 3. Components query each other through the registry pattern
 *
 * State layout (14 states total):
 *   [0]     - Simulation time (derivative = 1)
 *   [1-3]   - Position ENU (x, y, z)
 *   [4-6]   - Velocity ENU (vx, vy, vz)
 *   [7-10]  - Quaternion (q1, q2, q3, q4)
 *   [11-13] - Angular velocity (wx, wy, wz)
 *
 * Data flow (all through registry state functions):
 *   SimulationTime → provides propulsion::Time
 *   TranslationKinematics → provides kinematics::PositionENU, Altitude
 *   TranslationDynamics → provides kinematics::VelocityENU
 *   RotationKinematics → provides kinematics::AttitudeQuaternion
 *   RotationDynamics → provides kinematics::AngularVelocity
 *   StandardAtmosphere → provides environment::AtmosphericDensity, Pressure, SpeedOfSound
 *   Gravity → provides dynamics::GravityAcceleration
 *   RocketBody → queries Time, provides dynamics::Mass, MomentOfInertia
 *   InterpolatedEngine → queries Time, Pressure, provides propulsion::ThrustForceBody
 *   AxisymmetricAerodynamics → queries VelocityENU, Quaternion, etc., provides aero::AeroForceENU, AeroMomentBody
 *   ForceAggregator → aggregates forces from above, provides dynamics::TotalForceENU, TotalTorqueBody
 */
template<Scalar T = double>
class Rocket {
public:
    using scalar_type = T;

    // Component types
    using TimeComponent = SimulationTime<T>;
    using PosComponent = TranslationKinematics<T>;
    using VelComponent = TranslationDynamics<T>;
    using QuatComponent = RotationKinematics<T>;
    using OmegaComponent = RotationDynamics<T>;
    using AtmoComponent = StandardAtmosphere<T>;
    using GravComponent = Gravity<T>;
    using BodyComponent = RocketBody<T>;
    using EngineComponent = InterpolatedEngine<T>;
    using AeroComponent = AxisymmetricAerodynamics<T>;
    using ForceComponent = ForceAggregator<T>;

    // The composed ODE system type
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
    // Owned components (configured before creating system)
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

    // The composed system (created in setupBeforeSimulation)
    std::unique_ptr<SystemType> m_system;

    // Configuration
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

    // Access to aerodynamics component for additional configuration
    AeroComponent& aero() { return m_aero; }
    const AeroComponent& aero() const { return m_aero; }

    //=========================================================================
    // SETUP - Creates the TypedODESystem
    //=========================================================================

    void setupBeforeSimulation() {
        // Configure initial attitude from launcher angles
        m_attitude.setInitialFromLauncherAngles(m_elevation_deg, m_azimuth_deg);

        // Create the composed ODE system using makeTypedODESystem
        // This automatically:
        // - Sets state offsets for each component
        // - Creates the registry for compile-time state function dispatch
        // - Enables components to query each other through registry
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

        // Verify state functions are available (compile-time checks)
        static_assert(SystemType::template hasFunction<propulsion::Time>(),
            "System must provide Time state function");
        static_assert(SystemType::template hasFunction<dynamics::Mass>(),
            "System must provide Mass state function");
        static_assert(SystemType::template hasFunction<aero::AeroForceENU>(),
            "System must provide AeroForceENU state function");
        static_assert(SystemType::template hasFunction<dynamics::TotalForceENU>(),
            "System must provide TotalForceENU state function");
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

    // Convenience accessors
    T getTime(const std::vector<T>& state) const {
        return queryStateFunction<propulsion::Time>(state);
    }

    Vector3<T> getPosition(const std::vector<T>& state) const {
        return queryStateFunction<kinematics::PositionENU>(state);
    }

    Vector3<T> getVelocity(const std::vector<T>& state) const {
        return queryStateFunction<kinematics::VelocityENU>(state);
    }

    Quaternion<T> getQuaternion(const std::vector<T>& state) const {
        return queryStateFunction<kinematics::AttitudeQuaternion>(state);
    }

    T getAltitude(const std::vector<T>& state) const {
        return queryStateFunction<kinematics::Altitude>(state);
    }

    T getMass(const std::vector<T>& state) const {
        return queryStateFunction<dynamics::Mass>(state);
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
 * Simulate the rocket using the framework (returns raw SolutionResult)
 */
template<Scalar T = double>
SolutionResult simulateRocket(
    Rocket<T>& rocket,
    double t_end,
    double dt = 0.01
) {
    // Setup creates the TypedODESystem
    rocket.setupBeforeSimulation();

    // Get initial state from the system
    auto initial_state = rocket.getInitialState();
    size_t state_dim = rocket.getStateDimension();

    // Create derivative function that uses the system
    auto derivs_func = [&rocket](double t, StateView state_view) -> StateDerivative {
        std::vector<T> state(state_view.begin(), state_view.end());
        auto derivs = rocket.computeDerivatives(T(t), state);

        // Convert to StateDerivative (std::vector<double>)
        StateDerivative result(derivs.size());
        for (size_t i = 0; i < derivs.size(); ++i) {
            result[i] = value_of(derivs[i]);
        }
        return result;
    };

    // Convert initial state to double
    StateVector init_state(initial_state.size());
    for (size_t i = 0; i < initial_state.size(); ++i) {
        init_state[i] = value_of(initial_state[i]);
    }

    // Solve using RK4
    auto solver = createRK4Solver();
    return solver.solve(derivs_func, state_dim, 0.0, t_end, dt, init_state);
}

/**
 * Simulate and return SimulationResult with state function interpolation
 */
template<Scalar T = double>
SimulationResult<Rocket<T>> simulate(
    Rocket<T>& rocket,
    double t_end,
    double dt = 0.01
) {
    auto solution = simulateRocket(rocket, t_end, dt);
    auto result = SimulationResult<Rocket<T>>(rocket, std::move(solution));
    result.computeStatistics(rocket.getBurnTime());
    return result;
}

} // namespace sopot::rocket
