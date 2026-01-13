/**
 * WebAssembly wrapper for SOPOT Rocket simulation
 *
 * This file provides Emscripten embind bindings to expose the C++ rocket
 * simulation framework to JavaScript/TypeScript.
 *
 * Build with:
 *   emcc -std=c++20 -lembind -O3 -s WASM=1 \
 *        -I.. wasm_rocket.cpp -o sopot.js
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/emscripten.h>
#include "../rocket/rocket.hpp"
#include <memory>
#include <string>
#include <stdexcept>

using namespace emscripten;
using namespace sopot;
using namespace sopot::rocket;

/**
 * RocketSimulator: JavaScript-friendly wrapper around Rocket<double>
 *
 * This class provides:
 * - Clean initialization and configuration API
 * - Data-driven CSV loading (arrays instead of files)
 * - Single-step integration for JavaScript-controlled animation loops
 * - State queries that return JavaScript objects
 */
class RocketSimulator {
private:
    Rocket<double> m_rocket;
    std::vector<double> m_state;
    double m_time{0.0};
    double m_dt{0.01};  // Default timestep: 10ms
    bool m_initialized{false};

    // Helper: RK4 single step (manual implementation to avoid external dependencies)
    void rk4Step(double dt) {
        auto k1 = m_rocket.computeDerivatives(m_time, m_state);

        // Compute k2 = f(t + dt/2, y + k1*dt/2)
        std::vector<double> temp_state(m_state.size());
        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + 0.5 * dt * k1[i];
        }
        auto k2 = m_rocket.computeDerivatives(m_time + 0.5 * dt, temp_state);

        // Compute k3 = f(t + dt/2, y + k2*dt/2)
        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + 0.5 * dt * k2[i];
        }
        auto k3 = m_rocket.computeDerivatives(m_time + 0.5 * dt, temp_state);

        // Compute k4 = f(t + dt, y + k3*dt)
        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + dt * k3[i];
        }
        auto k4 = m_rocket.computeDerivatives(m_time + dt, temp_state);

        // Update: y_{n+1} = y_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        for (size_t i = 0; i < m_state.size(); ++i) {
            m_state[i] += (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
        }

        m_time += dt;
    }

public:
    RocketSimulator() = default;

    //=========================================================================
    // CONFIGURATION API
    //=========================================================================

    /**
     * Set launcher angles
     * @param elevation_deg Launch elevation angle in degrees (0=horizontal, 90=vertical)
     * @param azimuth_deg Launch azimuth angle in degrees (0=North, 90=East)
     */
    void setLauncher(double elevation_deg, double azimuth_deg) {
        m_rocket.setLauncher(elevation_deg, azimuth_deg);
    }

    /**
     * Set rocket reference diameter in meters
     */
    void setDiameter(double diameter_m) {
        m_rocket.setDiameter(diameter_m);
    }

    /**
     * Set integration timestep in seconds (default: 0.01)
     */
    void setTimestep(double dt) {
        m_dt = dt;
    }

    //=========================================================================
    // DATA LOADING (from JavaScript arrays, not files)
    //=========================================================================

    /**
     * Load mass data from JavaScript arrays
     * @param time Array of time points [s]
     * @param mass Array of mass values [kg]
     */
    void loadMassData(const val& time_js, const val& mass_js) {
        // TODO: Phase 2 - Implement array-based data loading
        // Need to adapt RocketBody component to accept std::vector instead of file paths
        throw std::runtime_error(
            "loadMassData() is not yet implemented. "
            "Please use loadMassDataFromPath() for Phase 1 prototype, "
            "or wait for Phase 2 array-based API."
        );
    }

    /**
     * Load engine data from JavaScript arrays
     * @param time Array of time points [s]
     * @param thrust Array of thrust values [N]
     */
    void loadEngineData(const val& time_js, const val& thrust_js) {
        // TODO: Phase 2 - Implement array-based data loading
        // Need to adapt InterpolatedEngine component to accept std::vector instead of file paths
        throw std::runtime_error(
            "loadEngineData() is not yet implemented. "
            "Please use loadEngineDataFromPath() for Phase 1 prototype, "
            "or wait for Phase 2 array-based API."
        );
    }

    /**
     * Load mass data from file path (for prototype)
     * @param path Path to directory containing CSV files
     */
    void loadMassDataFromPath(const std::string& path) {
        m_rocket.loadMassData(path);
    }

    /**
     * Load engine data from file path (for prototype)
     * @param path Path to directory containing CSV files
     */
    void loadEngineDataFromPath(const std::string& path) {
        m_rocket.loadEngineData(path);
    }

    /**
     * Load aerodynamic data from file path (for prototype)
     * @param path Path to directory containing CSV files
     */
    void loadAeroDataFromPath(const std::string& path) {
        m_rocket.loadAeroData(path);
    }

    /**
     * Load damping data from file path (for prototype)
     * @param path Path to directory containing CSV files
     */
    void loadDampingDataFromPath(const std::string& path) {
        m_rocket.loadDampingData(path);
    }

    //=========================================================================
    // INITIALIZATION
    //=========================================================================

    /**
     * Initialize the simulation system
     * Must be called after configuration and before step()
     */
    void setup() {
        m_rocket.setupBeforeSimulation();
        m_state = m_rocket.getInitialState();
        m_time = 0.0;
        m_initialized = true;
    }

    /**
     * Reset simulation to initial conditions
     */
    void reset() {
        if (!m_initialized) {
            throw std::runtime_error("Call setup() before reset()");
        }
        m_state = m_rocket.getInitialState();
        m_time = 0.0;
    }

    //=========================================================================
    // SIMULATION CONTROL
    //=========================================================================

    /**
     * Advance simulation by one timestep
     * Returns true if simulation should continue, false if rocket has landed
     */
    bool step() {
        if (!m_initialized) {
            throw std::runtime_error("Call setup() before step()");
        }

        rk4Step(m_dt);

        // Check termination condition: altitude < 0
        double altitude = getAltitude();
        return altitude >= 0.0;
    }

    /**
     * Step with custom timestep (overrides default)
     */
    bool stepWithDt(double dt) {
        if (!m_initialized) {
            throw std::runtime_error("Call setup() before step()");
        }

        rk4Step(dt);

        double altitude = getAltitude();
        return altitude >= 0.0;
    }

    //=========================================================================
    // STATE QUERIES (scalar values)
    //=========================================================================

    double getTime() const {
        return m_time;
    }

    double getAltitude() const {
        if (!m_initialized) return 0.0;
        return m_rocket.getAltitude(m_state);
    }

    double getSpeed() const {
        if (!m_initialized) return 0.0;
        return m_rocket.getSpeed(m_state);
    }

    double getMass() const {
        if (!m_initialized) return 0.0;
        return m_rocket.getMass(m_state);
    }

    double getBurnTime() const {
        return m_rocket.getBurnTime();
    }

    //=========================================================================
    // STATE QUERIES (vector/quaternion as JavaScript objects)
    //=========================================================================

    /**
     * Get position in ENU frame [m]
     * Returns {x: number, y: number, z: number}
     */
    val getPosition() const {
        if (!m_initialized) {
            return val::object();
        }

        auto pos = m_rocket.getPosition(m_state);

        val result = val::object();
        result.set("x", pos.x);
        result.set("y", pos.y);
        result.set("z", pos.z);
        return result;
    }

    /**
     * Get velocity in ENU frame [m/s]
     * Returns {x: number, y: number, z: number}
     */
    val getVelocity() const {
        if (!m_initialized) {
            return val::object();
        }

        auto vel = m_rocket.getVelocity(m_state);

        val result = val::object();
        result.set("x", vel.x);
        result.set("y", vel.y);
        result.set("z", vel.z);
        return result;
    }

    /**
     * Get attitude quaternion (body to ENU)
     * Returns {q1: number, q2: number, q3: number, q4: number}
     */
    val getQuaternion() const {
        if (!m_initialized) {
            return val::object();
        }

        auto q = m_rocket.getQuaternion(m_state);

        val result = val::object();
        result.set("q1", q.q1);
        result.set("q2", q.q2);
        result.set("q3", q.q3);
        result.set("q4", q.q4);
        return result;
    }

    /**
     * Get full state as JavaScript object (for debugging)
     */
    val getFullState() const {
        if (!m_initialized) {
            return val::object();
        }

        val result = val::object();
        result.set("time", m_time);
        result.set("position", getPosition());
        result.set("velocity", getVelocity());
        result.set("quaternion", getQuaternion());
        result.set("altitude", getAltitude());
        result.set("speed", getSpeed());
        result.set("mass", getMass());

        return result;
    }

    //=========================================================================
    // UTILITY
    //=========================================================================

    size_t getStateDimension() const {
        if (!m_initialized) return 0;
        return m_state.size();
    }

    bool isInitialized() const {
        return m_initialized;
    }
};

//=============================================================================
// EMSCRIPTEN BINDINGS
//=============================================================================

EMSCRIPTEN_BINDINGS(sopot_module) {
    // Register the main simulator class
    class_<RocketSimulator>("RocketSimulator")
        // Constructor
        .constructor<>()

        // Configuration
        .function("setLauncher", &RocketSimulator::setLauncher)
        .function("setDiameter", &RocketSimulator::setDiameter)
        .function("setTimestep", &RocketSimulator::setTimestep)

        // Data loading (arrays - Phase 2)
        .function("loadMassData", &RocketSimulator::loadMassData)
        .function("loadEngineData", &RocketSimulator::loadEngineData)

        // Data loading (file paths - prototype only)
        .function("loadMassDataFromPath", &RocketSimulator::loadMassDataFromPath)
        .function("loadEngineDataFromPath", &RocketSimulator::loadEngineDataFromPath)
        .function("loadAeroDataFromPath", &RocketSimulator::loadAeroDataFromPath)
        .function("loadDampingDataFromPath", &RocketSimulator::loadDampingDataFromPath)

        // Initialization
        .function("setup", &RocketSimulator::setup)
        .function("reset", &RocketSimulator::reset)

        // Simulation control
        .function("step", &RocketSimulator::step)
        .function("stepWithDt", &RocketSimulator::stepWithDt)

        // Scalar queries
        .function("getTime", &RocketSimulator::getTime)
        .function("getAltitude", &RocketSimulator::getAltitude)
        .function("getSpeed", &RocketSimulator::getSpeed)
        .function("getMass", &RocketSimulator::getMass)
        .function("getBurnTime", &RocketSimulator::getBurnTime)

        // Vector/quaternion queries (return JS objects)
        .function("getPosition", &RocketSimulator::getPosition)
        .function("getVelocity", &RocketSimulator::getVelocity)
        .function("getQuaternion", &RocketSimulator::getQuaternion)
        .function("getFullState", &RocketSimulator::getFullState)

        // Utility
        .function("getStateDimension", &RocketSimulator::getStateDimension)
        .function("isInitialized", &RocketSimulator::isInitialized);
}
