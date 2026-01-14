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
    bool m_demo_mode{false};

    // Demo mode parameters
    static constexpr double DEMO_THRUST = 1500.0;      // N (thrust force)
    static constexpr double DEMO_BURN_TIME = 3.5;      // seconds
    static constexpr double DEMO_MASS = 20.0;          // kg (initial)
    static constexpr double DEMO_MASS_FLOW = 1.5;      // kg/s

    // Helper: Apply demo thrust acceleration to derivatives
    // State layout: [0]=time, [1-3]=pos, [4-6]=vel, [7-10]=quat, [11-13]=omega
    void applyDemoThrust(std::vector<double>& derivs, double t) const {
        if (!m_demo_mode || t > DEMO_BURN_TIME) return;

        // Get current attitude quaternion from state
        // SOPOT convention: q1,q2,q3 = vector part, q4 = scalar part
        double q1 = m_state[7], q2 = m_state[8], q3 = m_state[9], q4 = m_state[10];

        // Thrust is along body X axis, rotate to ENU frame
        // First column of rotation matrix R (body X in reference frame)
        // From quaternion.hpp to_rotation_matrix():
        double bx_e = q4*q4 + q1*q1 - q2*q2 - q3*q3;  // East component
        double bx_n = 2.0*(q1*q2 + q3*q4);             // North component
        double bx_u = 2.0*(q1*q3 - q2*q4);             // Up component

        // Current mass (decreasing during burn)
        double mass = DEMO_MASS - DEMO_MASS_FLOW * t;
        if (mass < 5.0) mass = 5.0;  // Minimum dry mass

        // Thrust acceleration in ENU
        double accel = DEMO_THRUST / mass;
        derivs[4] += accel * bx_e;  // East acceleration
        derivs[5] += accel * bx_n;  // North acceleration
        derivs[6] += accel * bx_u;  // Up acceleration
    }

    // Helper: RK4 single step (manual implementation to avoid external dependencies)
    void rk4Step(double dt) {
        auto k1 = m_rocket.computeDerivatives(m_time, m_state);
        applyDemoThrust(k1, m_time);

        // Compute k2 = f(t + dt/2, y + k1*dt/2)
        std::vector<double> temp_state(m_state.size());
        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + 0.5 * dt * k1[i];
        }
        auto k2 = m_rocket.computeDerivatives(m_time + 0.5 * dt, temp_state);
        applyDemoThrust(k2, m_time + 0.5 * dt);

        // Compute k3 = f(t + dt/2, y + k2*dt/2)
        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + 0.5 * dt * k2[i];
        }
        auto k3 = m_rocket.computeDerivatives(m_time + 0.5 * dt, temp_state);
        applyDemoThrust(k3, m_time + 0.5 * dt);

        // Compute k4 = f(t + dt, y + k3*dt)
        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + dt * k3[i];
        }
        auto k4 = m_rocket.computeDerivatives(m_time + dt, temp_state);
        applyDemoThrust(k4, m_time + dt);

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
     * Load demo data with hardcoded values for a simple sounding rocket
     * This allows the demo to work without any external data files
     */
    void loadDemoData() {
        // Demo rocket parameters (typical small sounding rocket):
        // - Mass: 15 kg dry, 20 kg wet (5 kg propellant)
        // - Thrust: ~800 N for 3 seconds
        // - Diameter: 0.16 m

        // Note: Since we can't easily set up interpolators without the internal
        // API, we'll use constant values. The RocketBody already has defaults.
        // For a proper demo, we'd need to expose array-based loading.

        m_demo_mode = true;
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

    // Debug: get raw state value at index
    double getStateAt(size_t index) const {
        if (!m_initialized || index >= m_state.size()) return -999.0;
        return m_state[index];
    }

    // Debug: return string with state info
    std::string getDebugInfo() const {
        if (!m_initialized) return "Not initialized";

        std::string info = "t=" + std::to_string(m_time) +
            " pos=[" + std::to_string(m_state[1]) + "," +
                       std::to_string(m_state[2]) + "," +
                       std::to_string(m_state[3]) + "]" +
            " vel=[" + std::to_string(m_state[4]) + "," +
                       std::to_string(m_state[5]) + "," +
                       std::to_string(m_state[6]) + "]" +
            " alt_fn=" + std::to_string(getAltitude()) +
            " demo=" + (m_demo_mode ? "ON" : "OFF");
        return info;
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

        // Demo data (embedded, no external files needed)
        .function("loadDemoData", &RocketSimulator::loadDemoData)

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
        .function("isInitialized", &RocketSimulator::isInitialized)

        // Debug
        .function("getStateAt", &RocketSimulator::getStateAt)
        .function("getDebugInfo", &RocketSimulator::getDebugInfo);
}
