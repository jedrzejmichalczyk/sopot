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
#include <algorithm>
#include <cmath>

using namespace emscripten;
using namespace sopot;
using namespace sopot::rocket;

/**
 * Helper: Convert JavaScript array to C++ vector
 */
template<typename T>
std::vector<T> vecFromJSArray(const val& js_array) {
    const unsigned length = js_array["length"].as<unsigned>();
    std::vector<T> vec(length);
    for (unsigned i = 0; i < length; ++i) {
        vec[i] = js_array[i].as<T>();
    }
    return vec;
}

/**
 * Helper: Write CSV file to Emscripten's virtual filesystem
 */
void writeCSV(const std::string& filepath,
              const std::vector<std::string>& headers,
              const std::vector<std::vector<double>>& columns) {
    if (columns.empty() || columns[0].empty()) {
        throw std::runtime_error("writeCSV: empty data");
    }
    if (headers.size() != columns.size()) {
        throw std::runtime_error("writeCSV: headers and columns size mismatch");
    }

    // Build CSV content
    std::string content;

    // Write header row
    for (size_t i = 0; i < headers.size(); ++i) {
        if (i > 0) content += ",";
        content += headers[i];
    }
    content += "\n";

    // Write data rows
    size_t num_rows = columns[0].size();
    for (size_t row = 0; row < num_rows; ++row) {
        for (size_t col = 0; col < columns.size(); ++col) {
            if (col > 0) content += ",";
            content += std::to_string(columns[col][row]);
        }
        content += "\n";
    }

    // Write to Emscripten's virtual filesystem using JavaScript
    EM_ASM({
        const content = UTF8ToString($0);
        const path = UTF8ToString($1);
        FS.writeFile(path, content);
    }, content.c_str(), filepath.c_str());
}

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
    bool m_record_history{true};  // Enable time-series recording

    // Demo mode parameters
    static constexpr double DEMO_THRUST = 1500.0;      // N (thrust force)
    static constexpr double DEMO_BURN_TIME = 3.5;      // seconds
    static constexpr double DEMO_MASS = 20.0;          // kg (initial)
    static constexpr double DEMO_MASS_FLOW = 1.5;      // kg/s

    // Maximum history size to prevent unbounded memory growth
    static constexpr size_t MAX_HISTORY_SIZE = 50000;  // ~500s at 100Hz or ~5000s at 10Hz
    static constexpr double MIN_MASS_THRESHOLD = 0.1;  // kg, minimum mass to prevent division by zero

    // Time-series data storage
    struct TimeSeriesData {
        // Time
        std::vector<double> time;

        // Kinematics
        std::vector<double> altitude;
        std::vector<double> speed;
        std::vector<double> pos_x, pos_y, pos_z;
        std::vector<double> vel_x, vel_y, vel_z;

        // Dynamics
        std::vector<double> mass;
        std::vector<double> accel_x, accel_y, accel_z;

        // Forces
        std::vector<double> thrust_magnitude;
        std::vector<double> gravity_magnitude;

        void clear() {
            time.clear();
            altitude.clear(); speed.clear();
            pos_x.clear(); pos_y.clear(); pos_z.clear();
            vel_x.clear(); vel_y.clear(); vel_z.clear();
            mass.clear();
            accel_x.clear(); accel_y.clear(); accel_z.clear();
            thrust_magnitude.clear();
            gravity_magnitude.clear();
        }

        void reserve(size_t n) {
            time.reserve(n);
            altitude.reserve(n); speed.reserve(n);
            pos_x.reserve(n); pos_y.reserve(n); pos_z.reserve(n);
            vel_x.reserve(n); vel_y.reserve(n); vel_z.reserve(n);
            mass.reserve(n);
            accel_x.reserve(n); accel_y.reserve(n); accel_z.reserve(n);
            thrust_magnitude.reserve(n);
            gravity_magnitude.reserve(n);
        }
    };

    TimeSeriesData m_history;

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

    // Helper: Record current state to time-series history
    void recordState() {
        if (!m_record_history || !m_initialized) return;

        // Check maximum history size to prevent unbounded memory growth
        if (m_history.time.size() >= MAX_HISTORY_SIZE) {
            // Stop recording when limit is reached
            // Alternative: implement circular buffer by removing oldest entry
            return;
        }

        m_history.time.push_back(m_time);

        // Kinematics
        auto pos = m_rocket.getPosition(m_state);
        m_history.pos_x.push_back(pos.x);
        m_history.pos_y.push_back(pos.y);
        m_history.pos_z.push_back(pos.z);

        auto vel = m_rocket.getVelocity(m_state);
        m_history.vel_x.push_back(vel.x);
        m_history.vel_y.push_back(vel.y);
        m_history.vel_z.push_back(vel.z);

        m_history.altitude.push_back(m_rocket.getAltitude(m_state));
        m_history.speed.push_back(m_rocket.getSpeed(m_state));

        // Dynamics
        double mass = m_rocket.getMass(m_state);
        m_history.mass.push_back(mass);

        // Guard against division by zero with minimum mass threshold
        double safe_mass = std::max(mass, MIN_MASS_THRESHOLD);

        auto total_force = m_rocket.queryStateFunction<dynamics::TotalForceENU>(m_state);
        m_history.accel_x.push_back(total_force.x / safe_mass);
        m_history.accel_y.push_back(total_force.y / safe_mass);
        m_history.accel_z.push_back(total_force.z / safe_mass);

        // Forces
        auto thrust = m_rocket.queryStateFunction<propulsion::ThrustForceBody>(m_state);
        m_history.thrust_magnitude.push_back(thrust.magnitude());

        double g = m_rocket.queryStateFunction<dynamics::GravityAcceleration>(m_state);
        m_history.gravity_magnitude.push_back(g);
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

        // Record state to history
        recordState();
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
        // Convert JavaScript arrays to C++ vectors
        std::vector<double> time_vec = vecFromJSArray<double>(time_js);
        std::vector<double> mass_vec = vecFromJSArray<double>(mass_js);

        if (time_vec.size() != mass_vec.size()) {
            throw std::runtime_error("loadMassData: time and mass arrays must have the same length");
        }
        if (time_vec.size() < 2) {
            throw std::runtime_error("loadMassData: need at least 2 data points");
        }

        // Write CSV files to Emscripten's virtual filesystem
        // This allows us to reuse the existing Rocket file-loading infrastructure
        writeCSV("/tmp/sim_mass.csv", {"time", "mass"}, {time_vec, mass_vec});

        // Write default constant values for CoG and inertia
        // Users can call additional methods to customize these if needed
        std::vector<double> default_times = {time_vec.front(), time_vec.back()};
        writeCSV("/tmp/sim_cog.csv", {"time", "cog"}, {default_times, {1.5, 1.5}});
        writeCSV("/tmp/sim_ix.csv", {"time", "ix"}, {default_times, {10.0, 10.0}});
        writeCSV("/tmp/sim_iyz.csv", {"time", "iyz"}, {default_times, {100.0, 100.0}});

        // Load from the virtual filesystem
        m_rocket.loadMassData("/tmp/");
    }

    /**
     * Load engine data from JavaScript arrays (simplified interface)
     * @param time Array of time points [s]
     * @param thrust Array of thrust values [N]
     *
     * This method creates a simplified engine model based on the thrust profile.
     * It estimates reasonable engine parameters from the thrust curve.
     */
    void loadEngineData(const val& time_js, const val& thrust_js) {
        // Convert JavaScript arrays to C++ vectors
        std::vector<double> time_vec = vecFromJSArray<double>(time_js);
        std::vector<double> thrust_vec = vecFromJSArray<double>(thrust_js);

        if (time_vec.size() != thrust_vec.size()) {
            throw std::runtime_error("loadEngineData: time and thrust arrays must have the same length");
        }
        if (time_vec.size() < 2) {
            throw std::runtime_error("loadEngineData: need at least 2 data points");
        }

        // Generate simplified engine parameters from thrust profile
        // These are reasonable defaults for a solid rocket motor
        size_t n = time_vec.size();
        std::vector<std::vector<double>> columns(8);

        // Find max thrust to scale parameters
        double max_thrust = *std::max_element(thrust_vec.begin(), thrust_vec.end());

        for (size_t i = 0; i < n; ++i) {
            double t = time_vec[i];
            double F = thrust_vec[i];

            // Estimate throat and exit diameters based on thrust
            // F ≈ p_c * A_throat * CF, with CF ≈ 1.5-1.8 for typical solid motors
            // Assume p_c ≈ 5 MPa, CF ≈ 1.6
            double p_c = 5.0e6;  // Pa
            double CF = 1.6;
            double A_throat = (F > 0) ? F / (p_c * CF) : 1e-6;
            double d_throat = 2.0 * std::sqrt(A_throat / 3.14159265359);

            // Exit diameter (assume expansion ratio of 4-8)
            double expansion_ratio = 6.0;
            double d_exit = d_throat * std::sqrt(expansion_ratio);

            // Column 0: time
            columns[0].push_back(t);

            // Column 1: throat_diameter [m]
            columns[1].push_back(d_throat);

            // Column 2: exit_diameter [m]
            columns[2].push_back(d_exit);

            // Column 3: combustion_pressure [Pa]
            columns[3].push_back(p_c);

            // Column 4: combustion_temp [K]
            // Typical for solid propellants: 2500-3500 K
            columns[4].push_back(3000.0);

            // Column 5: gamma (ratio of specific heats)
            // Typical for combustion products: 1.2-1.25
            columns[5].push_back(1.24);

            // Column 6: mol_mass [kg/mol]
            // Typical for solid propellant combustion products: 0.020-0.030
            columns[6].push_back(0.024);

            // Column 7: efficiency
            // Typical nozzle efficiency: 0.95-0.98
            columns[7].push_back(0.96);
        }

        // Write engine CSV to virtual filesystem
        writeCSV("/tmp/sim_engine.csv",
                 {"time", "throat_d", "exit_d", "combustion_pressure",
                  "combustion_temp", "gamma", "mol_mass", "efficiency"},
                 columns);

        // Load from the virtual filesystem
        m_rocket.loadEngineData("/tmp/");
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
        m_history.clear();

        // Reserve space for typical flight (assume 100 seconds at 100Hz = 10k points)
        m_history.reserve(10000);

        // Record initial state at t=0
        recordState();
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
    // TIME-SERIES DATA RETRIEVAL
    //=========================================================================

    /**
     * Get time-series data as JavaScript arrays
     * Returns an object with all available state function histories
     */
    val getTimeSeries() const {
        val result = val::object();

        // Convert C++ vectors to JavaScript arrays
        result.set("time", val::array(m_history.time.begin(), m_history.time.end()));

        // Kinematics
        val kinematics = val::object();
        kinematics.set("altitude", val::array(m_history.altitude.begin(), m_history.altitude.end()));
        kinematics.set("speed", val::array(m_history.speed.begin(), m_history.speed.end()));
        kinematics.set("pos_x", val::array(m_history.pos_x.begin(), m_history.pos_x.end()));
        kinematics.set("pos_y", val::array(m_history.pos_y.begin(), m_history.pos_y.end()));
        kinematics.set("pos_z", val::array(m_history.pos_z.begin(), m_history.pos_z.end()));
        kinematics.set("vel_x", val::array(m_history.vel_x.begin(), m_history.vel_x.end()));
        kinematics.set("vel_y", val::array(m_history.vel_y.begin(), m_history.vel_y.end()));
        kinematics.set("vel_z", val::array(m_history.vel_z.begin(), m_history.vel_z.end()));
        result.set("kinematics", kinematics);

        // Dynamics
        val dynamics = val::object();
        dynamics.set("mass", val::array(m_history.mass.begin(), m_history.mass.end()));
        dynamics.set("accel_x", val::array(m_history.accel_x.begin(), m_history.accel_x.end()));
        dynamics.set("accel_y", val::array(m_history.accel_y.begin(), m_history.accel_y.end()));
        dynamics.set("accel_z", val::array(m_history.accel_z.begin(), m_history.accel_z.end()));
        result.set("dynamics", dynamics);

        // Forces
        val forces = val::object();
        forces.set("thrust", val::array(m_history.thrust_magnitude.begin(), m_history.thrust_magnitude.end()));
        forces.set("gravity", val::array(m_history.gravity_magnitude.begin(), m_history.gravity_magnitude.end()));
        result.set("forces", forces);

        return result;
    }

    /**
     * Get number of recorded data points
     */
    size_t getHistorySize() const {
        return m_history.time.size();
    }

    /**
     * Enable/disable time-series recording
     */
    void setRecordHistory(bool enable) {
        m_record_history = enable;
    }

    /**
     * Clear time-series history
     */
    void clearHistory() {
        m_history.clear();
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
            " demo=" + (m_demo_mode ? "ON" : "OFF") +
            " hist_size=" + std::to_string(m_history.time.size());
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

        // Time-series data retrieval
        .function("getTimeSeries", &RocketSimulator::getTimeSeries)
        .function("getHistorySize", &RocketSimulator::getHistorySize)
        .function("setRecordHistory", &RocketSimulator::setRecordHistory)
        .function("clearHistory", &RocketSimulator::clearHistory)

        // Utility
        .function("getStateDimension", &RocketSimulator::getStateDimension)
        .function("isInitialized", &RocketSimulator::isInitialized)

        // Debug
        .function("getStateAt", &RocketSimulator::getStateAt)
        .function("getDebugInfo", &RocketSimulator::getDebugInfo);
}
