/**
 * WebAssembly wrapper for SOPOT Inverted Double Pendulum Control System
 *
 * This file provides Emscripten embind bindings to expose the C++ inverted
 * double pendulum control system to JavaScript/TypeScript.
 *
 * Build with:
 *   emcc -std=c++20 -lembind -O3 -s WASM=1 \
 *        -I.. wasm_inverted_pendulum.cpp -o sopot_pendulum.js
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/emscripten.h>
#include "../physics/control/cart_double_pendulum.hpp"
#include "../physics/control/lqr.hpp"
#include "../physics/control/state_feedback_controller.hpp"
#include "../core/typed_component.hpp"
#include <memory>
#include <string>
#include <stdexcept>
#include <cmath>
#include <vector>
#include <array>

using namespace emscripten;
using namespace sopot;
using namespace sopot::control;

/**
 * InvertedPendulumSimulator: JavaScript-friendly wrapper for the control system
 *
 * This class provides:
 * - Clean initialization with physical parameters
 * - LQR controller with tunable weights
 * - Single-step integration for JavaScript-controlled animation
 * - State queries returning JavaScript objects
 * - Disturbance injection for demos
 */
class InvertedPendulumSimulator {
private:
    // System parameters
    double m_cart_mass{1.0};
    double m_mass1{0.5};
    double m_mass2{0.5};
    double m_length1{0.5};
    double m_length2{0.5};
    double m_gravity{9.81};

    // Controller
    std::unique_ptr<InvertedDoublePendulumController> m_controller;

    // State: [x, θ₁, θ₂, ẋ, ω₁, ω₂]
    std::array<double, 6> m_state{};
    std::array<double, 6> m_initial_state{};

    // Simulation
    double m_time{0.0};
    double m_dt{0.01};  // 10ms timestep
    bool m_initialized{false};
    bool m_controller_enabled{true};

    // Control limits
    double m_max_force{100.0};  // N

    // History for visualization
    struct HistoryEntry {
        double time;
        double x, theta1, theta2;
        double xdot, omega1, omega2;
        double control_force;
    };
    std::vector<HistoryEntry> m_history;
    bool m_record_history{true};
    static constexpr size_t MAX_HISTORY_SIZE = 10000;

    // RK4 integration step
    void rk4Step(double dt) {
        // Get control force
        double F = 0.0;
        if (m_controller_enabled && m_controller) {
            F = m_controller->compute(m_state)[0];
            F = std::clamp(F, -m_max_force, m_max_force);
        }

        // RK4 integration
        auto derivs = [this, F](const std::array<double, 6>& s) -> std::array<double, 6> {
            // Create temporary plant with current state
            double x = s[0], theta1 = s[1], theta2 = s[2];
            double xdot = s[3], omega1 = s[4], omega2 = s[5];

            // Compute derivatives (physics from CartDoublePendulum)
            double mc = m_cart_mass, m1 = m_mass1, m2 = m_mass2;
            double L1 = m_length1, L2 = m_length2, g = m_gravity;

            double s1 = std::sin(theta1), c1 = std::cos(theta1);
            double s2 = std::sin(theta2), c2 = std::cos(theta2);
            double s12 = std::sin(theta1 - theta2), c12 = std::cos(theta1 - theta2);

            // Mass matrix elements
            double M11 = mc + m1 + m2;
            double M12 = (m1 + m2) * L1 * c1;
            double M13 = m2 * L2 * c2;
            double M22 = (m1 + m2) * L1 * L1;
            double M23 = m2 * L1 * L2 * c12;
            double M33 = m2 * L2 * L2;

            // Right-hand side
            double rhs1 = F + (m1 + m2) * L1 * s1 * omega1 * omega1
                           + m2 * L2 * s2 * omega2 * omega2;
            double rhs2 = (m1 + m2) * g * L1 * s1
                        - m2 * L1 * L2 * s12 * omega2 * omega2;
            double rhs3 = m2 * g * L2 * s2
                        + m2 * L1 * L2 * s12 * omega1 * omega1;

            // Solve 3x3 system using Cramer's rule
            double det = M11 * (M22 * M33 - M23 * M23)
                       - M12 * (M12 * M33 - M23 * M13)
                       + M13 * (M12 * M23 - M22 * M13);

            double C11 = M22 * M33 - M23 * M23;
            double C12 = -(M12 * M33 - M13 * M23);
            double C13 = M12 * M23 - M13 * M22;
            double C21 = -(M12 * M33 - M13 * M23);
            double C22 = M11 * M33 - M13 * M13;
            double C23 = -(M11 * M23 - M12 * M13);
            double C31 = M12 * M23 - M13 * M22;
            double C32 = -(M11 * M23 - M12 * M13);
            double C33 = M11 * M22 - M12 * M12;

            if (std::abs(det) < 1e-12) {
                throw std::runtime_error("InvertedPendulumSimulator: mass matrix is singular");
            }
            double inv_det = 1.0 / det;
            double xddot = inv_det * (C11 * rhs1 + C12 * rhs2 + C13 * rhs3);
            double alpha1 = inv_det * (C21 * rhs1 + C22 * rhs2 + C23 * rhs3);
            double alpha2 = inv_det * (C31 * rhs1 + C32 * rhs2 + C33 * rhs3);

            return {xdot, omega1, omega2, xddot, alpha1, alpha2};
        };

        // RK4 stages
        auto k1 = derivs(m_state);

        std::array<double, 6> s2;
        for (int i = 0; i < 6; i++) s2[i] = m_state[i] + 0.5 * dt * k1[i];
        auto k2 = derivs(s2);

        std::array<double, 6> s3;
        for (int i = 0; i < 6; i++) s3[i] = m_state[i] + 0.5 * dt * k2[i];
        auto k3 = derivs(s3);

        std::array<double, 6> s4;
        for (int i = 0; i < 6; i++) s4[i] = m_state[i] + dt * k3[i];
        auto k4 = derivs(s4);

        // Update state
        for (int i = 0; i < 6; i++) {
            m_state[i] += dt * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) / 6.0;
        }

        m_time += dt;

        // Record history
        if (m_record_history && m_history.size() < MAX_HISTORY_SIZE) {
            m_history.push_back({
                m_time,
                m_state[0], m_state[1], m_state[2],
                m_state[3], m_state[4], m_state[5],
                F
            });
        }
    }

public:
    InvertedPendulumSimulator() = default;

    //=========================================================================
    // CONFIGURATION
    //=========================================================================

    /**
     * Set physical parameters
     */
    void setParameters(double cart_mass, double m1, double m2, double L1, double L2, double g) {
        m_cart_mass = cart_mass;
        m_mass1 = m1;
        m_mass2 = m2;
        m_length1 = L1;
        m_length2 = L2;
        m_gravity = g;
        m_initialized = false;
    }

    /**
     * Set initial state
     * @param x Cart position (m)
     * @param theta1 Angle of link 1 from vertical (rad)
     * @param theta2 Angle of link 2 from vertical (rad)
     * @param xdot Cart velocity (m/s)
     * @param omega1 Angular velocity of link 1 (rad/s)
     * @param omega2 Angular velocity of link 2 (rad/s)
     */
    void setInitialState(double x, double theta1, double theta2,
                         double xdot, double omega1, double omega2) {
        m_initial_state = {x, theta1, theta2, xdot, omega1, omega2};
    }

    /**
     * Set simulation timestep
     */
    void setTimestep(double dt) {
        m_dt = dt;
    }

    /**
     * Set maximum control force
     */
    void setMaxForce(double max_force) {
        m_max_force = max_force;
    }

    /**
     * Configure LQR controller weights
     * @param q_diag Diagonal elements of Q matrix [q_x, q_θ1, q_θ2, q_ẋ, q_ω1, q_ω2]
     * @param r Control weight R
     */
    void configureLQR(val q_diag_js, double r) {
        std::array<double, 6> q_diag;
        for (int i = 0; i < 6; i++) {
            q_diag[i] = q_diag_js[i].as<double>();
        }

        m_controller = std::make_unique<InvertedDoublePendulumController>(
            InvertedDoublePendulumController::createWithLQR(
                m_cart_mass, m_mass1, m_mass2, m_length1, m_length2, m_gravity,
                q_diag, r
            )
        );

        // Set saturation limits
        m_controller->setSaturationLimits({-m_max_force}, {m_max_force});
    }

    /**
     * Initialize with default parameters
     */
    void setupDefault() {
        // Default parameters: lab-scale inverted pendulum
        m_cart_mass = 1.0;
        m_mass1 = 0.5;
        m_mass2 = 0.5;
        m_length1 = 0.5;
        m_length2 = 0.5;
        m_gravity = 9.81;

        // Default initial state: small perturbation from upright
        m_initial_state = {0.0, 0.1, 0.05, 0.0, 0.0, 0.0};

        // Default LQR weights
        std::array<double, 6> q_diag = {10.0, 100.0, 100.0, 1.0, 10.0, 10.0};
        double r = 0.1;

        m_controller = std::make_unique<InvertedDoublePendulumController>(
            InvertedDoublePendulumController::createWithLQR(
                m_cart_mass, m_mass1, m_mass2, m_length1, m_length2, m_gravity,
                q_diag, r
            )
        );
        m_controller->setSaturationLimits({-m_max_force}, {m_max_force});

        m_state = m_initial_state;
        m_time = 0.0;
        m_history.clear();
        m_initialized = true;
    }

    /**
     * Reset to initial state
     */
    void reset() {
        m_state = m_initial_state;
        m_time = 0.0;
        m_history.clear();

        // Record initial state
        if (m_record_history) {
            m_history.push_back({
                0.0,
                m_state[0], m_state[1], m_state[2],
                m_state[3], m_state[4], m_state[5],
                0.0
            });
        }
    }

    //=========================================================================
    // SIMULATION CONTROL
    //=========================================================================

    /**
     * Advance simulation by one timestep
     * Returns true if simulation is stable (angles < 45°)
     */
    bool step() {
        if (!m_initialized) {
            throw std::runtime_error("Call setupDefault() or configure before step()");
        }

        rk4Step(m_dt);

        // Check if pendulum has fallen (angles > 45°)
        double theta1 = std::abs(m_state[1]);
        double theta2 = std::abs(m_state[2]);
        return theta1 < M_PI/4 && theta2 < M_PI/4;
    }

    /**
     * Step with custom timestep
     */
    bool stepWithDt(double dt) {
        if (!m_initialized) {
            throw std::runtime_error("Call setupDefault() or configure before step()");
        }

        rk4Step(dt);

        double theta1 = std::abs(m_state[1]);
        double theta2 = std::abs(m_state[2]);
        return theta1 < M_PI/4 && theta2 < M_PI/4;
    }

    /**
     * Enable/disable controller (for demonstrating unstable behavior)
     */
    void setControllerEnabled(bool enabled) {
        m_controller_enabled = enabled;
    }

    /**
     * Apply impulse disturbance to cart
     */
    void applyCartImpulse(double impulse) {
        m_state[3] += impulse / m_cart_mass;  // Δv = J/m
    }

    /**
     * Apply angular impulse to link 1
     */
    void applyLink1Impulse(double impulse) {
        // Approximate: impulse affects angular velocity
        double I1 = m_mass1 * m_length1 * m_length1;
        m_state[4] += impulse / I1;
    }

    /**
     * Apply angular impulse to link 2
     */
    void applyLink2Impulse(double impulse) {
        double I2 = m_mass2 * m_length2 * m_length2;
        m_state[5] += impulse / I2;
    }

    //=========================================================================
    // STATE QUERIES
    //=========================================================================

    double getTime() const { return m_time; }

    double getCartPosition() const { return m_state[0]; }
    double getTheta1() const { return m_state[1]; }
    double getTheta2() const { return m_state[2]; }
    double getCartVelocity() const { return m_state[3]; }
    double getOmega1() const { return m_state[4]; }
    double getOmega2() const { return m_state[5]; }

    /**
     * Get link 1 tip position [x, y]
     */
    val getLink1TipPosition() const {
        double x = m_state[0];
        double theta1 = m_state[1];
        double L1 = m_length1;

        val result = val::object();
        result.set("x", x + L1 * std::sin(theta1));
        result.set("y", L1 * std::cos(theta1));
        return result;
    }

    /**
     * Get link 2 tip position [x, y]
     */
    val getLink2TipPosition() const {
        double x = m_state[0];
        double theta1 = m_state[1];
        double theta2 = m_state[2];
        double L1 = m_length1, L2 = m_length2;

        val result = val::object();
        result.set("x", x + L1 * std::sin(theta1) + L2 * std::sin(theta2));
        result.set("y", L1 * std::cos(theta1) + L2 * std::cos(theta2));
        return result;
    }

    /**
     * Get current control force
     */
    double getControlForce() const {
        if (!m_controller_enabled || !m_controller) return 0.0;
        double F = m_controller->compute(m_state)[0];
        return std::clamp(F, -m_max_force, m_max_force);
    }

    /**
     * Get full state as JavaScript object
     */
    val getFullState() const {
        val result = val::object();
        result.set("time", m_time);
        result.set("x", m_state[0]);
        result.set("theta1", m_state[1]);
        result.set("theta2", m_state[2]);
        result.set("xdot", m_state[3]);
        result.set("omega1", m_state[4]);
        result.set("omega2", m_state[5]);
        result.set("controlForce", getControlForce());
        result.set("link1Tip", getLink1TipPosition());
        result.set("link2Tip", getLink2TipPosition());
        return result;
    }

    /**
     * Get visualization data: cart and pendulum geometry
     */
    val getVisualizationData() const {
        double x = m_state[0];
        double theta1 = m_state[1];
        double theta2 = m_state[2];
        double L1 = m_length1, L2 = m_length2;

        // Cart center
        double cart_x = x;
        double cart_y = 0.0;

        // Link 1 joint (cart pivot)
        double j1_x = cart_x;
        double j1_y = cart_y;

        // Link 1 tip = Link 2 joint
        double j2_x = j1_x + L1 * std::sin(theta1);
        double j2_y = j1_y + L1 * std::cos(theta1);

        // Link 2 tip
        double tip_x = j2_x + L2 * std::sin(theta2);
        double tip_y = j2_y + L2 * std::cos(theta2);

        val result = val::object();

        val cart = val::object();
        cart.set("x", cart_x);
        cart.set("y", cart_y);
        result.set("cart", cart);

        val joint1 = val::object();
        joint1.set("x", j1_x);
        joint1.set("y", j1_y);
        result.set("joint1", joint1);

        val joint2 = val::object();
        joint2.set("x", j2_x);
        joint2.set("y", j2_y);
        result.set("joint2", joint2);

        val tip = val::object();
        tip.set("x", tip_x);
        tip.set("y", tip_y);
        result.set("tip", tip);

        result.set("theta1", theta1);
        result.set("theta2", theta2);
        result.set("controlForce", getControlForce());

        return result;
    }

    //=========================================================================
    // HISTORY
    //=========================================================================

    void setRecordHistory(bool record) { m_record_history = record; }
    size_t getHistorySize() const { return m_history.size(); }
    void clearHistory() { m_history.clear(); }

    /**
     * Get history as JavaScript arrays
     */
    val getHistory() const {
        val result = val::object();

        std::vector<double> time, x, theta1, theta2, xdot, omega1, omega2, control;
        for (const auto& h : m_history) {
            time.push_back(h.time);
            x.push_back(h.x);
            theta1.push_back(h.theta1);
            theta2.push_back(h.theta2);
            xdot.push_back(h.xdot);
            omega1.push_back(h.omega1);
            omega2.push_back(h.omega2);
            control.push_back(h.control_force);
        }

        result.set("time", val::array(time.begin(), time.end()));
        result.set("x", val::array(x.begin(), x.end()));
        result.set("theta1", val::array(theta1.begin(), theta1.end()));
        result.set("theta2", val::array(theta2.begin(), theta2.end()));
        result.set("xdot", val::array(xdot.begin(), xdot.end()));
        result.set("omega1", val::array(omega1.begin(), omega1.end()));
        result.set("omega2", val::array(omega2.begin(), omega2.end()));
        result.set("controlForce", val::array(control.begin(), control.end()));

        return result;
    }

    //=========================================================================
    // PARAMETERS
    //=========================================================================

    double getCartMass() const { return m_cart_mass; }
    double getMass1() const { return m_mass1; }
    double getMass2() const { return m_mass2; }
    double getLength1() const { return m_length1; }
    double getLength2() const { return m_length2; }
    double getGravity() const { return m_gravity; }
    double getMaxForce() const { return m_max_force; }
    bool isInitialized() const { return m_initialized; }
    bool isControllerEnabled() const { return m_controller_enabled; }

    /**
     * Get LQR gain matrix as array
     */
    val getLQRGain() const {
        if (!m_controller) {
            return val::array();
        }
        const auto& K = m_controller->getGain();
        std::vector<double> gains(K[0].begin(), K[0].end());
        return val::array(gains.begin(), gains.end());
    }
};

//=============================================================================
// EMSCRIPTEN BINDINGS
//=============================================================================

EMSCRIPTEN_BINDINGS(inverted_pendulum) {
    class_<InvertedPendulumSimulator>("InvertedPendulumSimulator")
        .constructor<>()

        // Configuration
        .function("setParameters", &InvertedPendulumSimulator::setParameters)
        .function("setInitialState", &InvertedPendulumSimulator::setInitialState)
        .function("setTimestep", &InvertedPendulumSimulator::setTimestep)
        .function("setMaxForce", &InvertedPendulumSimulator::setMaxForce)
        .function("configureLQR", &InvertedPendulumSimulator::configureLQR)
        .function("setupDefault", &InvertedPendulumSimulator::setupDefault)
        .function("reset", &InvertedPendulumSimulator::reset)

        // Simulation control
        .function("step", &InvertedPendulumSimulator::step)
        .function("stepWithDt", &InvertedPendulumSimulator::stepWithDt)
        .function("setControllerEnabled", &InvertedPendulumSimulator::setControllerEnabled)
        .function("applyCartImpulse", &InvertedPendulumSimulator::applyCartImpulse)
        .function("applyLink1Impulse", &InvertedPendulumSimulator::applyLink1Impulse)
        .function("applyLink2Impulse", &InvertedPendulumSimulator::applyLink2Impulse)

        // State queries
        .function("getTime", &InvertedPendulumSimulator::getTime)
        .function("getCartPosition", &InvertedPendulumSimulator::getCartPosition)
        .function("getTheta1", &InvertedPendulumSimulator::getTheta1)
        .function("getTheta2", &InvertedPendulumSimulator::getTheta2)
        .function("getCartVelocity", &InvertedPendulumSimulator::getCartVelocity)
        .function("getOmega1", &InvertedPendulumSimulator::getOmega1)
        .function("getOmega2", &InvertedPendulumSimulator::getOmega2)
        .function("getLink1TipPosition", &InvertedPendulumSimulator::getLink1TipPosition)
        .function("getLink2TipPosition", &InvertedPendulumSimulator::getLink2TipPosition)
        .function("getControlForce", &InvertedPendulumSimulator::getControlForce)
        .function("getFullState", &InvertedPendulumSimulator::getFullState)
        .function("getVisualizationData", &InvertedPendulumSimulator::getVisualizationData)

        // History
        .function("setRecordHistory", &InvertedPendulumSimulator::setRecordHistory)
        .function("getHistorySize", &InvertedPendulumSimulator::getHistorySize)
        .function("clearHistory", &InvertedPendulumSimulator::clearHistory)
        .function("getHistory", &InvertedPendulumSimulator::getHistory)

        // Parameters
        .function("getCartMass", &InvertedPendulumSimulator::getCartMass)
        .function("getMass1", &InvertedPendulumSimulator::getMass1)
        .function("getMass2", &InvertedPendulumSimulator::getMass2)
        .function("getLength1", &InvertedPendulumSimulator::getLength1)
        .function("getLength2", &InvertedPendulumSimulator::getLength2)
        .function("getGravity", &InvertedPendulumSimulator::getGravity)
        .function("getMaxForce", &InvertedPendulumSimulator::getMaxForce)
        .function("isInitialized", &InvertedPendulumSimulator::isInitialized)
        .function("isControllerEnabled", &InvertedPendulumSimulator::isControllerEnabled)
        .function("getLQRGain", &InvertedPendulumSimulator::getLQRGain);
}
