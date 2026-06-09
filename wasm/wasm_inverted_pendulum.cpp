/**
 * WebAssembly wrapper for the SOPOT Cart-N-Pendulum Control System.
 *
 * Configured for a cart balancing a chain of NUM_LINKS inverted pendulums
 * (six by default) stabilized by an LQR controller. Provides Emscripten
 * embind bindings exposing the C++ control system to JavaScript/TypeScript.
 *
 * Build with:
 *   emcc -std=c++20 -lembind -O3 -s WASM=1 \
 *        -I.. wasm_inverted_pendulum.cpp -o sopot_pendulum.js
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/emscripten.h>
#include "../physics/control/cart_n_pendulum.hpp"
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
 * InvertedPendulumSimulator: JavaScript-friendly wrapper for the control system.
 *
 * The cart carries NUM_LINKS serially-connected pendulum links. The state
 * vector has size 2*(NUM_LINKS+1) = [x, θ₁…θ_N, ẋ, ω₁…ω_N]. All physics and
 * control runs in C++; the frontend handles visualization and interaction.
 */
class InvertedPendulumSimulator {
public:
    static constexpr size_t NUM_LINKS = 6;
    static constexpr size_t NSTATE = 2 * (NUM_LINKS + 1);  // 14

private:
    using Plant = CartNPendulum<NUM_LINKS, double>;
    using Controller = CartNPendulumController<NUM_LINKS>;

    // System parameters (uniform links by default).
    double m_cart_mass{2.0};
    std::array<double, NUM_LINKS> m_link_mass{};
    std::array<double, NUM_LINKS> m_link_length{};
    double m_gravity{9.81};

    std::unique_ptr<Controller> m_controller;

    std::array<double, NSTATE> m_state{};
    std::array<double, NSTATE> m_initial_state{};

    double m_time{0.0};
    double m_dt{0.01};
    bool m_initialized{false};
    bool m_controller_enabled{true};
    double m_max_force{500.0};

    // LQR design weights / parameters.
    std::array<double, NSTATE> m_q_diag{};
    double m_r{0.001};
    double m_riccati_dt{0.005};

    struct HistoryEntry {
        double time;
        double x;
        double control_force;
        double max_angle;
    };
    std::vector<HistoryEntry> m_history;
    bool m_record_history{true};
    static constexpr size_t MAX_HISTORY_SIZE = 10000;

    static constexpr size_t idxX()  { return 0; }
    static constexpr size_t idxTheta(size_t i) { return 1 + i; }
    static constexpr size_t idxXDot() { return NUM_LINKS + 1; }
    static constexpr size_t idxOmega(size_t i) { return NUM_LINKS + 2 + i; }

    double maxAbsAngle() const {
        double m = 0.0;
        for (size_t i = 0; i < NUM_LINKS; ++i) m = std::max(m, std::abs(m_state[idxTheta(i)]));
        return m;
    }

    double currentControlForce() const {
        if (!m_controller_enabled || !m_controller) return 0.0;
        std::array<double, NSTATE> s = m_state;
        double F = m_controller->computeForce(s);
        return std::clamp(F, -m_max_force, m_max_force);
    }

    // Evaluate the plant derivative with a fixed control force.
    std::array<double, NSTATE> derivativesAt(const std::array<double, NSTATE>& s, double F) const {
        Plant plant(m_cart_mass, m_link_mass, m_link_length, m_gravity);
        plant.setControlForce(F);
        std::vector<double> sv(s.begin(), s.end());
        std::span<const double> sp(sv);
        TypedRegistry<double> reg{};
        auto d = plant.derivatives(0.0, sp, sp, reg);
        std::array<double, NSTATE> out{};
        for (size_t i = 0; i < NSTATE; ++i) out[i] = d[i];
        return out;
    }

    void rk4Step(double dt) {
        // Control force held constant across the RK4 stages (computed once).
        double F = currentControlForce();

        auto add = [](const std::array<double, NSTATE>& a,
                      const std::array<double, NSTATE>& b, double h) {
            std::array<double, NSTATE> r{};
            for (size_t i = 0; i < NSTATE; ++i) r[i] = a[i] + h * b[i];
            return r;
        };

        auto k1 = derivativesAt(m_state, F);
        auto k2 = derivativesAt(add(m_state, k1, 0.5 * dt), F);
        auto k3 = derivativesAt(add(m_state, k2, 0.5 * dt), F);
        auto k4 = derivativesAt(add(m_state, k3, dt), F);

        for (size_t i = 0; i < NSTATE; ++i) {
            m_state[i] += dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
        }
        m_time += dt;

        if (m_record_history && m_history.size() < MAX_HISTORY_SIZE) {
            m_history.push_back({m_time, m_state[idxX()], F, maxAbsAngle()});
        }
    }

    void rebuildController() {
        m_controller = std::make_unique<Controller>(
            Controller::createWithLQR(
                m_cart_mass, m_link_mass, m_link_length, m_gravity,
                m_q_diag, m_r, m_riccati_dt
            )
        );
        m_controller->setSaturationLimits({-m_max_force}, {m_max_force});
    }

public:
    InvertedPendulumSimulator() {
        m_link_mass.fill(0.1);
        m_link_length.fill(0.3);
    }

    //=========================================================================
    // CONFIGURATION
    //=========================================================================

    size_t getNumLinks() const { return NUM_LINKS; }

    /** Set uniform physical parameters for every link. */
    void setUniformParameters(double cart_mass, double link_mass, double link_length, double g) {
        m_cart_mass = cart_mass;
        m_link_mass.fill(link_mass);
        m_link_length.fill(link_length);
        m_gravity = g;
        m_initialized = false;
    }

    /** Set per-link masses and lengths from JS arrays (length NUM_LINKS). */
    void setParameters(double cart_mass, val masses_js, val lengths_js, double g) {
        m_cart_mass = cart_mass;
        for (size_t i = 0; i < NUM_LINKS; ++i) {
            m_link_mass[i] = masses_js[i].as<double>();
            m_link_length[i] = lengths_js[i].as<double>();
        }
        m_gravity = g;
        m_initialized = false;
    }

    /** Set all link angles to the same value (cart at rest, upright chain). */
    void setInitialAnglesUniform(double angle_rad) {
        m_initial_state.fill(0.0);
        for (size_t i = 0; i < NUM_LINKS; ++i) m_initial_state[idxTheta(i)] = angle_rad;
    }

    /** Set the full initial state from a JS array (length NSTATE). */
    void setInitialState(val state_js) {
        for (size_t i = 0; i < NSTATE; ++i) m_initial_state[i] = state_js[i].as<double>();
    }

    /** Set initial angles from a JS array (length NUM_LINKS), cart at rest. */
    void setInitialAngles(val angles_js) {
        m_initial_state.fill(0.0);
        for (size_t i = 0; i < NUM_LINKS; ++i) {
            m_initial_state[idxTheta(i)] = angles_js[i].as<double>();
        }
    }

    void setTimestep(double dt) { m_dt = dt; }

    void setMaxForce(double max_force) {
        m_max_force = max_force;
        if (m_controller) m_controller->setSaturationLimits({-m_max_force}, {m_max_force});
    }

    /** Configure the LQR controller (qDiag length NSTATE, scalar r). */
    void configureLQR(val q_diag_js, double r) {
        for (size_t i = 0; i < NSTATE; ++i) m_q_diag[i] = q_diag_js[i].as<double>();
        m_r = r;
        rebuildController();
    }

    /** Initialize with sensible defaults for a six-link inverted pendulum. */
    void setupDefault() {
        m_cart_mass = 2.0;
        m_link_mass.fill(0.1);
        m_link_length.fill(0.3);
        m_gravity = 9.81;

        // Default LQR weights: heavy penalty on link angles.
        m_q_diag.fill(0.0);
        m_q_diag[idxX()] = 1.0;
        for (size_t i = 0; i < NUM_LINKS; ++i) m_q_diag[idxTheta(i)] = 200.0;
        m_q_diag[idxXDot()] = 1.0;
        for (size_t i = 0; i < NUM_LINKS; ++i) m_q_diag[idxOmega(i)] = 10.0;
        m_r = 0.001;
        m_riccati_dt = 0.005;

        // Small initial perturbation from upright.
        setInitialAnglesUniform(0.02);

        rebuildController();

        m_state = m_initial_state;
        m_time = 0.0;
        m_history.clear();
        m_initialized = true;
    }

    void reset() {
        m_state = m_initial_state;
        m_time = 0.0;
        m_history.clear();
        if (m_record_history) {
            m_history.push_back({0.0, m_state[idxX()], 0.0, maxAbsAngle()});
        }
    }

    //=========================================================================
    // SIMULATION CONTROL
    //=========================================================================

    /** Advance one timestep; returns true while the chain is still upright. */
    bool step() {
        if (!m_initialized) throw std::runtime_error("Call setupDefault() or configure before step()");
        rk4Step(m_dt);
        return maxAbsAngle() < M_PI / 4;
    }

    bool stepWithDt(double dt) {
        if (!m_initialized) throw std::runtime_error("Call setupDefault() or configure before step()");
        rk4Step(dt);
        return maxAbsAngle() < M_PI / 4;
    }

    void setControllerEnabled(bool enabled) { m_controller_enabled = enabled; }

    /** Apply a horizontal impulse to the cart. */
    void applyCartImpulse(double impulse) {
        m_state[idxXDot()] += impulse / m_cart_mass;
    }

    /** Apply an angular impulse to a given link (0-based index). */
    void applyLinkImpulse(int link, double impulse) {
        if (link < 0 || link >= static_cast<int>(NUM_LINKS)) return;
        double I = m_link_mass[link] * m_link_length[link] * m_link_length[link];
        m_state[idxOmega(static_cast<size_t>(link))] += impulse / I;
    }

    //=========================================================================
    // STATE QUERIES
    //=========================================================================

    double getTime() const { return m_time; }
    double getCartPosition() const { return m_state[idxX()]; }
    double getCartVelocity() const { return m_state[idxXDot()]; }
    double getMaxAngle() const { return maxAbsAngle(); }
    double getControlForce() const { return currentControlForce(); }

    /** Angle of each link (radians) as a JS array. */
    val getAngles() const {
        std::vector<double> a(NUM_LINKS);
        for (size_t i = 0; i < NUM_LINKS; ++i) a[i] = m_state[idxTheta(i)];
        return val::array(a.begin(), a.end());
    }

    /** Angular velocity of each link as a JS array. */
    val getAngularVelocities() const {
        std::vector<double> w(NUM_LINKS);
        for (size_t i = 0; i < NUM_LINKS; ++i) w[i] = m_state[idxOmega(i)];
        return val::array(w.begin(), w.end());
    }

    /** Joint positions: [pivot, tip₁, …, tip_N] as a JS array of {x,y}. */
    val getJoints() const {
        val arr = val::array();
        double px = m_state[idxX()];
        double py = 0.0;
        val pivot = val::object();
        pivot.set("x", px);
        pivot.set("y", py);
        arr.set(0, pivot);
        for (size_t i = 0; i < NUM_LINKS; ++i) {
            double theta = m_state[idxTheta(i)];
            px += m_link_length[i] * std::sin(theta);
            py += m_link_length[i] * std::cos(theta);
            val joint = val::object();
            joint.set("x", px);
            joint.set("y", py);
            arr.set(static_cast<int>(i + 1), joint);
        }
        return arr;
    }

    val getFullState() const {
        val result = val::object();
        result.set("time", m_time);
        result.set("x", m_state[idxX()]);
        result.set("xdot", m_state[idxXDot()]);
        result.set("controlForce", getControlForce());
        result.set("numLinks", static_cast<double>(NUM_LINKS));
        result.set("angles", getAngles());
        result.set("omegas", getAngularVelocities());
        result.set("joints", getJoints());
        // Backward-compatible scalar fields for the first two links.
        result.set("theta1", m_state[idxTheta(0)]);
        result.set("theta2", NUM_LINKS > 1 ? m_state[idxTheta(1)] : 0.0);
        result.set("omega1", m_state[idxOmega(0)]);
        result.set("omega2", NUM_LINKS > 1 ? m_state[idxOmega(1)] : 0.0);
        return result;
    }

    val getVisualizationData() const {
        val result = val::object();
        val cart = val::object();
        cart.set("x", m_state[idxX()]);
        cart.set("y", 0.0);
        result.set("cart", cart);
        result.set("joints", getJoints());
        result.set("angles", getAngles());
        result.set("controlForce", getControlForce());
        result.set("numLinks", static_cast<double>(NUM_LINKS));
        return result;
    }

    //=========================================================================
    // HISTORY
    //=========================================================================

    void setRecordHistory(bool record) { m_record_history = record; }
    size_t getHistorySize() const { return m_history.size(); }
    void clearHistory() { m_history.clear(); }

    val getHistory() const {
        val result = val::object();
        std::vector<double> time, x, control, maxangle;
        for (const auto& h : m_history) {
            time.push_back(h.time);
            x.push_back(h.x);
            control.push_back(h.control_force);
            maxangle.push_back(h.max_angle);
        }
        result.set("time", val::array(time.begin(), time.end()));
        result.set("x", val::array(x.begin(), x.end()));
        result.set("controlForce", val::array(control.begin(), control.end()));
        result.set("maxAngle", val::array(maxangle.begin(), maxangle.end()));
        return result;
    }

    //=========================================================================
    // PARAMETERS
    //=========================================================================

    double getCartMass() const { return m_cart_mass; }
    double getLinkMass(int i) const { return m_link_mass[static_cast<size_t>(i)]; }
    double getLinkLength(int i) const { return m_link_length[static_cast<size_t>(i)]; }
    double getGravity() const { return m_gravity; }
    double getMaxForce() const { return m_max_force; }
    bool isInitialized() const { return m_initialized; }
    bool isControllerEnabled() const { return m_controller_enabled; }

    /** LQR gain row as a JS array (length NSTATE). */
    val getLQRGain() const {
        if (!m_controller) return val::array();
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
        .function("getNumLinks", &InvertedPendulumSimulator::getNumLinks)
        .function("setUniformParameters", &InvertedPendulumSimulator::setUniformParameters)
        .function("setParameters", &InvertedPendulumSimulator::setParameters)
        .function("setInitialAnglesUniform", &InvertedPendulumSimulator::setInitialAnglesUniform)
        .function("setInitialState", &InvertedPendulumSimulator::setInitialState)
        .function("setInitialAngles", &InvertedPendulumSimulator::setInitialAngles)
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
        .function("applyLinkImpulse", &InvertedPendulumSimulator::applyLinkImpulse)

        // State queries
        .function("getTime", &InvertedPendulumSimulator::getTime)
        .function("getCartPosition", &InvertedPendulumSimulator::getCartPosition)
        .function("getCartVelocity", &InvertedPendulumSimulator::getCartVelocity)
        .function("getMaxAngle", &InvertedPendulumSimulator::getMaxAngle)
        .function("getControlForce", &InvertedPendulumSimulator::getControlForce)
        .function("getAngles", &InvertedPendulumSimulator::getAngles)
        .function("getAngularVelocities", &InvertedPendulumSimulator::getAngularVelocities)
        .function("getJoints", &InvertedPendulumSimulator::getJoints)
        .function("getFullState", &InvertedPendulumSimulator::getFullState)
        .function("getVisualizationData", &InvertedPendulumSimulator::getVisualizationData)

        // History
        .function("setRecordHistory", &InvertedPendulumSimulator::setRecordHistory)
        .function("getHistorySize", &InvertedPendulumSimulator::getHistorySize)
        .function("clearHistory", &InvertedPendulumSimulator::clearHistory)
        .function("getHistory", &InvertedPendulumSimulator::getHistory)

        // Parameters
        .function("getCartMass", &InvertedPendulumSimulator::getCartMass)
        .function("getLinkMass", &InvertedPendulumSimulator::getLinkMass)
        .function("getLinkLength", &InvertedPendulumSimulator::getLinkLength)
        .function("getGravity", &InvertedPendulumSimulator::getGravity)
        .function("getMaxForce", &InvertedPendulumSimulator::getMaxForce)
        .function("isInitialized", &InvertedPendulumSimulator::isInitialized)
        .function("isControllerEnabled", &InvertedPendulumSimulator::isControllerEnabled)
        .function("getLQRGain", &InvertedPendulumSimulator::getLQRGain);
}
