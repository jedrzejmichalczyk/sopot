/**
 * Exact replica of WASM RocketSimulator to test locally
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include "../rocket/rocket.hpp"

using namespace sopot;
using namespace sopot::rocket;

class RocketSimulatorTest {
private:
    Rocket<double> m_rocket;
    std::vector<double> m_state;
    double m_time{0.0};
    double m_dt{0.01};
    bool m_initialized{false};
    bool m_demo_mode{false};

    static constexpr double DEMO_THRUST = 1500.0;
    static constexpr double DEMO_BURN_TIME = 3.5;
    static constexpr double DEMO_MASS = 20.0;
    static constexpr double DEMO_MASS_FLOW = 1.5;

    void applyDemoThrust(std::vector<double>& derivs, double t) const {
        if (!m_demo_mode || t > DEMO_BURN_TIME) return;

        double q1 = m_state[7], q2 = m_state[8], q3 = m_state[9], q4 = m_state[10];

        double bx_e = q4*q4 + q1*q1 - q2*q2 - q3*q3;
        double bx_n = 2.0*(q1*q2 + q3*q4);
        double bx_u = 2.0*(q1*q3 - q2*q4);

        double mass = DEMO_MASS - DEMO_MASS_FLOW * t;
        if (mass < 5.0) mass = 5.0;

        double accel = DEMO_THRUST / mass;
        derivs[4] += accel * bx_e;
        derivs[5] += accel * bx_n;
        derivs[6] += accel * bx_u;
    }

    void rk4Step(double dt) {
        auto k1 = m_rocket.computeDerivatives(m_time, m_state);
        applyDemoThrust(k1, m_time);

        std::vector<double> temp_state(m_state.size());
        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + 0.5 * dt * k1[i];
        }
        auto k2 = m_rocket.computeDerivatives(m_time + 0.5 * dt, temp_state);
        applyDemoThrust(k2, m_time + 0.5 * dt);

        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + 0.5 * dt * k2[i];
        }
        auto k3 = m_rocket.computeDerivatives(m_time + 0.5 * dt, temp_state);
        applyDemoThrust(k3, m_time + 0.5 * dt);

        for (size_t i = 0; i < m_state.size(); ++i) {
            temp_state[i] = m_state[i] + dt * k3[i];
        }
        auto k4 = m_rocket.computeDerivatives(m_time + dt, temp_state);
        applyDemoThrust(k4, m_time + dt);

        for (size_t i = 0; i < m_state.size(); ++i) {
            m_state[i] += (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
        }

        m_time += dt;
    }

public:
    void setLauncher(double elevation_deg, double azimuth_deg) {
        m_rocket.setLauncher(elevation_deg, azimuth_deg);
    }

    void setDiameter(double diameter_m) {
        m_rocket.setDiameter(diameter_m);
    }

    void setTimestep(double dt) {
        m_dt = dt;
    }

    void loadDemoData() {
        m_demo_mode = true;
    }

    void setup() {
        m_rocket.setupBeforeSimulation();
        m_state = m_rocket.getInitialState();
        m_time = 0.0;
        m_initialized = true;
    }

    bool step() {
        if (!m_initialized) return false;

        rk4Step(m_dt);

        double altitude = getAltitude();
        return altitude >= 0.0;
    }

    double getTime() const { return m_time; }

    double getAltitude() const {
        if (!m_initialized) return 0.0;
        return m_rocket.getAltitude(m_state);
    }

    void printState() const {
        std::cout << "t=" << std::fixed << std::setprecision(4) << m_time
                  << " pos=[" << m_state[1] << "," << m_state[2] << "," << m_state[3] << "]"
                  << " vel=[" << m_state[4] << "," << m_state[5] << "," << m_state[6] << "]"
                  << " state[3]=" << m_state[3]
                  << " alt_fn=" << getAltitude()
                  << " demo=" << (m_demo_mode ? "ON" : "OFF")
                  << "\n";
    }

    // Debug: check all relevant state indices
    void printDebugIndices() const {
        std::cout << "State indices check:\n";
        for (size_t i = 0; i < m_state.size(); ++i) {
            std::cout << "  state[" << i << "] = " << m_state[i] << "\n";
        }
        std::cout << "getAltitude() = " << getAltitude() << "\n";
    }
};

int main() {
    std::cout << "Exact WASM Replica Test\n";
    std::cout << "========================\n\n";

    RocketSimulatorTest sim;

    // Same as web demo
    sim.setLauncher(85.0, 0.0);
    sim.setDiameter(0.16);
    sim.setTimestep(0.01);
    sim.loadDemoData();
    sim.setup();

    std::cout << "Initial state:\n";
    sim.printState();

    // Run same as JavaScript loop
    for (int frame = 0; frame < 20; ++frame) {
        std::cout << "\nFrame " << frame << ":\n";

        bool shouldContinue = sim.step();

        std::cout << "After step: ";
        sim.printState();
        std::cout << "shouldContinue = " << (shouldContinue ? "true" : "false") << "\n";

        if (!shouldContinue) {
            std::cout << "\n*** SIMULATION ENDED ***\n";
            break;
        }
    }

    return 0;
}
