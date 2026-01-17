/**
 * WebAssembly wrapper for SOPOT 2D Grid simulation
 *
 * This file provides Emscripten embind bindings to expose the C++ 2D grid
 * mass-spring simulation to JavaScript/TypeScript.
 *
 * All physics computations run in C++ - the frontend only visualizes.
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "../physics/connected_masses/connectivity_matrix_2d.hpp"
#include "../core/solver.hpp"
#include <memory>
#include <vector>
#include <cmath>
#include <utility>

using namespace emscripten;
using namespace sopot;
using namespace sopot::connected_masses;

/**
 * Grid2DSimulator: JavaScript-friendly wrapper for 2D mass-spring grid
 *
 * Since the C++ grid uses compile-time template parameters, we support
 * a few common grid sizes. The simulation runs entirely in C++.
 */
class Grid2DSimulator {
private:
    // Grid configuration
    size_t m_rows{5};
    size_t m_cols{5};
    double m_mass{1.0};
    double m_spacing{0.5};
    double m_stiffness{50.0};
    double m_damping{0.5};

    // Simulation state
    std::vector<double> m_state;
    double m_time{0.0};
    double m_dt{0.005};  // Default timestep: 5ms (smaller for stability)
    bool m_initialized{false};

    // We store derivatives function pointer based on grid size
    // Limited to 6x6 to keep WASM compilation time reasonable
    enum class GridSize { G3x3, G4x4, G5x5, G6x6 };
    GridSize m_grid_size{GridSize::G5x5};

    // Template-based systems for different grid sizes
    // Each is created on-demand when initialize() is called

    // RK4 step function - works with any system
    template<typename System>
    void rk4Step(System& system, double dt) {
        size_t n = m_state.size();

        auto k1 = system.computeDerivatives(m_time, m_state);

        std::vector<double> temp(n);
        for (size_t i = 0; i < n; ++i) temp[i] = m_state[i] + 0.5 * dt * k1[i];
        auto k2 = system.computeDerivatives(m_time + 0.5 * dt, temp);

        for (size_t i = 0; i < n; ++i) temp[i] = m_state[i] + 0.5 * dt * k2[i];
        auto k3 = system.computeDerivatives(m_time + 0.5 * dt, temp);

        for (size_t i = 0; i < n; ++i) temp[i] = m_state[i] + dt * k3[i];
        auto k4 = system.computeDerivatives(m_time + dt, temp);

        for (size_t i = 0; i < n; ++i) {
            m_state[i] += (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
        }

        m_time += dt;
    }

    // Physics systems for each grid size (non-static to avoid cross-instance contamination)
    std::unique_ptr<decltype(makeGrid2DSystem<double, 3, 3, false>(1.0, 0.5, 50.0, 0.5))> system_3x3;
    std::unique_ptr<decltype(makeGrid2DSystem<double, 4, 4, false>(1.0, 0.5, 50.0, 0.5))> system_4x4;
    std::unique_ptr<decltype(makeGrid2DSystem<double, 5, 5, false>(1.0, 0.5, 50.0, 0.5))> system_5x5;
    std::unique_ptr<decltype(makeGrid2DSystem<double, 6, 6, false>(1.0, 0.5, 50.0, 0.5))> system_6x6;

    // Step functions for each grid size
    void step3x3() {
        if (!system_3x3) {
            system_3x3 = std::make_unique<decltype(makeGrid2DSystem<double, 3, 3, false>(1.0, 0.5, 50.0, 0.5))>(
                makeGrid2DSystem<double, 3, 3, false>(m_mass, m_spacing, m_stiffness, m_damping));
        }
        rk4Step(*system_3x3, m_dt);
    }

    void step4x4() {
        if (!system_4x4) {
            system_4x4 = std::make_unique<decltype(makeGrid2DSystem<double, 4, 4, false>(1.0, 0.5, 50.0, 0.5))>(
                makeGrid2DSystem<double, 4, 4, false>(m_mass, m_spacing, m_stiffness, m_damping));
        }
        rk4Step(*system_4x4, m_dt);
    }

    void step5x5() {
        if (!system_5x5) {
            system_5x5 = std::make_unique<decltype(makeGrid2DSystem<double, 5, 5, false>(1.0, 0.5, 50.0, 0.5))>(
                makeGrid2DSystem<double, 5, 5, false>(m_mass, m_spacing, m_stiffness, m_damping));
        }
        rk4Step(*system_5x5, m_dt);
    }

    void step6x6() {
        if (!system_6x6) {
            system_6x6 = std::make_unique<decltype(makeGrid2DSystem<double, 6, 6, false>(1.0, 0.5, 50.0, 0.5))>(
                makeGrid2DSystem<double, 6, 6, false>(m_mass, m_spacing, m_stiffness, m_damping));
        }
        rk4Step(*system_6x6, m_dt);
    }

public:
    Grid2DSimulator() = default;

    //=========================================================================
    // CONFIGURATION
    //=========================================================================

    /**
     * Set grid dimensions (must be one of: 3x3, 4x4, 5x5, 6x6)
     */
    void setGridSize(int rows, int cols) {
        if (rows != cols) {
            throw std::runtime_error("Grid must be square (rows == cols)");
        }

        m_rows = rows;
        m_cols = cols;

        switch (rows) {
            case 3:  m_grid_size = GridSize::G3x3; break;
            case 4:  m_grid_size = GridSize::G4x4; break;
            case 5:  m_grid_size = GridSize::G5x5; break;
            case 6:  m_grid_size = GridSize::G6x6; break;
            default:
                throw std::runtime_error("Grid size must be 3, 4, 5, or 6");
        }
    }

    void setMass(double mass) { m_mass = mass; }
    void setSpacing(double spacing) { m_spacing = spacing; }
    void setStiffness(double stiffness) { m_stiffness = stiffness; }
    void setDamping(double damping) { m_damping = damping; }
    void setTimestep(double dt) { m_dt = dt; }

    //=========================================================================
    // INITIALIZATION
    //=========================================================================

    /**
     * Initialize the grid simulation
     */
    void initialize() {
        size_t num_masses = m_rows * m_cols;
        size_t state_dim = num_masses * 4;  // x, y, vx, vy per mass

        m_state.resize(state_dim);

        // Initialize positions on a grid, velocities to zero
        for (size_t r = 0; r < m_rows; ++r) {
            for (size_t c = 0; c < m_cols; ++c) {
                size_t idx = r * m_cols + c;
                m_state[idx * 4 + 0] = c * m_spacing;  // x
                m_state[idx * 4 + 1] = r * m_spacing;  // y
                m_state[idx * 4 + 2] = 0.0;            // vx
                m_state[idx * 4 + 3] = 0.0;            // vy
            }
        }

        m_time = 0.0;
        m_initialized = true;
    }

    /**
     * Reset to initial state
     */
    void reset() {
        initialize();
    }

    /**
     * Perturb a mass at given row/col by displacement
     */
    void perturbMass(int row, int col, double dx, double dy) {
        if (!m_initialized) return;
        if (row < 0 || row >= static_cast<int>(m_rows)) return;
        if (col < 0 || col >= static_cast<int>(m_cols)) return;

        size_t idx = row * m_cols + col;
        m_state[idx * 4 + 0] += dx;
        m_state[idx * 4 + 1] += dy;
    }

    /**
     * Perturb center mass (convenience)
     */
    void perturbCenter(double dx, double dy) {
        int center_row = m_rows / 2;
        int center_col = m_cols / 2;
        perturbMass(center_row, center_col, dx, dy);
    }

    //=========================================================================
    // SIMULATION
    //=========================================================================

    /**
     * Advance simulation by one timestep
     */
    void step() {
        if (!m_initialized) return;

        switch (m_grid_size) {
            case GridSize::G3x3:  step3x3(); break;
            case GridSize::G4x4:  step4x4(); break;
            case GridSize::G5x5:  step5x5(); break;
            case GridSize::G6x6:  step6x6(); break;
        }
    }

    /**
     * Step with custom dt
     */
    void stepWithDt(double dt) {
        double old_dt = m_dt;
        m_dt = dt;
        step();
        m_dt = old_dt;
    }

    //=========================================================================
    // STATE QUERIES
    //=========================================================================

    double getTime() const { return m_time; }
    int getRows() const { return m_rows; }
    int getCols() const { return m_cols; }
    bool isInitialized() const { return m_initialized; }

    /**
     * Get all positions as flat array [x0, y0, x1, y1, ...]
     */
    val getPositions() const {
        if (!m_initialized) return val::array();

        size_t num_masses = m_rows * m_cols;
        std::vector<double> positions(num_masses * 2);

        for (size_t i = 0; i < num_masses; ++i) {
            positions[i * 2 + 0] = m_state[i * 4 + 0];  // x
            positions[i * 2 + 1] = m_state[i * 4 + 1];  // y
        }

        return val::array(positions.begin(), positions.end());
    }

    /**
     * Get all velocities as flat array [vx0, vy0, vx1, vy1, ...]
     */
    val getVelocities() const {
        if (!m_initialized) return val::array();

        size_t num_masses = m_rows * m_cols;
        std::vector<double> velocities(num_masses * 2);

        for (size_t i = 0; i < num_masses; ++i) {
            velocities[i * 2 + 0] = m_state[i * 4 + 2];  // vx
            velocities[i * 2 + 1] = m_state[i * 4 + 3];  // vy
        }

        return val::array(velocities.begin(), velocities.end());
    }

    /**
     * Get full state as JavaScript object
     */
    val getState() const {
        val result = val::object();
        result.set("time", m_time);
        result.set("rows", static_cast<int>(m_rows));
        result.set("cols", static_cast<int>(m_cols));
        result.set("positions", getPositions());
        result.set("velocities", getVelocities());
        return result;
    }

    /**
     * Get position of specific mass
     */
    val getMassPosition(int row, int col) const {
        val result = val::object();
        if (!m_initialized) return result;
        if (row < 0 || row >= static_cast<int>(m_rows)) return result;
        if (col < 0 || col >= static_cast<int>(m_cols)) return result;

        size_t idx = row * m_cols + col;
        result.set("x", m_state[idx * 4 + 0]);
        result.set("y", m_state[idx * 4 + 1]);
        return result;
    }

    /**
     * Compute total kinetic energy
     */
    double getKineticEnergy() const {
        if (!m_initialized) return 0.0;

        double ke = 0.0;
        size_t num_masses = m_rows * m_cols;
        for (size_t i = 0; i < num_masses; ++i) {
            double vx = m_state[i * 4 + 2];
            double vy = m_state[i * 4 + 3];
            ke += 0.5 * m_mass * (vx*vx + vy*vy);
        }
        return ke;
    }

    /**
     * Compute total potential energy (sum over all springs)
     * PE = 0.5 * k * (length - rest_length)^2
     */
    double getPotentialEnergy() const {
        if (!m_initialized) return 0.0;

        double pe = 0.0;
        size_t num_masses = m_rows * m_cols;

        // Horizontal springs
        for (size_t r = 0; r < m_rows; r++) {
            for (size_t c = 0; c < m_cols - 1; c++) {
                size_t idx1 = r * m_cols + c;
                size_t idx2 = r * m_cols + c + 1;

                double x1 = m_state[idx1 * 4 + 0];
                double y1 = m_state[idx1 * 4 + 1];
                double x2 = m_state[idx2 * 4 + 0];
                double y2 = m_state[idx2 * 4 + 1];

                double dx = x2 - x1;
                double dy = y2 - y1;
                double length = std::sqrt(dx * dx + dy * dy);
                double extension = length - m_spacing;

                pe += 0.5 * m_stiffness * extension * extension;
            }
        }

        // Vertical springs
        for (size_t r = 0; r < m_rows - 1; r++) {
            for (size_t c = 0; c < m_cols; c++) {
                size_t idx1 = r * m_cols + c;
                size_t idx2 = (r + 1) * m_cols + c;

                double x1 = m_state[idx1 * 4 + 0];
                double y1 = m_state[idx1 * 4 + 1];
                double x2 = m_state[idx2 * 4 + 0];
                double y2 = m_state[idx2 * 4 + 1];

                double dx = x2 - x1;
                double dy = y2 - y1;
                double length = std::sqrt(dx * dx + dy * dy);
                double extension = length - m_spacing;

                pe += 0.5 * m_stiffness * extension * extension;
            }
        }

        return pe;
    }

    /**
     * Compute total energy (kinetic + potential)
     */
    double getTotalEnergy() const {
        return getKineticEnergy() + getPotentialEnergy();
    }

    /**
     * Compute center of mass position
     */
    val getCenterOfMass() const {
        val result = val::object();
        if (!m_initialized) {
            result.set("x", 0.0);
            result.set("y", 0.0);
            return result;
        }

        double cx = 0.0;
        double cy = 0.0;
        size_t num_masses = m_rows * m_cols;
        double total_mass = m_mass * num_masses;

        for (size_t i = 0; i < num_masses; ++i) {
            cx += m_mass * m_state[i * 4 + 0];
            cy += m_mass * m_state[i * 4 + 1];
        }

        result.set("x", cx / total_mass);
        result.set("y", cy / total_mass);
        return result;
    }
};

//=============================================================================
// EMSCRIPTEN BINDINGS
//=============================================================================

EMSCRIPTEN_BINDINGS(grid2d_module) {
    class_<Grid2DSimulator>("Grid2DSimulator")
        .constructor<>()

        // Configuration
        .function("setGridSize", &Grid2DSimulator::setGridSize)
        .function("setMass", &Grid2DSimulator::setMass)
        .function("setSpacing", &Grid2DSimulator::setSpacing)
        .function("setStiffness", &Grid2DSimulator::setStiffness)
        .function("setDamping", &Grid2DSimulator::setDamping)
        .function("setTimestep", &Grid2DSimulator::setTimestep)

        // Initialization
        .function("initialize", &Grid2DSimulator::initialize)
        .function("reset", &Grid2DSimulator::reset)
        .function("perturbMass", &Grid2DSimulator::perturbMass)
        .function("perturbCenter", &Grid2DSimulator::perturbCenter)

        // Simulation
        .function("step", &Grid2DSimulator::step)
        .function("stepWithDt", &Grid2DSimulator::stepWithDt)

        // State queries
        .function("getTime", &Grid2DSimulator::getTime)
        .function("getRows", &Grid2DSimulator::getRows)
        .function("getCols", &Grid2DSimulator::getCols)
        .function("isInitialized", &Grid2DSimulator::isInitialized)
        .function("getPositions", &Grid2DSimulator::getPositions)
        .function("getVelocities", &Grid2DSimulator::getVelocities)
        .function("getState", &Grid2DSimulator::getState)
        .function("getMassPosition", &Grid2DSimulator::getMassPosition)
        .function("getKineticEnergy", &Grid2DSimulator::getKineticEnergy)
        .function("getPotentialEnergy", &Grid2DSimulator::getPotentialEnergy)
        .function("getTotalEnergy", &Grid2DSimulator::getTotalEnergy)
        .function("getCenterOfMass", &Grid2DSimulator::getCenterOfMass);
}
