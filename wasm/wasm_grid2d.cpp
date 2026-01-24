/**
 * WebAssembly wrapper for SOPOT 2D Grid simulation
 *
 * This file provides Emscripten embind bindings to expose the C++ 2D grid
 * mass-spring simulation to JavaScript/TypeScript.
 *
 * Uses the Unified Graph Architecture for O(K) template instantiations,
 * enabling simulations up to 100x100 grids (10,000 masses) with compile-time
 * validated state function resolution.
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "../physics/unified/validated_system.hpp"
#include "../physics/unified/constexpr_topology.hpp"
#include "../physics/unified/components.hpp"
#include <memory>
#include <vector>
#include <cmath>

using namespace emscripten;
using namespace sopot;
using namespace sopot::unified;

// =============================================================================
// COMPILE-TIME GRID TOPOLOGIES
// =============================================================================
// These are validated at compile time - if state function resolution fails,
// compilation will fail with a static_assert.

// Small grids for quick simulations
constexpr auto topology_5x5 = makeGridTopology<5, 5>();
constexpr auto topology_10x10 = makeGridTopology<10, 10>();
constexpr auto topology_20x20 = makeGridTopology<20, 20>();
constexpr auto topology_50x50 = makeGridTopology<50, 50>();

// Large grid - 10,000 masses, 19,800 springs - validated at COMPILE TIME
constexpr auto topology_100x100 = makeGridTopology<100, 100>();

// =============================================================================
// TYPE ALIASES
// =============================================================================

using MassType = PointMass2D<double>;
using SpringType = Spring2D<double>;

template<auto& Topology>
using GridSystem = ValidatedSystem<double, Topology, MassType, SpringType>;

// =============================================================================
// GRID SIMULATOR
// =============================================================================

/**
 * Grid2DSimulator: JavaScript-friendly wrapper for 2D mass-spring grid
 *
 * Now supports grids up to 100x100 using the Unified Graph Architecture.
 * Topology validation happens at COMPILE TIME.
 */
class Grid2DSimulator {
private:
    // Grid configuration
    size_t m_rows{10};
    size_t m_cols{10};
    double m_mass{1.0};
    double m_spacing{0.5};
    double m_stiffness{100.0};
    double m_damping{1.0};

    // Simulation state
    std::vector<double> m_state;
    double m_time{0.0};
    double m_dt{0.001};  // Smaller timestep for larger grids
    bool m_initialized{false};

    // Supported grid sizes
    enum class GridSize { G5, G10, G20, G50, G100 };
    GridSize m_grid_size{GridSize::G10};

    // Systems for each supported size (created on demand)
    std::unique_ptr<GridSystem<topology_5x5>> m_system_5;
    std::unique_ptr<GridSystem<topology_10x10>> m_system_10;
    std::unique_ptr<GridSystem<topology_20x20>> m_system_20;
    std::unique_ptr<GridSystem<topology_50x50>> m_system_50;
    std::unique_ptr<GridSystem<topology_100x100>> m_system_100;

    // RK4 integrator
    template<typename System>
    void rk4Step(System& system, double dt) {
        const size_t n = m_state.size();

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

    // Create and populate a grid system
    template<size_t Rows, size_t Cols, auto& Topology>
    auto createSystem() {
        auto system = std::make_unique<GridSystem<Topology>>();

        constexpr size_t num_masses = Rows * Cols;
        constexpr size_t num_h_springs = Rows * (Cols - 1);
        constexpr size_t num_v_springs = (Rows - 1) * Cols;

        // Add masses with initial positions
        auto& mass_batch = system->template getBatch<0>();
        for (size_t r = 0; r < Rows; ++r) {
            for (size_t c = 0; c < Cols; ++c) {
                double x = c * m_spacing;
                double y = r * m_spacing;
                mass_batch.add(MassType(m_mass, {x, y}, {0.0, 0.0}));
            }
        }

        // Add springs
        auto& spring_batch = system->template getBatch<1>();

        // Horizontal springs
        for (size_t r = 0; r < Rows; ++r) {
            for (size_t c = 0; c < Cols - 1; ++c) {
                spring_batch.add(SpringType(m_stiffness, m_spacing, m_damping));
            }
        }

        // Vertical springs
        for (size_t r = 0; r < Rows - 1; ++r) {
            for (size_t c = 0; c < Cols; ++c) {
                spring_batch.add(SpringType(m_stiffness, m_spacing, m_damping));
            }
        }

        return system;
    }

    void createSystemForSize() {
        switch (m_grid_size) {
            case GridSize::G5:
                m_system_5 = createSystem<5, 5, topology_5x5>();
                m_state = m_system_5->getInitialState();
                break;
            case GridSize::G10:
                m_system_10 = createSystem<10, 10, topology_10x10>();
                m_state = m_system_10->getInitialState();
                break;
            case GridSize::G20:
                m_system_20 = createSystem<20, 20, topology_20x20>();
                m_state = m_system_20->getInitialState();
                break;
            case GridSize::G50:
                m_system_50 = createSystem<50, 50, topology_50x50>();
                m_state = m_system_50->getInitialState();
                break;
            case GridSize::G100:
                m_system_100 = createSystem<100, 100, topology_100x100>();
                m_state = m_system_100->getInitialState();
                break;
        }
    }

    void stepSystem() {
        switch (m_grid_size) {
            case GridSize::G5:   rk4Step(*m_system_5, m_dt); break;
            case GridSize::G10:  rk4Step(*m_system_10, m_dt); break;
            case GridSize::G20:  rk4Step(*m_system_20, m_dt); break;
            case GridSize::G50:  rk4Step(*m_system_50, m_dt); break;
            case GridSize::G100: rk4Step(*m_system_100, m_dt); break;
        }
    }

public:
    Grid2DSimulator() = default;

    //=========================================================================
    // CONFIGURATION
    //=========================================================================

    /**
     * Set grid dimensions (supported: 5, 10, 20, 50, 100)
     */
    void setGridSize(int rows, int cols) {
        if (rows != cols) {
            throw std::runtime_error("Grid must be square (rows == cols)");
        }

        m_rows = rows;
        m_cols = cols;

        switch (rows) {
            case 5:   m_grid_size = GridSize::G5; break;
            case 10:  m_grid_size = GridSize::G10; break;
            case 20:  m_grid_size = GridSize::G20; break;
            case 50:  m_grid_size = GridSize::G50; break;
            case 100: m_grid_size = GridSize::G100; break;
            default:
                throw std::runtime_error("Grid size must be 5, 10, 20, 50, or 100");
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

    void initialize() {
        // Clear old systems
        m_system_5.reset();
        m_system_10.reset();
        m_system_20.reset();
        m_system_50.reset();
        m_system_100.reset();

        // Create the system for current size
        createSystemForSize();

        m_time = 0.0;
        m_initialized = true;
    }

    void reset() {
        initialize();
    }

    void perturbMass(int row, int col, double dx, double dy) {
        if (!m_initialized) return;
        if (row < 0 || row >= static_cast<int>(m_rows)) return;
        if (col < 0 || col >= static_cast<int>(m_cols)) return;

        size_t idx = row * m_cols + col;
        // State layout: [x, y, vx, vy] per mass
        m_state[idx * 4 + 0] += dx;
        m_state[idx * 4 + 1] += dy;
    }

    void perturbCenter(double dx, double dy) {
        int center_row = m_rows / 2;
        int center_col = m_cols / 2;
        perturbMass(center_row, center_col, dx, dy);
    }

    //=========================================================================
    // SIMULATION
    //=========================================================================

    void step() {
        if (!m_initialized) return;
        stepSystem();
    }

    void stepWithDt(double dt) {
        double old_dt = m_dt;
        m_dt = dt;
        step();
        m_dt = old_dt;
    }

    /**
     * Run multiple steps at once (more efficient for large grids)
     */
    void stepMultiple(int count) {
        if (!m_initialized) return;
        for (int i = 0; i < count; ++i) {
            stepSystem();
        }
    }

    //=========================================================================
    // STATE QUERIES
    //=========================================================================

    double getTime() const { return m_time; }
    int getRows() const { return m_rows; }
    int getCols() const { return m_cols; }
    bool isInitialized() const { return m_initialized; }

    val getPositions() const {
        if (!m_initialized) return val::array();

        size_t num_masses = m_rows * m_cols;
        std::vector<double> positions(num_masses * 2);

        for (size_t i = 0; i < num_masses; ++i) {
            positions[i * 2 + 0] = m_state[i * 4 + 0];
            positions[i * 2 + 1] = m_state[i * 4 + 1];
        }

        return val::array(positions.begin(), positions.end());
    }

    val getVelocities() const {
        if (!m_initialized) return val::array();

        size_t num_masses = m_rows * m_cols;
        std::vector<double> velocities(num_masses * 2);

        for (size_t i = 0; i < num_masses; ++i) {
            velocities[i * 2 + 0] = m_state[i * 4 + 2];
            velocities[i * 2 + 1] = m_state[i * 4 + 3];
        }

        return val::array(velocities.begin(), velocities.end());
    }

    val getState() const {
        val result = val::object();
        result.set("time", m_time);
        result.set("rows", static_cast<int>(m_rows));
        result.set("cols", static_cast<int>(m_cols));
        result.set("positions", getPositions());
        result.set("velocities", getVelocities());
        return result;
    }

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

    val getCenterOfMass() const {
        val result = val::object();
        if (!m_initialized) {
            result.set("x", 0.0);
            result.set("y", 0.0);
            return result;
        }

        double total_mass = m_mass * m_rows * m_cols;
        double com_x = 0.0;
        double com_y = 0.0;

        size_t num_masses = m_rows * m_cols;
        for (size_t i = 0; i < num_masses; ++i) {
            com_x += m_mass * m_state[i * 4 + 0];
            com_y += m_mass * m_state[i * 4 + 1];
        }

        result.set("x", com_x / total_mass);
        result.set("y", com_y / total_mass);
        return result;
    }

    val getTotalMomentum() const {
        val result = val::object();
        if (!m_initialized) {
            result.set("px", 0.0);
            result.set("py", 0.0);
            return result;
        }

        double px = 0.0;
        double py = 0.0;

        size_t num_masses = m_rows * m_cols;
        for (size_t i = 0; i < num_masses; ++i) {
            px += m_mass * m_state[i * 4 + 2];
            py += m_mass * m_state[i * 4 + 3];
        }

        result.set("px", px);
        result.set("py", py);
        return result;
    }

    /**
     * Get system info for debugging
     */
    val getSystemInfo() const {
        val result = val::object();
        result.set("rows", static_cast<int>(m_rows));
        result.set("cols", static_cast<int>(m_cols));
        result.set("numMasses", static_cast<int>(m_rows * m_cols));

        size_t h_springs = m_rows * (m_cols - 1);
        size_t v_springs = (m_rows - 1) * m_cols;
        result.set("numSprings", static_cast<int>(h_springs + v_springs));
        result.set("stateSize", static_cast<int>(m_state.size()));
        result.set("architecture", std::string("Unified Graph (O(K) templates)"));
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
        .function("stepMultiple", &Grid2DSimulator::stepMultiple)

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
        .function("getCenterOfMass", &Grid2DSimulator::getCenterOfMass)
        .function("getTotalMomentum", &Grid2DSimulator::getTotalMomentum)
        .function("getSystemInfo", &Grid2DSimulator::getSystemInfo);
}
