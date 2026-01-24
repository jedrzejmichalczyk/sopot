/**
 * WebAssembly wrapper for SOPOT 2D Grid simulation
 *
 * This file provides Emscripten embind bindings to expose the C++ 2D grid
 * mass-spring simulation to JavaScript/TypeScript.
 *
 * Uses the Unified Graph Architecture for O(K) template instantiations,
 * enabling simulations up to 100x100 grids (10,000 masses) with compile-time
 * validated state function resolution.
 *
 * State per mass: [x, y, vx, vy, θ, ω] (6 doubles)
 * - x, y: position
 * - vx, vy: velocity
 * - θ: angle (orientation)
 * - ω: angular velocity
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "../physics/unified/validated_system.hpp"
#include "../physics/unified/constexpr_topology.hpp"
#include "../physics/unified/components.hpp"
#include <memory>
#include <vector>
#include <cmath>

// WebAssembly SIMD intrinsics for hardware-accelerated vector math
// Processes 2 doubles (128-bit) or 4 floats at once
#ifdef __wasm_simd128__
#include <wasm_simd128.h>
#define SOPOT_USE_SIMD 1

// =============================================================================
// SIMD HELPER FUNCTIONS
// =============================================================================
// State layout per mass: [x, y, vx, vy, θ, ω] (6 doubles = 48 bytes)
// SIMD register: 128 bits = 2 doubles at a time
// We process (x,y), (vx,vy), and (θ,ω) as separate vector pairs

namespace simd {

// Update positions and angles: pos += dt * vel, θ += dt * ω (for all masses)
// Processes 2 components at a time using SIMD
inline void updatePositions(double* state, size_t num_masses, double dt) {
    const v128_t dt_vec = wasm_f64x2_splat(dt);

    for (size_t i = 0; i < num_masses; ++i) {
        double* base = state + i * 6;

        // Load position (x, y) and velocity (vx, vy)
        v128_t pos = wasm_v128_load(base);      // [x, y]
        v128_t vel = wasm_v128_load(base + 2);  // [vx, vy]

        // pos += dt * vel
        v128_t delta = wasm_f64x2_mul(dt_vec, vel);
        pos = wasm_f64x2_add(pos, delta);

        // Store updated position
        wasm_v128_store(base, pos);

        // Update angle: θ += dt * ω
        base[4] += dt * base[5];
    }
}

// Update velocities: vel += dt * accel, ω += dt * α (for all masses)
// derivs layout matches state: [dx, dy, dvx, dvy, dθ, dω] per mass
inline void updateVelocities(double* state, const double* derivs, size_t num_masses, double dt) {
    const v128_t dt_vec = wasm_f64x2_splat(dt);

    for (size_t i = 0; i < num_masses; ++i) {
        double* vel_ptr = state + i * 6 + 2;
        const double* accel_ptr = derivs + i * 6 + 2;

        // Load velocity and acceleration
        v128_t vel = wasm_v128_load(vel_ptr);     // [vx, vy]
        v128_t accel = wasm_v128_load(accel_ptr); // [ax, ay]

        // vel += dt * accel
        v128_t delta = wasm_f64x2_mul(dt_vec, accel);
        vel = wasm_f64x2_add(vel, delta);

        // Store updated velocity
        wasm_v128_store(vel_ptr, vel);

        // Update angular velocity: ω += dt * α
        double* omega_ptr = state + i * 6 + 5;
        const double* alpha_ptr = derivs + i * 6 + 5;
        *omega_ptr += dt * (*alpha_ptr);
    }
}

// Velocity Verlet position update: pos += dt * vel + 0.5 * dt^2 * accel
inline void updatePositionsVerlet(double* state, const double* derivs, size_t num_masses, double dt) {
    const v128_t dt_vec = wasm_f64x2_splat(dt);
    const v128_t half_dt_sq_vec = wasm_f64x2_splat(0.5 * dt * dt);
    const double half_dt_sq = 0.5 * dt * dt;

    for (size_t i = 0; i < num_masses; ++i) {
        double* base = state + i * 6;
        const double* deriv_base = derivs + i * 6;

        // Load position, velocity, and acceleration
        v128_t pos = wasm_v128_load(base);          // [x, y]
        v128_t vel = wasm_v128_load(base + 2);      // [vx, vy]
        v128_t accel = wasm_v128_load(deriv_base + 2); // [ax, ay]

        // pos += dt * vel + 0.5 * dt^2 * accel
        v128_t vel_term = wasm_f64x2_mul(dt_vec, vel);
        v128_t accel_term = wasm_f64x2_mul(half_dt_sq_vec, accel);
        pos = wasm_f64x2_add(pos, vel_term);
        pos = wasm_f64x2_add(pos, accel_term);

        // Store updated position
        wasm_v128_store(base, pos);

        // Update angle: θ += dt * ω + 0.5 * dt^2 * α
        base[4] += dt * base[5] + half_dt_sq * deriv_base[5];
    }
}

// Velocity Verlet velocity update: vel += 0.5 * dt * (accel_old + accel_new)
inline void updateVelocitiesVerlet(double* state, const double* derivs_old,
                                    const double* derivs_new, size_t num_masses, double dt) {
    const v128_t half_dt_vec = wasm_f64x2_splat(0.5 * dt);
    const double half_dt = 0.5 * dt;

    for (size_t i = 0; i < num_masses; ++i) {
        double* vel_ptr = state + i * 6 + 2;
        const double* accel_old_ptr = derivs_old + i * 6 + 2;
        const double* accel_new_ptr = derivs_new + i * 6 + 2;

        // Load velocity and accelerations
        v128_t vel = wasm_v128_load(vel_ptr);
        v128_t accel_old = wasm_v128_load(accel_old_ptr);
        v128_t accel_new = wasm_v128_load(accel_new_ptr);

        // vel += 0.5 * dt * (accel_old + accel_new)
        v128_t accel_sum = wasm_f64x2_add(accel_old, accel_new);
        v128_t delta = wasm_f64x2_mul(half_dt_vec, accel_sum);
        vel = wasm_f64x2_add(vel, delta);

        // Store updated velocity
        wasm_v128_store(vel_ptr, vel);

        // Update angular velocity: ω += 0.5 * dt * (α_old + α_new)
        double* omega_ptr = state + i * 6 + 5;
        const double* alpha_old_ptr = derivs_old + i * 6 + 5;
        const double* alpha_new_ptr = derivs_new + i * 6 + 5;
        *omega_ptr += half_dt * (*alpha_old_ptr + *alpha_new_ptr);
    }
}

// RK4 state combination: state += (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
inline void rk4Combine(double* state, const double* k1, const double* k2,
                       const double* k3, const double* k4, size_t state_size, double dt) {
    const v128_t dt_over_6 = wasm_f64x2_splat(dt / 6.0);
    const v128_t two = wasm_f64x2_splat(2.0);

    // Process 2 doubles at a time
    size_t i = 0;
    for (; i + 1 < state_size; i += 2) {
        v128_t s = wasm_v128_load(state + i);
        v128_t v1 = wasm_v128_load(k1 + i);
        v128_t v2 = wasm_v128_load(k2 + i);
        v128_t v3 = wasm_v128_load(k3 + i);
        v128_t v4 = wasm_v128_load(k4 + i);

        // k1 + 2*k2 + 2*k3 + k4
        v128_t sum = v1;
        sum = wasm_f64x2_add(sum, wasm_f64x2_mul(two, v2));
        sum = wasm_f64x2_add(sum, wasm_f64x2_mul(two, v3));
        sum = wasm_f64x2_add(sum, v4);

        // state += (dt/6) * sum
        s = wasm_f64x2_add(s, wasm_f64x2_mul(dt_over_6, sum));
        wasm_v128_store(state + i, s);
    }

    // Handle remaining element if state_size is odd
    if (i < state_size) {
        state[i] += (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    }
}

} // namespace simd

#endif // __wasm_simd128__

using namespace emscripten;
using namespace sopot;
using namespace sopot::unified;

// =============================================================================
// COMPILE-TIME GRID TOPOLOGIES
// =============================================================================
// These are validated at compile time - if state function resolution fails,
// compilation will fail with a static_assert.

// Quad topology (horizontal + vertical springs only)
constexpr auto quad_5x5 = makeGridTopology<5, 5>();
constexpr auto quad_10x10 = makeGridTopology<10, 10>();
constexpr auto quad_20x20 = makeGridTopology<20, 20>();
constexpr auto quad_50x50 = makeGridTopology<50, 50>();
constexpr auto quad_100x100 = makeGridTopology<100, 100>();

// Triangle topology (horizontal + vertical + diagonal springs)
// More stable, better approximates cloth-like behavior
constexpr auto triangle_5x5 = makeTriangleGridTopology<5, 5>();
constexpr auto triangle_10x10 = makeTriangleGridTopology<10, 10>();
constexpr auto triangle_20x20 = makeTriangleGridTopology<20, 20>();
constexpr auto triangle_50x50 = makeTriangleGridTopology<50, 50>();
constexpr auto triangle_100x100 = makeTriangleGridTopology<100, 100>();

// =============================================================================
// TYPE ALIASES
// =============================================================================

using MassType = PointMass2D<double>;
using SpringType = Spring2D<double>;

template<auto& Topology>
using GridSystem = ValidatedSystem<double, Topology, MassType, SpringType>;

// Grid type enumeration (exposed to JavaScript)
enum class GridType { Quad, Triangle };

// Integrator type enumeration (exposed to JavaScript)
// - RK4: 4th-order Runge-Kutta, highest accuracy, 4 derivative evals/step
// - SymplecticEuler: 1st-order symplectic (velocity-first), bounded energy error
// - VelocityVerlet: 2nd-order symplectic, excellent energy conservation
enum class Integrator { RK4, SymplecticEuler, VelocityVerlet };

// =============================================================================
// GRID SIMULATOR
// =============================================================================

/**
 * Grid2DSimulator: JavaScript-friendly wrapper for 2D mass-spring grid
 *
 * Now supports grids up to 100x100 using the Unified Graph Architecture.
 * Topology validation happens at COMPILE TIME.
 *
 * State per mass: [x, y, vx, vy, θ, ω] (6 doubles)
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
    double m_min_distance{0.0};          // Repulsion collision radius (0 = disabled)
    double m_repulsion_stiffness{-1.0};  // Repulsion strength (-1 = auto: 10x stiffness)
    double m_radius{0.05};               // Mass radius for rotational dynamics
    GridType m_grid_type{GridType::Quad};
    Integrator m_integrator{Integrator::RK4};

    // Simulation state
    std::vector<double> m_state;
    double m_time{0.0};
    double m_dt{0.001};  // Smaller timestep for larger grids
    bool m_initialized{false};

    // Supported grid sizes
    enum class GridSize { G5, G10, G20, G50, G100 };
    GridSize m_grid_size{GridSize::G10};

    // Quad topology systems (created on demand)
    std::unique_ptr<GridSystem<quad_5x5>> m_quad_5;
    std::unique_ptr<GridSystem<quad_10x10>> m_quad_10;
    std::unique_ptr<GridSystem<quad_20x20>> m_quad_20;
    std::unique_ptr<GridSystem<quad_50x50>> m_quad_50;
    std::unique_ptr<GridSystem<quad_100x100>> m_quad_100;

    // Triangle topology systems (created on demand)
    std::unique_ptr<GridSystem<triangle_5x5>> m_triangle_5;
    std::unique_ptr<GridSystem<triangle_10x10>> m_triangle_10;
    std::unique_ptr<GridSystem<triangle_20x20>> m_triangle_20;
    std::unique_ptr<GridSystem<triangle_50x50>> m_triangle_50;
    std::unique_ptr<GridSystem<triangle_100x100>> m_triangle_100;

    // RK4 integrator (with optional SIMD optimization)
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

        // Final combination: state += (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
#ifdef SOPOT_USE_SIMD
        simd::rk4Combine(m_state.data(), k1.data(), k2.data(), k3.data(), k4.data(), n, dt);
#else
        for (size_t i = 0; i < n; ++i) {
            m_state[i] += (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
        }
#endif

        m_time += dt;
    }

    /**
     * Symplectic Euler integrator (velocity-first, a.k.a. Euler-Cromer)
     *
     * Updates velocity first using current accelerations, then updates
     * position using the NEW velocity. This is the correct symplectic Euler
     * that preserves the symplectic structure of Hamiltonian systems.
     *
     * Order: 1st order
     * Properties: Symplectic (preserves phase space volume), bounded energy error
     * Best for: Long simulations of undamped or lightly damped oscillatory systems
     *
     * Algorithm:
     *   v_{n+1} = v_n + dt * a(x_n, v_n)
     *   x_{n+1} = x_n + dt * v_{n+1}
     */
    template<typename System>
    void symplecticEulerStep(System& system, double dt) {
        const size_t num_masses = m_rows * m_cols;

        // Get accelerations at current state
        auto derivs = system.computeDerivatives(m_time, m_state);

#ifdef SOPOT_USE_SIMD
        // SIMD-optimized velocity and position updates
        simd::updateVelocities(m_state.data(), derivs.data(), num_masses, dt);
        simd::updatePositions(m_state.data(), num_masses, dt);
#else
        // Update velocities and angular velocities first: v_{n+1} = v_n + dt * a(x_n, v_n)
        for (size_t i = 0; i < num_masses; ++i) {
            m_state[i * 6 + 2] += dt * derivs[i * 6 + 2];  // vx += dt * ax
            m_state[i * 6 + 3] += dt * derivs[i * 6 + 3];  // vy += dt * ay
            m_state[i * 6 + 5] += dt * derivs[i * 6 + 5];  // ω += dt * α
        }

        // Update positions and angles using NEW velocities: x_{n+1} = x_n + dt * v_{n+1}
        for (size_t i = 0; i < num_masses; ++i) {
            m_state[i * 6 + 0] += dt * m_state[i * 6 + 2];  // x += dt * vx_new
            m_state[i * 6 + 1] += dt * m_state[i * 6 + 3];  // y += dt * vy_new
            m_state[i * 6 + 4] += dt * m_state[i * 6 + 5];  // θ += dt * ω_new
        }
#endif

        m_time += dt;
    }

    /**
     * Velocity Verlet integrator (Störmer-Verlet)
     *
     * A 2nd-order symplectic integrator that provides better accuracy than
     * symplectic Euler while still preserving the symplectic structure.
     * Commonly used in molecular dynamics simulations.
     *
     * Order: 2nd order
     * Properties: Symplectic, time-reversible, excellent energy conservation
     * Best for: High-precision simulations where energy conservation matters
     *
     * Algorithm:
     *   x_{n+1} = x_n + dt * v_n + 0.5 * dt^2 * a_n
     *   a_{n+1} = a(x_{n+1})
     *   v_{n+1} = v_n + 0.5 * dt * (a_n + a_{n+1})
     */
    template<typename System>
    void velocityVerletStep(System& system, double dt) {
        const size_t num_masses = m_rows * m_cols;

        // Get accelerations at current state: a_n
        auto derivs = system.computeDerivatives(m_time, m_state);

#ifdef SOPOT_USE_SIMD
        // SIMD-optimized Verlet position update
        simd::updatePositionsVerlet(m_state.data(), derivs.data(), num_masses, dt);

        // Get accelerations at new positions: a_{n+1}
        auto derivs_new = system.computeDerivatives(m_time + dt, m_state);

        // SIMD-optimized Verlet velocity update
        simd::updateVelocitiesVerlet(m_state.data(), derivs.data(), derivs_new.data(), num_masses, dt);
#else
        // Update positions and angles: x_{n+1} = x_n + dt * v_n + 0.5 * dt^2 * a_n
        const double half_dt_sq = 0.5 * dt * dt;
        for (size_t i = 0; i < num_masses; ++i) {
            m_state[i * 6 + 0] += dt * m_state[i * 6 + 2] + half_dt_sq * derivs[i * 6 + 2];
            m_state[i * 6 + 1] += dt * m_state[i * 6 + 3] + half_dt_sq * derivs[i * 6 + 3];
            m_state[i * 6 + 4] += dt * m_state[i * 6 + 5] + half_dt_sq * derivs[i * 6 + 5];
        }

        // Get accelerations at new positions: a_{n+1}
        auto derivs_new = system.computeDerivatives(m_time + dt, m_state);

        // Update velocities and angular velocities: v_{n+1} = v_n + 0.5 * dt * (a_n + a_{n+1})
        const double half_dt = 0.5 * dt;
        for (size_t i = 0; i < num_masses; ++i) {
            m_state[i * 6 + 2] += half_dt * (derivs[i * 6 + 2] + derivs_new[i * 6 + 2]);
            m_state[i * 6 + 3] += half_dt * (derivs[i * 6 + 3] + derivs_new[i * 6 + 3]);
            m_state[i * 6 + 5] += half_dt * (derivs[i * 6 + 5] + derivs_new[i * 6 + 5]);
        }
#endif

        m_time += dt;
    }

    // Create and populate a quad grid system (horizontal + vertical springs only)
    template<size_t Rows, size_t Cols, auto& Topology>
    auto createQuadSystem() {
        auto system = std::make_unique<GridSystem<Topology>>();

        static_assert(Rows * Cols > 0, "Grid must have at least one mass");

        // Constants for attachment angles
        constexpr double PI = 3.14159265358979323846;

        // Add masses with initial positions and radius
        auto& mass_batch = system->template getBatch<0>();
        for (size_t r = 0; r < Rows; ++r) {
            for (size_t c = 0; c < Cols; ++c) {
                double x = c * m_spacing;
                double y = r * m_spacing;
                // MassType(mass, pos, vel, radius, initial_angle, initial_angular_vel)
                mass_batch.add(MassType(m_mass, {x, y}, {0.0, 0.0}, m_radius, 0.0, 0.0));
            }
        }

        // Add springs with proper attachment angles
        auto& spring_batch = system->template getBatch<1>();

        // Horizontal springs: attach at angle 0 (right) on left mass, angle π (left) on right mass
        for (size_t r = 0; r < Rows; ++r) {
            for (size_t c = 0; c < Cols - 1; ++c) {
                // Spring connects mass at (r,c) to mass at (r,c+1)
                // On left mass: attach on right side (angle 0)
                // On right mass: attach on left side (angle π)
                spring_batch.add(SpringType(m_stiffness, m_spacing, m_damping,
                                           m_min_distance, m_repulsion_stiffness,
                                           0.0, PI));  // attach_angle_0=0, attach_angle_1=π
            }
        }

        // Vertical springs: attach at angle π/2 (top) on bottom mass, angle -π/2 (bottom) on top mass
        for (size_t r = 0; r < Rows - 1; ++r) {
            for (size_t c = 0; c < Cols; ++c) {
                // Spring connects mass at (r,c) to mass at (r+1,c)
                // On bottom mass: attach on top side (angle π/2)
                // On top mass: attach on bottom side (angle -π/2)
                spring_batch.add(SpringType(m_stiffness, m_spacing, m_damping,
                                           m_min_distance, m_repulsion_stiffness,
                                           PI / 2.0, -PI / 2.0));  // attach_angle_0=π/2, attach_angle_1=-π/2
            }
        }

        return system;
    }

    // Create and populate a triangle grid system (horizontal + vertical + diagonal springs)
    template<size_t Rows, size_t Cols, auto& Topology>
    auto createTriangleSystem() {
        auto system = std::make_unique<GridSystem<Topology>>();

        static_assert(Rows * Cols > 0, "Grid must have at least one mass");

        // Constants for attachment angles
        constexpr double PI = 3.14159265358979323846;

        // Add masses with initial positions and radius
        auto& mass_batch = system->template getBatch<0>();
        for (size_t r = 0; r < Rows; ++r) {
            for (size_t c = 0; c < Cols; ++c) {
                double x = c * m_spacing;
                double y = r * m_spacing;
                // MassType(mass, pos, vel, radius, initial_angle, initial_angular_vel)
                mass_batch.add(MassType(m_mass, {x, y}, {0.0, 0.0}, m_radius, 0.0, 0.0));
            }
        }

        // Add springs with proper attachment angles
        auto& spring_batch = system->template getBatch<1>();

        // Horizontal springs: attach at angle 0 (right) on left mass, angle π (left) on right mass
        for (size_t r = 0; r < Rows; ++r) {
            for (size_t c = 0; c < Cols - 1; ++c) {
                spring_batch.add(SpringType(m_stiffness, m_spacing, m_damping,
                                           m_min_distance, m_repulsion_stiffness,
                                           0.0, PI));
            }
        }

        // Vertical springs: attach at angle π/2 (top) on bottom mass, angle -π/2 (bottom) on top mass
        for (size_t r = 0; r < Rows - 1; ++r) {
            for (size_t c = 0; c < Cols; ++c) {
                spring_batch.add(SpringType(m_stiffness, m_spacing, m_damping,
                                           m_min_distance, m_repulsion_stiffness,
                                           PI / 2.0, -PI / 2.0));
            }
        }

        // Diagonal springs (rest length = sqrt(2) * spacing)
        double diagonal_length = std::sqrt(2.0) * m_spacing;
        for (size_t r = 0; r < Rows - 1; ++r) {
            for (size_t c = 0; c < Cols - 1; ++c) {
                // Main diagonal: connects (r,c) to (r+1,c+1) - bottom-left to top-right
                // Attach at π/4 (45°) on (r,c), and -3π/4 (-135°) on (r+1,c+1)
                spring_batch.add(SpringType(m_stiffness, diagonal_length, m_damping,
                                           m_min_distance, m_repulsion_stiffness,
                                           PI / 4.0, -3.0 * PI / 4.0));

                // Anti-diagonal: connects (r,c+1) to (r+1,c) - bottom-right to top-left
                // Attach at 3π/4 (135°) on (r,c+1), and -π/4 (-45°) on (r+1,c)
                spring_batch.add(SpringType(m_stiffness, diagonal_length, m_damping,
                                           m_min_distance, m_repulsion_stiffness,
                                           3.0 * PI / 4.0, -PI / 4.0));
            }
        }

        return system;
    }

    void createSystemForSize() {
        if (m_grid_type == GridType::Quad) {
            switch (m_grid_size) {
                case GridSize::G5:
                    m_quad_5 = createQuadSystem<5, 5, quad_5x5>();
                    m_state = m_quad_5->getInitialState();
                    break;
                case GridSize::G10:
                    m_quad_10 = createQuadSystem<10, 10, quad_10x10>();
                    m_state = m_quad_10->getInitialState();
                    break;
                case GridSize::G20:
                    m_quad_20 = createQuadSystem<20, 20, quad_20x20>();
                    m_state = m_quad_20->getInitialState();
                    break;
                case GridSize::G50:
                    m_quad_50 = createQuadSystem<50, 50, quad_50x50>();
                    m_state = m_quad_50->getInitialState();
                    break;
                case GridSize::G100:
                    m_quad_100 = createQuadSystem<100, 100, quad_100x100>();
                    m_state = m_quad_100->getInitialState();
                    break;
            }
        } else {
            switch (m_grid_size) {
                case GridSize::G5:
                    m_triangle_5 = createTriangleSystem<5, 5, triangle_5x5>();
                    m_state = m_triangle_5->getInitialState();
                    break;
                case GridSize::G10:
                    m_triangle_10 = createTriangleSystem<10, 10, triangle_10x10>();
                    m_state = m_triangle_10->getInitialState();
                    break;
                case GridSize::G20:
                    m_triangle_20 = createTriangleSystem<20, 20, triangle_20x20>();
                    m_state = m_triangle_20->getInitialState();
                    break;
                case GridSize::G50:
                    m_triangle_50 = createTriangleSystem<50, 50, triangle_50x50>();
                    m_state = m_triangle_50->getInitialState();
                    break;
                case GridSize::G100:
                    m_triangle_100 = createTriangleSystem<100, 100, triangle_100x100>();
                    m_state = m_triangle_100->getInitialState();
                    break;
            }
        }
    }

    // Helper to step a specific system with the selected integrator
    template<typename System>
    void stepWithIntegrator(System& system, double dt) {
        switch (m_integrator) {
            case Integrator::RK4:
                rk4Step(system, dt);
                break;
            case Integrator::SymplecticEuler:
                symplecticEulerStep(system, dt);
                break;
            case Integrator::VelocityVerlet:
                velocityVerletStep(system, dt);
                break;
        }
    }

    void stepSystem() {
        if (m_grid_type == GridType::Quad) {
            switch (m_grid_size) {
                case GridSize::G5:   stepWithIntegrator(*m_quad_5, m_dt); break;
                case GridSize::G10:  stepWithIntegrator(*m_quad_10, m_dt); break;
                case GridSize::G20:  stepWithIntegrator(*m_quad_20, m_dt); break;
                case GridSize::G50:  stepWithIntegrator(*m_quad_50, m_dt); break;
                case GridSize::G100: stepWithIntegrator(*m_quad_100, m_dt); break;
            }
        } else {
            switch (m_grid_size) {
                case GridSize::G5:   stepWithIntegrator(*m_triangle_5, m_dt); break;
                case GridSize::G10:  stepWithIntegrator(*m_triangle_10, m_dt); break;
                case GridSize::G20:  stepWithIntegrator(*m_triangle_20, m_dt); break;
                case GridSize::G50:  stepWithIntegrator(*m_triangle_50, m_dt); break;
                case GridSize::G100: stepWithIntegrator(*m_triangle_100, m_dt); break;
            }
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
    void setSpacing(double spacing) {
        m_spacing = spacing;
        // Auto-compute radius as 10% of spacing (reasonable default for visualization)
        m_radius = spacing * 0.1;
    }
    void setStiffness(double stiffness) { m_stiffness = stiffness; }
    void setDamping(double damping) { m_damping = damping; }
    void setTimestep(double dt) { m_dt = dt; }
    void setRadius(double radius) { m_radius = radius; }
    double getRadius() const { return m_radius; }

    /**
     * Set repulsion parameters for collision avoidance
     *
     * @param min_distance Collision radius (m). When masses get closer than this,
     *                     a steep repulsive force is applied. Set to 0 to disable.
     * @param repulsion_stiffness Strength of repulsion (N/m). Default -1 = auto (10x stiffness)
     */
    void setRepulsion(double min_distance, double repulsion_stiffness = -1.0) {
        // Clamp negative min_distance to 0 (disabled)
        m_min_distance = min_distance < 0.0 ? 0.0 : min_distance;
        m_repulsion_stiffness = repulsion_stiffness;
    }

    double getMinDistance() const { return m_min_distance; }
    double getRepulsionStiffness() const {
        return m_repulsion_stiffness > 0.0 ? m_repulsion_stiffness : 10.0 * m_stiffness;
    }
    bool isRepulsionEnabled() const { return m_min_distance > 0.0; }

    /**
     * Set grid type (topology): "quad" or "triangle"
     */
    void setGridType(const std::string& type) {
        if (type == "quad") {
            m_grid_type = GridType::Quad;
        } else if (type == "triangle") {
            m_grid_type = GridType::Triangle;
        } else {
            throw std::runtime_error("Grid type must be 'quad' or 'triangle'");
        }
    }

    std::string getGridType() const {
        return m_grid_type == GridType::Quad ? "quad" : "triangle";
    }

    /**
     * Set integrator type: "rk4", "symplectic", or "verlet"
     *
     * - rk4: 4th-order Runge-Kutta (default) - highest accuracy, 4 derivative evals/step
     * - symplectic: Symplectic Euler (1st order) - bounded energy error, 1 eval/step
     * - verlet: Velocity Verlet (2nd order) - excellent energy conservation, 2 evals/step
     *
     * Can be changed at any time during simulation.
     */
    void setIntegrator(const std::string& type) {
        if (type == "rk4") {
            m_integrator = Integrator::RK4;
        } else if (type == "symplectic") {
            m_integrator = Integrator::SymplecticEuler;
        } else if (type == "verlet") {
            m_integrator = Integrator::VelocityVerlet;
        } else {
            throw std::runtime_error("Integrator must be 'rk4', 'symplectic', or 'verlet'");
        }
    }

    std::string getIntegrator() const {
        switch (m_integrator) {
            case Integrator::RK4: return "rk4";
            case Integrator::SymplecticEuler: return "symplectic";
            case Integrator::VelocityVerlet: return "verlet";
            default: return "rk4";
        }
    }

    //=========================================================================
    // INITIALIZATION
    //=========================================================================

    void initialize() {
        // Clear all systems
        m_quad_5.reset();
        m_quad_10.reset();
        m_quad_20.reset();
        m_quad_50.reset();
        m_quad_100.reset();
        m_triangle_5.reset();
        m_triangle_10.reset();
        m_triangle_20.reset();
        m_triangle_50.reset();
        m_triangle_100.reset();

        // Create the system for current size and type
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
        // State layout: [x, y, vx, vy, θ, ω] per mass
        m_state[idx * 6 + 0] += dx;
        m_state[idx * 6 + 1] += dy;
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
            positions[i * 2 + 0] = m_state[i * 6 + 0];
            positions[i * 2 + 1] = m_state[i * 6 + 1];
        }

        return val::array(positions.begin(), positions.end());
    }

    val getVelocities() const {
        if (!m_initialized) return val::array();

        size_t num_masses = m_rows * m_cols;
        std::vector<double> velocities(num_masses * 2);

        for (size_t i = 0; i < num_masses; ++i) {
            velocities[i * 2 + 0] = m_state[i * 6 + 2];
            velocities[i * 2 + 1] = m_state[i * 6 + 3];
        }

        return val::array(velocities.begin(), velocities.end());
    }

    val getAngularVelocities() const {
        if (!m_initialized) return val::array();

        size_t num_masses = m_rows * m_cols;
        std::vector<double> ang_vels(num_masses);

        for (size_t i = 0; i < num_masses; ++i) {
            ang_vels[i] = m_state[i * 6 + 5];  // ω is at index 5
        }

        return val::array(ang_vels.begin(), ang_vels.end());
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
        result.set("x", m_state[idx * 6 + 0]);
        result.set("y", m_state[idx * 6 + 1]);
        return result;
    }

    double getKineticEnergy() const {
        if (!m_initialized) return 0.0;

        double ke = 0.0;
        size_t num_masses = m_rows * m_cols;
        for (size_t i = 0; i < num_masses; ++i) {
            double vx = m_state[i * 6 + 2];
            double vy = m_state[i * 6 + 3];
            double omega = m_state[i * 6 + 5];
            // KE = 0.5*m*v² + 0.5*I*ω² where I = 0.5*m*r²
            double I = 0.5 * m_mass * m_radius * m_radius;
            ke += 0.5 * m_mass * (vx*vx + vy*vy) + 0.5 * I * omega * omega;
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
            com_x += m_mass * m_state[i * 6 + 0];
            com_y += m_mass * m_state[i * 6 + 1];
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
            px += m_mass * m_state[i * 6 + 2];
            py += m_mass * m_state[i * 6 + 3];
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
        size_t d_springs = m_grid_type == GridType::Triangle
            ? 2 * (m_rows - 1) * (m_cols - 1)
            : 0;
        result.set("numSprings", static_cast<int>(h_springs + v_springs + d_springs));
        result.set("stateSize", static_cast<int>(m_state.size()));
        result.set("gridType", getGridType());
        result.set("integrator", getIntegrator());
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
        .function("setGridType", &Grid2DSimulator::setGridType)
        .function("getGridType", &Grid2DSimulator::getGridType)
        .function("setIntegrator", &Grid2DSimulator::setIntegrator)
        .function("getIntegrator", &Grid2DSimulator::getIntegrator)
        .function("setMass", &Grid2DSimulator::setMass)
        .function("setSpacing", &Grid2DSimulator::setSpacing)
        .function("setStiffness", &Grid2DSimulator::setStiffness)
        .function("setDamping", &Grid2DSimulator::setDamping)
        .function("setTimestep", &Grid2DSimulator::setTimestep)
        .function("setRadius", &Grid2DSimulator::setRadius)
        .function("getRadius", &Grid2DSimulator::getRadius)
        .function("setRepulsion", &Grid2DSimulator::setRepulsion)
        .function("getMinDistance", &Grid2DSimulator::getMinDistance)
        .function("getRepulsionStiffness", &Grid2DSimulator::getRepulsionStiffness)
        .function("isRepulsionEnabled", &Grid2DSimulator::isRepulsionEnabled)

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
        .function("getAngularVelocities", &Grid2DSimulator::getAngularVelocities)
        .function("getState", &Grid2DSimulator::getState)
        .function("getMassPosition", &Grid2DSimulator::getMassPosition)
        .function("getKineticEnergy", &Grid2DSimulator::getKineticEnergy)
        .function("getCenterOfMass", &Grid2DSimulator::getCenterOfMass)
        .function("getTotalMomentum", &Grid2DSimulator::getTotalMomentum)
        .function("getSystemInfo", &Grid2DSimulator::getSystemInfo);
}
