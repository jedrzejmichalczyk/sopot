#pragma once

#include <cmath>
#include <concepts>

namespace sopot {

/**
 * @brief Compile-time stability analysis for ODE systems
 *
 * This analyzer computes conservative stability bounds for explicit time
 * integrators (RK4) applied to mass-spring systems.
 *
 * Theory:
 * - For mass-spring systems, stability depends on maximum natural frequency
 * - ω_max = sqrt(k_eff_max / m_min)
 * - k_eff_max = k * max_connections (effective stiffness)
 * - For RK4, stability bound: dt < 2.8 / ω_max (conservative)
 *
 * Usage:
 * @code
 * constexpr auto stability = GridStabilityInfo<3, 3, false>::analyze(10.0, 1.0, 0.5);
 * static_assert(my_dt < stability.dt_recommended);
 * @endcode
 */

/**
 * @brief Stability analysis result
 */
struct StabilityResult {
    double max_frequency;        // Maximum natural frequency (rad/s)
    double dt_recommended;       // Recommended maximum time step (s)
    size_t max_connectivity;     // Maximum connections per mass
    double effective_stiffness;  // k_eff = k * max_connectivity

    constexpr StabilityResult(
        double omega_max,
        double dt_rec,
        size_t max_conn,
        double k_eff
    )
        : max_frequency(omega_max)
        , dt_recommended(dt_rec)
        , max_connectivity(max_conn)
        , effective_stiffness(k_eff)
    {}
};

/**
 * @brief Grid stability analyzer for 2D mass-spring grids
 *
 * Provides compile-time computation of stability bounds based on:
 * - Grid topology (rows, cols, diagonal connections)
 * - Physical parameters (mass, stiffness, damping)
 *
 * @tparam Rows Number of rows in grid
 * @tparam Cols Number of columns in grid
 * @tparam IncludeDiagonals Whether grid includes diagonal springs
 */
template<size_t Rows, size_t Cols, bool IncludeDiagonals = false>
struct GridStabilityInfo {
    static_assert(Rows >= 2 && Cols >= 2, "Grid must be at least 2x2");

    /**
     * @brief Compute maximum connectivity (edges per mass)
     *
     * Interior masses have more connections than edge/corner masses.
     * We return the maximum for conservative stability bounds.
     */
    static constexpr size_t computeMaxConnectivity() {
        if constexpr (IncludeDiagonals) {
            // With diagonals: interior masses connect to 8 neighbors
            // (up, down, left, right, and 4 diagonals)
            return 8;
        } else {
            // Without diagonals: interior masses connect to 4 neighbors
            // (up, down, left, right)
            return 4;
        }
    }

    /**
     * @brief Analyze stability for given physical parameters
     *
     * @param stiffness Spring stiffness k (N/m)
     * @param mass Point mass m (kg)
     * @param damping Spring damping c (N·s/m) [optional, improves stability]
     * @return StabilityResult with recommended time step
     */
    static constexpr StabilityResult analyze(
        double stiffness,
        double mass,
        double damping = 0.0
    ) {
        constexpr size_t max_conn = computeMaxConnectivity();

        // Effective stiffness for most-connected mass
        double k_eff = stiffness * max_conn;

        // Maximum natural frequency (undamped)
        // ω_max = sqrt(k_eff / m)
        double omega_max = std::sqrt(k_eff / mass);

        // RK4 stability factor (empirical)
        // RK4 has stability region |R(z)| ≤ 1 for |z| ≤ 2.78
        // where z = i*ω*dt
        constexpr double RK4_STABILITY_FACTOR = 2.78;

        // Apply safety margin (0.9x) for real-world usage
        constexpr double SAFETY_MARGIN = 0.9;

        // Recommended time step
        double dt_rec = SAFETY_MARGIN * RK4_STABILITY_FACTOR / omega_max;

        // Note: Damping improves stability but we use conservative bound
        // A more sophisticated analysis could reduce dt_rec based on damping ratio

        return StabilityResult{omega_max, dt_rec, max_conn, k_eff};
    }

    /**
     * @brief Get minimum recommended time step
     *
     * Convenience function for compile-time dt computation
     */
    static constexpr double getRecommendedTimeStep(
        double stiffness,
        double mass,
        double damping = 0.0
    ) {
        return analyze(stiffness, mass, damping).dt_recommended;
    }
};

/**
 * @brief Triangular grid stability analyzer
 *
 * Triangular grids have full diagonal connectivity (X pattern),
 * equivalent to rectangular grids with diagonals enabled.
 *
 * @tparam Rows Number of rows in grid
 * @tparam Cols Number of columns in grid
 */
template<size_t Rows, size_t Cols>
struct TriangularGridStabilityInfo {
    static_assert(Rows >= 2 && Cols >= 2, "Grid must be at least 2x2");

    // Triangular grids always have full diagonal connectivity
    using BaseAnalyzer = GridStabilityInfo<Rows, Cols, true>;

    static constexpr StabilityResult analyze(
        double stiffness,
        double mass,
        double damping = 0.0
    ) {
        return BaseAnalyzer::analyze(stiffness, mass, damping);
    }

    static constexpr double getRecommendedTimeStep(
        double stiffness,
        double mass,
        double damping = 0.0
    ) {
        return BaseAnalyzer::getRecommendedTimeStep(stiffness, mass, damping);
    }
};

/**
 * @brief Check if a given time step is within stability bounds
 *
 * @param dt Proposed time step (s)
 * @param stability Stability analysis result
 * @return true if dt is safe, false if likely unstable
 */
constexpr bool isStableTimeStep(double dt, const StabilityResult& stability) {
    return dt <= stability.dt_recommended;
}

/**
 * @brief Compute stability safety factor
 *
 * Returns ratio dt_used / dt_recommended:
 * - < 1.0: Safe (recommended)
 * - 1.0 - 1.2: Marginal (may work but risky)
 * - > 1.2: Likely unstable
 *
 * @param dt Proposed time step (s)
 * @param stability Stability analysis result
 * @return Safety factor (lower is safer)
 */
constexpr double computeSafetyFactor(double dt, const StabilityResult& stability) {
    return dt / stability.dt_recommended;
}

} // namespace sopot
