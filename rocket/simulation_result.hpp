#pragma once

#include "../core/solver.hpp"
#include "../io/interpolation.hpp"
#include "rocket_tags.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>

namespace sopot::rocket {

/**
 * SimulationResult: Stores trajectory data with state function interpolation
 *
 * Wraps SolutionResult and provides:
 * - Linear interpolation of state at any time
 * - State function evaluation at any time
 * - Trajectory statistics (apogee, max speed, etc.)
 */
template<typename RocketType>
class SimulationResult {
public:
    using T = typename RocketType::scalar_type;

private:
    const RocketType* m_rocket;
    SolutionResult m_solution;

    // Cached statistics
    mutable double m_apogee{-1};
    mutable double m_apogee_time{0};
    mutable double m_max_speed{0};
    mutable double m_max_speed_time{0};
    mutable double m_burnout_altitude{0};
    mutable double m_burnout_speed{0};
    mutable bool m_stats_computed{false};

public:
    SimulationResult() : m_rocket(nullptr) {}

    SimulationResult(const RocketType& rocket, SolutionResult&& solution)
        : m_rocket(&rocket), m_solution(std::move(solution)) {}

    // Basic accessors
    size_t size() const { return m_solution.size(); }
    bool empty() const { return m_solution.empty(); }
    double startTime() const { return m_solution.times.empty() ? 0 : m_solution.times.front(); }
    double endTime() const { return m_solution.times.empty() ? 0 : m_solution.times.back(); }
    double simulationTimeMs() const { return m_solution.total_time; }

    const std::vector<double>& times() const { return m_solution.times; }
    const std::vector<StateVector>& states() const { return m_solution.states; }

    // Find index for interpolation
    std::pair<size_t, double> findIndex(double t) const {
        if (t <= m_solution.times.front()) {
            return {0, 0.0};
        }
        if (t >= m_solution.times.back()) {
            return {m_solution.size() - 1, 0.0};
        }

        auto it = std::lower_bound(m_solution.times.begin(), m_solution.times.end(), t);
        size_t i = std::distance(m_solution.times.begin(), it);
        if (i > 0) --i;

        double t0 = m_solution.times[i];
        double t1 = m_solution.times[i + 1];
        double alpha = (t - t0) / (t1 - t0);

        return {i, alpha};
    }

    // Interpolate state at time t
    StateVector interpolateState(double t) const {
        auto [i, alpha] = findIndex(t);

        if (alpha == 0.0 || i >= m_solution.size() - 1) {
            return m_solution.states[i];
        }

        // Linear interpolation
        StateVector result(m_solution.states[i].size());
        const auto& s0 = m_solution.states[i];
        const auto& s1 = m_solution.states[i + 1];

        for (size_t j = 0; j < result.size(); ++j) {
            result[j] = s0[j] + alpha * (s1[j] - s0[j]);
        }

        return result;
    }

    // Get state at index (no interpolation)
    const StateVector& stateAt(size_t index) const {
        return m_solution.states[index];
    }

    double timeAt(size_t index) const {
        return m_solution.times[index];
    }

    //=========================================================================
    // STATE FUNCTION QUERIES
    //=========================================================================

    // Query any state function at time t
    template<StateTagConcept Tag>
    auto query(double t) const {
        auto state = interpolateState(t);
        return m_rocket->template queryStateFunction<Tag>(state);
    }

    // Convenience: Position at time t
    Vector3<T> position(double t) const {
        return query<kinematics::PositionENU>(t);
    }

    // Convenience: Velocity at time t
    Vector3<T> velocity(double t) const {
        return query<kinematics::VelocityENU>(t);
    }

    // Convenience: Altitude at time t
    T altitude(double t) const {
        return query<kinematics::Altitude>(t);
    }

    // Convenience: Speed at time t
    T speed(double t) const {
        return velocity(t).norm();
    }

    // Convenience: Mass at time t
    T mass(double t) const {
        return query<dynamics::Mass>(t);
    }

    // Convenience: Quaternion at time t
    Quaternion<T> attitude(double t) const {
        return query<kinematics::AttitudeQuaternion>(t);
    }

    //=========================================================================
    // TRAJECTORY STATISTICS
    //=========================================================================

    void computeStatistics(double burn_time = 0) const {
        if (m_stats_computed) return;

        m_apogee = 0;
        m_max_speed = 0;

        for (size_t i = 0; i < m_solution.size(); ++i) {
            double t = m_solution.times[i];
            const auto& state = m_solution.states[i];

            // Altitude (position Z, index 3 after time at index 0)
            double alt = state[3];

            // Speed (velocity at indices 4-6)
            double spd = std::sqrt(
                state[4] * state[4] +
                state[5] * state[5] +
                state[6] * state[6]
            );

            if (alt > m_apogee) {
                m_apogee = alt;
                m_apogee_time = t;
            }

            if (spd > m_max_speed) {
                m_max_speed = spd;
                m_max_speed_time = t;
            }

            // Burnout conditions
            if (burn_time > 0 && std::abs(t - burn_time) < 0.1) {
                m_burnout_altitude = alt;
                m_burnout_speed = spd;
            }
        }

        m_stats_computed = true;
    }

    double apogee() const { computeStatistics(); return m_apogee; }
    double apogeeTime() const { computeStatistics(); return m_apogee_time; }
    double maxSpeed() const { computeStatistics(); return m_max_speed; }
    double maxSpeedTime() const { computeStatistics(); return m_max_speed_time; }
    double burnoutAltitude() const { return m_burnout_altitude; }
    double burnoutSpeed() const { return m_burnout_speed; }

    //=========================================================================
    // DATA EXPORT
    //=========================================================================

    // Sample state function at regular intervals
    template<typename Func>
    std::vector<double> sampleFunction(Func&& func, double t_start, double t_end, double dt) const {
        std::vector<double> values;
        size_t n = static_cast<size_t>((t_end - t_start) / dt) + 1;
        values.reserve(n);

        for (double t = t_start; t <= t_end; t += dt) {
            auto state = interpolateState(t);
            values.push_back(func(state, t));
        }

        return values;
    }

    // Get time series of altitude
    std::vector<double> altitudeProfile() const {
        std::vector<double> alt;
        alt.reserve(m_solution.size());
        for (const auto& state : m_solution.states) {
            alt.push_back(state[3]);
        }
        return alt;
    }

    // Get time series of speed
    std::vector<double> speedProfile() const {
        std::vector<double> spd;
        spd.reserve(m_solution.size());
        for (const auto& state : m_solution.states) {
            spd.push_back(std::sqrt(
                state[4] * state[4] +
                state[5] * state[5] +
                state[6] * state[6]
            ));
        }
        return spd;
    }
};

// Factory function
template<typename RocketType>
SimulationResult<RocketType> createSimulationResult(
    const RocketType& rocket,
    SolutionResult&& solution
) {
    return SimulationResult<RocketType>(rocket, std::move(solution));
}

} // namespace sopot::rocket
