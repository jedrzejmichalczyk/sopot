#pragma once

#include <vector>
#include <span>
#include <chrono>
#include <concepts>
#include <functional>

namespace sopot {

// State types
using StateVector = std::vector<double>;
using StateDerivative = std::vector<double>;
using StateView = std::span<const double>;
using MutableStateView = std::span<double>;

// Solution data structure
struct SolutionResult {
    std::vector<double> times;
    std::vector<StateVector> states;
    double total_time{0.0};

    size_t size() const noexcept { return times.size(); }
    bool empty() const noexcept { return times.empty(); }

    void reserve(size_t capacity) {
        times.reserve(capacity);
        states.reserve(capacity);
    }
};

// ODE system concept for compile-time typed systems
template<typename System>
concept ODESystemConcept = requires(const System& sys, double t, StateView state) {
    { sys.getStateDimension() } -> std::convertible_to<size_t>;
    { sys.getInitialState() } -> std::convertible_to<StateVector>;
};

// Derivative function concept
template<typename F>
concept DerivativeFunctionConcept = requires(F f, double t, StateView state) {
    { f(t, state) } -> std::convertible_to<StateDerivative>;
};

// High-performance RK4 solver - pure compile-time dispatch
class RK4Solver {
private:
    mutable StateVector m_k1, m_k2, m_k3, m_k4;
    mutable StateVector m_temp_state;

    void ensureCapacity(size_t state_dim) const {
        if (m_k1.size() != state_dim) {
            m_k1.resize(state_dim);
            m_k2.resize(state_dim);
            m_k3.resize(state_dim);
            m_k4.resize(state_dim);
            m_temp_state.resize(state_dim);
        }
    }

public:
    // Generic solve with a derivatives function
    template<DerivativeFunctionConcept DerivFunc>
    SolutionResult solve(
        DerivFunc&& derivs,
        size_t state_dim,
        double t_start,
        double t_end,
        double dt,
        const StateVector& initial_state
    ) const {
        auto start_time = std::chrono::high_resolution_clock::now();

        const size_t num_steps = static_cast<size_t>((t_end - t_start) / dt) + 1;

        // Pre-allocate workspace
        ensureCapacity(state_dim);

        SolutionResult result;
        result.reserve(num_steps);

        // Initialize
        StateVector current_state = initial_state;
        double current_time = t_start;

        // Store initial conditions
        result.times.push_back(current_time);
        result.states.push_back(current_state);

        // Integration loop
        while (current_time < t_end - dt * 0.5) {
            StateView state_view(current_state);

            // k1 = dt * f(t, y)
            m_k1 = derivs(current_time, state_view);
            for (size_t i = 0; i < state_dim; ++i) {
                m_k1[i] *= dt;
                m_temp_state[i] = current_state[i] + 0.5 * m_k1[i];
            }

            // k2 = dt * f(t + dt/2, y + k1/2)
            StateView temp_view(m_temp_state);
            m_k2 = derivs(current_time + 0.5 * dt, temp_view);
            for (size_t i = 0; i < state_dim; ++i) {
                m_k2[i] *= dt;
                m_temp_state[i] = current_state[i] + 0.5 * m_k2[i];
            }

            // k3 = dt * f(t + dt/2, y + k2/2)
            m_k3 = derivs(current_time + 0.5 * dt, temp_view);
            for (size_t i = 0; i < state_dim; ++i) {
                m_k3[i] *= dt;
                m_temp_state[i] = current_state[i] + m_k3[i];
            }

            // k4 = dt * f(t + dt, y + k3)
            m_k4 = derivs(current_time + dt, temp_view);
            for (size_t i = 0; i < state_dim; ++i) {
                m_k4[i] *= dt;
            }

            // Update state: y_{n+1} = y_n + (k1 + 2*k2 + 2*k3 + k4)/6
            for (size_t i = 0; i < state_dim; ++i) {
                current_state[i] += (m_k1[i] + 2.0 * m_k2[i] + 2.0 * m_k3[i] + m_k4[i]) / 6.0;
            }

            current_time += dt;

            // Store result
            result.times.push_back(current_time);
            result.states.push_back(current_state);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        result.total_time = duration.count() / 1000.0; // Convert to milliseconds

        return result;
    }

    // Convenience overload that takes dimension and uses zero initial state
    template<DerivativeFunctionConcept DerivFunc>
    SolutionResult solve(
        DerivFunc&& derivs,
        size_t state_dim,
        double t_start,
        double t_end,
        double dt
    ) const {
        StateVector initial_state(state_dim, 0.0);
        return solve(std::forward<DerivFunc>(derivs), state_dim, t_start, t_end, dt, initial_state);
    }

    // Solve for ODE system that provides getInitialState
    template<ODESystemConcept System, DerivativeFunctionConcept DerivFunc>
    SolutionResult solve(
        const System& system,
        DerivFunc&& derivs,
        double t_start,
        double t_end,
        double dt
    ) const {
        return solve(
            std::forward<DerivFunc>(derivs),
            system.getStateDimension(),
            t_start, t_end, dt,
            system.getInitialState()
        );
    }

    constexpr std::string_view getSolverName() const {
        return "RK4";
    }

    constexpr size_t getOrder() const {
        return 4;
    }
};

// Factory function
inline RK4Solver createRK4Solver() {
    return RK4Solver{};
}

} // namespace sopot
