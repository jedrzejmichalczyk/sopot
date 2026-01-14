#pragma once

//=============================================================================
// SOPOT Reflection-Based API (Prototype)
//=============================================================================
// This simulates what C++26 reflection will enable.
// Users write plain structs; framework generates the machinery.
//
// Current implementation: Uses macros to bridge until P2996 is available.
// Future implementation: Replace macros with actual reflection.
//=============================================================================

#include "../core/scalar.hpp"
#include "../core/solver.hpp"
#include <array>
#include <vector>
#include <span>
#include <cmath>
#include <functional>
#include <string_view>
#include <iostream>
#include <iomanip>

namespace sopot::reflect {

//=============================================================================
// State - Simple wrapper for accessing state by name
//=============================================================================

template<typename System>
class State {
    std::span<const double> m_data;
    const System* m_system;

public:
    State(std::span<const double> data, const System* sys)
        : m_data(data), m_system(sys) {}

    // Access by member pointer (framework resolves to index)
    template<auto MemberPtr>
    double get() const {
        constexpr size_t idx = System::template index_of<MemberPtr>();
        return m_data[idx];
    }

    // Raw access
    double operator[](size_t i) const { return m_data[i]; }
    size_t size() const { return m_data.size(); }
    std::span<const double> span() const { return m_data; }
};

//=============================================================================
// MutableState - For setting state values
//=============================================================================

template<typename System>
class MutableState {
    std::span<double> m_data;
    const System* m_system;

public:
    MutableState(std::span<double> data, const System* sys)
        : m_data(data), m_system(sys) {}

    template<auto MemberPtr>
    double& get() {
        constexpr size_t idx = System::template index_of<MemberPtr>();
        return m_data[idx];
    }

    template<auto MemberPtr>
    double get() const {
        constexpr size_t idx = System::template index_of<MemberPtr>();
        return m_data[idx];
    }

    double& operator[](size_t i) { return m_data[i]; }
    double operator[](size_t i) const { return m_data[i]; }
    size_t size() const { return m_data.size(); }
    std::span<double> span() { return m_data; }
};

//=============================================================================
// SimulationResult - Trajectory storage
//=============================================================================

struct SimulationResult {
    std::vector<double> time;
    std::vector<std::vector<double>> states;

    size_t size() const { return time.size(); }

    void reserve(size_t n) {
        time.reserve(n);
        states.reserve(n);
    }

    void push(double t, const std::vector<double>& state) {
        time.push_back(t);
        states.push_back(state);
    }

    // Print trajectory
    template<typename System>
    void print(const System& sys, size_t every = 1) const {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Time";
        for (const auto& name : sys.state_names()) {
            std::cout << "\t" << name;
        }
        std::cout << "\n";

        for (size_t i = 0; i < time.size(); i += every) {
            std::cout << time[i];
            for (double v : states[i]) {
                std::cout << "\t" << v;
            }
            std::cout << "\n";
        }
    }
};

//=============================================================================
// RK4 Integrator (simplified, non-templated for clarity)
//=============================================================================

template<typename System>
SimulationResult integrate(
    System& system,
    double t0,
    double t_end,
    double dt = 0.001
) {
    SimulationResult result;

    std::vector<double> state = system.initial_state();
    const size_t n = state.size();

    std::vector<double> k1(n), k2(n), k3(n), k4(n), temp(n);

    double t = t0;
    result.push(t, state);

    while (t < t_end) {
        // RK4 steps
        auto derivs1 = system.derivatives(t, state);
        for (size_t i = 0; i < n; i++) {
            k1[i] = derivs1[i];
            temp[i] = state[i] + 0.5 * dt * k1[i];
        }

        auto derivs2 = system.derivatives(t + 0.5*dt, temp);
        for (size_t i = 0; i < n; i++) {
            k2[i] = derivs2[i];
            temp[i] = state[i] + 0.5 * dt * k2[i];
        }

        auto derivs3 = system.derivatives(t + 0.5*dt, temp);
        for (size_t i = 0; i < n; i++) {
            k3[i] = derivs3[i];
            temp[i] = state[i] + dt * k3[i];
        }

        auto derivs4 = system.derivatives(t + dt, temp);
        for (size_t i = 0; i < n; i++) {
            k4[i] = derivs4[i];
            state[i] += (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        }

        t += dt;
        result.push(t, state);
    }

    return result;
}

//=============================================================================
// MACROS - Simulate reflection until C++26
//=============================================================================
// These macros generate the boilerplate that reflection would auto-generate.
// The USER code stays clean; macros are in the "framework" layer.

// Start a reflectable system definition
#define SOPOT_SYSTEM(SystemName, StateCount) \
    struct SystemName { \
        static constexpr size_t state_count = StateCount; \
        using StateType = sopot::reflect::State<SystemName>; \
        using MutableStateType = sopot::reflect::MutableState<SystemName>;

// Declare a state variable with its index
#define SOPOT_STATE(index, type, name, initial) \
        type name##_init = initial; \
        static constexpr size_t name##_index = index; \
        template<auto P> \
        static constexpr size_t index_of() { \
            if constexpr (P == &std::remove_pointer_t<decltype(this)>::name##_init) \
                return index; \
            else return static_cast<size_t>(-1); \
        }

// End system definition
#define SOPOT_END_SYSTEM() \
    };

} // namespace sopot::reflect
