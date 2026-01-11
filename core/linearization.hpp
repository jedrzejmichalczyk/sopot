#pragma once

#include "dual.hpp"
#include "typed_component.hpp"
#include <array>
#include <vector>
#include <functional>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace sopot {

// Matrix type for Jacobians
template<size_t Rows, size_t Cols>
using Matrix = std::array<std::array<double, Cols>, Rows>;

// Vector type
template<size_t N>
using Vector = std::array<double, N>;

// Linearization result for control systems
// ẋ = f(x, u) linearized to: δẋ = A * δx + B * δu
template<size_t StateSize, size_t InputSize>
struct LinearizedSystem {
    Matrix<StateSize, StateSize> A;  // State Jacobian ∂f/∂x
    Matrix<StateSize, InputSize> B;  // Input Jacobian ∂f/∂u
    Vector<StateSize> x0;             // Linearization point (state)
    Vector<InputSize> u0;             // Linearization point (input)
    double t0;                        // Time at linearization point

    // Print for debugging
    void print() const {
        std::cout << "Linearized System at t = " << t0 << std::endl;
        std::cout << "A matrix (" << StateSize << "x" << StateSize << "):" << std::endl;
        for (size_t i = 0; i < StateSize; ++i) {
            std::cout << "  [";
            for (size_t j = 0; j < StateSize; ++j) {
                std::cout << std::setw(10) << std::setprecision(4) << A[i][j];
                if (j < StateSize - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }

        std::cout << "B matrix (" << StateSize << "x" << InputSize << "):" << std::endl;
        for (size_t i = 0; i < StateSize; ++i) {
            std::cout << "  [";
            for (size_t j = 0; j < InputSize; ++j) {
                std::cout << std::setw(10) << std::setprecision(4) << B[i][j];
                if (j < InputSize - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
    }
};

// Linearizer for systems with explicit control input
// The dynamics function has signature: f(t, x, u) -> ẋ
template<size_t StateSize, size_t InputSize>
class SystemLinearizer {
public:
    // Type aliases for autodiff
    static constexpr size_t TotalDerivs = StateSize + InputSize;
    using DualT = Dual<double, TotalDerivs>;
    using DualState = std::array<DualT, StateSize>;
    using DualInput = std::array<DualT, InputSize>;
    using DualDerivative = std::array<DualT, StateSize>;

    // Dynamics function type: f(t, x, u) -> ẋ
    using DynamicsFunc = std::function<DualDerivative(DualT, const DualState&, const DualInput&)>;

    explicit SystemLinearizer(DynamicsFunc dynamics) : m_dynamics(std::move(dynamics)) {}

    // Linearize at a given point
    LinearizedSystem<StateSize, InputSize> linearize(
        double t,
        const Vector<StateSize>& x,
        const Vector<InputSize>& u
    ) const {
        LinearizedSystem<StateSize, InputSize> result;
        result.x0 = x;
        result.u0 = u;
        result.t0 = t;

        // Set up state variables with derivatives for indices [0, StateSize)
        DualState dual_x;
        for (size_t i = 0; i < StateSize; ++i) {
            dual_x[i] = DualT::variable(x[i], i);
        }

        // Set up input variables with derivatives for indices [StateSize, StateSize + InputSize)
        DualInput dual_u;
        for (size_t i = 0; i < InputSize; ++i) {
            dual_u[i] = DualT::variable(u[i], StateSize + i);
        }

        // Compute dynamics with autodiff
        DualT dual_t = DualT::constant(t);
        DualDerivative dual_xdot = m_dynamics(dual_t, dual_x, dual_u);

        // Extract Jacobians
        for (size_t i = 0; i < StateSize; ++i) {
            // A matrix: ∂ẋᵢ/∂xⱼ
            for (size_t j = 0; j < StateSize; ++j) {
                result.A[i][j] = dual_xdot[i].derivative(j);
            }
            // B matrix: ∂ẋᵢ/∂uⱼ
            for (size_t j = 0; j < InputSize; ++j) {
                result.B[i][j] = dual_xdot[i].derivative(StateSize + j);
            }
        }

        return result;
    }

private:
    DynamicsFunc m_dynamics;
};

// Helper to create a linearizer from a dynamics function
template<size_t StateSize, size_t InputSize>
auto makeLinearizer(
    typename SystemLinearizer<StateSize, InputSize>::DynamicsFunc dynamics
) {
    return SystemLinearizer<StateSize, InputSize>(std::move(dynamics));
}

// Linearizer for autonomous systems (no explicit input)
// The dynamics function has signature: f(t, x) -> ẋ
template<size_t StateSize>
class AutonomousLinearizer {
public:
    using DualT = Dual<double, StateSize>;
    using DualState = std::array<DualT, StateSize>;
    using DualDerivative = std::array<DualT, StateSize>;

    using DynamicsFunc = std::function<DualDerivative(DualT, const DualState&)>;

    explicit AutonomousLinearizer(DynamicsFunc dynamics) : m_dynamics(std::move(dynamics)) {}

    // Compute state Jacobian A = ∂f/∂x
    Matrix<StateSize, StateSize> computeJacobian(double t, const Vector<StateSize>& x) const {
        Matrix<StateSize, StateSize> A;

        // Set up state variables with derivatives
        DualState dual_x;
        for (size_t i = 0; i < StateSize; ++i) {
            dual_x[i] = DualT::variable(x[i], i);
        }

        // Compute dynamics with autodiff
        DualT dual_t = DualT::constant(t);
        DualDerivative dual_xdot = m_dynamics(dual_t, dual_x);

        // Extract Jacobian
        for (size_t i = 0; i < StateSize; ++i) {
            for (size_t j = 0; j < StateSize; ++j) {
                A[i][j] = dual_xdot[i].derivative(j);
            }
        }

        return A;
    }

private:
    DynamicsFunc m_dynamics;
};

// Helper to create an autonomous linearizer
template<size_t StateSize>
auto makeAutonomousLinearizer(
    typename AutonomousLinearizer<StateSize>::DynamicsFunc dynamics
) {
    return AutonomousLinearizer<StateSize>(std::move(dynamics));
}

// Wrapper to linearize a TypedODESystem
// This converts a TypedODESystem to work with the linearizer
template<typename... Components>
class ODESystemLinearizer {
public:
    static constexpr size_t StateSize = (Components::state_size + ...);
    using DualT = Dual<double, StateSize>;

    explicit ODESystemLinearizer(TypedODESystem<DualT, Components...>& system)
        : m_system(system) {}

    // Compute the state Jacobian A = ∂f/∂x at a given state
    Matrix<StateSize, StateSize> computeJacobian(double t, const Vector<StateSize>& x) const {
        // Create dual state with identity derivatives
        std::vector<DualT> dual_state(StateSize);
        for (size_t i = 0; i < StateSize; ++i) {
            dual_state[i] = DualT::variable(x[i], i);
        }

        // Compute derivatives through the system
        auto dual_derivs = m_system.computeDerivatives(DualT::constant(t), dual_state);

        // Extract Jacobian
        Matrix<StateSize, StateSize> A;
        for (size_t i = 0; i < StateSize; ++i) {
            for (size_t j = 0; j < StateSize; ++j) {
                A[i][j] = dual_derivs[i].derivative(j);
            }
        }

        return A;
    }

private:
    TypedODESystem<DualT, Components...>& m_system;
};

// Factory function for ODESystemLinearizer
template<typename... Components>
auto makeODESystemLinearizer(TypedODESystem<Dual<double, (Components::state_size + ...)>, Components...>& system) {
    return ODESystemLinearizer<Components...>(system);
}

} // namespace sopot
