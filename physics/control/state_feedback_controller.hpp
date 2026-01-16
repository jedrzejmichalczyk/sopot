#pragma once

#include <array>
#include <cmath>
#include <algorithm>

namespace sopot::control {

/**
 * @brief Generic state feedback controller: u = -K(x - x_ref)
 *
 * This controller implements linear state feedback with:
 * - Gain matrix K (computed from LQR or pole placement)
 * - Reference state x_ref (default: zero for regulation)
 * - Optional control saturation limits
 *
 * @tparam N Number of states
 * @tparam M Number of control inputs
 */
template<size_t N, size_t M>
class StateFeedbackController {
public:
    using GainMatrix = std::array<std::array<double, N>, M>;
    using StateVector = std::array<double, N>;
    using ControlVector = std::array<double, M>;

private:
    GainMatrix m_K;
    StateVector m_reference;
    ControlVector m_u_min;
    ControlVector m_u_max;
    bool m_saturate;

public:
    /**
     * @brief Construct controller with gain matrix
     * @param K Feedback gain matrix (M×N)
     */
    explicit StateFeedbackController(const GainMatrix& K)
        : m_K(K)
        , m_reference{}
        , m_u_min{}
        , m_u_max{}
        , m_saturate(false)
    {
        // Initialize saturation limits to infinity
        for (size_t i = 0; i < M; ++i) {
            m_u_min[i] = -1e10;
            m_u_max[i] = 1e10;
        }
    }

    /**
     * @brief Set the reference state
     */
    void setReference(const StateVector& ref) {
        m_reference = ref;
    }

    /**
     * @brief Get the current reference state
     */
    const StateVector& getReference() const {
        return m_reference;
    }

    /**
     * @brief Set control saturation limits
     */
    void setSaturationLimits(const ControlVector& u_min, const ControlVector& u_max) {
        m_u_min = u_min;
        m_u_max = u_max;
        m_saturate = true;
    }

    /**
     * @brief Disable saturation
     */
    void disableSaturation() {
        m_saturate = false;
    }

    /**
     * @brief Compute control input given current state
     * @param state Current state vector
     * @return Control input u = -K(x - x_ref), saturated if limits set
     */
    ControlVector compute(const StateVector& state) const {
        ControlVector u{};

        for (size_t i = 0; i < M; ++i) {
            u[i] = 0.0;
            for (size_t j = 0; j < N; ++j) {
                u[i] -= m_K[i][j] * (state[j] - m_reference[j]);
            }

            // Apply saturation
            if (m_saturate) {
                u[i] = std::clamp(u[i], m_u_min[i], m_u_max[i]);
            }
        }

        return u;
    }

    /**
     * @brief Get the feedback gain matrix
     */
    const GainMatrix& getGain() const {
        return m_K;
    }

    /**
     * @brief Update the feedback gain matrix
     */
    void setGain(const GainMatrix& K) {
        m_K = K;
    }
};

/**
 * @brief Specialized controller for cart-double-pendulum (6 states, 1 input)
 */
class InvertedDoublePendulumController : public StateFeedbackController<6, 1> {
public:
    using Base = StateFeedbackController<6, 1>;

    /**
     * @brief Construct from LQR-computed gain
     */
    explicit InvertedDoublePendulumController(const GainMatrix& K)
        : Base(K)
    {}

    /**
     * @brief Construct with default parameters and compute LQR gain
     *
     * Uses typical parameters for a lab-scale inverted double pendulum:
     *   - Cart mass: 1 kg
     *   - Link masses: 0.5 kg each
     *   - Link lengths: 0.5 m each
     *
     * @param mc Cart mass
     * @param m1 Mass at end of link 1
     * @param m2 Mass at end of link 2
     * @param L1 Length of link 1
     * @param L2 Length of link 2
     * @param g Gravity
     * @param Q State weight matrix (diagonal: position, angles, velocities)
     * @param R Control weight
     */
    static InvertedDoublePendulumController createWithLQR(
        double mc, double m1, double m2, double L1, double L2, double g,
        const std::array<double, 6>& q_diag = {10.0, 100.0, 100.0, 1.0, 10.0, 10.0},
        double r = 0.01
    );

    /**
     * @brief Convenience method to compute control from individual state components
     */
    double computeForce(
        double x, double theta1, double theta2,
        double xdot, double omega1, double omega2
    ) const {
        StateVector state = {x, theta1, theta2, xdot, omega1, omega2};
        return compute(state)[0];
    }
};

} // namespace sopot::control

// Implementation of createWithLQR (requires lqr.hpp)
#include "lqr.hpp"

namespace sopot::control {

inline InvertedDoublePendulumController InvertedDoublePendulumController::createWithLQR(
    double mc, double m1, double m2, double L1, double L2, double g,
    const std::array<double, 6>& q_diag,
    double r
) {
    // Get linearized system
    auto [A, B] = linearizeCartDoublePendulum(mc, m1, m2, L1, L2, g);

    // Build diagonal Q matrix
    std::array<std::array<double, 6>, 6> Q{};
    for (size_t i = 0; i < 6; ++i) {
        Q[i][i] = q_diag[i];
    }

    // R matrix (scalar for single input)
    std::array<std::array<double, 1>, 1> R{};
    R[0][0] = r;

    // Solve LQR
    LQR<6, 1> lqr;
    bool converged = lqr.solve(A, B, Q, R);

    if (!converged) {
        // Still use the gain, but could log a warning
    }

    return InvertedDoublePendulumController(lqr.getGain());
}

} // namespace sopot::control
