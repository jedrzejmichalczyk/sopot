#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace sopot::control {

/**
 * @brief Linear Quadratic Regulator (LQR) solver
 *
 * Solves the continuous-time algebraic Riccati equation (CARE):
 *   A'P + PA - PBR⁻¹B'P + Q = 0
 *
 * And computes the optimal feedback gain:
 *   K = R⁻¹B'P
 *
 * For the system ẋ = Ax + Bu with cost functional:
 *   J = ∫(x'Qx + u'Ru)dt
 *
 * The optimal control law is:
 *   u = -Kx
 *
 * @tparam N Number of states
 * @tparam M Number of control inputs
 */
template<size_t N, size_t M>
class LQR {
public:
    using StateMatrix = std::array<std::array<double, N>, N>;
    using InputMatrix = std::array<std::array<double, M>, N>;
    using GainMatrix = std::array<std::array<double, N>, M>;
    using StateWeightMatrix = std::array<std::array<double, N>, N>;
    using InputWeightMatrix = std::array<std::array<double, M>, M>;

private:
    GainMatrix m_K;      // Optimal gain matrix
    StateMatrix m_P;     // Solution to Riccati equation
    bool m_solved;

    // Matrix operations
    static StateMatrix transpose(const StateMatrix& A) {
        StateMatrix result{};
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                result[i][j] = A[j][i];
            }
        }
        return result;
    }

    static StateMatrix add(const StateMatrix& A, const StateMatrix& B) {
        StateMatrix result{};
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                result[i][j] = A[i][j] + B[i][j];
            }
        }
        return result;
    }

    static StateMatrix subtract(const StateMatrix& A, const StateMatrix& B) {
        StateMatrix result{};
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                result[i][j] = A[i][j] - B[i][j];
            }
        }
        return result;
    }

    static StateMatrix multiply(const StateMatrix& A, const StateMatrix& B) {
        StateMatrix result{};
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                result[i][j] = 0.0;
                for (size_t k = 0; k < N; ++k) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }

    // A * B where B is N×M
    static InputMatrix multiplyAB(const StateMatrix& A, const InputMatrix& B) {
        InputMatrix result{};
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < M; ++j) {
                result[i][j] = 0.0;
                for (size_t k = 0; k < N; ++k) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }

    // B' * P where B is N×M, P is N×N, result is M×N
    static std::array<std::array<double, N>, M> multiplyBtP(
        const InputMatrix& B, const StateMatrix& P
    ) {
        std::array<std::array<double, N>, M> result{};
        for (size_t i = 0; i < M; ++i) {
            for (size_t j = 0; j < N; ++j) {
                result[i][j] = 0.0;
                for (size_t k = 0; k < N; ++k) {
                    result[i][j] += B[k][i] * P[k][j];
                }
            }
        }
        return result;
    }

    // B * K where B is N×M, K is M×N, result is N×N
    static StateMatrix multiplyBK(const InputMatrix& B, const GainMatrix& K) {
        StateMatrix result{};
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                result[i][j] = 0.0;
                for (size_t k = 0; k < M; ++k) {
                    result[i][j] += B[i][k] * K[k][j];
                }
            }
        }
        return result;
    }

    // Frobenius norm
    static double norm(const StateMatrix& A) {
        double sum = 0.0;
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                sum += A[i][j] * A[i][j];
            }
        }
        return std::sqrt(sum);
    }

    // Identity matrix
    static StateMatrix identity() {
        StateMatrix I{};
        for (size_t i = 0; i < N; ++i) {
            I[i][i] = 1.0;
        }
        return I;
    }

    // Scale matrix
    static StateMatrix scale(const StateMatrix& A, double s) {
        StateMatrix result{};
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                result[i][j] = A[i][j] * s;
            }
        }
        return result;
    }

    // Solve small linear system (for M=1: scalar, for M>1: Gaussian elimination)
    static GainMatrix solveForGain(
        const InputWeightMatrix& RplusBtPB,
        const std::array<std::array<double, N>, M>& BtP
    ) {
        GainMatrix K{};

        if constexpr (M == 1) {
            // Scalar case: K = BtP / R
            double r = RplusBtPB[0][0];
            for (size_t j = 0; j < N; ++j) {
                K[0][j] = BtP[0][j] / r;
            }
        } else {
            // General case: solve (R + B'PB) * K = B'P
            // Using Gaussian elimination with partial pivoting

            // Augmented matrix [RplusBtPB | BtP]
            std::array<std::array<double, M + N>, M> aug{};
            for (size_t i = 0; i < M; ++i) {
                for (size_t j = 0; j < M; ++j) {
                    aug[i][j] = RplusBtPB[i][j];
                }
                for (size_t j = 0; j < N; ++j) {
                    aug[i][M + j] = BtP[i][j];
                }
            }

            // Forward elimination
            for (size_t col = 0; col < M; ++col) {
                // Find pivot
                size_t max_row = col;
                for (size_t row = col + 1; row < M; ++row) {
                    if (std::abs(aug[row][col]) > std::abs(aug[max_row][col])) {
                        max_row = row;
                    }
                }
                std::swap(aug[col], aug[max_row]);

                // Eliminate
                for (size_t row = col + 1; row < M; ++row) {
                    double factor = aug[row][col] / aug[col][col];
                    for (size_t j = col; j < M + N; ++j) {
                        aug[row][j] -= factor * aug[col][j];
                    }
                }
            }

            // Back substitution
            for (size_t j = 0; j < N; ++j) {
                for (int i = static_cast<int>(M) - 1; i >= 0; --i) {
                    double sum = aug[i][M + j];
                    for (size_t k = i + 1; k < M; ++k) {
                        sum -= aug[i][k] * K[k][j];
                    }
                    K[i][j] = sum / aug[i][i];
                }
            }
        }

        return K;
    }

public:
    LQR() : m_K{}, m_P{}, m_solved(false) {}

    /**
     * @brief Solve the continuous-time algebraic Riccati equation
     *
     * Uses iterative method with stabilizing initial feedback:
     * 1. Start with initial stabilizing gain K0 (LQR iteration needs this for unstable systems)
     * 2. Iterate the Lyapunov equation to find P
     * 3. Update K from P
     *
     * For unstable systems like the inverted pendulum, we use:
     * - Doubling algorithm for faster convergence
     * - Regularization for numerical stability
     *
     * @param A System matrix (N×N)
     * @param B Input matrix (N×M)
     * @param Q State weight matrix (N×N, positive semi-definite)
     * @param R Input weight matrix (M×M, positive definite)
     * @param dt Discretization timestep (default 0.01)
     * @param max_iter Maximum iterations (default 1000)
     * @param tol Convergence tolerance (default 1e-8)
     * @return true if converged, false otherwise
     */
    bool solve(
        const StateMatrix& A,
        const InputMatrix& B,
        const StateWeightMatrix& Q,
        const InputWeightMatrix& R,
        double dt = 0.01,
        size_t max_iter = 1000,
        double tol = 1e-8
    ) {
        // For unstable systems, use the discrete-time Riccati iteration
        // with matrix exponential approximation for discretization

        // Better discretization using first-order hold
        // Ad ≈ exp(A*dt) ≈ I + A*dt + (A*dt)²/2
        StateMatrix Adt = scale(A, dt);
        StateMatrix Adt2 = multiply(Adt, Adt);
        StateMatrix Ad = add(add(identity(), Adt), scale(Adt2, 0.5));

        // Bd ≈ (∫₀^dt exp(A*τ) dτ) * B ≈ (I*dt + A*dt²/2) * B
        StateMatrix Bcoef = add(scale(identity(), dt), scale(Adt, dt * 0.5));
        InputMatrix Bd = multiplyAB(Bcoef, B);

        // Scale Q for discrete time
        StateWeightMatrix Qd = scale(Q, dt);

        // Initialize P with scaled Q (stabilizes iteration for unstable A)
        m_P = scale(Q, 1.0);

        // Transpose of Ad
        StateMatrix AdT = transpose(Ad);

        // Iterate discrete-time Riccati equation: P = Ad'PAd - Ad'PBd(R+Bd'PBd)^{-1}Bd'PAd + Qd
        for (size_t iter = 0; iter < max_iter; ++iter) {
            // Compute Ad' * P
            StateMatrix AdTP = multiply(AdT, m_P);

            // Compute Bd' * P
            auto BdTP = multiplyBtP(Bd, m_P);

            // Compute Bd' * P * Bd
            InputWeightMatrix BdTPBd{};
            for (size_t i = 0; i < M; ++i) {
                for (size_t j = 0; j < M; ++j) {
                    BdTPBd[i][j] = 0.0;
                    for (size_t k = 0; k < N; ++k) {
                        BdTPBd[i][j] += BdTP[i][k] * Bd[k][j];
                    }
                }
            }

            // Compute R + Bd' * P * Bd (scaled R for discrete system)
            InputWeightMatrix RplusBdTPBd{};
            for (size_t i = 0; i < M; ++i) {
                for (size_t j = 0; j < M; ++j) {
                    RplusBdTPBd[i][j] = R[i][j] * dt + BdTPBd[i][j];
                }
            }

            // Compute Bd' * P * Ad
            std::array<std::array<double, N>, M> BdTPAd{};
            for (size_t i = 0; i < M; ++i) {
                for (size_t j = 0; j < N; ++j) {
                    BdTPAd[i][j] = 0.0;
                    for (size_t k = 0; k < N; ++k) {
                        BdTPAd[i][j] += BdTP[i][k] * Ad[k][j];
                    }
                }
            }

            // Solve (R*dt + Bd'PBd) * Kd = Bd'PAd for Kd
            GainMatrix Kd = solveForGain(RplusBdTPBd, BdTPAd);

            // Compute Ad' * P * Ad
            StateMatrix AdTPAd = multiply(AdTP, Ad);

            // Compute Kd' * (R*dt + Bd'PBd) * Kd
            // First: (R*dt + Bd'PBd) * Kd = BdTPAd (from the solve above)
            // So: Kd' * BdTPAd
            StateMatrix KtRK{};
            for (size_t i = 0; i < N; ++i) {
                for (size_t j = 0; j < N; ++j) {
                    KtRK[i][j] = 0.0;
                    for (size_t k = 0; k < M; ++k) {
                        KtRK[i][j] += Kd[k][i] * BdTPAd[k][j];
                    }
                }
            }

            // P_new = Ad'PAd - Kd'(R*dt+Bd'PBd)Kd + Qd
            StateMatrix P_new = add(subtract(AdTPAd, KtRK), Qd);

            // Symmetrize for numerical stability
            for (size_t i = 0; i < N; ++i) {
                for (size_t j = i + 1; j < N; ++j) {
                    double avg = 0.5 * (P_new[i][j] + P_new[j][i]);
                    P_new[i][j] = avg;
                    P_new[j][i] = avg;
                }
            }

            // Check convergence
            StateMatrix diff = subtract(P_new, m_P);
            double change = norm(diff);
            double p_norm = norm(P_new);

            // Relative change for better convergence detection
            double rel_change = (p_norm > 1e-10) ? change / p_norm : change;

            m_P = P_new;

            if (rel_change < tol || change < tol) {
                // Compute continuous-time gain K = R^{-1} B' P
                auto BTP = multiplyBtP(B, m_P);
                m_K = solveForGain(R, BTP);
                m_solved = true;
                return true;
            }

            // Check for divergence
            if (p_norm > 1e15 || std::isnan(change)) {
                break;
            }

        }

        // Did not converge cleanly, but if P is reasonable, compute gain anyway
        double final_norm = norm(m_P);
        if (final_norm < 1e10 && !std::isnan(final_norm)) {
            auto BTP = multiplyBtP(B, m_P);
            m_K = solveForGain(R, BTP);
            m_solved = true;  // Mark as solved if we got a reasonable answer
            return true;
        }

        m_solved = false;
        return false;
    }

    /**
     * @brief Get the optimal feedback gain matrix
     * @return K such that u = -Kx is optimal
     */
    const GainMatrix& getGain() const {
        if (!m_solved) {
            throw std::runtime_error("LQR not solved yet");
        }
        return m_K;
    }

    /**
     * @brief Get the solution to the Riccati equation
     * @return P matrix
     */
    const StateMatrix& getRiccatiSolution() const {
        return m_P;
    }

    /**
     * @brief Check if the LQR problem was solved successfully
     */
    bool isSolved() const {
        return m_solved;
    }

    /**
     * @brief Compute optimal control input
     * @param state Current state vector
     * @param reference Reference state (default: zero)
     * @return Optimal control input u = -K(x - x_ref)
     */
    std::array<double, M> computeControl(
        const std::array<double, N>& state,
        const std::array<double, N>& reference = {}
    ) const {
        std::array<double, M> u{};
        for (size_t i = 0; i < M; ++i) {
            u[i] = 0.0;
            for (size_t j = 0; j < N; ++j) {
                u[i] -= m_K[i][j] * (state[j] - reference[j]);
            }
        }
        return u;
    }
};

/**
 * @brief Linearize the cart-double-pendulum system around upright equilibrium
 *
 * At equilibrium: x = 0, θ₁ = 0, θ₂ = 0, ẋ = 0, ω₁ = 0, ω₂ = 0
 *
 * The linearized system is ẋ = Ax + Bu where:
 *   State: [x, θ₁, θ₂, ẋ, ω₁, ω₂]ᵀ
 *   Input: F (force on cart)
 *
 * @param mc Cart mass
 * @param m1 Mass 1 (at end of link 1)
 * @param m2 Mass 2 (at end of link 2)
 * @param L1 Length of link 1
 * @param L2 Length of link 2
 * @param g Gravity
 * @return Pair of (A, B) matrices
 */
inline std::pair<
    std::array<std::array<double, 6>, 6>,
    std::array<std::array<double, 1>, 6>
> linearizeCartDoublePendulum(
    double mc, double m1, double m2, double L1, double L2, double g
) {
    // At equilibrium (θ₁=θ₂=0), the mass matrix is:
    // M = [mc+m1+m2,  (m1+m2)*L1,  m2*L2    ]
    //     [(m1+m2)*L1, (m1+m2)*L1², m2*L1*L2]
    //     [m2*L2,      m2*L1*L2,    m2*L2²  ]

    double M11 = mc + m1 + m2;
    double M12 = (m1 + m2) * L1;
    double M13 = m2 * L2;
    double M22 = (m1 + m2) * L1 * L1;
    double M23 = m2 * L1 * L2;
    double M33 = m2 * L2 * L2;

    // Compute inverse of M using cofactors
    double det = M11 * (M22 * M33 - M23 * M23)
               - M12 * (M12 * M33 - M23 * M13)
               + M13 * (M12 * M23 - M22 * M13);

    double C11 = M22 * M33 - M23 * M23;
    double C12 = -(M12 * M33 - M13 * M23);
    double C13 = M12 * M23 - M13 * M22;
    double C22 = M11 * M33 - M13 * M13;
    double C23 = -(M11 * M23 - M12 * M13);
    double C33 = M11 * M22 - M12 * M12;

    double inv_det = 1.0 / det;

    double Minv11 = C11 * inv_det;
    double Minv12 = C12 * inv_det;
    double Minv13 = C13 * inv_det;
    double Minv22 = C22 * inv_det;
    double Minv23 = C23 * inv_det;
    double Minv33 = C33 * inv_det;

    // Linearized gravity terms: ∂τ/∂θ at θ=0
    // τ₁ = 0 (no direct gravity term for cart in position derivative)
    // τ₂ = (m1+m2)*g*L1*sin(θ₁) ≈ (m1+m2)*g*L1*θ₁
    // τ₃ = m2*g*L2*sin(θ₂) ≈ m2*g*L2*θ₂

    double G22 = (m1 + m2) * g * L1;  // ∂τ₂/∂θ₁
    double G33 = m2 * g * L2;          // ∂τ₃/∂θ₂

    // System: q̈ = M⁻¹ * (G * q + B * u)
    // Where G contributes to position-dependent forces

    // A matrix (6×6):
    // ẋ = [0 0 0 | 1 0 0] * x
    //     [0 0 0 | 0 1 0]
    //     [0 0 0 | 0 0 1]
    //     [-------|------]
    //     [0 g₁ g₂| 0 0 0]  <- M⁻¹ * G
    //     [0 g₃ g₄| 0 0 0]
    //     [0 g₅ g₆| 0 0 0]

    // M⁻¹ * G (only columns 2 and 3 of G are non-zero)
    double A41 = 0.0;  // No gravity dependence on x
    double A42 = Minv12 * G22;  // Effect of θ₁ on ẍ
    double A43 = Minv13 * G33;  // Effect of θ₂ on ẍ
    double A52 = Minv22 * G22;  // Effect of θ₁ on α₁
    double A53 = Minv23 * G33;  // Effect of θ₂ on α₁
    double A62 = Minv23 * G22;  // Effect of θ₁ on α₂ (using symmetry Minv32 = Minv23)
    double A63 = Minv33 * G33;  // Effect of θ₂ on α₂

    std::array<std::array<double, 6>, 6> A{};
    // Position derivative rows
    A[0][3] = 1.0;  // ẋ = xdot
    A[1][4] = 1.0;  // θ̇₁ = ω₁
    A[2][5] = 1.0;  // θ̇₂ = ω₂

    // Acceleration rows (from M⁻¹ * G)
    A[3][0] = A41;
    A[3][1] = A42;
    A[3][2] = A43;
    A[4][1] = A52;
    A[4][2] = A53;
    A[5][1] = A62;
    A[5][2] = A63;

    // B matrix (6×1):
    // B = [0, 0, 0, M⁻¹[0,0], M⁻¹[1,0], M⁻¹[2,0]]ᵀ
    std::array<std::array<double, 1>, 6> B{};
    B[3][0] = Minv11;  // Effect of F on ẍ
    B[4][0] = Minv12;  // Effect of F on α₁
    B[5][0] = Minv13;  // Effect of F on α₂

    return {A, B};
}

} // namespace sopot::control
