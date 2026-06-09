#pragma once

#include "core/typed_component.hpp"
#include "core/scalar.hpp"
#include "control_tags.hpp"
#include <string>
#include <stdexcept>
#include <array>
#include <cmath>

namespace sopot::control {

/**
 * @brief Cart with an inverted N-link pendulum chain.
 *
 * This is the generalization of the classic "cart-pole" problem to an
 * arbitrary number of serially-connected pendulum links balanced on a
 * single actuated cart. With @p NLinks == 1 it is the textbook cart-pole;
 * with @p NLinks == 2 it reproduces the cart-double-pendulum; the project
 * configures it with @p NLinks == 6 for the six-pendulum demonstration.
 *
 * Generalized coordinates: q = [x, θ₁, θ₂, …, θ_N]
 *   - x:   cart position (m)
 *   - θᵢ:  angle of link i measured from vertical (rad), 0 = upright
 *
 * Full state vector (size 2·(N+1)):
 *   [x, θ₁ … θ_N, ẋ, ω₁ … ω_N]
 *
 * Control input: horizontal force F applied to the cart (N).
 *
 * Each link i carries a point mass mᵢ at its tip. Positions (relative to the
 * ground frame, y pointing up, θ = 0 upright):
 *   xᵢ = x + Σ_{j≤i} Lⱼ sin θⱼ
 *   yᵢ =     Σ_{j≤i} Lⱼ cos θⱼ
 *
 * Equations of motion are derived from the Lagrangian and assembled as
 *   M(q) q̈ = rhs(q, q̇, F)
 * with the (N+1)×(N+1) symmetric mass matrix and right-hand side
 *   M₀₀          = m_cart + Σ mᵢ
 *   M₀ⱼ = Mⱼ₀    = M̄ⱼ Lⱼ cos θⱼ
 *   Mⱼₖ          = M̄_{max(j,k)} Lⱼ Lₖ cos(θⱼ − θₖ)
 *   rhs₀         = F + Σ M̄ⱼ Lⱼ sin θⱼ ωⱼ²
 *   rhsⱼ         = M̄ⱼ g Lⱼ sin θⱼ − Σₖ M̄_{max(j,k)} Lⱼ Lₖ sin(θⱼ − θₖ) ωₖ²
 * where M̄ⱼ = Σ_{i≥j} mᵢ is the "tail mass" carried by joint j.
 * The (N+1)×(N+1) linear system is solved with Gaussian elimination, which
 * works for both plain `double` and autodiff `Dual` scalar types.
 */
template<size_t NLinks, Scalar T = double>
class CartNPendulum final : public TypedComponent<2 * (NLinks + 1), T> {
public:
    static_assert(NLinks >= 1, "CartNPendulum requires at least one link");

    static constexpr size_t num_links = NLinks;
    static constexpr size_t num_dof   = NLinks + 1;   // cart + N angles
    static constexpr size_t num_state = 2 * num_dof;  // positions + velocities

    using Base = TypedComponent<num_state, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    double m_cart_mass;                       // mc: cart mass (kg)
    std::array<double, NLinks> m_mass;        // mᵢ: point mass at link i tip (kg)
    std::array<double, NLinks> m_length;      // Lᵢ: length of link i (m)
    double m_gravity;                         // g: gravity (m/s²)

    std::array<double, num_state> m_initial_state;  // [x, θ…, ẋ, ω…]

    // Control input (set externally by the controller before each derivative eval)
    mutable T m_control_force = T(0.0);

    std::string m_name;

    // Indices into the state / generalized-coordinate vectors.
    static constexpr size_t idxX()        { return 0; }
    static constexpr size_t idxTheta(size_t link) { return 1 + link; }
    static constexpr size_t idxXDot()     { return num_dof; }
    static constexpr size_t idxOmega(size_t link) { return num_dof + 1 + link; }

public:
    /**
     * @brief Construct an N-link cart pendulum.
     * @param cart_mass     Cart mass (kg)
     * @param link_masses   Point mass at each link tip (kg)
     * @param link_lengths  Length of each link (m)
     * @param gravity       Gravitational acceleration (m/s²)
     * @param initial_state Full initial state [x, θ₁…θ_N, ẋ, ω₁…ω_N]
     */
    CartNPendulum(
        double cart_mass,
        const std::array<double, NLinks>& link_masses,
        const std::array<double, NLinks>& link_lengths,
        double gravity = 9.81,
        const std::array<double, num_state>& initial_state = {}
    )
        : m_cart_mass(cart_mass)
        , m_mass(link_masses)
        , m_length(link_lengths)
        , m_gravity(gravity)
        , m_initial_state(initial_state)
        , m_name("CartNPendulum")
    {
        if (cart_mass <= 0.0) {
            throw std::invalid_argument("Cart mass must be positive");
        }
        for (size_t i = 0; i < NLinks; ++i) {
            if (m_mass[i] <= 0.0) {
                throw std::invalid_argument("All link masses must be positive");
            }
            if (m_length[i] <= 0.0) {
                throw std::invalid_argument("All link lengths must be positive");
            }
        }
    }

    // Set control input (force on cart)
    void setControlForce(T force) const { m_control_force = force; }
    T getControlForce() const { return m_control_force; }

    LocalState getInitialLocalState() const {
        LocalState s;
        for (size_t i = 0; i < num_state; ++i) {
            s[i] = T(m_initial_state[i]);
        }
        return s;
    }

    std::string_view getComponentType() const { return "CartNPendulum"; }
    std::string_view getComponentName() const { return m_name; }

    /**
     * @brief Tail mass M̄ⱼ = Σ_{i≥j} mᵢ carried by joint of link @p link.
     */
    double tailMass(size_t link) const {
        double sum = 0.0;
        for (size_t i = link; i < NLinks; ++i) sum += m_mass[i];
        return sum;
    }

    /**
     * @brief Compute state derivatives via the Euler-Lagrange equations.
     */
    template<typename Registry>
    LocalDerivative derivatives(
        T /*t*/,
        std::span<const T> local,
        std::span<const T> /*global*/,
        const Registry& /*registry*/
    ) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;

        const T g = T(m_gravity);
        const T F = m_control_force;

        // Cache trig of each link angle.
        std::array<T, NLinks> s{}, c{};
        std::array<T, NLinks> omega{};
        for (size_t i = 0; i < NLinks; ++i) {
            T theta = local[idxTheta(i)];
            s[i] = sin(theta);
            c[i] = cos(theta);
            omega[i] = local[idxOmega(i)];
        }

        // Tail masses (constant per configuration).
        std::array<double, NLinks> Mtail{};
        for (size_t i = 0; i < NLinks; ++i) Mtail[i] = tailMass(i);

        // Assemble the symmetric (N+1)×(N+1) mass matrix M and rhs vector.
        std::array<std::array<T, num_dof>, num_dof> M{};
        std::array<T, num_dof> rhs{};

        // Cart row/column.
        T total_mass = T(m_cart_mass);
        for (size_t i = 0; i < NLinks; ++i) total_mass = total_mass + T(m_mass[i]);
        M[0][0] = total_mass;

        rhs[0] = F;
        for (size_t a = 0; a < NLinks; ++a) {
            T coupling = T(Mtail[a]) * T(m_length[a]) * c[a];
            M[0][idxTheta(a)] = coupling;
            M[idxTheta(a)][0] = coupling;
            rhs[0] = rhs[0] + T(Mtail[a]) * T(m_length[a]) * s[a] * omega[a] * omega[a];
        }

        // Angle rows/columns.
        for (size_t a = 0; a < NLinks; ++a) {
            // Gravity term.
            T rhs_a = T(Mtail[a]) * g * T(m_length[a]) * s[a];

            for (size_t b = 0; b < NLinks; ++b) {
                size_t mx = (a > b) ? a : b;
                T cab = c[a] * c[b] + s[a] * s[b];                 // cos(θa − θb)
                M[idxTheta(a)][idxTheta(b)] =
                    T(Mtail[mx]) * T(m_length[a]) * T(m_length[b]) * cab;

                if (b != a) {
                    T sab = s[a] * c[b] - c[a] * s[b];             // sin(θa − θb)
                    rhs_a = rhs_a - T(Mtail[mx]) * T(m_length[a]) * T(m_length[b])
                                    * sab * omega[b] * omega[b];
                }
            }
            rhs[idxTheta(a)] = rhs_a;
        }

        // Solve M · accel = rhs (Gaussian elimination with partial pivoting).
        std::array<T, num_dof> accel = solveLinearSystem(M, rhs);

        // Assemble derivative: position rates = velocities, velocity rates = accel.
        LocalDerivative deriv;
        deriv[idxX()] = local[idxXDot()];
        for (size_t i = 0; i < NLinks; ++i) {
            deriv[idxTheta(i)] = local[idxOmega(i)];
        }
        deriv[idxXDot()] = accel[0];
        for (size_t i = 0; i < NLinks; ++i) {
            deriv[idxOmega(i)] = accel[idxTheta(i)];
        }
        return deriv;
    }

    // =========================================================================
    // State queries (work on the global state span)
    // =========================================================================

    T cartPosition(std::span<const T> state) const { return this->getGlobalState(state, idxX()); }
    T cartVelocity(std::span<const T> state) const { return this->getGlobalState(state, idxXDot()); }

    T linkAngle(size_t link, std::span<const T> state) const {
        return this->getGlobalState(state, idxTheta(link));
    }
    T linkAngularVelocity(size_t link, std::span<const T> state) const {
        return this->getGlobalState(state, idxOmega(link));
    }

    /**
     * @brief Cartesian [x, y] position of the tip of link @p link.
     */
    std::array<T, 2> linkTipPosition(size_t link, std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;
        T x = this->getGlobalState(state, idxX());
        T px = x;
        T py = T(0.0);
        for (size_t i = 0; i <= link; ++i) {
            T theta = this->getGlobalState(state, idxTheta(i));
            px = px + T(m_length[i]) * sin(theta);
            py = py + T(m_length[i]) * cos(theta);
        }
        return {px, py};
    }

    T kineticEnergy(std::span<const T> state) const {
        using std::sin; using std::cos;
        using sopot::sin; using sopot::cos;

        T xdot = this->getGlobalState(state, idxXDot());

        // Cart kinetic energy.
        T KE = T(0.5) * T(m_cart_mass) * xdot * xdot;

        // Each point mass: accumulate the chain velocity contributions.
        T vx_accum = xdot;  // running Σ Lⱼ cosθⱼ ωⱼ added to ẋ
        T vy_accum = T(0.0);
        for (size_t i = 0; i < NLinks; ++i) {
            T theta = this->getGlobalState(state, idxTheta(i));
            T omega = this->getGlobalState(state, idxOmega(i));
            vx_accum = vx_accum + T(m_length[i]) * cos(theta) * omega;
            vy_accum = vy_accum - T(m_length[i]) * sin(theta) * omega;
            KE = KE + T(0.5) * T(m_mass[i]) * (vx_accum * vx_accum + vy_accum * vy_accum);
        }
        return KE;
    }

    T potentialEnergy(std::span<const T> state) const {
        using std::cos;
        using sopot::cos;
        T g = T(m_gravity);
        T height_accum = T(0.0);
        T PE = T(0.0);
        for (size_t i = 0; i < NLinks; ++i) {
            T theta = this->getGlobalState(state, idxTheta(i));
            height_accum = height_accum + T(m_length[i]) * cos(theta);
            PE = PE + T(m_mass[i]) * g * height_accum;
        }
        return PE;
    }

    T totalEnergy(std::span<const T> state) const {
        return kineticEnergy(state) + potentialEnergy(state);
    }

    // -------------------------------------------------------------------------
    // Minimal tag-based interface (cart only; angles are queried per-index above)
    // -------------------------------------------------------------------------
    T compute(cart::Position, std::span<const T> state) const { return cartPosition(state); }
    T compute(cart::Velocity, std::span<const T> state) const { return cartVelocity(state); }
    T compute(cart::Mass, std::span<const T> /*state*/) const { return T(m_cart_mass); }

    // Controller interface
    T compute(controller::ControlInput, std::span<const T> /*state*/) const { return m_control_force; }

    // Parameter accessors
    double getCartMass() const { return m_cart_mass; }
    double getMass(size_t link) const { return m_mass[link]; }
    double getLength(size_t link) const { return m_length[link]; }
    const std::array<double, NLinks>& getMasses() const { return m_mass; }
    const std::array<double, NLinks>& getLengths() const { return m_length; }
    double getGravity() const { return m_gravity; }

private:
    /**
     * @brief Solve A·x = b for the dense (N+1)×(N+1) system using Gaussian
     *        elimination with partial pivoting. Templated on the scalar type
     *        so it works with both `double` and autodiff `Dual` numbers
     *        (pivot selection compares the underlying real values).
     */
    static std::array<T, num_dof> solveLinearSystem(
        std::array<std::array<T, num_dof>, num_dof> A,
        std::array<T, num_dof> b
    ) {
        for (size_t col = 0; col < num_dof; ++col) {
            // Partial pivot: largest |value| in this column.
            size_t pivot = col;
            double best = std::abs(value_of(A[col][col]));
            for (size_t row = col + 1; row < num_dof; ++row) {
                double mag = std::abs(value_of(A[row][col]));
                if (mag > best) { best = mag; pivot = row; }
            }
            if (best < 1e-15) {
                throw std::runtime_error("CartNPendulum: mass matrix is singular; cannot solve for accelerations.");
            }
            if (pivot != col) {
                std::swap(A[col], A[pivot]);
                std::swap(b[col], b[pivot]);
            }

            // Eliminate below the pivot.
            for (size_t row = col + 1; row < num_dof; ++row) {
                T factor = A[row][col] / A[col][col];
                for (size_t j = col; j < num_dof; ++j) {
                    A[row][j] = A[row][j] - factor * A[col][j];
                }
                b[row] = b[row] - factor * b[col];
            }
        }

        // Back substitution.
        std::array<T, num_dof> x{};
        for (size_t i = num_dof; i-- > 0; ) {
            T sum = b[i];
            for (size_t j = i + 1; j < num_dof; ++j) {
                sum = sum - A[i][j] * x[j];
            }
            x[i] = sum / A[i][i];
        }
        return x;
    }
};

} // namespace sopot::control
