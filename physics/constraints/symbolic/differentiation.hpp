#pragma once

#include "expression.hpp"
#include <array>

namespace sopot::symbolic {

/**
 * @file differentiation.hpp
 * @brief Compile-time symbolic differentiation
 *
 * Implements automatic symbolic differentiation using template metaprogramming.
 * All differentiation happens at compile time - the derivative expression is
 * computed as a type, and can then be evaluated at runtime.
 *
 * Example:
 *   using x = Var<0>;
 *   using y = Var<1>;
 *   using f = Mul<x, Sin<y>>;      // f = x * sin(y)
 *   using df_dx = Diff_t<f, 0>;    // df/dx = sin(y)
 *   using df_dy = Diff_t<f, 1>;    // df/dy = x * cos(y)
 */

// ============================================================================
// Forward declarations
// ============================================================================

template<typename E, size_t I>
struct Diff;

template<typename E, size_t I>
using Diff_t = typename Diff<E, I>::type;

// ============================================================================
// Differentiation Rules
// ============================================================================

/**
 * @brief d/dx_i(x_j) = delta_ij (Kronecker delta)
 */
template<size_t J, size_t I>
struct Diff<Var<J>, I> {
    using type = std::conditional_t<I == J, One, Zero>;
};

/**
 * @brief d/dx_i(constant) = 0
 */
template<int N, int D, size_t I>
struct Diff<Const<N, D>, I> {
    using type = Zero;
};

/**
 * @brief d/dx_i(param) = 0 (parameters are constants)
 */
template<size_t ID, size_t I>
struct Diff<Param<ID>, I> {
    using type = Zero;
};

/**
 * @brief d/dx_i(f + g) = df/dx_i + dg/dx_i (sum rule)
 */
template<typename L, typename R, size_t I>
struct Diff<Add<L, R>, I> {
    using type = Simplify_t<Add<Diff_t<L, I>, Diff_t<R, I>>>;
};

/**
 * @brief d/dx_i(f - g) = df/dx_i - dg/dx_i (difference rule)
 */
template<typename L, typename R, size_t I>
struct Diff<Sub<L, R>, I> {
    using type = Simplify_t<Sub<Diff_t<L, I>, Diff_t<R, I>>>;
};

/**
 * @brief d/dx_i(f * g) = f * dg/dx_i + df/dx_i * g (product rule)
 */
template<typename L, typename R, size_t I>
struct Diff<Mul<L, R>, I> {
private:
    using dL = Diff_t<L, I>;
    using dR = Diff_t<R, I>;
    using term1 = Mul<L, dR>;     // f * dg
    using term2 = Mul<dL, R>;     // df * g
public:
    using type = Simplify_t<Add<term1, term2>>;
};

/**
 * @brief d/dx_i(f / g) = (g * df/dx_i - f * dg/dx_i) / g^2 (quotient rule)
 */
template<typename L, typename R, size_t I>
struct Diff<Div<L, R>, I> {
private:
    using dL = Diff_t<L, I>;
    using dR = Diff_t<R, I>;
    using numerator = Sub<Mul<R, dL>, Mul<L, dR>>;  // g*df - f*dg
    using denominator = Pow<R, 2>;                   // g^2
public:
    using type = Simplify_t<Div<numerator, denominator>>;
};

/**
 * @brief d/dx_i(-f) = -df/dx_i
 */
template<typename E, size_t I>
struct Diff<Neg<E>, I> {
    using type = Simplify_t<Neg<Diff_t<E, I>>>;
};

/**
 * @brief d/dx_i(f^n) = n * f^(n-1) * df/dx_i (power rule)
 */
template<typename E, int N, size_t I>
struct Diff<Pow<E, N>, I> {
private:
    using dE = Diff_t<E, I>;
    // n * f^(n-1) * df
    using coeff = Const<N>;
    using power_term = Pow<E, N - 1>;
    using chain = Mul<coeff, Mul<power_term, dE>>;
public:
    using type = Simplify_t<chain>;
};

// Special case: d/dx(f^0) = 0
template<typename E, size_t I>
struct Diff<Pow<E, 0>, I> {
    using type = Zero;
};

// Special case: d/dx(f^1) = df/dx
template<typename E, size_t I>
struct Diff<Pow<E, 1>, I> {
    using type = Diff_t<E, I>;
};

/**
 * @brief d/dx_i(sin(f)) = cos(f) * df/dx_i (chain rule)
 */
template<typename E, size_t I>
struct Diff<Sin<E>, I> {
private:
    using dE = Diff_t<E, I>;
public:
    using type = Simplify_t<Mul<Cos<E>, dE>>;
};

/**
 * @brief d/dx_i(cos(f)) = -sin(f) * df/dx_i (chain rule)
 */
template<typename E, size_t I>
struct Diff<Cos<E>, I> {
private:
    using dE = Diff_t<E, I>;
public:
    using type = Simplify_t<Neg<Mul<Sin<E>, dE>>>;
};

/**
 * @brief d/dx_i(sqrt(f)) = df/dx_i / (2 * sqrt(f))
 */
template<typename E, size_t I>
struct Diff<Sqrt<E>, I> {
private:
    using dE = Diff_t<E, I>;
    // 1 / (2 * sqrt(f))
    using two_sqrt = Mul<Two, Sqrt<E>>;
public:
    using type = Simplify_t<Div<dE, two_sqrt>>;
};

// ============================================================================
// Gradient and Jacobian computation
// ============================================================================

/**
 * @brief Compute gradient at compile time: [df/dx_0, df/dx_1, ...]
 * @tparam E Expression type
 * @tparam NumVars Number of variables
 */
template<typename E, size_t NumVars>
struct Gradient {
    /**
     * @brief Evaluate gradient at given point
     */
    template<typename T>
    static std::array<T, NumVars> eval(const std::array<T, NumVars>& vars) {
        return eval_impl(vars, std::make_index_sequence<NumVars>{});
    }

    /**
     * @brief Evaluate gradient with parameters
     */
    template<typename T, size_t P>
    static std::array<T, NumVars> eval(const std::array<T, NumVars>& vars,
                                        const std::array<T, P>& params) {
        return eval_impl(vars, params, std::make_index_sequence<NumVars>{});
    }

private:
    template<typename T, size_t... Is>
    static std::array<T, NumVars> eval_impl(const std::array<T, NumVars>& vars,
                                             std::index_sequence<Is...>) {
        return {symbolic::eval<Diff_t<E, Is>>(vars)...};
    }

    template<typename T, size_t P, size_t... Is>
    static std::array<T, NumVars> eval_impl(const std::array<T, NumVars>& vars,
                                             const std::array<T, P>& params,
                                             std::index_sequence<Is...>) {
        return {symbolic::eval<Diff_t<E, Is>>(vars, params)...};
    }
};

/**
 * @brief Jacobian row generator for constraints
 *
 * Given a constraint g(q) = 0, this generates the row [dg/dq_0, dg/dq_1, ...]
 */
template<typename Constraint, size_t NumVars>
struct JacobianRow {
    template<typename T>
    static std::array<T, NumVars> eval(const std::array<T, NumVars>& vars) {
        return Gradient<Constraint, NumVars>::eval(vars);
    }

    template<typename T, size_t P>
    static std::array<T, NumVars> eval(const std::array<T, NumVars>& vars,
                                        const std::array<T, P>& params) {
        return Gradient<Constraint, NumVars>::eval(vars, params);
    }
};

/**
 * @brief Full Jacobian for multiple constraints
 * @tparam NumVars Number of variables
 * @tparam Constraints... Constraint expression types
 */
template<size_t NumVars, typename... Constraints>
struct Jacobian {
    static constexpr size_t num_constraints = sizeof...(Constraints);
    static constexpr size_t num_vars = NumVars;

    /**
     * @brief Evaluate full Jacobian matrix
     * @return Array of rows, each row is an array of derivatives
     */
    template<typename T>
    static std::array<std::array<T, NumVars>, num_constraints>
    eval(const std::array<T, NumVars>& vars) {
        return {JacobianRow<Constraints, NumVars>::eval(vars)...};
    }

    template<typename T, size_t P>
    static std::array<std::array<T, NumVars>, num_constraints>
    eval(const std::array<T, NumVars>& vars, const std::array<T, P>& params) {
        return {JacobianRow<Constraints, NumVars>::eval(vars, params)...};
    }
};

/**
 * @brief Time derivative of constraint (constraint velocity)
 *
 * For g(q), computes dg/dt = sum_i (dg/dq_i * dq_i/dt) = grad(g) . q_dot
 */
template<typename Constraint, size_t NumVars>
struct ConstraintVelocity {
    /**
     * @brief Evaluate constraint velocity
     * @param q Position variables
     * @param q_dot Velocity variables
     */
    template<typename T>
    static T eval(const std::array<T, NumVars>& q, const std::array<T, NumVars>& q_dot) {
        auto grad = Gradient<Constraint, NumVars>::eval(q);
        T result = T(0);
        for (size_t i = 0; i < NumVars; ++i) {
            result = result + grad[i] * q_dot[i];
        }
        return result;
    }
};

// ============================================================================
// Expression printing (for debugging)
// ============================================================================

// Forward declaration
template<typename E>
struct ExprName;

template<size_t I>
struct ExprName<Var<I>> {
    static constexpr const char* prefix = "x";
    static constexpr size_t index = I;
};

template<int N, int D>
struct ExprName<Const<N, D>> {
    static constexpr int num = N;
    static constexpr int den = D;
};

// ============================================================================
// Hessian computation (second derivatives)
// ============================================================================

/**
 * @brief Second partial derivative d²f/(dx_i dx_j)
 */
template<typename E, size_t I, size_t J>
using Diff2_t = Diff_t<Diff_t<E, I>, J>;

/**
 * @brief Hessian matrix for a scalar expression
 */
template<typename E, size_t NumVars>
struct Hessian {
    template<typename T>
    static std::array<std::array<T, NumVars>, NumVars>
    eval(const std::array<T, NumVars>& vars) {
        return eval_impl(vars, std::make_index_sequence<NumVars>{});
    }

private:
    template<typename T, size_t... Is>
    static std::array<std::array<T, NumVars>, NumVars>
    eval_impl(const std::array<T, NumVars>& vars, std::index_sequence<Is...>) {
        return {eval_row<T, Is>(vars)...};
    }

    template<typename T, size_t I>
    static std::array<T, NumVars> eval_row(const std::array<T, NumVars>& vars) {
        return eval_row_impl<T, I>(vars, std::make_index_sequence<NumVars>{});
    }

    template<typename T, size_t I, size_t... Js>
    static std::array<T, NumVars> eval_row_impl(const std::array<T, NumVars>& vars,
                                                 std::index_sequence<Js...>) {
        return {symbolic::eval<Diff2_t<E, I, Js>>(vars)...};
    }
};

} // namespace sopot::symbolic
