#pragma once

#include "expression.hpp"
#include "differentiation.hpp"
#include <tuple>
#include <string_view>

namespace sopot::symbolic {

/**
 * @file named_expression.hpp
 * @brief Named variables for ergonomic constraint definition
 *
 * This provides a user-friendly API for defining constraints using named
 * variables instead of numeric indices. The named expressions compile down
 * to the same efficient indexed types used internally.
 *
 * Example usage:
 *   // Define a constraint context with named variables
 *   constexpr auto [x1, y1, x2, y2] = make_symbols<4>("x1", "y1", "x2", "y2");
 *
 *   // Build constraints naturally
 *   auto rod1 = x1*x1 + y1*y1;           // Length constraint for rod 1
 *   auto rod2 = sq(x2-x1) + sq(y2-y1);   // Length constraint for rod 2
 *
 *   // Evaluate with values
 *   std::array<double, 4> pos = {0.6, -0.8, 1.4, -1.4};
 *   double g1 = rod1.eval(pos);
 *
 *   // Get Jacobian automatically
 *   auto J = jacobian<4>(rod1, rod2);
 */

// ============================================================================
// Named Symbol - A variable with both a name and compile-time index
// ============================================================================

/**
 * @brief A named symbolic variable that maps to Var<Index>
 * @tparam Index The compile-time variable index
 */
template<size_t Index>
struct NamedSymbol {
    static constexpr size_t index = Index;
    const char* name;

    constexpr NamedSymbol(const char* n = "") : name(n) {}

    // Convert to underlying Var type for expression building
    using var_type = Var<Index>;
};

// ============================================================================
// Expression Wrapper - Holds expression type with convenient methods
// ============================================================================

/**
 * @brief Wrapper that holds an expression type and provides evaluation methods
 * @tparam E The underlying expression type (Add, Mul, Var, etc.)
 */
template<typename E>
struct Expr {
    using type = E;

    constexpr Expr() = default;

    // Evaluate with variable values
    template<typename T, size_t N>
    T eval(const std::array<T, N>& vars) const {
        return symbolic::eval<E>(vars);
    }

    // Get the expression type (for template metaprogramming)
    using expression_type = E;
};

// ============================================================================
// Expression Building - Operators that return Expr<...> wrappers
// ============================================================================

// Symbol + Symbol
template<size_t I, size_t J>
constexpr auto operator+(NamedSymbol<I>, NamedSymbol<J>) {
    return Expr<Add<Var<I>, Var<J>>>{};
}

// Symbol - Symbol
template<size_t I, size_t J>
constexpr auto operator-(NamedSymbol<I>, NamedSymbol<J>) {
    return Expr<Sub<Var<I>, Var<J>>>{};
}

// Symbol * Symbol
template<size_t I, size_t J>
constexpr auto operator*(NamedSymbol<I>, NamedSymbol<J>) {
    return Expr<Mul<Var<I>, Var<J>>>{};
}

// Symbol / Symbol
template<size_t I, size_t J>
constexpr auto operator/(NamedSymbol<I>, NamedSymbol<J>) {
    return Expr<Div<Var<I>, Var<J>>>{};
}

// -Symbol
template<size_t I>
constexpr auto operator-(NamedSymbol<I>) {
    return Expr<Neg<Var<I>>>{};
}

// Expr + Expr
template<typename L, typename R>
constexpr auto operator+(Expr<L>, Expr<R>) {
    return Expr<Add<L, R>>{};
}

// Expr - Expr
template<typename L, typename R>
constexpr auto operator-(Expr<L>, Expr<R>) {
    return Expr<Sub<L, R>>{};
}

// Expr * Expr
template<typename L, typename R>
constexpr auto operator*(Expr<L>, Expr<R>) {
    return Expr<Mul<L, R>>{};
}

// Expr / Expr
template<typename L, typename R>
constexpr auto operator/(Expr<L>, Expr<R>) {
    return Expr<Div<L, R>>{};
}

// -Expr
template<typename E>
constexpr auto operator-(Expr<E>) {
    return Expr<Neg<E>>{};
}

// Symbol + Expr
template<size_t I, typename E>
constexpr auto operator+(NamedSymbol<I>, Expr<E>) {
    return Expr<Add<Var<I>, E>>{};
}

template<typename E, size_t I>
constexpr auto operator+(Expr<E>, NamedSymbol<I>) {
    return Expr<Add<E, Var<I>>>{};
}

// Symbol - Expr
template<size_t I, typename E>
constexpr auto operator-(NamedSymbol<I>, Expr<E>) {
    return Expr<Sub<Var<I>, E>>{};
}

template<typename E, size_t I>
constexpr auto operator-(Expr<E>, NamedSymbol<I>) {
    return Expr<Sub<E, Var<I>>>{};
}

// Symbol * Expr
template<size_t I, typename E>
constexpr auto operator*(NamedSymbol<I>, Expr<E>) {
    return Expr<Mul<Var<I>, E>>{};
}

template<typename E, size_t I>
constexpr auto operator*(Expr<E>, NamedSymbol<I>) {
    return Expr<Mul<E, Var<I>>>{};
}

// Symbol / Expr
template<size_t I, typename E>
constexpr auto operator/(NamedSymbol<I>, Expr<E>) {
    return Expr<Div<Var<I>, E>>{};
}

template<typename E, size_t I>
constexpr auto operator/(Expr<E>, NamedSymbol<I>) {
    return Expr<Div<E, Var<I>>>{};
}

// ============================================================================
// Constant expressions
// ============================================================================

// Compile-time constant wrapper
template<int N, int D = 1>
struct ConstExpr : Expr<Const<N, D>> {
    static constexpr int numerator = N;
    static constexpr int denominator = D;
};

inline constexpr ConstExpr<0> zero{};
inline constexpr ConstExpr<1> one{};
inline constexpr ConstExpr<2> two{};

// Expr + Const
template<typename E, int N, int D>
constexpr auto operator+(Expr<E>, ConstExpr<N, D>) {
    return Expr<Add<E, Const<N, D>>>{};
}

template<int N, int D, typename E>
constexpr auto operator+(ConstExpr<N, D>, Expr<E>) {
    return Expr<Add<Const<N, D>, E>>{};
}

// Expr - Const
template<typename E, int N, int D>
constexpr auto operator-(Expr<E>, ConstExpr<N, D>) {
    return Expr<Sub<E, Const<N, D>>>{};
}

// Expr * Const
template<typename E, int N, int D>
constexpr auto operator*(Expr<E>, ConstExpr<N, D>) {
    return Expr<Mul<E, Const<N, D>>>{};
}

template<int N, int D, typename E>
constexpr auto operator*(ConstExpr<N, D>, Expr<E>) {
    return Expr<Mul<Const<N, D>, E>>{};
}

// ============================================================================
// Mathematical Functions
// ============================================================================

// Square function: sq(x) = x^2
template<size_t I>
constexpr auto sq(NamedSymbol<I>) {
    return Expr<Square<Var<I>>>{};
}

template<typename E>
constexpr auto sq(Expr<E>) {
    return Expr<Square<E>>{};
}

// Sine
template<size_t I>
constexpr auto sin(NamedSymbol<I>) {
    return Expr<Sin<Var<I>>>{};
}

template<typename E>
constexpr auto sin(Expr<E>) {
    return Expr<Sin<E>>{};
}

// Cosine
template<size_t I>
constexpr auto cos(NamedSymbol<I>) {
    return Expr<Cos<Var<I>>>{};
}

template<typename E>
constexpr auto cos(Expr<E>) {
    return Expr<Cos<E>>{};
}

// Square root
template<size_t I>
constexpr auto sqrt(NamedSymbol<I>) {
    return Expr<Sqrt<Var<I>>>{};
}

template<typename E>
constexpr auto sqrt(Expr<E>) {
    return Expr<Sqrt<E>>{};
}

// ============================================================================
// Symbol Factory - Create named symbols with sequential indices
// ============================================================================

namespace detail {

template<size_t... Is>
constexpr auto make_symbols_impl(std::index_sequence<Is...>, const char* const* names) {
    return std::make_tuple(NamedSymbol<Is>{names[Is]}...);
}

} // namespace detail

/**
 * @brief Create N named symbols with sequential indices 0, 1, 2, ...
 *
 * Usage:
 *   constexpr const char* names[] = {"x", "y", "vx", "vy"};
 *   auto [x, y, vx, vy] = make_symbols<4>(names);
 */
template<size_t N>
constexpr auto make_symbols(const char* const (&names)[N]) {
    return detail::make_symbols_impl(std::make_index_sequence<N>{}, names);
}

/**
 * @brief Create named symbols using variadic arguments
 *
 * Usage:
 *   auto [x1, y1, x2, y2] = symbols<4>("x1", "y1", "x2", "y2");
 */
template<size_t N, typename... Names>
constexpr auto symbols(Names... names) {
    static_assert(sizeof...(Names) == N, "Number of names must match N");
    const char* name_array[] = {names...};
    return detail::make_symbols_impl(std::make_index_sequence<N>{}, name_array);
}

// ============================================================================
// Predefined Symbol Sets for Common Physics Scenarios
// ============================================================================

namespace cartesian {

// 2D single particle: (x, y, vx, vy)
namespace particle_2d {
    inline constexpr NamedSymbol<0> x{"x"};
    inline constexpr NamedSymbol<1> y{"y"};
    inline constexpr NamedSymbol<2> vx{"vx"};
    inline constexpr NamedSymbol<3> vy{"vy"};
}

// 2D two-particle system: (x1, y1, x2, y2)
namespace two_body_2d {
    inline constexpr NamedSymbol<0> x1{"x1"};
    inline constexpr NamedSymbol<1> y1{"y1"};
    inline constexpr NamedSymbol<2> x2{"x2"};
    inline constexpr NamedSymbol<3> y2{"y2"};
}

// 2D two-particle with velocities: (x1, y1, x2, y2, vx1, vy1, vx2, vy2)
namespace two_body_2d_full {
    inline constexpr NamedSymbol<0> x1{"x1"};
    inline constexpr NamedSymbol<1> y1{"y1"};
    inline constexpr NamedSymbol<2> x2{"x2"};
    inline constexpr NamedSymbol<3> y2{"y2"};
    inline constexpr NamedSymbol<4> vx1{"vx1"};
    inline constexpr NamedSymbol<5> vy1{"vy1"};
    inline constexpr NamedSymbol<6> vx2{"vx2"};
    inline constexpr NamedSymbol<7> vy2{"vy2"};
}

} // namespace cartesian

namespace generalized {

// Double pendulum angles: (theta1, theta2, omega1, omega2)
namespace pendulum {
    inline constexpr NamedSymbol<0> theta1{"θ1"};
    inline constexpr NamedSymbol<1> theta2{"θ2"};
    inline constexpr NamedSymbol<2> omega1{"ω1"};
    inline constexpr NamedSymbol<3> omega2{"ω2"};
}

} // namespace generalized

// ============================================================================
// Jacobian Builder - Create Jacobian from named expressions
// ============================================================================

/**
 * @brief Build a Jacobian type from multiple constraint expressions
 *
 * Usage:
 *   using namespace cartesian::two_body_2d;
 *   auto g1 = sq(x1) + sq(y1);
 *   auto g2 = sq(x2-x1) + sq(y2-y1);
 *
 *   using J = ConstraintJacobian<4, decltype(g1)::type, decltype(g2)::type>;
 *   auto jacobian = J::eval(position_array);
 */
template<size_t NumVars, typename... Constraints>
using ConstraintJacobian = Jacobian<NumVars, typename Constraints::type...>;

/**
 * @brief Evaluate Jacobian from Expr wrappers
 */
template<size_t NumVars, typename T, typename... Exprs>
auto eval_jacobian(const std::array<T, NumVars>& vars, Exprs...) {
    using JacobianType = Jacobian<NumVars, typename Exprs::type...>;
    return JacobianType::eval(vars);
}

/**
 * @brief Evaluate gradient from a single Expr
 */
template<size_t NumVars, typename T, typename E>
auto eval_gradient(const std::array<T, NumVars>& vars, Expr<E>) {
    return Gradient<E, NumVars>::eval(vars);
}

} // namespace sopot::symbolic
