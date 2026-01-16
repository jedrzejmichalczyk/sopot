#pragma once

#include <array>
#include <cmath>
#include <type_traits>

namespace sopot::symbolic {

/**
 * @file expression.hpp
 * @brief Compile-time expression templates for symbolic mathematics
 *
 * This implements a compile-time computer algebra system (CAS) using
 * expression templates. All expression types are stateless (zero-size)
 * and the entire expression tree is encoded in the type system.
 *
 * Example usage:
 *   using x = Var<0>;          // First variable
 *   using y = Var<1>;          // Second variable
 *   using expr = Add<Mul<x, x>, y>;  // x^2 + y
 *
 *   std::array<double, 2> vars = {3.0, 2.0};
 *   double result = eval<expr>(vars);  // = 3^2 + 2 = 11
 */

// ============================================================================
// Expression Types (all stateless, zero-size)
// ============================================================================

/**
 * @brief Variable expression: represents x_I
 * @tparam I Variable index (0-based)
 */
template<size_t I>
struct Var {
    static constexpr size_t index = I;
};

/**
 * @brief Rational constant: N/D
 * @tparam N Numerator
 * @tparam D Denominator (default 1)
 */
template<int N, int D = 1>
struct Const {
    static_assert(D != 0, "Denominator cannot be zero");
    static constexpr int numerator = N;
    static constexpr int denominator = D;
    static constexpr double value = static_cast<double>(N) / static_cast<double>(D);
};

// Common constants
using Zero = Const<0>;
using One = Const<1>;
using Two = Const<2>;
using Half = Const<1, 2>;
using NegOne = Const<-1>;

/**
 * @brief Runtime constant (for parameters like mass, length, etc.)
 * @tparam ID Unique identifier for this parameter
 */
template<size_t ID>
struct Param {
    static constexpr size_t id = ID;
};

/**
 * @brief Addition: L + R
 */
template<typename L, typename R>
struct Add {
    using Left = L;
    using Right = R;
};

/**
 * @brief Subtraction: L - R
 */
template<typename L, typename R>
struct Sub {
    using Left = L;
    using Right = R;
};

/**
 * @brief Multiplication: L * R
 */
template<typename L, typename R>
struct Mul {
    using Left = L;
    using Right = R;
};

/**
 * @brief Division: L / R
 */
template<typename L, typename R>
struct Div {
    using Left = L;
    using Right = R;
};

/**
 * @brief Negation: -E
 */
template<typename E>
struct Neg {
    using Expr = E;
};

/**
 * @brief Power: E^N (compile-time integer exponent)
 */
template<typename E, int N>
struct Pow {
    using Base = E;
    static constexpr int exponent = N;
};

/**
 * @brief Square: E^2 (convenience alias)
 */
template<typename E>
using Square = Pow<E, 2>;

/**
 * @brief Sine: sin(E)
 */
template<typename E>
struct Sin {
    using Arg = E;
};

/**
 * @brief Cosine: cos(E)
 */
template<typename E>
struct Cos {
    using Arg = E;
};

/**
 * @brief Square root: sqrt(E)
 */
template<typename E>
struct Sqrt {
    using Arg = E;
};

// ============================================================================
// Type Traits
// ============================================================================

// Check if type is a constant
template<typename E>
struct is_const : std::false_type {};

template<int N, int D>
struct is_const<Const<N, D>> : std::true_type {};

template<typename E>
inline constexpr bool is_const_v = is_const<E>::value;

// Check if constant is zero
template<typename E>
struct is_zero : std::false_type {};

template<int D>
struct is_zero<Const<0, D>> : std::true_type {};

template<typename E>
inline constexpr bool is_zero_v = is_zero<E>::value;

// Check if constant is one
template<typename E>
struct is_one : std::false_type {};

template<int D>
struct is_one<Const<D, D>> : std::true_type {};

template<typename E>
inline constexpr bool is_one_v = is_one<E>::value;

// Check if type is a variable
template<typename E>
struct is_var : std::false_type {};

template<size_t I>
struct is_var<Var<I>> : std::true_type {};

template<typename E>
inline constexpr bool is_var_v = is_var<E>::value;

// ============================================================================
// Expression Evaluation
// ============================================================================

// Forward declaration
template<typename E, typename T, size_t N>
struct Evaluator;

/**
 * @brief Evaluate expression with given variable values
 * @tparam E Expression type
 * @tparam T Scalar type (double, Dual, etc.)
 * @tparam N Number of variables
 * @param vars Array of variable values
 * @return Evaluated result
 */
template<typename E, typename T, size_t N>
T eval(const std::array<T, N>& vars) {
    return Evaluator<E, T, N>::eval(vars);
}

// Overload for runtime parameters
template<typename E, typename T, size_t N, size_t P>
T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
    return Evaluator<E, T, N>::eval(vars, params);
}

// Variable evaluation
template<size_t I, typename T, size_t N>
struct Evaluator<Var<I>, T, N> {
    static_assert(I < N, "Variable index out of bounds");

    static T eval(const std::array<T, N>& vars) {
        return vars[I];
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& /*params*/) {
        return vars[I];
    }
};

// Constant evaluation
template<int Num, int Denom, typename T, size_t N>
struct Evaluator<Const<Num, Denom>, T, N> {
    static T eval(const std::array<T, N>& /*vars*/) {
        return T(static_cast<double>(Num) / static_cast<double>(Denom));
    }

    template<size_t P>
    static T eval(const std::array<T, N>& /*vars*/, const std::array<T, P>& /*params*/) {
        return T(static_cast<double>(Num) / static_cast<double>(Denom));
    }
};

// Parameter evaluation
template<size_t ID, typename T, size_t N>
struct Evaluator<Param<ID>, T, N> {
    template<size_t P>
    static T eval(const std::array<T, N>& /*vars*/, const std::array<T, P>& params) {
        static_assert(ID < P, "Parameter index out of bounds");
        return params[ID];
    }
};

// Addition evaluation
template<typename L, typename R, typename T, size_t N>
struct Evaluator<Add<L, R>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        return Evaluator<L, T, N>::eval(vars) + Evaluator<R, T, N>::eval(vars);
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        return Evaluator<L, T, N>::eval(vars, params) + Evaluator<R, T, N>::eval(vars, params);
    }
};

// Subtraction evaluation
template<typename L, typename R, typename T, size_t N>
struct Evaluator<Sub<L, R>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        return Evaluator<L, T, N>::eval(vars) - Evaluator<R, T, N>::eval(vars);
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        return Evaluator<L, T, N>::eval(vars, params) - Evaluator<R, T, N>::eval(vars, params);
    }
};

// Multiplication evaluation
template<typename L, typename R, typename T, size_t N>
struct Evaluator<Mul<L, R>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        return Evaluator<L, T, N>::eval(vars) * Evaluator<R, T, N>::eval(vars);
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        return Evaluator<L, T, N>::eval(vars, params) * Evaluator<R, T, N>::eval(vars, params);
    }
};

// Division evaluation
template<typename L, typename R, typename T, size_t N>
struct Evaluator<Div<L, R>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        return Evaluator<L, T, N>::eval(vars) / Evaluator<R, T, N>::eval(vars);
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        return Evaluator<L, T, N>::eval(vars, params) / Evaluator<R, T, N>::eval(vars, params);
    }
};

// Negation evaluation
template<typename E, typename T, size_t N>
struct Evaluator<Neg<E>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        return -Evaluator<E, T, N>::eval(vars);
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        return -Evaluator<E, T, N>::eval(vars, params);
    }
};

// Power evaluation (integer exponent)
template<typename E, int Exp, typename T, size_t N>
struct Evaluator<Pow<E, Exp>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        T base = Evaluator<E, T, N>::eval(vars);
        return pow_impl(base);
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        T base = Evaluator<E, T, N>::eval(vars, params);
        return pow_impl(base);
    }

private:
    static T pow_impl(T base) {
        if constexpr (Exp == 0) {
            return T(1);
        } else if constexpr (Exp == 1) {
            return base;
        } else if constexpr (Exp == 2) {
            return base * base;
        } else if constexpr (Exp == -1) {
            return T(1) / base;
        } else if constexpr (Exp == -2) {
            return T(1) / (base * base);
        } else if constexpr (Exp > 0) {
            // Use exponentiation by squaring
            return std::pow(static_cast<double>(base), static_cast<double>(Exp));
        } else {
            return T(1) / std::pow(static_cast<double>(base), static_cast<double>(-Exp));
        }
    }
};

// Sine evaluation
template<typename E, typename T, size_t N>
struct Evaluator<Sin<E>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        using std::sin;
        return sin(Evaluator<E, T, N>::eval(vars));
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        using std::sin;
        return sin(Evaluator<E, T, N>::eval(vars, params));
    }
};

// Cosine evaluation
template<typename E, typename T, size_t N>
struct Evaluator<Cos<E>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        using std::cos;
        return cos(Evaluator<E, T, N>::eval(vars));
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        using std::cos;
        return cos(Evaluator<E, T, N>::eval(vars, params));
    }
};

// Square root evaluation
template<typename E, typename T, size_t N>
struct Evaluator<Sqrt<E>, T, N> {
    static T eval(const std::array<T, N>& vars) {
        using std::sqrt;
        return sqrt(Evaluator<E, T, N>::eval(vars));
    }

    template<size_t P>
    static T eval(const std::array<T, N>& vars, const std::array<T, P>& params) {
        using std::sqrt;
        return sqrt(Evaluator<E, T, N>::eval(vars, params));
    }
};

// ============================================================================
// Expression Simplification (compile-time)
// ============================================================================

// Forward declaration
template<typename E>
struct Simplify;

template<typename E>
using Simplify_t = typename Simplify<E>::type;

// Default: no simplification
template<typename E>
struct Simplify {
    using type = E;
};

// 0 + 0 = 0 (must come before other Add specializations to avoid ambiguity)
template<>
struct Simplify<Add<Zero, Zero>> {
    using type = Zero;
};

// 0 + x = x
template<typename R>
struct Simplify<Add<Zero, R>> {
    using type = Simplify_t<R>;
};

// x + 0 = x
template<typename L>
struct Simplify<Add<L, Zero>> {
    using type = Simplify_t<L>;
};

// 0 - 0 = 0 (must come before other Sub specializations to avoid ambiguity)
template<>
struct Simplify<Sub<Zero, Zero>> {
    using type = Zero;
};

// 0 - x = -x
template<typename R>
struct Simplify<Sub<Zero, R>> {
    using type = Neg<Simplify_t<R>>;
};

// x - 0 = x
template<typename L>
struct Simplify<Sub<L, Zero>> {
    using type = Simplify_t<L>;
};

// 0 * 0 = 0 (ambiguity resolution)
template<>
struct Simplify<Mul<Zero, Zero>> {
    using type = Zero;
};

// 0 * 1 = 0 (ambiguity resolution)
template<>
struct Simplify<Mul<Zero, One>> {
    using type = Zero;
};

// 1 * 0 = 0 (ambiguity resolution)
template<>
struct Simplify<Mul<One, Zero>> {
    using type = Zero;
};

// 1 * 1 = 1 (ambiguity resolution)
template<>
struct Simplify<Mul<One, One>> {
    using type = One;
};

// 0 * x = 0
template<typename R>
struct Simplify<Mul<Zero, R>> {
    using type = Zero;
};

// x * 0 = 0
template<typename L>
struct Simplify<Mul<L, Zero>> {
    using type = Zero;
};

// 1 * x = x
template<typename R>
struct Simplify<Mul<One, R>> {
    using type = Simplify_t<R>;
};

// x * 1 = x
template<typename L>
struct Simplify<Mul<L, One>> {
    using type = Simplify_t<L>;
};

// x / 1 = x
template<typename L>
struct Simplify<Div<L, One>> {
    using type = Simplify_t<L>;
};

// 0 / x = 0
template<typename R>
struct Simplify<Div<Zero, R>> {
    using type = Zero;
};

// --x = x
template<typename E>
struct Simplify<Neg<Neg<E>>> {
    using type = Simplify_t<E>;
};

// -0 = 0
template<>
struct Simplify<Neg<Zero>> {
    using type = Zero;
};

// x^0 = 1
template<typename E>
struct Simplify<Pow<E, 0>> {
    using type = One;
};

// x^1 = x
template<typename E>
struct Simplify<Pow<E, 1>> {
    using type = Simplify_t<E>;
};

// ============================================================================
// Convenience operators for building expressions
// ============================================================================

// These create expression types, not values
template<typename L, typename R>
Add<L, R> operator+(L, R) { return {}; }

template<typename L, typename R>
Sub<L, R> operator-(L, R) { return {}; }

template<typename L, typename R>
Mul<L, R> operator*(L, R) { return {}; }

template<typename L, typename R>
Div<L, R> operator/(L, R) { return {}; }

template<typename E>
Neg<E> operator-(E) { return {}; }

} // namespace sopot::symbolic
