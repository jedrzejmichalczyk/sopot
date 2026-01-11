#pragma once

#include <cmath>
#include <array>
#include <type_traits>
#include <ostream>

namespace sopot {

// Forward-mode automatic differentiation using dual numbers
// Dual<T, N> represents a value with N partial derivatives
// For single-variable: Dual<double, 1>
// For gradient computation: Dual<double, N> where N = number of inputs

template<typename T = double, size_t NumDerivatives = 1>
class Dual {
public:
    using value_type = T;
    static constexpr size_t num_derivatives = NumDerivatives;

private:
    T m_value;
    std::array<T, NumDerivatives> m_derivatives;

public:
    // Constructors
    constexpr Dual() noexcept : m_value(T{0}), m_derivatives{} {}

    constexpr explicit Dual(T value) noexcept : m_value(value), m_derivatives{} {}

    constexpr Dual(T value, std::array<T, NumDerivatives> derivatives) noexcept
        : m_value(value), m_derivatives(derivatives) {}

    // Create a variable with derivative = 1 for index i (for computing partial derivatives)
    static constexpr Dual variable(T value, size_t derivative_index = 0) noexcept {
        Dual result(value);
        if (derivative_index < NumDerivatives) {
            result.m_derivatives[derivative_index] = T{1};
        }
        return result;
    }

    // Create a constant (derivative = 0)
    static constexpr Dual constant(T value) noexcept {
        return Dual(value);
    }

    // Accessors
    constexpr T value() const noexcept { return m_value; }
    constexpr T derivative(size_t i = 0) const noexcept {
        return i < NumDerivatives ? m_derivatives[i] : T{0};
    }
    constexpr const std::array<T, NumDerivatives>& derivatives() const noexcept {
        return m_derivatives;
    }

    // Strip derivatives (convert to constant)
    constexpr Dual noDerivs() const noexcept {
        return Dual(m_value);
    }

    // Implicit conversion to value type (for comparisons, etc.)
    constexpr explicit operator T() const noexcept { return m_value; }

    // Arithmetic operators with Dual
    constexpr Dual operator+(const Dual& other) const noexcept {
        std::array<T, NumDerivatives> new_derivs;
        for (size_t i = 0; i < NumDerivatives; ++i) {
            new_derivs[i] = m_derivatives[i] + other.m_derivatives[i];
        }
        return Dual(m_value + other.m_value, new_derivs);
    }

    constexpr Dual operator-(const Dual& other) const noexcept {
        std::array<T, NumDerivatives> new_derivs;
        for (size_t i = 0; i < NumDerivatives; ++i) {
            new_derivs[i] = m_derivatives[i] - other.m_derivatives[i];
        }
        return Dual(m_value - other.m_value, new_derivs);
    }

    constexpr Dual operator*(const Dual& other) const noexcept {
        // d(u*v) = u*dv + v*du
        std::array<T, NumDerivatives> new_derivs;
        for (size_t i = 0; i < NumDerivatives; ++i) {
            new_derivs[i] = m_value * other.m_derivatives[i] + m_derivatives[i] * other.m_value;
        }
        return Dual(m_value * other.m_value, new_derivs);
    }

    constexpr Dual operator/(const Dual& other) const noexcept {
        // d(u/v) = (v*du - u*dv) / v^2
        T inv_v2 = T{1} / (other.m_value * other.m_value);
        std::array<T, NumDerivatives> new_derivs;
        for (size_t i = 0; i < NumDerivatives; ++i) {
            new_derivs[i] = (other.m_value * m_derivatives[i] - m_value * other.m_derivatives[i]) * inv_v2;
        }
        return Dual(m_value / other.m_value, new_derivs);
    }

    constexpr Dual operator-() const noexcept {
        std::array<T, NumDerivatives> new_derivs;
        for (size_t i = 0; i < NumDerivatives; ++i) {
            new_derivs[i] = -m_derivatives[i];
        }
        return Dual(-m_value, new_derivs);
    }

    // Arithmetic with scalars (on the right)
    constexpr Dual operator+(T scalar) const noexcept {
        return Dual(m_value + scalar, m_derivatives);
    }

    constexpr Dual operator-(T scalar) const noexcept {
        return Dual(m_value - scalar, m_derivatives);
    }

    constexpr Dual operator*(T scalar) const noexcept {
        std::array<T, NumDerivatives> new_derivs;
        for (size_t i = 0; i < NumDerivatives; ++i) {
            new_derivs[i] = m_derivatives[i] * scalar;
        }
        return Dual(m_value * scalar, new_derivs);
    }

    constexpr Dual operator/(T scalar) const noexcept {
        T inv = T{1} / scalar;
        std::array<T, NumDerivatives> new_derivs;
        for (size_t i = 0; i < NumDerivatives; ++i) {
            new_derivs[i] = m_derivatives[i] * inv;
        }
        return Dual(m_value * inv, new_derivs);
    }

    // Compound assignment
    constexpr Dual& operator+=(const Dual& other) noexcept { return *this = *this + other; }
    constexpr Dual& operator-=(const Dual& other) noexcept { return *this = *this - other; }
    constexpr Dual& operator*=(const Dual& other) noexcept { return *this = *this * other; }
    constexpr Dual& operator/=(const Dual& other) noexcept { return *this = *this / other; }
    constexpr Dual& operator+=(T scalar) noexcept { return *this = *this + scalar; }
    constexpr Dual& operator-=(T scalar) noexcept { return *this = *this - scalar; }
    constexpr Dual& operator*=(T scalar) noexcept { return *this = *this * scalar; }
    constexpr Dual& operator/=(T scalar) noexcept { return *this = *this / scalar; }

    // Comparison (based on value only)
    constexpr bool operator==(const Dual& other) const noexcept { return m_value == other.m_value; }
    constexpr bool operator!=(const Dual& other) const noexcept { return m_value != other.m_value; }
    constexpr bool operator<(const Dual& other) const noexcept { return m_value < other.m_value; }
    constexpr bool operator<=(const Dual& other) const noexcept { return m_value <= other.m_value; }
    constexpr bool operator>(const Dual& other) const noexcept { return m_value > other.m_value; }
    constexpr bool operator>=(const Dual& other) const noexcept { return m_value >= other.m_value; }

    // Comparison with scalar
    constexpr bool operator==(T scalar) const noexcept { return m_value == scalar; }
    constexpr bool operator<(T scalar) const noexcept { return m_value < scalar; }
    constexpr bool operator>(T scalar) const noexcept { return m_value > scalar; }
};

// Scalar on the left
template<typename T, size_t N>
constexpr Dual<T, N> operator+(T scalar, const Dual<T, N>& dual) noexcept {
    return dual + scalar;
}

template<typename T, size_t N>
constexpr Dual<T, N> operator-(T scalar, const Dual<T, N>& dual) noexcept {
    return Dual<T, N>::constant(scalar) - dual;
}

template<typename T, size_t N>
constexpr Dual<T, N> operator*(T scalar, const Dual<T, N>& dual) noexcept {
    return dual * scalar;
}

template<typename T, size_t N>
constexpr Dual<T, N> operator/(T scalar, const Dual<T, N>& dual) noexcept {
    return Dual<T, N>::constant(scalar) / dual;
}

// Mathematical functions
template<typename T, size_t N>
Dual<T, N> sin(const Dual<T, N>& x) {
    // d(sin(u)) = cos(u) * du
    T cos_val = std::cos(x.value());
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = cos_val * x.derivative(i);
    }
    return Dual<T, N>(std::sin(x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> cos(const Dual<T, N>& x) {
    // d(cos(u)) = -sin(u) * du
    T neg_sin_val = -std::sin(x.value());
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = neg_sin_val * x.derivative(i);
    }
    return Dual<T, N>(std::cos(x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> tan(const Dual<T, N>& x) {
    // d(tan(u)) = sec^2(u) * du = du / cos^2(u)
    T cos_val = std::cos(x.value());
    T sec2 = T{1} / (cos_val * cos_val);
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = sec2 * x.derivative(i);
    }
    return Dual<T, N>(std::tan(x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> asin(const Dual<T, N>& x) {
    // d(asin(u)) = du / sqrt(1 - u^2)
    T factor = T{1} / std::sqrt(T{1} - x.value() * x.value());
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = factor * x.derivative(i);
    }
    return Dual<T, N>(std::asin(x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> acos(const Dual<T, N>& x) {
    // d(acos(u)) = -du / sqrt(1 - u^2)
    T factor = T{-1} / std::sqrt(T{1} - x.value() * x.value());
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = factor * x.derivative(i);
    }
    return Dual<T, N>(std::acos(x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> atan(const Dual<T, N>& x) {
    // d(atan(u)) = du / (1 + u^2)
    T factor = T{1} / (T{1} + x.value() * x.value());
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = factor * x.derivative(i);
    }
    return Dual<T, N>(std::atan(x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> atan2(const Dual<T, N>& y, const Dual<T, N>& x) {
    // d(atan2(y,x)) = (x*dy - y*dx) / (x^2 + y^2)
    T denom = x.value() * x.value() + y.value() * y.value();
    T inv_denom = T{1} / denom;
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = (x.value() * y.derivative(i) - y.value() * x.derivative(i)) * inv_denom;
    }
    return Dual<T, N>(std::atan2(y.value(), x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> exp(const Dual<T, N>& x) {
    // d(exp(u)) = exp(u) * du
    T exp_val = std::exp(x.value());
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = exp_val * x.derivative(i);
    }
    return Dual<T, N>(exp_val, new_derivs);
}

template<typename T, size_t N>
Dual<T, N> log(const Dual<T, N>& x) {
    // d(log(u)) = du / u
    T inv = T{1} / x.value();
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = inv * x.derivative(i);
    }
    return Dual<T, N>(std::log(x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> sqrt(const Dual<T, N>& x) {
    // d(sqrt(u)) = du / (2 * sqrt(u))
    T sqrt_val = std::sqrt(x.value());
    T factor = T{0.5} / sqrt_val;
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = factor * x.derivative(i);
    }
    return Dual<T, N>(sqrt_val, new_derivs);
}

template<typename T, size_t N>
Dual<T, N> abs(const Dual<T, N>& x) {
    // d(|u|) = sign(u) * du
    T sign = (x.value() >= T{0}) ? T{1} : T{-1};
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = sign * x.derivative(i);
    }
    return Dual<T, N>(std::abs(x.value()), new_derivs);
}

template<typename T, size_t N>
Dual<T, N> pow(const Dual<T, N>& base, T exponent) {
    // d(u^n) = n * u^(n-1) * du
    T pow_val = std::pow(base.value(), exponent);
    T factor = exponent * std::pow(base.value(), exponent - T{1});
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = factor * base.derivative(i);
    }
    return Dual<T, N>(pow_val, new_derivs);
}

template<typename T, size_t N>
Dual<T, N> pow(const Dual<T, N>& base, const Dual<T, N>& exponent) {
    // d(u^v) = u^v * (v' * ln(u) + v * u' / u)
    T pow_val = std::pow(base.value(), exponent.value());
    T log_base = std::log(base.value());
    T inv_base = T{1} / base.value();
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = pow_val * (exponent.derivative(i) * log_base +
                                    exponent.value() * base.derivative(i) * inv_base);
    }
    return Dual<T, N>(pow_val, new_derivs);
}

template<typename T, size_t N>
Dual<T, N> tanh(const Dual<T, N>& x) {
    // d(tanh(u)) = (1 - tanh^2(u)) * du
    T tanh_val = std::tanh(x.value());
    T factor = T{1} - tanh_val * tanh_val;
    std::array<T, N> new_derivs;
    for (size_t i = 0; i < N; ++i) {
        new_derivs[i] = factor * x.derivative(i);
    }
    return Dual<T, N>(tanh_val, new_derivs);
}

// Sign function (derivative is 0 everywhere except at 0)
template<typename T, size_t N>
Dual<T, N> sign(const Dual<T, N>& x) {
    T sign_val = (x.value() > T{0}) ? T{1} : ((x.value() < T{0}) ? T{-1} : T{0});
    return Dual<T, N>(sign_val); // Derivative is 0
}

// Output stream
template<typename T, size_t N>
std::ostream& operator<<(std::ostream& os, const Dual<T, N>& dual) {
    os << "(" << dual.value() << "; [";
    for (size_t i = 0; i < N; ++i) {
        if (i > 0) os << ", ";
        os << dual.derivative(i);
    }
    os << "])";
    return os;
}

// Type traits
template<typename T>
struct is_dual : std::false_type {};

template<typename T, size_t N>
struct is_dual<Dual<T, N>> : std::true_type {};

template<typename T>
inline constexpr bool is_dual_v = is_dual<T>::value;

// Extract value from Dual or pass through scalar
template<typename T>
constexpr auto get_value(const T& x) {
    if constexpr (is_dual_v<T>) {
        return x.value();
    } else {
        return x;
    }
}

// Common type aliases
using Dual1 = Dual<double, 1>;   // Single derivative (e.g., df/dx)
using Dual3 = Dual<double, 3>;   // 3D gradient (e.g., ∂f/∂x, ∂f/∂y, ∂f/∂z)
using Dual6 = Dual<double, 6>;   // 6-DOF state (common for rockets)
using Dual13 = Dual<double, 13>; // Full rocket state (pos, vel, quat, omega)

} // namespace sopot
