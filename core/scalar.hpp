#pragma once

#include "dual.hpp"
#include "units.hpp"
#include <concepts>
#include <type_traits>
#include <cmath>

namespace sopot {

// Concept for scalar types that can be used in computations
template<typename T>
concept Scalar = requires(T a, T b, double d) {
    { a + b } -> std::convertible_to<T>;
    { a - b } -> std::convertible_to<T>;
    { a * b } -> std::convertible_to<T>;
    { a / b } -> std::convertible_to<T>;
    { -a } -> std::convertible_to<T>;
    { a * d } -> std::convertible_to<T>;
    { d * a } -> std::convertible_to<T>;
};

// Concept for types that support automatic differentiation
template<typename T>
concept Differentiable = Scalar<T> && requires(T a) {
    { a.derivative(0) } -> std::convertible_to<double>;
    { a.value() } -> std::convertible_to<double>;
};

// Concept for types with physical dimensions
template<typename T>
concept Dimensional = requires(T a) {
    typename T::dimension;
    { a.value() };
};

// Get the underlying value from any scalar type
template<typename T>
constexpr auto value_of(const T& x) {
    if constexpr (is_dual_v<T>) {
        return x.value();
    } else if constexpr (units::is_quantity_v<T>) {
        return value_of(x.value()); // Recursive for Quantity<Dual<...>>
    } else {
        return x;
    }
}

// Get derivative from any scalar type
template<typename T>
constexpr double derivative_of(const T& x, size_t index = 0) {
    if constexpr (is_dual_v<T>) {
        return x.derivative(index);
    } else if constexpr (units::is_quantity_v<T>) {
        return derivative_of(x.value(), index);
    } else {
        return 0.0;
    }
}

// Create a variable for differentiation
template<typename T>
constexpr T make_variable(double value, size_t derivative_index = 0) {
    if constexpr (is_dual_v<T>) {
        return T::variable(value, derivative_index);
    } else {
        return T(value);
    }
}

// Create a constant (no derivatives)
template<typename T>
constexpr T make_constant(double value) {
    if constexpr (is_dual_v<T>) {
        return T::constant(value);
    } else {
        return T(value);
    }
}

// Type trait to get the base scalar type
template<typename T>
struct base_scalar {
    using type = T;
};

template<typename T, size_t N>
struct base_scalar<Dual<T, N>> {
    using type = T;
};

template<typename Dim, typename T>
struct base_scalar<units::Quantity<Dim, T>> {
    using type = typename base_scalar<T>::type;
};

template<typename T>
using base_scalar_t = typename base_scalar<T>::type;

// Type trait to get the number of derivatives
template<typename T>
struct num_derivatives {
    static constexpr size_t value = 0;
};

template<typename T, size_t N>
struct num_derivatives<Dual<T, N>> {
    static constexpr size_t value = N;
};

template<typename Dim, typename T>
struct num_derivatives<units::Quantity<Dim, T>> {
    static constexpr size_t value = num_derivatives<T>::value;
};

template<typename T>
inline constexpr size_t num_derivatives_v = num_derivatives<T>::value;

// ScalarState: A state vector with a specific scalar type
template<typename T, size_t N>
class ScalarState {
public:
    using scalar_type = T;
    static constexpr size_t size = N;

private:
    std::array<T, N> m_data;

public:
    constexpr ScalarState() : m_data{} {}

    template<typename... Args>
    requires (sizeof...(Args) == N) && (std::convertible_to<Args, T> && ...)
    constexpr ScalarState(Args... args) : m_data{T(args)...} {}

    constexpr T& operator[](size_t i) { return m_data[i]; }
    constexpr const T& operator[](size_t i) const { return m_data[i]; }

    constexpr T* data() { return m_data.data(); }
    constexpr const T* data() const { return m_data.data(); }

    constexpr auto begin() { return m_data.begin(); }
    constexpr auto end() { return m_data.end(); }
    constexpr auto begin() const { return m_data.begin(); }
    constexpr auto end() const { return m_data.end(); }

    // Convert to array of base values
    std::array<double, N> values() const {
        std::array<double, N> result;
        for (size_t i = 0; i < N; ++i) {
            result[i] = value_of(m_data[i]);
        }
        return result;
    }

    // Get Jacobian row for a specific output
    template<size_t NumDerivs>
    std::array<double, NumDerivs> derivatives(size_t output_index) const {
        std::array<double, NumDerivs> result;
        for (size_t j = 0; j < NumDerivs; ++j) {
            result[j] = derivative_of(m_data[output_index], j);
        }
        return result;
    }
};

// Vector operations for ScalarState
template<typename T, size_t N>
constexpr ScalarState<T, N> operator+(const ScalarState<T, N>& a, const ScalarState<T, N>& b) {
    ScalarState<T, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

template<typename T, size_t N>
constexpr ScalarState<T, N> operator-(const ScalarState<T, N>& a, const ScalarState<T, N>& b) {
    ScalarState<T, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = a[i] - b[i];
    }
    return result;
}

template<typename T, size_t N>
constexpr ScalarState<T, N> operator*(const ScalarState<T, N>& a, double scalar) {
    ScalarState<T, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = a[i] * scalar;
    }
    return result;
}

template<typename T, size_t N>
constexpr ScalarState<T, N> operator*(double scalar, const ScalarState<T, N>& a) {
    return a * scalar;
}

} // namespace sopot
