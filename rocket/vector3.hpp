#pragma once

#include "../core/scalar.hpp"
#include <cmath>
#include <array>

namespace sopot::rocket {

// 3D vector template supporting autodiff
template<Scalar T = double>
struct Vector3 {
    T x{}, y{}, z{};

    constexpr Vector3() = default;
    constexpr Vector3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    // Construct from array
    explicit Vector3(const std::array<T, 3>& arr) : x(arr[0]), y(arr[1]), z(arr[2]) {}

    // Element access
    T& operator[](size_t i) {
        switch(i) {
            case 0: return x;
            case 1: return y;
            default: return z;
        }
    }
    const T& operator[](size_t i) const {
        switch(i) {
            case 0: return x;
            case 1: return y;
            default: return z;
        }
    }

    // Arithmetic operators
    Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vector3 operator-() const {
        return {-x, -y, -z};
    }

    Vector3 operator*(T scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vector3 operator/(T scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    Vector3& operator+=(const Vector3& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    Vector3& operator*=(T scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    // Dot product
    T dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product
    Vector3 cross(const Vector3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    // Magnitude squared
    T norm_squared() const {
        return x * x + y * y + z * z;
    }

    // Magnitude
    T norm() const {
        using std::sqrt;
        return sqrt(norm_squared());
    }

    // Normalized vector
    Vector3 normalized() const {
        T n = norm();
        if (value_of(n) < 1e-15) return {T(0), T(0), T(0)};
        return *this / n;
    }

    // Convert to array
    std::array<T, 3> to_array() const {
        return {x, y, z};
    }

    // Zero vector
    static Vector3 zero() { return {T(0), T(0), T(0)}; }

    // Unit vectors
    static Vector3 unit_x() { return {T(1), T(0), T(0)}; }
    static Vector3 unit_y() { return {T(0), T(1), T(0)}; }
    static Vector3 unit_z() { return {T(0), T(0), T(1)}; }
};

// Scalar * Vector
template<Scalar T>
Vector3<T> operator*(T scalar, const Vector3<T>& v) {
    return v * scalar;
}

// Free function versions
template<Scalar T>
T dot(const Vector3<T>& a, const Vector3<T>& b) {
    return a.dot(b);
}

template<Scalar T>
Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b) {
    return a.cross(b);
}

template<Scalar T>
T norm(const Vector3<T>& v) {
    return v.norm();
}

template<Scalar T>
Vector3<T> normalize(const Vector3<T>& v) {
    return v.normalized();
}

} // namespace sopot::rocket
