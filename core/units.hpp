#pragma once

#include <ratio>
#include <type_traits>
#include <ostream>
#include <cmath>

namespace sopot::units {

// Compile-time dimensional analysis
// Each dimension is represented by a std::ratio exponent
// Base SI dimensions: Length, Mass, Time, Angle

template<
    typename LengthExp = std::ratio<0>,    // meters
    typename MassExp = std::ratio<0>,      // kilograms
    typename TimeExp = std::ratio<0>,      // seconds
    typename AngleExp = std::ratio<0>      // radians (dimensionless but tracked)
>
struct Dimension {
    using length = LengthExp;
    using mass = MassExp;
    using time = TimeExp;
    using angle = AngleExp;
};

// Common dimensions
using Dimensionless = Dimension<>;
using Length = Dimension<std::ratio<1>>;
using Mass = Dimension<std::ratio<0>, std::ratio<1>>;
using Time = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<1>>;
using Angle = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;

// Derived dimensions
using Area = Dimension<std::ratio<2>>;
using Volume = Dimension<std::ratio<3>>;
using Velocity = Dimension<std::ratio<1>, std::ratio<0>, std::ratio<-1>>;
using Acceleration = Dimension<std::ratio<1>, std::ratio<0>, std::ratio<-2>>;
using Force = Dimension<std::ratio<1>, std::ratio<1>, std::ratio<-2>>;
using Pressure = Dimension<std::ratio<-1>, std::ratio<1>, std::ratio<-2>>;
using Energy = Dimension<std::ratio<2>, std::ratio<1>, std::ratio<-2>>;
using Power = Dimension<std::ratio<2>, std::ratio<1>, std::ratio<-3>>;
using Density = Dimension<std::ratio<-3>, std::ratio<1>>;
using AngularVelocity = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<-1>, std::ratio<1>>;
using AngularAcceleration = Dimension<std::ratio<0>, std::ratio<0>, std::ratio<-2>, std::ratio<1>>;
using MomentOfInertia = Dimension<std::ratio<2>, std::ratio<1>>;
using Torque = Dimension<std::ratio<2>, std::ratio<1>, std::ratio<-2>>; // Same as Energy
using MassFlowRate = Dimension<std::ratio<0>, std::ratio<1>, std::ratio<-1>>;

// Dimension arithmetic
template<typename D1, typename D2>
struct DimensionMultiply {
    using type = Dimension<
        std::ratio_add<typename D1::length, typename D2::length>,
        std::ratio_add<typename D1::mass, typename D2::mass>,
        std::ratio_add<typename D1::time, typename D2::time>,
        std::ratio_add<typename D1::angle, typename D2::angle>
    >;
};

template<typename D1, typename D2>
struct DimensionDivide {
    using type = Dimension<
        std::ratio_subtract<typename D1::length, typename D2::length>,
        std::ratio_subtract<typename D1::mass, typename D2::mass>,
        std::ratio_subtract<typename D1::time, typename D2::time>,
        std::ratio_subtract<typename D1::angle, typename D2::angle>
    >;
};

template<typename D>
struct DimensionInvert {
    using type = Dimension<
        std::ratio_subtract<std::ratio<0>, typename D::length>,
        std::ratio_subtract<std::ratio<0>, typename D::mass>,
        std::ratio_subtract<std::ratio<0>, typename D::time>,
        std::ratio_subtract<std::ratio<0>, typename D::angle>
    >;
};

template<typename D, typename Exp>
struct DimensionPow {
    using type = Dimension<
        std::ratio_multiply<typename D::length, Exp>,
        std::ratio_multiply<typename D::mass, Exp>,
        std::ratio_multiply<typename D::time, Exp>,
        std::ratio_multiply<typename D::angle, Exp>
    >;
};

template<typename D1, typename D2>
using dim_multiply_t = typename DimensionMultiply<D1, D2>::type;

template<typename D1, typename D2>
using dim_divide_t = typename DimensionDivide<D1, D2>::type;

template<typename D>
using dim_invert_t = typename DimensionInvert<D>::type;

template<typename D, typename Exp>
using dim_pow_t = typename DimensionPow<D, Exp>::type;

// Dimension equality check
template<typename D1, typename D2>
struct DimensionEqual : std::bool_constant<
    std::ratio_equal_v<typename D1::length, typename D2::length> &&
    std::ratio_equal_v<typename D1::mass, typename D2::mass> &&
    std::ratio_equal_v<typename D1::time, typename D2::time> &&
    std::ratio_equal_v<typename D1::angle, typename D2::angle>
> {};

template<typename D1, typename D2>
inline constexpr bool dim_equal_v = DimensionEqual<D1, D2>::value;

// Quantity: value with compile-time dimension
template<typename Dim, typename T = double>
class Quantity {
public:
    using dimension = Dim;
    using value_type = T;

private:
    T m_value;

public:
    // Constructors
    constexpr Quantity() noexcept : m_value(T{0}) {}
    constexpr explicit Quantity(T value) noexcept : m_value(value) {}

    // Get raw value
    constexpr T value() const noexcept { return m_value; }

    // Implicit conversion for dimensionless quantities
    template<typename D = Dim, typename = std::enable_if_t<dim_equal_v<D, Dimensionless>>>
    constexpr operator T() const noexcept { return m_value; }

    // Arithmetic with same dimension
    constexpr Quantity operator+(const Quantity& other) const noexcept {
        return Quantity(m_value + other.m_value);
    }

    constexpr Quantity operator-(const Quantity& other) const noexcept {
        return Quantity(m_value - other.m_value);
    }

    constexpr Quantity operator-() const noexcept {
        return Quantity(-m_value);
    }

    // Multiplication with different dimensions
    template<typename OtherDim>
    constexpr auto operator*(const Quantity<OtherDim, T>& other) const noexcept {
        using ResultDim = dim_multiply_t<Dim, OtherDim>;
        return Quantity<ResultDim, T>(m_value * other.value());
    }

    // Division with different dimensions
    template<typename OtherDim>
    constexpr auto operator/(const Quantity<OtherDim, T>& other) const noexcept {
        using ResultDim = dim_divide_t<Dim, OtherDim>;
        return Quantity<ResultDim, T>(m_value / other.value());
    }

    // Scalar multiplication
    constexpr Quantity operator*(T scalar) const noexcept {
        return Quantity(m_value * scalar);
    }

    constexpr Quantity operator/(T scalar) const noexcept {
        return Quantity(m_value / scalar);
    }

    // Compound assignment
    constexpr Quantity& operator+=(const Quantity& other) noexcept {
        m_value += other.m_value;
        return *this;
    }

    constexpr Quantity& operator-=(const Quantity& other) noexcept {
        m_value -= other.m_value;
        return *this;
    }

    constexpr Quantity& operator*=(T scalar) noexcept {
        m_value *= scalar;
        return *this;
    }

    constexpr Quantity& operator/=(T scalar) noexcept {
        m_value /= scalar;
        return *this;
    }

    // Comparison
    constexpr bool operator==(const Quantity& other) const noexcept { return m_value == other.m_value; }
    constexpr bool operator!=(const Quantity& other) const noexcept { return m_value != other.m_value; }
    constexpr bool operator<(const Quantity& other) const noexcept { return m_value < other.m_value; }
    constexpr bool operator<=(const Quantity& other) const noexcept { return m_value <= other.m_value; }
    constexpr bool operator>(const Quantity& other) const noexcept { return m_value > other.m_value; }
    constexpr bool operator>=(const Quantity& other) const noexcept { return m_value >= other.m_value; }
};

// Scalar on the left
template<typename Dim, typename T>
constexpr Quantity<Dim, T> operator*(T scalar, const Quantity<Dim, T>& q) noexcept {
    return q * scalar;
}

// Mathematical functions
template<typename Dim, typename T>
auto sqrt(const Quantity<Dim, T>& q) {
    using ResultDim = dim_pow_t<Dim, std::ratio<1, 2>>;
    return Quantity<ResultDim, T>(std::sqrt(q.value()));
}

template<typename T>
Quantity<Dimensionless, T> sin(const Quantity<Angle, T>& q) {
    return Quantity<Dimensionless, T>(std::sin(q.value()));
}

template<typename T>
Quantity<Dimensionless, T> cos(const Quantity<Angle, T>& q) {
    return Quantity<Dimensionless, T>(std::cos(q.value()));
}

template<typename T>
Quantity<Dimensionless, T> tan(const Quantity<Angle, T>& q) {
    return Quantity<Dimensionless, T>(std::tan(q.value()));
}

template<typename T>
Quantity<Angle, T> atan2(const Quantity<Dimensionless, T>& y, const Quantity<Dimensionless, T>& x) {
    return Quantity<Angle, T>(std::atan2(y.value(), x.value()));
}

// For velocity/length ratios
template<typename Dim, typename T>
Quantity<Angle, T> atan2(const Quantity<Dim, T>& y, const Quantity<Dim, T>& x) {
    return Quantity<Angle, T>(std::atan2(y.value(), x.value()));
}

template<typename Dim, typename T>
auto abs(const Quantity<Dim, T>& q) {
    return Quantity<Dim, T>(std::abs(q.value()));
}

// Output stream
template<typename Dim, typename T>
std::ostream& operator<<(std::ostream& os, const Quantity<Dim, T>& q) {
    os << q.value();
    // Could add unit suffix based on dimension
    return os;
}

// Common quantity type aliases
template<typename T = double> using Meters = Quantity<Length, T>;
template<typename T = double> using Kilograms = Quantity<Mass, T>;
template<typename T = double> using Seconds = Quantity<Time, T>;
template<typename T = double> using Radians = Quantity<Angle, T>;
template<typename T = double> using MetersPerSecond = Quantity<Velocity, T>;
template<typename T = double> using MetersPerSecond2 = Quantity<Acceleration, T>;
template<typename T = double> using Newtons = Quantity<Force, T>;
template<typename T = double> using Pascals = Quantity<Pressure, T>;
template<typename T = double> using Joules = Quantity<Energy, T>;
template<typename T = double> using KilogramsPerMeter3 = Quantity<Density, T>;
template<typename T = double> using RadiansPerSecond = Quantity<AngularVelocity, T>;
template<typename T = double> using NewtonMeters = Quantity<Torque, T>;
template<typename T = double> using KilogramsMeter2 = Quantity<MomentOfInertia, T>;
template<typename T = double> using KilogramsPerSecond = Quantity<MassFlowRate, T>;

// User-defined literals for common units
namespace literals {

constexpr Meters<> operator""_m(long double val) { return Meters<>(static_cast<double>(val)); }
constexpr Meters<> operator""_m(unsigned long long val) { return Meters<>(static_cast<double>(val)); }

constexpr auto operator""_km(long double val) { return Meters<>(static_cast<double>(val) * 1000.0); }
constexpr auto operator""_km(unsigned long long val) { return Meters<>(static_cast<double>(val) * 1000.0); }

constexpr auto operator""_cm(long double val) { return Meters<>(static_cast<double>(val) * 0.01); }
constexpr auto operator""_mm(long double val) { return Meters<>(static_cast<double>(val) * 0.001); }

constexpr Kilograms<> operator""_kg(long double val) { return Kilograms<>(static_cast<double>(val)); }
constexpr Kilograms<> operator""_kg(unsigned long long val) { return Kilograms<>(static_cast<double>(val)); }

constexpr Seconds<> operator""_s(long double val) { return Seconds<>(static_cast<double>(val)); }
constexpr Seconds<> operator""_s(unsigned long long val) { return Seconds<>(static_cast<double>(val)); }

constexpr auto operator""_ms(long double val) { return Seconds<>(static_cast<double>(val) * 0.001); }

constexpr Radians<> operator""_rad(long double val) { return Radians<>(static_cast<double>(val)); }
constexpr auto operator""_deg(long double val) { return Radians<>(static_cast<double>(val) * M_PI / 180.0); }

constexpr Newtons<> operator""_N(long double val) { return Newtons<>(static_cast<double>(val)); }
constexpr auto operator""_kN(long double val) { return Newtons<>(static_cast<double>(val) * 1000.0); }

constexpr Pascals<> operator""_Pa(long double val) { return Pascals<>(static_cast<double>(val)); }
constexpr auto operator""_kPa(long double val) { return Pascals<>(static_cast<double>(val) * 1000.0); }

constexpr MetersPerSecond<> operator""_mps(long double val) { return MetersPerSecond<>(static_cast<double>(val)); }

} // namespace literals

// Type trait to check if a type is a Quantity
template<typename T>
struct is_quantity : std::false_type {};

template<typename Dim, typename T>
struct is_quantity<Quantity<Dim, T>> : std::true_type {};

template<typename T>
inline constexpr bool is_quantity_v = is_quantity<T>::value;

// Extract raw value from Quantity or pass through scalar
template<typename T>
constexpr auto get_value(const T& x) {
    if constexpr (is_quantity_v<T>) {
        return x.value();
    } else {
        return x;
    }
}

} // namespace sopot::units
