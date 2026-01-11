#pragma once

#include "../core/scalar.hpp"
#include "vector3.hpp"
#include <cmath>
#include <array>

namespace sopot::rocket {

// Quaternion: q = [q1, q2, q3, q4] where q1,q2,q3 are vector part, q4 is scalar
// Convention: q represents rotation from body to reference frame
template<Scalar T = double>
struct Quaternion {
    T q1{}, q2{}, q3{}, q4{1};  // Default to identity quaternion

    constexpr Quaternion() = default;
    constexpr Quaternion(T q1_, T q2_, T q3_, T q4_) : q1(q1_), q2(q2_), q3(q3_), q4(q4_) {}

    // Construct from array [q1, q2, q3, q4]
    explicit Quaternion(const std::array<T, 4>& arr) : q1(arr[0]), q2(arr[1]), q3(arr[2]), q4(arr[3]) {}

    // Element access
    T& operator[](size_t i) {
        switch(i) {
            case 0: return q1;
            case 1: return q2;
            case 2: return q3;
            default: return q4;
        }
    }
    const T& operator[](size_t i) const {
        switch(i) {
            case 0: return q1;
            case 1: return q2;
            case 2: return q3;
            default: return q4;
        }
    }

    // Quaternion multiplication: q * p
    Quaternion operator*(const Quaternion& p) const {
        return {
            q4 * p.q1 + q1 * p.q4 + q2 * p.q3 - q3 * p.q2,
            q4 * p.q2 - q1 * p.q3 + q2 * p.q4 + q3 * p.q1,
            q4 * p.q3 + q1 * p.q2 - q2 * p.q1 + q3 * p.q4,
            q4 * p.q4 - q1 * p.q1 - q2 * p.q2 - q3 * p.q3
        };
    }

    // Conjugate: q*
    Quaternion conjugate() const {
        return {-q1, -q2, -q3, q4};
    }

    // Norm squared
    T norm_squared() const {
        return q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4;
    }

    // Norm
    T norm() const {
        using std::sqrt;
        return sqrt(norm_squared());
    }

    // Normalize to unit quaternion
    Quaternion normalized() const {
        T n = norm();
        return {q1 / n, q2 / n, q3 / n, q4 / n};
    }

    // Inverse (for unit quaternion, same as conjugate)
    Quaternion inverse() const {
        T n2 = norm_squared();
        return {-q1 / n2, -q2 / n2, -q3 / n2, q4 / n2};
    }

    // Quaternion derivative: dq/dt = 0.5 * q * omega_quat
    // where omega_quat = [omega_x, omega_y, omega_z, 0]
    Quaternion derivative(const Vector3<T>& omega) const {
        // dq/dt = 0.5 * q ⊗ [ωx, ωy, ωz, 0]
        Quaternion omega_quat{omega.x, omega.y, omega.z, T(0)};
        Quaternion result = (*this) * omega_quat;
        return {
            result.q1 * T(0.5),
            result.q2 * T(0.5),
            result.q3 * T(0.5),
            result.q4 * T(0.5)
        };
    }

    // Rotation matrix (body to reference): R = q.toRotationMatrix()
    // v_ref = R * v_body
    std::array<std::array<T, 3>, 3> to_rotation_matrix() const {
        T q1_2 = q1 * q1;
        T q2_2 = q2 * q2;
        T q3_2 = q3 * q3;
        T q4_2 = q4 * q4;

        T q1q2 = q1 * q2;
        T q1q3 = q1 * q3;
        T q1q4 = q1 * q4;
        T q2q3 = q2 * q3;
        T q2q4 = q2 * q4;
        T q3q4 = q3 * q4;

        return {{
            {q4_2 + q1_2 - q2_2 - q3_2, T(2) * (q1q2 - q3q4),       T(2) * (q1q3 + q2q4)},
            {T(2) * (q1q2 + q3q4),       q4_2 - q1_2 + q2_2 - q3_2, T(2) * (q2q3 - q1q4)},
            {T(2) * (q1q3 - q2q4),       T(2) * (q2q3 + q1q4),       q4_2 - q1_2 - q2_2 + q3_2}
        }};
    }

    // Rotate vector from body to reference frame
    Vector3<T> rotate_body_to_reference(const Vector3<T>& v_body) const {
        auto R = to_rotation_matrix();
        return {
            R[0][0] * v_body.x + R[0][1] * v_body.y + R[0][2] * v_body.z,
            R[1][0] * v_body.x + R[1][1] * v_body.y + R[1][2] * v_body.z,
            R[2][0] * v_body.x + R[2][1] * v_body.y + R[2][2] * v_body.z
        };
    }

    // Rotate vector from reference to body frame
    Vector3<T> rotate_reference_to_body(const Vector3<T>& v_ref) const {
        auto R = to_rotation_matrix();
        // R^T * v_ref
        return {
            R[0][0] * v_ref.x + R[1][0] * v_ref.y + R[2][0] * v_ref.z,
            R[0][1] * v_ref.x + R[1][1] * v_ref.y + R[2][1] * v_ref.z,
            R[0][2] * v_ref.x + R[1][2] * v_ref.y + R[2][2] * v_ref.z
        };
    }

    // Convert to array
    std::array<T, 4> to_array() const {
        return {q1, q2, q3, q4};
    }

    // Identity quaternion
    static Quaternion identity() { return {T(0), T(0), T(0), T(1)}; }

    // Create from axis-angle
    static Quaternion from_axis_angle(const Vector3<T>& axis, T angle) {
        using std::sin;
        using std::cos;
        T half_angle = angle * T(0.5);
        T s = sin(half_angle);
        Vector3<T> n = axis.normalized();
        return {n.x * s, n.y * s, n.z * s, cos(half_angle)};
    }

    // Create from Euler angles (elevation, azimuth for launcher)
    // Elevation: angle above horizontal (0° = horizontal, 90° = vertical)
    // Azimuth: angle from north, clockwise (0° = north, 90° = east)
    //
    // In ENU frame with identity quaternion, body X points East.
    // For a rocket at elevation θ and azimuth φ, body X should point to:
    //   [cos(θ)*sin(φ), cos(θ)*cos(φ), sin(θ)] in ENU
    //
    // We need to:
    // 1. Yaw by (90° - azimuth) to face the azimuth direction (from East baseline)
    // 2. Pitch by elevation to raise the nose
    static Quaternion from_launcher_angles(T elevation_deg, T azimuth_deg) {
        using std::sin;
        using std::cos;
        constexpr T deg2rad = T(3.14159265358979323846) / T(180);

        // Yaw: 90° - azimuth (because azimuth=0 means north, not east)
        T yaw = (T(90) - azimuth_deg) * deg2rad;
        T pitch = elevation_deg * deg2rad;

        // Half angles
        T y = yaw * T(0.5);
        T p = pitch * T(0.5);

        T cy = cos(y);
        T sy = sin(y);
        T cp = cos(p);
        T sp = sin(p);

        // Combined quaternion for Rz(yaw) then Ry(pitch)
        // This particular combination gives the correct body X direction
        return {
            sy * sp,           // q1
            -cy * sp,          // q2
            sy * cp,           // q3
            cy * cp            // q4
        };
    }
};

// Euler angles structure
template<Scalar T = double>
struct EulerAngles {
    T roll{}, pitch{}, yaw{};

    constexpr EulerAngles() = default;
    constexpr EulerAngles(T r, T p, T y) : roll(r), pitch(p), yaw(y) {}
};

// Extract Euler angles from quaternion (roll-pitch-yaw, XYZ convention)
template<Scalar T>
EulerAngles<T> to_euler_angles(const Quaternion<T>& q) {
    using std::atan2;
    using std::asin;

    T q1 = q.q1, q2 = q.q2, q3 = q.q3, q4 = q.q4;

    // Roll (rotation around X)
    T sinr_cosp = T(2) * (q4 * q1 + q2 * q3);
    T cosr_cosp = T(1) - T(2) * (q1 * q1 + q2 * q2);
    T roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (rotation around Y)
    T sinp = T(2) * (q4 * q2 - q3 * q1);
    T pitch;
    if (value_of(sinp) >= T(1))
        pitch = T(1.5707963267948966);  // 90 degrees
    else if (value_of(sinp) <= T(-1))
        pitch = T(-1.5707963267948966);
    else
        pitch = asin(sinp);

    // Yaw (rotation around Z)
    T siny_cosp = T(2) * (q4 * q3 + q1 * q2);
    T cosy_cosp = T(1) - T(2) * (q2 * q2 + q3 * q3);
    T yaw = atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

} // namespace sopot::rocket
