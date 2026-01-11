#include "../rocket/vector3.hpp"
#include "../rocket/quaternion.hpp"
#include "../rocket/rocket_tags.hpp"
#include "../rocket/translation_kinematics.hpp"
#include "../rocket/translation_dynamics.hpp"
#include "../rocket/rotation_kinematics.hpp"
#include "../rocket/rotation_dynamics.hpp"
#include "../rocket/gravity.hpp"
#include "../rocket/standard_atmosphere.hpp"
#include "../rocket/rocket_body.hpp"
#include "../core/typed_component.hpp"
#include "../core/solver.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot;
using namespace sopot::rocket;

// Test macro
#define ASSERT_NEAR(a, b, tolerance) \
    do { \
        double _a = value_of(a); \
        double _b = value_of(b); \
        if (std::abs(_a - _b) > (tolerance)) { \
            std::cerr << "ASSERTION FAILED at line " << __LINE__ << ": " \
                      << #a << " (" << _a << ") != " << #b << " (" << _b \
                      << ") with tolerance " << (tolerance) << std::endl; \
            std::abort(); \
        } \
    } while(0)

#define ASSERT_TRUE(cond) \
    do { \
        if (!(cond)) { \
            std::cerr << "ASSERTION FAILED at line " << __LINE__ << ": " << #cond << std::endl; \
            std::abort(); \
        } \
    } while(0)

/**
 * SimplifiedRocket: A minimal 6DOF rocket for testing
 *
 * This simplified version uses constant mass and no engine/aero,
 * just to verify the ODE integration structure works.
 */
template<Scalar T = double>
class SimplifiedRocket {
public:
    // Components
    TranslationKinematics<T> position;
    TranslationDynamics<T> velocity;
    RotationKinematics<T> attitude;
    RotationDynamics<T> angular_velocity;

    // Parameters
    T mass{T(300)};
    T gravity{T(9.80665)};
    Vector3<T> inertia{T(5), T(150), T(150)};  // Axisymmetric

    SimplifiedRocket(
        Vector3<T> initial_pos,
        Vector3<T> initial_vel,
        T elevation_deg,
        T azimuth_deg
    ) : position(initial_pos),
        velocity(initial_vel),
        attitude(Quaternion<T>::from_launcher_angles(elevation_deg, azimuth_deg)),
        angular_velocity(Vector3<T>::zero())
    {}

    size_t getStateDimension() const { return 13; }

    std::vector<T> getInitialState() const {
        std::vector<T> state;
        state.reserve(13);

        auto pos = position.getInitialLocalState();
        auto vel = velocity.getInitialLocalState();
        auto att = attitude.getInitialLocalState();
        auto omega = angular_velocity.getInitialLocalState();

        for (const auto& v : pos) state.push_back(v);
        for (const auto& v : vel) state.push_back(v);
        for (const auto& v : att) state.push_back(v);
        for (const auto& v : omega) state.push_back(v);

        return state;
    }

    // Compute derivatives
    std::vector<T> computeDerivatives(T t, const std::vector<T>& state) const {
        std::vector<T> derivs(13, T(0));

        // Extract state components
        Vector3<T> pos_enu{state[0], state[1], state[2]};
        Vector3<T> vel_enu{state[3], state[4], state[5]};
        Quaternion<T> q{state[6], state[7], state[8], state[9]};
        Vector3<T> omega{state[10], state[11], state[12]};

        // Position derivative: dPos/dt = velocity
        derivs[0] = vel_enu.x;
        derivs[1] = vel_enu.y;
        derivs[2] = vel_enu.z;

        // Velocity derivative: dVel/dt = F/m
        // Force = gravity only (points down in ENU)
        Vector3<T> F_gravity{T(0), T(0), -gravity * mass};
        Vector3<T> acceleration = F_gravity / mass;
        derivs[3] = acceleration.x;
        derivs[4] = acceleration.y;
        derivs[5] = acceleration.z;

        // Quaternion derivative: dq/dt = 0.5 * q ⊗ ω
        Quaternion<T> dq = q.derivative(omega);
        derivs[6] = dq.q1;
        derivs[7] = dq.q2;
        derivs[8] = dq.q3;
        derivs[9] = dq.q4;

        // Angular velocity derivative: I·dω/dt = T - ω × (I·ω)
        // No external torque for now
        T dOmegaX = -(inertia.z - inertia.y) * omega.y * omega.z / inertia.x;
        T dOmegaY = -(inertia.x - inertia.z) * omega.x * omega.z / inertia.y;
        T dOmegaZ = -(inertia.y - inertia.x) * omega.x * omega.y / inertia.z;
        derivs[10] = dOmegaX;
        derivs[11] = dOmegaY;
        derivs[12] = dOmegaZ;

        return derivs;
    }
};

void test_vector3() {
    std::cout << "\n1. Vector3 operations..." << std::endl;

    Vector3<double> a{1, 2, 3};
    Vector3<double> b{4, 5, 6};

    auto c = a + b;
    ASSERT_NEAR(c.x, 5, 1e-10);
    ASSERT_NEAR(c.y, 7, 1e-10);
    ASSERT_NEAR(c.z, 9, 1e-10);

    double dot_product = a.dot(b);
    ASSERT_NEAR(dot_product, 32, 1e-10);  // 1*4 + 2*5 + 3*6 = 32

    auto cross_product = a.cross(b);
    ASSERT_NEAR(cross_product.x, -3, 1e-10);  // 2*6 - 3*5 = -3
    ASSERT_NEAR(cross_product.y, 6, 1e-10);   // 3*4 - 1*6 = 6
    ASSERT_NEAR(cross_product.z, -3, 1e-10); // 1*5 - 2*4 = -3

    ASSERT_NEAR(a.norm(), std::sqrt(14), 1e-10);

    std::cout << "   ✓ Vector3 operations passed" << std::endl;
}

void test_quaternion() {
    std::cout << "\n2. Quaternion operations..." << std::endl;

    // Test identity quaternion
    auto q_identity = Quaternion<double>::identity();
    ASSERT_NEAR(q_identity.q4, 1.0, 1e-10);
    ASSERT_NEAR(q_identity.norm(), 1.0, 1e-10);

    // Test rotation from launcher angles (84° elevation, 0° azimuth)
    auto q = Quaternion<double>::from_launcher_angles(84.0, 0.0);
    ASSERT_NEAR(q.norm(), 1.0, 1e-10);

    // The rocket should point mostly up (6° from vertical)
    Vector3<double> body_x{1, 0, 0};  // Body X axis
    Vector3<double> ref = q.rotate_body_to_reference(body_x);

    // At 84° elevation, azimuth 0, rocket X should point:
    // mostly up (+Z in ENU), slightly north (+Y in ENU)
    std::cout << "   Rocket X in ENU: [" << ref.x << ", " << ref.y << ", " << ref.z << "]" << std::endl;
    ASSERT_TRUE(ref.z > 0.9);  // Should be mostly up

    // Test quaternion derivative
    Vector3<double> omega{0.1, 0.0, 0.0};  // Small roll rate
    auto dq = q_identity.derivative(omega);
    ASSERT_NEAR(dq.q1, 0.05, 1e-10);  // 0.5 * omega.x

    std::cout << "   ✓ Quaternion operations passed" << std::endl;
}

void test_atmosphere() {
    std::cout << "\n3. Standard atmosphere..." << std::endl;

    StandardAtmosphere<double> atm;

    // Sea level
    ASSERT_NEAR(atm.computeTemperature(0), 288.15, 0.1);
    ASSERT_NEAR(atm.computePressure(0), 101325, 1);
    ASSERT_NEAR(atm.computeDensity(0), 1.225, 0.01);

    // 10 km
    double T_10km = atm.computeTemperature(10000);
    double P_10km = atm.computePressure(10000);
    std::cout << "   At 10 km: T = " << T_10km << " K, P = " << P_10km << " Pa" << std::endl;
    ASSERT_NEAR(T_10km, 223.15, 1);  // ~223 K at 10km
    ASSERT_TRUE(P_10km < 30000);     // Should be much lower than sea level

    // 50 km altitude
    double T_50km = atm.computeTemperature(50000);
    double P_50km = atm.computePressure(50000);
    std::cout << "   At 50 km: T = " << T_50km << " K, P = " << P_50km << " Pa" << std::endl;
    ASSERT_TRUE(P_50km < 100);  // Very low pressure at 50km

    std::cout << "   ✓ Standard atmosphere passed" << std::endl;
}

void test_ballistic_trajectory() {
    std::cout << "\n4. Ballistic trajectory (gravity only)..." << std::endl;

    // Create simplified rocket
    SimplifiedRocket<double> rocket(
        {0, 0, 0},           // Initial position
        {0, 100, 300},       // Initial velocity: 100 m/s north, 300 m/s up
        84.0, 0.0            // Launcher angles (not used for velocity)
    );

    // Override velocity with the one we want
    rocket.velocity.setInitialVelocity({0, 100, 300});

    auto derivs_func = [&rocket](double t, StateView state) {
        std::vector<double> state_vec(state.begin(), state.end());
        return rocket.computeDerivatives(t, state_vec);
    };

    auto solver = createRK4Solver();
    auto result = solver.solve(
        derivs_func,
        rocket.getStateDimension(),
        0.0, 70.0, 0.1,  // Simulate for 70 seconds
        rocket.getInitialState()
    );

    std::cout << "   Integration: " << result.size() << " steps, " << result.total_time << " ms" << std::endl;

    // Find apogee
    double max_altitude = 0;
    double apogee_time = 0;
    for (size_t i = 0; i < result.size(); ++i) {
        double alt = result.states[i][2];  // Z component
        if (alt > max_altitude) {
            max_altitude = alt;
            apogee_time = result.times[i];
        }
    }

    // Expected apogee: h = v²/(2g) = 300²/(2*9.80665) ≈ 4591 m
    double expected_apogee = 300.0 * 300.0 / (2 * 9.80665);
    std::cout << "   Apogee: " << max_altitude << " m at t = " << apogee_time << " s" << std::endl;
    std::cout << "   Expected (analytical): " << expected_apogee << " m" << std::endl;

    ASSERT_NEAR(max_altitude, expected_apogee, 10);  // Within 10 m

    // Check final position (should have traveled north during flight)
    double final_north = result.states.back()[1];
    double expected_north = 100.0 * result.times.back();  // Constant northward velocity
    std::cout << "   Final north position: " << final_north << " m" << std::endl;
    ASSERT_NEAR(final_north, expected_north, 10);

    std::cout << "   ✓ Ballistic trajectory passed" << std::endl;
}

void test_rocket_rotation() {
    std::cout << "\n5. Rocket with initial rotation..." << std::endl;

    // Create rocket with small initial angular velocity
    SimplifiedRocket<double> rocket(
        {0, 0, 1000},        // Start at 1000m altitude
        {0, 0, 0},           // Zero velocity (free fall)
        90.0, 0.0            // Pointing straight up
    );

    // Add small roll rate
    rocket.angular_velocity.setInitialAngularVelocity({0.1, 0, 0});  // 0.1 rad/s roll

    auto derivs_func = [&rocket](double t, StateView state) {
        std::vector<double> state_vec(state.begin(), state.end());
        return rocket.computeDerivatives(t, state_vec);
    };

    auto solver = createRK4Solver();
    auto result = solver.solve(
        derivs_func,
        rocket.getStateDimension(),
        0.0, 10.0, 0.01,
        rocket.getInitialState()
    );

    // Check that quaternion remains normalized
    for (size_t i = 0; i < result.size(); i += 100) {
        double q_norm = std::sqrt(
            result.states[i][6] * result.states[i][6] +
            result.states[i][7] * result.states[i][7] +
            result.states[i][8] * result.states[i][8] +
            result.states[i][9] * result.states[i][9]
        );
        ASSERT_NEAR(q_norm, 1.0, 0.01);  // Should stay normalized
    }

    // Check angular velocity is conserved (no external torque)
    double final_omega_x = result.states.back()[10];
    ASSERT_NEAR(final_omega_x, 0.1, 0.001);

    std::cout << "   ✓ Rocket rotation passed" << std::endl;
}

void test_6dof_integration() {
    std::cout << "\n6. Full 6DOF integration..." << std::endl;

    // Create rocket: 84° elevation, 310° azimuth
    SimplifiedRocket<double> rocket(
        {0, 0, 0},           // Launch from origin
        {0, 0, 0},           // Zero initial velocity (will be set by launcher)
        84.0, 310.0          // Launch angles
    );

    // Set initial velocity in body frame (along rocket axis) = ~50 m/s leaving launcher
    Quaternion<double> q = Quaternion<double>::from_launcher_angles(84.0, 310.0);
    Vector3<double> v_body{50, 0, 0};  // 50 m/s along rocket axis
    Vector3<double> v_enu = q.rotate_body_to_reference(v_body);

    std::cout << "   Initial velocity ENU: [" << v_enu.x << ", " << v_enu.y << ", " << v_enu.z << "]" << std::endl;

    rocket.velocity.setInitialVelocity(v_enu);

    auto derivs_func = [&rocket](double t, StateView state) {
        std::vector<double> state_vec(state.begin(), state.end());
        return rocket.computeDerivatives(t, state_vec);
    };

    auto solver = createRK4Solver();
    auto result = solver.solve(
        derivs_func,
        rocket.getStateDimension(),
        0.0, 20.0, 0.01,
        rocket.getInitialState()
    );

    // Print trajectory summary
    std::cout << "   Time [s] | East [m] | North [m] | Up [m]" << std::endl;
    std::cout << "   ---------|----------|-----------|--------" << std::endl;
    for (size_t i = 0; i < result.size(); i += result.size() / 5) {
        std::cout << "   " << std::setw(7) << result.times[i]
                  << " | " << std::setw(8) << std::fixed << std::setprecision(1) << result.states[i][0]
                  << " | " << std::setw(9) << result.states[i][1]
                  << " | " << std::setw(6) << result.states[i][2]
                  << std::endl;
    }

    // Verify final state makes sense
    double final_altitude = result.states.back()[2];
    ASSERT_TRUE(final_altitude < 0 || result.times.back() < 15);  // Should come down or be still going

    std::cout << "   ✓ 6DOF integration passed" << std::endl;
}

int main() {
    std::cout << "=== Rocket Flight Simulation Test ===" << std::endl;

    try {
        test_vector3();
        test_quaternion();
        test_atmosphere();
        test_ballistic_trajectory();
        test_rocket_rotation();
        test_6dof_integration();

        std::cout << "\n=== ALL ROCKET FLIGHT TESTS PASSED ===" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
