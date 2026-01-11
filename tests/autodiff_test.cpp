#include "../core/dual.hpp"
#include "../core/units.hpp"
#include "../core/scalar.hpp"
#include "../core/typed_component.hpp"
#include "../core/linearization.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <span>

// MSVC requires _USE_MATH_DEFINES before cmath for M_PI
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace sopot;
using namespace sopot::units;
using namespace sopot::units::literals;

// Test macro
#define ASSERT_NEAR(a, b, tolerance) \
    do { \
        double _a = (a); double _b = (b); \
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

// ============================================================================
// Test 1: Basic Dual number operations
// ============================================================================
void testDualBasicOps() {
    std::cout << "Test 1: Dual number basic operations..." << std::endl;

    // Create variable x with derivative = 1
    Dual1 x = Dual1::variable(3.0);
    ASSERT_NEAR(x.value(), 3.0, 1e-10);
    ASSERT_NEAR(x.derivative(), 1.0, 1e-10);

    // Create constant c with derivative = 0
    Dual1 c = Dual1::constant(2.0);
    ASSERT_NEAR(c.value(), 2.0, 1e-10);
    ASSERT_NEAR(c.derivative(), 0.0, 1e-10);

    // Addition: d(x + c)/dx = 1
    auto sum = x + c;
    ASSERT_NEAR(sum.value(), 5.0, 1e-10);
    ASSERT_NEAR(sum.derivative(), 1.0, 1e-10);

    // Multiplication: d(x * c)/dx = c = 2
    auto prod = x * c;
    ASSERT_NEAR(prod.value(), 6.0, 1e-10);
    ASSERT_NEAR(prod.derivative(), 2.0, 1e-10);

    // Division: d(x / c)/dx = 1/c = 0.5
    auto quot = x / c;
    ASSERT_NEAR(quot.value(), 1.5, 1e-10);
    ASSERT_NEAR(quot.derivative(), 0.5, 1e-10);

    // x * x: d(x^2)/dx = 2x = 6
    auto square = x * x;
    ASSERT_NEAR(square.value(), 9.0, 1e-10);
    ASSERT_NEAR(square.derivative(), 6.0, 1e-10);

    std::cout << "   ✓ Basic operations passed" << std::endl;
}

// ============================================================================
// Test 2: Dual number math functions
// ============================================================================
void testDualMathFunctions() {
    std::cout << "Test 2: Dual number math functions..." << std::endl;

    double x_val = 0.5;
    Dual1 x = Dual1::variable(x_val);

    // sin(x): d(sin(x))/dx = cos(x)
    auto s = sin(x);
    ASSERT_NEAR(s.value(), std::sin(x_val), 1e-10);
    ASSERT_NEAR(s.derivative(), std::cos(x_val), 1e-10);

    // cos(x): d(cos(x))/dx = -sin(x)
    auto c = cos(x);
    ASSERT_NEAR(c.value(), std::cos(x_val), 1e-10);
    ASSERT_NEAR(c.derivative(), -std::sin(x_val), 1e-10);

    // exp(x): d(exp(x))/dx = exp(x)
    auto e = exp(x);
    ASSERT_NEAR(e.value(), std::exp(x_val), 1e-10);
    ASSERT_NEAR(e.derivative(), std::exp(x_val), 1e-10);

    // log(x): d(log(x))/dx = 1/x
    auto l = log(x);
    ASSERT_NEAR(l.value(), std::log(x_val), 1e-10);
    ASSERT_NEAR(l.derivative(), 1.0 / x_val, 1e-10);

    // sqrt(x): d(sqrt(x))/dx = 1 / (2 * sqrt(x))
    auto sq = sqrt(x);
    ASSERT_NEAR(sq.value(), std::sqrt(x_val), 1e-10);
    ASSERT_NEAR(sq.derivative(), 0.5 / std::sqrt(x_val), 1e-10);

    // pow(x, 3): d(x^3)/dx = 3x^2
    auto p = pow(x, 3.0);
    ASSERT_NEAR(p.value(), std::pow(x_val, 3.0), 1e-10);
    ASSERT_NEAR(p.derivative(), 3.0 * std::pow(x_val, 2.0), 1e-10);

    // atan2(y, x) where y is constant
    Dual1 y = Dual1::constant(0.3);  // dy/dx = 0 (constant)
    auto at = atan2(y, x);
    // d(atan2(y,x))/dx = -y / (x^2 + y^2) when y is constant
    double denom = x_val * x_val + 0.3 * 0.3;
    ASSERT_NEAR(at.derivative(), -0.3 / denom, 1e-10);

    std::cout << "   ✓ Math functions passed" << std::endl;
}

// ============================================================================
// Test 3: Multi-variable gradient computation
// ============================================================================
void testMultiVariableGradient() {
    std::cout << "Test 3: Multi-variable gradient..." << std::endl;

    // f(x, y, z) = x^2 + 2*y*z + z^3
    // ∂f/∂x = 2x, ∂f/∂y = 2z, ∂f/∂z = 2y + 3z^2

    double x_val = 2.0, y_val = 3.0, z_val = 1.0;

    // Set up variables with identity derivatives for gradient
    Dual3 x = Dual3::variable(x_val, 0);  // derivative index 0
    Dual3 y = Dual3::variable(y_val, 1);  // derivative index 1
    Dual3 z = Dual3::variable(z_val, 2);  // derivative index 2

    // Compute f
    Dual3 f = x * x + Dual3::constant(2.0) * y * z + z * z * z;

    // Check value
    double expected_value = x_val * x_val + 2.0 * y_val * z_val + z_val * z_val * z_val;
    ASSERT_NEAR(f.value(), expected_value, 1e-10);

    // Check gradient
    ASSERT_NEAR(f.derivative(0), 2.0 * x_val, 1e-10);          // ∂f/∂x = 2x = 4
    ASSERT_NEAR(f.derivative(1), 2.0 * z_val, 1e-10);          // ∂f/∂y = 2z = 2
    ASSERT_NEAR(f.derivative(2), 2.0 * y_val + 3.0 * z_val * z_val, 1e-10);  // ∂f/∂z = 2y + 3z^2 = 9

    std::cout << "   ✓ Multi-variable gradient passed" << std::endl;
}

// ============================================================================
// Test 4: Chain rule through composition
// ============================================================================
void testChainRule() {
    std::cout << "Test 4: Chain rule..." << std::endl;

    // f(x) = sin(exp(x^2))
    // f'(x) = cos(exp(x^2)) * exp(x^2) * 2x

    double x_val = 0.5;
    Dual1 x = Dual1::variable(x_val);

    auto f = sin(exp(x * x));

    double exp_x2 = std::exp(x_val * x_val);
    double expected_deriv = std::cos(exp_x2) * exp_x2 * 2.0 * x_val;

    ASSERT_NEAR(f.value(), std::sin(exp_x2), 1e-10);
    ASSERT_NEAR(f.derivative(), expected_deriv, 1e-10);

    std::cout << "   ✓ Chain rule passed" << std::endl;
}

// ============================================================================
// Test 5: Units system basic operations
// ============================================================================
void testUnitsBasic() {
    std::cout << "Test 5: Units basic operations..." << std::endl;

    // Length
    Meters<> d1(10.0);
    Meters<> d2(5.0);
    auto d_sum = d1 + d2;
    ASSERT_NEAR(d_sum.value(), 15.0, 1e-10);

    // Area = Length * Length
    auto area = d1 * d2;
    static_assert(std::is_same_v<decltype(area)::dimension, Area>);
    ASSERT_NEAR(area.value(), 50.0, 1e-10);

    // Velocity = Length / Time
    Seconds<> t(2.0);
    auto velocity = d1 / t;
    static_assert(std::is_same_v<decltype(velocity)::dimension, Velocity>);
    ASSERT_NEAR(velocity.value(), 5.0, 1e-10);

    // Force = Mass * Acceleration
    Kilograms<> mass(100.0);
    MetersPerSecond2<> accel(9.81);
    auto force = mass * accel;
    static_assert(std::is_same_v<decltype(force)::dimension, Force>);
    ASSERT_NEAR(force.value(), 981.0, 1e-10);

    std::cout << "   ✓ Units basic operations passed" << std::endl;
}

// ============================================================================
// Test 6: Units with user-defined literals
// ============================================================================
void testUnitsLiterals() {
    std::cout << "Test 6: Units literals..." << std::endl;

    auto distance = 100.0_m;
    ASSERT_NEAR(distance.value(), 100.0, 1e-10);

    auto distance_km = 1.5_km;
    ASSERT_NEAR(distance_km.value(), 1500.0, 1e-10);

    auto mass = 50.0_kg;
    ASSERT_NEAR(mass.value(), 50.0, 1e-10);

    auto time = 10.0_s;
    ASSERT_NEAR(time.value(), 10.0, 1e-10);

    auto angle_rad = 1.57_rad;
    ASSERT_NEAR(angle_rad.value(), 1.57, 1e-10);

    auto angle_deg = 90.0_deg;
    ASSERT_NEAR(angle_deg.value(), M_PI / 2.0, 1e-6);

    auto force = 1000.0_N;
    ASSERT_NEAR(force.value(), 1000.0, 1e-10);

    auto force_kn = 2.5_kN;
    ASSERT_NEAR(force_kn.value(), 2500.0, 1e-10);

    std::cout << "   ✓ Units literals passed" << std::endl;
}

// ============================================================================
// Test 7: Trigonometric functions with units
// ============================================================================
void testUnitsTrig() {
    std::cout << "Test 7: Units trigonometric functions..." << std::endl;

    auto angle = 30.0_deg;  // 30 degrees = π/6 radians

    auto sine = units::sin(angle);
    ASSERT_NEAR(sine.value(), 0.5, 1e-10);

    auto cosine = units::cos(angle);
    ASSERT_NEAR(cosine.value(), std::sqrt(3.0) / 2.0, 1e-10);

    std::cout << "   ✓ Units trigonometric functions passed" << std::endl;
}

// ============================================================================
// Test 8: Dual numbers with component system
// ============================================================================

// Simple oscillator component using Dual numbers for autodiff
template<Scalar T>
class AutodiffOscillator final : public TypedComponent<2, T> {
private:
    double m_mass;
    double m_spring_k;
    double m_x0;
    double m_v0;
    mutable size_t m_offset{0};

public:
    using Base = TypedComponent<2, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

    AutodiffOscillator(double mass, double k, double x0 = 1.0, double v0 = 0.0)
        : m_mass(mass), m_spring_k(k), m_x0(x0), m_v0(v0) {}

    void setOffset(size_t off) const { m_offset = off; }

    // Non-virtual derivatives method - required by TypedODESystem
    template<typename Registry>
    LocalDerivative derivatives(
        [[maybe_unused]] T t,
        std::span<const T> local,
        [[maybe_unused]] std::span<const T> global,
        [[maybe_unused]] const Registry& registry
    ) const {
        T x = local[0];
        T v = local[1];

        LocalDerivative derivs;
        derivs[0] = v;  // dx/dt = v
        derivs[1] = T(-m_spring_k / m_mass) * x;  // dv/dt = -k/m * x

        return derivs;
    }

    LocalState getInitialLocalState() const {
        LocalState state;
        state[0] = T(m_x0);
        state[1] = T(m_v0);
        return state;
    }

    std::string_view getComponentType() const { return "AutodiffOscillator"; }
    std::string_view getComponentName() const { return "oscillator"; }

    // State functions
    T compute(kinematics::Position, std::span<const T> global_state) const {
        return global_state[m_offset];
    }

    T compute(kinematics::Velocity, std::span<const T> global_state) const {
        return global_state[m_offset + 1];
    }

    T compute(kinematics::Acceleration, std::span<const T> global_state) const {
        T x = global_state[m_offset];
        return T(-m_spring_k / m_mass) * x;
    }

    T compute(energy::Kinetic, std::span<const T> global_state) const {
        T v = global_state[m_offset + 1];
        return T(0.5 * m_mass) * v * v;
    }

    T compute(energy::Potential, std::span<const T> global_state) const {
        T x = global_state[m_offset];
        return T(0.5 * m_spring_k) * x * x;
    }

    T compute(energy::Total, std::span<const T> global_state) const {
        return compute(energy::Kinetic{}, global_state) + compute(energy::Potential{}, global_state);
    }
};

void testAutodiffComponent() {
    std::cout << "Test 8: Autodiff with component system..." << std::endl;

    // Create oscillator with Dual<double, 2> for 2x2 Jacobian
    using Dual2 = Dual<double, 2>;
    AutodiffOscillator<Dual2> osc(1.0, 4.0, 1.0, 0.0);  // m=1, k=4, x0=1, v0=0

    // Create state with derivatives set up
    std::vector<Dual2> state(2);
    state[0] = Dual2::variable(1.0, 0);  // x = 1, ∂/∂x
    state[1] = Dual2::variable(0.0, 1);  // v = 0, ∂/∂v

    // Test state functions
    auto pos = osc.compute(kinematics::Position{}, state);
    ASSERT_NEAR(pos.value(), 1.0, 1e-10);
    ASSERT_NEAR(pos.derivative(0), 1.0, 1e-10);  // ∂x/∂x = 1
    ASSERT_NEAR(pos.derivative(1), 0.0, 1e-10);  // ∂x/∂v = 0

    auto accel = osc.compute(kinematics::Acceleration{}, state);
    // a = -k/m * x = -4 * x
    ASSERT_NEAR(accel.value(), -4.0, 1e-10);
    ASSERT_NEAR(accel.derivative(0), -4.0, 1e-10);  // ∂a/∂x = -4
    ASSERT_NEAR(accel.derivative(1), 0.0, 1e-10);   // ∂a/∂v = 0

    auto energy = osc.compute(energy::Total{}, state);
    // E = 0.5*m*v^2 + 0.5*k*x^2 = 0.5*1*0 + 0.5*4*1 = 2.0
    ASSERT_NEAR(energy.value(), 2.0, 1e-10);
    // ∂E/∂x = k*x = 4
    ASSERT_NEAR(energy.derivative(0), 4.0, 1e-10);
    // ∂E/∂v = m*v = 0
    ASSERT_NEAR(energy.derivative(1), 0.0, 1e-10);

    std::cout << "   ✓ Autodiff component passed" << std::endl;
}

// ============================================================================
// Test 9: Jacobian computation
// ============================================================================
void testJacobianComputation() {
    std::cout << "Test 9: Jacobian computation..." << std::endl;

    // Oscillator: dx/dt = v, dv/dt = -k/m * x
    // Jacobian: [[∂(dx/dt)/∂x, ∂(dx/dt)/∂v],
    //            [∂(dv/dt)/∂x, ∂(dv/dt)/∂v]]
    //         = [[0, 1],
    //            [-k/m, 0]]
    //         = [[0, 1],
    //            [-4, 0]] for k=4, m=1

    using Dual2 = Dual<double, 2>;

    // Create system with oscillator - this sets up the registry
    auto system = makeTypedODESystem<Dual2>(AutodiffOscillator<Dual2>(1.0, 4.0));

    // Set up state with derivative seeds
    std::vector<Dual2> state(2);
    state[0] = Dual2::variable(1.5, 0);  // x = 1.5
    state[1] = Dual2::variable(0.5, 1);  // v = 0.5

    // Compute derivatives through the system
    auto derivs = system.computeDerivatives(Dual2::constant(0.0), state);

    // Check Jacobian elements
    // dx/dt = v
    ASSERT_NEAR(derivs[0].derivative(0), 0.0, 1e-10);  // ∂(dx/dt)/∂x = 0
    ASSERT_NEAR(derivs[0].derivative(1), 1.0, 1e-10);  // ∂(dx/dt)/∂v = 1

    // dv/dt = -k/m * x = -4 * x
    ASSERT_NEAR(derivs[1].derivative(0), -4.0, 1e-10);  // ∂(dv/dt)/∂x = -4
    ASSERT_NEAR(derivs[1].derivative(1), 0.0, 1e-10);   // ∂(dv/dt)/∂v = 0

    std::cout << "   Jacobian:" << std::endl;
    std::cout << "   [[" << derivs[0].derivative(0) << ", " << derivs[0].derivative(1) << "]," << std::endl;
    std::cout << "    [" << derivs[1].derivative(0) << ", " << derivs[1].derivative(1) << "]]" << std::endl;

    std::cout << "   ✓ Jacobian computation passed" << std::endl;
}

// ============================================================================
// Test 10: Scalar type utilities
// ============================================================================
void testScalarUtilities() {
    std::cout << "Test 10: Scalar type utilities..." << std::endl;

    // Test value_of
    double d = 3.14;
    ASSERT_NEAR(value_of(d), 3.14, 1e-10);

    Dual1 dual = Dual1::variable(2.71);
    ASSERT_NEAR(value_of(dual), 2.71, 1e-10);

    Meters<> length(100.0);
    ASSERT_NEAR(value_of(length), 100.0, 1e-10);

    // Test derivative_of
    ASSERT_NEAR(derivative_of(d), 0.0, 1e-10);  // double has no derivative
    ASSERT_NEAR(derivative_of(dual), 1.0, 1e-10);  // variable has derivative = 1

    // Test num_derivatives_v
    static_assert(num_derivatives_v<double> == 0);
    static_assert(num_derivatives_v<Dual1> == 1);
    static_assert(num_derivatives_v<Dual3> == 3);

    // Test is_dual_v
    static_assert(!is_dual_v<double>);
    static_assert(is_dual_v<Dual1>);
    static_assert(is_dual_v<Dual<double, 6>>);

    // Test is_quantity_v
    static_assert(!is_quantity_v<double>);
    static_assert(is_quantity_v<Meters<>>);
    static_assert(is_quantity_v<Quantity<Force, double>>);

    std::cout << "   ✓ Scalar utilities passed" << std::endl;
}

// ============================================================================
// Test 11: SystemLinearizer for control design
// ============================================================================
void testSystemLinearizer() {
    std::cout << "Test 11: System linearizer for control design..." << std::endl;

    // Rocket pitch dynamics (simplified):
    // State: [θ, ω] (pitch angle, angular velocity)
    // Input: [δ] (TVC deflection)
    //
    // ẋ = f(x, u):
    //   θ̇ = ω
    //   ω̇ = -k*θ + b*δ   (restoring force + TVC control)
    //
    // Jacobians:
    //   A = ∂f/∂x = [[0, 1],
    //                [-k, 0]]
    //   B = ∂f/∂u = [[0],
    //                [b]]

    constexpr size_t StateSize = 2;
    constexpr size_t InputSize = 1;
    constexpr double k = 4.0;  // restoring constant
    constexpr double b = 2.0;  // control effectiveness

    using DualT = Dual<double, StateSize + InputSize>;
    using DualState = std::array<DualT, StateSize>;
    using DualInput = std::array<DualT, InputSize>;
    using DualDerivative = std::array<DualT, StateSize>;

    // Define dynamics function
    auto dynamics = [k, b](DualT /*t*/, const DualState& x, const DualInput& u) -> DualDerivative {
        DualT theta = x[0];
        DualT omega = x[1];
        DualT delta = u[0];

        DualDerivative xdot;
        xdot[0] = omega;                           // θ̇ = ω
        xdot[1] = DualT(-k) * theta + DualT(b) * delta;  // ω̇ = -k*θ + b*δ

        return xdot;
    };

    auto linearizer = makeLinearizer<StateSize, InputSize>(dynamics);

    // Linearize at equilibrium (θ=0, ω=0, δ=0)
    Vector<StateSize> x0 = {0.0, 0.0};
    Vector<InputSize> u0 = {0.0};
    auto result = linearizer.linearize(0.0, x0, u0);

    // Check A matrix
    ASSERT_NEAR(result.A[0][0], 0.0, 1e-10);   // ∂θ̇/∂θ = 0
    ASSERT_NEAR(result.A[0][1], 1.0, 1e-10);   // ∂θ̇/∂ω = 1
    ASSERT_NEAR(result.A[1][0], -k, 1e-10);    // ∂ω̇/∂θ = -k
    ASSERT_NEAR(result.A[1][1], 0.0, 1e-10);   // ∂ω̇/∂ω = 0

    // Check B matrix
    ASSERT_NEAR(result.B[0][0], 0.0, 1e-10);   // ∂θ̇/∂δ = 0
    ASSERT_NEAR(result.B[1][0], b, 1e-10);     // ∂ω̇/∂δ = b

    std::cout << "   A matrix:" << std::endl;
    std::cout << "   [[" << result.A[0][0] << ", " << result.A[0][1] << "]," << std::endl;
    std::cout << "    [" << result.A[1][0] << ", " << result.A[1][1] << "]]" << std::endl;
    std::cout << "   B matrix:" << std::endl;
    std::cout << "   [[" << result.B[0][0] << "]," << std::endl;
    std::cout << "    [" << result.B[1][0] << "]]" << std::endl;

    std::cout << "   ✓ System linearizer passed" << std::endl;
}

// ============================================================================
// Test 12: Linearization at non-equilibrium point
// ============================================================================
void testLinearizationNonEquilibrium() {
    std::cout << "Test 12: Linearization at non-equilibrium point..." << std::endl;

    // Nonlinear pendulum: ẍ = -sin(x)
    // State: [x, v] (angle, velocity)
    // Dynamics: ẋ = v, v̇ = -sin(x)
    // Jacobian: A = [[0, 1], [-cos(x), 0]]

    constexpr size_t StateSize = 2;

    using DualT = Dual<double, StateSize>;
    using DualState = std::array<DualT, StateSize>;
    using DualDerivative = std::array<DualT, StateSize>;

    auto dynamics = [](DualT /*t*/, const DualState& x) -> DualDerivative {
        DualDerivative xdot;
        xdot[0] = x[1];           // ẋ = v
        xdot[1] = -sin(x[0]);     // v̇ = -sin(x)
        return xdot;
    };

    auto linearizer = makeAutonomousLinearizer<StateSize>(dynamics);

    // Linearize at x = π/4 (45 degrees)
    double x_val = M_PI / 4.0;
    Vector<StateSize> x0 = {x_val, 0.0};
    auto A = linearizer.computeJacobian(0.0, x0);

    // Expected: A = [[0, 1], [-cos(π/4), 0]] = [[0, 1], [-√2/2, 0]]
    double expected_A10 = -std::cos(x_val);

    ASSERT_NEAR(A[0][0], 0.0, 1e-10);
    ASSERT_NEAR(A[0][1], 1.0, 1e-10);
    ASSERT_NEAR(A[1][0], expected_A10, 1e-10);
    ASSERT_NEAR(A[1][1], 0.0, 1e-10);

    std::cout << "   Linearization at x = π/4:" << std::endl;
    std::cout << "   A = [[" << A[0][0] << ", " << A[0][1] << "]," << std::endl;
    std::cout << "        [" << A[1][0] << ", " << A[1][1] << "]]" << std::endl;
    std::cout << "   Expected A[1][0] = -cos(π/4) = " << expected_A10 << std::endl;

    std::cout << "   ✓ Non-equilibrium linearization passed" << std::endl;
}

// ============================================================================
// Test 13: Cross-component state function access via registry
// ============================================================================

// Aerodynamics component that needs velocity from another component
template<Scalar T>
class AeroDragComponent final : public TypedComponent<0, T> {
private:
    double m_drag_coeff;
    double m_air_density;
    double m_reference_area;

public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

    AeroDragComponent(double cd = 1.0, double rho = 1.225, double area = 0.01)
        : m_drag_coeff(cd), m_air_density(rho), m_reference_area(area) {}

    void setOffset(size_t) const {} // No state

    LocalState getInitialLocalState() const {
        return {};
    }

    std::string_view getComponentType() const { return "AeroDragComponent"; }
    std::string_view getComponentName() const { return "aero"; }

    // Compute drag force using velocity from registry (cross-component access)
    template<typename Registry>
    T computeDragForce(std::span<const T> state, const Registry& registry) const {
        // Query velocity through the registry - not hardcoded index
        static_assert(Registry::template hasFunction<kinematics::Velocity>(),
                     "Registry must provide Velocity function");

        T velocity = registry.template computeFunction<kinematics::Velocity>(state);

        // Drag: F = -0.5 * rho * v * |v| * Cd * A
        T coeff = T(0.5 * m_air_density * m_drag_coeff * m_reference_area);
        return -coeff * velocity * abs(velocity);
    }
};

void testCrossComponentAccess() {
    std::cout << "Test 13: Cross-component state function access via registry..." << std::endl;

    using Dual2 = Dual<double, 2>;

    // Create oscillator that provides Velocity state function
    AutodiffOscillator<Dual2> oscillator(1.0, 4.0, 0.0, 5.0);  // x0=0, v0=5

    // Create aero component that queries Velocity
    AeroDragComponent<Dual2> aero(1.0, 1.225, 0.01);  // Cd=1, rho=1.225, A=0.01

    // Create system with both components
    auto system = makeTypedODESystem<Dual2>(
        std::move(oscillator),
        std::move(aero)
    );

    // Set up state with proper derivatives for autodiff
    // State: [x, v] with x=0, v=5
    // We want to compute gradients with respect to x and v
    std::vector<Dual2> state(2);
    state[0] = Dual2::variable(0.0, 0);  // x = 0, ∂/∂x = 1, ∂/∂v = 0
    state[1] = Dual2::variable(5.0, 1);  // v = 5, ∂/∂x = 0, ∂/∂v = 1

    // Verify velocity is 5.0
    auto velocity = system.computeStateFunction<kinematics::Velocity>(state);
    ASSERT_NEAR(value_of(velocity), 5.0, 1e-10);

    // Velocity's derivative with respect to v should be 1
    ASSERT_NEAR(velocity.derivative(1), 1.0, 1e-10);

    // Now test that aero component can query velocity through registry
    const auto& registry = system.getRegistry();

    // Manually compute drag force using registry
    auto& aero_ref = system.template getComponent<1>();
    auto drag = aero_ref.computeDragForce(state, registry);

    // Expected: F = -0.5 * 1.225 * 5 * |5| * 1.0 * 0.01 = -0.153125
    double expected_drag = -0.5 * 1.225 * 5.0 * 5.0 * 1.0 * 0.01;
    ASSERT_NEAR(value_of(drag), expected_drag, 1e-10);

    // Check derivative: F = -coeff * v * |v|
    // For v > 0: F = -coeff * v * v
    // dF/dv = -coeff * 2 * v = -0.5 * 1.225 * 1.0 * 0.01 * 2 * 5 = -0.06125
    double coeff = 0.5 * 1.225 * 1.0 * 0.01;
    double expected_deriv_wrt_v = -coeff * 2.0 * 5.0;
    ASSERT_NEAR(drag.derivative(1), expected_deriv_wrt_v, 1e-10);

    std::cout << "   Velocity from registry: " << value_of(velocity) << " m/s" << std::endl;
    std::cout << "   Drag force: " << value_of(drag) << " N" << std::endl;
    std::cout << "   ∂Drag/∂v: " << drag.derivative(1) << " N·s/m" << std::endl;
    std::cout << "   ✓ Cross-component access passed" << std::endl;
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "=== Autodiff and Units Test Suite ===" << std::endl;
    std::cout << std::setprecision(10);

    try {
        testDualBasicOps();
        testDualMathFunctions();
        testMultiVariableGradient();
        testChainRule();
        testUnitsBasic();
        testUnitsLiterals();
        testUnitsTrig();
        testAutodiffComponent();
        testJacobianComputation();
        testScalarUtilities();
        testSystemLinearizer();
        testLinearizationNonEquilibrium();
        testCrossComponentAccess();

        std::cout << "\n=== ALL TESTS PASSED ===" << std::endl;
        std::cout << "✓ Dual numbers with forward-mode autodiff" << std::endl;
        std::cout << "✓ Multi-variable gradient computation" << std::endl;
        std::cout << "✓ Chain rule through composition" << std::endl;
        std::cout << "✓ Compile-time dimensional analysis (units)" << std::endl;
        std::cout << "✓ Autodiff integration with component system" << std::endl;
        std::cout << "✓ Jacobian computation for LQR/control design" << std::endl;
        std::cout << "✓ System linearization (A,B matrices)" << std::endl;
        std::cout << "✓ Nonlinear system linearization at any point" << std::endl;
        std::cout << "✓ Cross-component state function access via registry" << std::endl;

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
