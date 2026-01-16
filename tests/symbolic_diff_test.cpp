#include "../physics/constraints/symbolic/expression.hpp"
#include "../physics/constraints/symbolic/differentiation.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace sopot::symbolic;

#define ASSERT_NEAR(a, b, tol) \
    do { \
        double _a = (a), _b = (b); \
        if (std::abs(_a - _b) > (tol)) { \
            std::cerr << "FAIL at line " << __LINE__ << ": " \
                      << #a << " = " << _a << " != " << #b << " = " << _b << std::endl; \
            std::abort(); \
        } \
    } while(0)

//=============================================================================
// Test 1: Expression Evaluation
//=============================================================================
void testExpressionEvaluation() {
    std::cout << "Test 1: Expression evaluation..." << std::endl;

    // Variables
    using x = Var<0>;
    using y = Var<1>;

    // Constants
    using two = Const<2>;
    using half = Const<1, 2>;

    std::array<double, 2> vars = {3.0, 4.0};

    // Test variable evaluation
    double x_val = eval<x>(vars);
    double y_val = eval<y>(vars);
    ASSERT_NEAR(x_val, 3.0, 1e-10);
    ASSERT_NEAR(y_val, 4.0, 1e-10);
    std::cout << "   x = " << x_val << ", y = " << y_val << std::endl;

    // Test constant evaluation
    double two_val = eval<two>(vars);
    double half_val = eval<half>(vars);
    ASSERT_NEAR(two_val, 2.0, 1e-10);
    ASSERT_NEAR(half_val, 0.5, 1e-10);
    std::cout << "   2 = " << two_val << ", 1/2 = " << half_val << std::endl;

    // Test addition: x + y = 7
    using sum = Add<x, y>;
    double sum_val = eval<sum>(vars);
    ASSERT_NEAR(sum_val, 7.0, 1e-10);
    std::cout << "   x + y = " << sum_val << std::endl;

    // Test subtraction: x - y = -1
    using diff = Sub<x, y>;
    double diff_val = eval<diff>(vars);
    ASSERT_NEAR(diff_val, -1.0, 1e-10);
    std::cout << "   x - y = " << diff_val << std::endl;

    // Test multiplication: x * y = 12
    using prod = Mul<x, y>;
    double prod_val = eval<prod>(vars);
    ASSERT_NEAR(prod_val, 12.0, 1e-10);
    std::cout << "   x * y = " << prod_val << std::endl;

    // Test division: x / y = 0.75
    using quot = Div<x, y>;
    double quot_val = eval<quot>(vars);
    ASSERT_NEAR(quot_val, 0.75, 1e-10);
    std::cout << "   x / y = " << quot_val << std::endl;

    // Test negation: -x = -3
    using neg = Neg<x>;
    double neg_val = eval<neg>(vars);
    ASSERT_NEAR(neg_val, -3.0, 1e-10);
    std::cout << "   -x = " << neg_val << std::endl;

    // Test power: x^2 = 9
    using sq = Pow<x, 2>;
    double sq_val = eval<sq>(vars);
    ASSERT_NEAR(sq_val, 9.0, 1e-10);
    std::cout << "   x^2 = " << sq_val << std::endl;

    std::cout << "   [OK] Expression evaluation passed" << std::endl;
}

//=============================================================================
// Test 2: Trigonometric Functions
//=============================================================================
void testTrigFunctions() {
    std::cout << "\nTest 2: Trigonometric functions..." << std::endl;

    using x = Var<0>;
    std::array<double, 1> vars = {M_PI / 4};  // 45 degrees

    // sin(pi/4) = sqrt(2)/2
    using sine = Sin<x>;
    double sin_val = eval<sine>(vars);
    ASSERT_NEAR(sin_val, std::sqrt(2.0) / 2.0, 1e-10);
    std::cout << "   sin(pi/4) = " << sin_val << std::endl;

    // cos(pi/4) = sqrt(2)/2
    using cosine = Cos<x>;
    double cos_val = eval<cosine>(vars);
    ASSERT_NEAR(cos_val, std::sqrt(2.0) / 2.0, 1e-10);
    std::cout << "   cos(pi/4) = " << cos_val << std::endl;

    // Test sin^2 + cos^2 = 1
    using sin_sq = Square<sine>;
    using cos_sq = Square<cosine>;
    using identity = Add<sin_sq, cos_sq>;
    double identity_val = eval<identity>(vars);
    ASSERT_NEAR(identity_val, 1.0, 1e-10);
    std::cout << "   sin^2 + cos^2 = " << identity_val << std::endl;

    std::cout << "   [OK] Trigonometric functions passed" << std::endl;
}

//=============================================================================
// Test 3: Basic Differentiation
//=============================================================================
void testBasicDifferentiation() {
    std::cout << "\nTest 3: Basic differentiation rules..." << std::endl;

    using x = Var<0>;
    using y = Var<1>;
    std::array<double, 2> vars = {2.0, 3.0};

    // d/dx(x) = 1
    using dx_dx = Diff_t<x, 0>;
    double dx_dx_val = eval<dx_dx>(vars);
    ASSERT_NEAR(dx_dx_val, 1.0, 1e-10);
    std::cout << "   d/dx(x) = " << dx_dx_val << std::endl;

    // d/dy(x) = 0
    using dx_dy = Diff_t<x, 1>;
    double dx_dy_val = eval<dx_dy>(vars);
    ASSERT_NEAR(dx_dy_val, 0.0, 1e-10);
    std::cout << "   d/dy(x) = " << dx_dy_val << std::endl;

    // d/dx(constant) = 0
    using dc_dx = Diff_t<Const<5>, 0>;
    double dc_dx_val = eval<dc_dx>(vars);
    ASSERT_NEAR(dc_dx_val, 0.0, 1e-10);
    std::cout << "   d/dx(5) = " << dc_dx_val << std::endl;

    // d/dx(x + y) = 1
    using sum = Add<x, y>;
    using dsum_dx = Diff_t<sum, 0>;
    double dsum_dx_val = eval<dsum_dx>(vars);
    ASSERT_NEAR(dsum_dx_val, 1.0, 1e-10);
    std::cout << "   d/dx(x + y) = " << dsum_dx_val << std::endl;

    // d/dy(x + y) = 1
    using dsum_dy = Diff_t<sum, 1>;
    double dsum_dy_val = eval<dsum_dy>(vars);
    ASSERT_NEAR(dsum_dy_val, 1.0, 1e-10);
    std::cout << "   d/dy(x + y) = " << dsum_dy_val << std::endl;

    std::cout << "   [OK] Basic differentiation passed" << std::endl;
}

//=============================================================================
// Test 4: Product and Quotient Rules
//=============================================================================
void testProductQuotientRules() {
    std::cout << "\nTest 4: Product and quotient rules..." << std::endl;

    using x = Var<0>;
    using y = Var<1>;
    std::array<double, 2> vars = {2.0, 3.0};

    // Product rule: d/dx(x * y) = y
    using prod = Mul<x, y>;
    using dprod_dx = Diff_t<prod, 0>;
    double dprod_dx_val = eval<dprod_dx>(vars);
    ASSERT_NEAR(dprod_dx_val, 3.0, 1e-10);
    std::cout << "   d/dx(x * y) = " << dprod_dx_val << " (expected: y = 3)" << std::endl;

    // Product rule: d/dy(x * y) = x
    using dprod_dy = Diff_t<prod, 1>;
    double dprod_dy_val = eval<dprod_dy>(vars);
    ASSERT_NEAR(dprod_dy_val, 2.0, 1e-10);
    std::cout << "   d/dy(x * y) = " << dprod_dy_val << " (expected: x = 2)" << std::endl;

    // Quotient rule: d/dx(x / y) = 1/y
    using quot = Div<x, y>;
    using dquot_dx = Diff_t<quot, 0>;
    double dquot_dx_val = eval<dquot_dx>(vars);
    ASSERT_NEAR(dquot_dx_val, 1.0/3.0, 1e-10);
    std::cout << "   d/dx(x / y) = " << dquot_dx_val << " (expected: 1/y = 1/3)" << std::endl;

    // Quotient rule: d/dy(x / y) = -x/y^2
    using dquot_dy = Diff_t<quot, 1>;
    double dquot_dy_val = eval<dquot_dy>(vars);
    ASSERT_NEAR(dquot_dy_val, -2.0/9.0, 1e-10);
    std::cout << "   d/dy(x / y) = " << dquot_dy_val << " (expected: -x/y^2 = -2/9)" << std::endl;

    std::cout << "   [OK] Product and quotient rules passed" << std::endl;
}

//=============================================================================
// Test 5: Power Rule
//=============================================================================
void testPowerRule() {
    std::cout << "\nTest 5: Power rule..." << std::endl;

    using x = Var<0>;
    std::array<double, 1> vars = {3.0};

    // d/dx(x^2) = 2x
    using x_sq = Pow<x, 2>;
    using dx_sq = Diff_t<x_sq, 0>;
    double dx_sq_val = eval<dx_sq>(vars);
    ASSERT_NEAR(dx_sq_val, 6.0, 1e-10);
    std::cout << "   d/dx(x^2) = " << dx_sq_val << " (expected: 2x = 6)" << std::endl;

    // d/dx(x^3) = 3x^2
    using x_cube = Pow<x, 3>;
    using dx_cube = Diff_t<x_cube, 0>;
    double dx_cube_val = eval<dx_cube>(vars);
    ASSERT_NEAR(dx_cube_val, 27.0, 1e-10);
    std::cout << "   d/dx(x^3) = " << dx_cube_val << " (expected: 3x^2 = 27)" << std::endl;

    // d/dx(x^0) = 0
    using x_0 = Pow<x, 0>;
    using dx_0 = Diff_t<x_0, 0>;
    double dx_0_val = eval<dx_0>(vars);
    ASSERT_NEAR(dx_0_val, 0.0, 1e-10);
    std::cout << "   d/dx(x^0) = " << dx_0_val << " (expected: 0)" << std::endl;

    std::cout << "   [OK] Power rule passed" << std::endl;
}

//=============================================================================
// Test 6: Chain Rule (Trigonometric)
//=============================================================================
void testChainRule() {
    std::cout << "\nTest 6: Chain rule with trig functions..." << std::endl;

    using x = Var<0>;
    std::array<double, 1> vars = {M_PI / 3};  // 60 degrees

    // d/dx(sin(x)) = cos(x)
    using sinx = Sin<x>;
    using dsinx = Diff_t<sinx, 0>;
    double dsinx_val = eval<dsinx>(vars);
    double expected_cos = std::cos(vars[0]);
    ASSERT_NEAR(dsinx_val, expected_cos, 1e-10);
    std::cout << "   d/dx(sin(x)) = " << dsinx_val << " (expected: cos(x) = " << expected_cos << ")" << std::endl;

    // d/dx(cos(x)) = -sin(x)
    using cosx = Cos<x>;
    using dcosx = Diff_t<cosx, 0>;
    double dcosx_val = eval<dcosx>(vars);
    double expected_neg_sin = -std::sin(vars[0]);
    ASSERT_NEAR(dcosx_val, expected_neg_sin, 1e-10);
    std::cout << "   d/dx(cos(x)) = " << dcosx_val << " (expected: -sin(x) = " << expected_neg_sin << ")" << std::endl;

    // d/dx(sin(2x)) = 2*cos(2x)
    using two_x = Mul<Const<2>, x>;
    using sin_2x = Sin<two_x>;
    using dsin_2x = Diff_t<sin_2x, 0>;
    double dsin_2x_val = eval<dsin_2x>(vars);
    double expected = 2.0 * std::cos(2.0 * vars[0]);
    ASSERT_NEAR(dsin_2x_val, expected, 1e-10);
    std::cout << "   d/dx(sin(2x)) = " << dsin_2x_val << " (expected: 2*cos(2x) = " << expected << ")" << std::endl;

    std::cout << "   [OK] Chain rule passed" << std::endl;
}

//=============================================================================
// Test 7: Gradient Computation
//=============================================================================
void testGradient() {
    std::cout << "\nTest 7: Gradient computation..." << std::endl;

    using x = Var<0>;
    using y = Var<1>;

    // f(x, y) = x^2 + 2*x*y + y^2 = (x + y)^2
    // grad f = (2x + 2y, 2x + 2y) = 2*(x + y, x + y)
    using x_sq = Square<x>;
    using y_sq = Square<y>;
    using xy = Mul<x, y>;
    using two_xy = Mul<Const<2>, xy>;
    using f = Add<Add<x_sq, two_xy>, y_sq>;

    std::array<double, 2> vars = {1.0, 2.0};

    auto grad = Gradient<f, 2>::eval(vars);
    double expected_dx = 2 * (vars[0] + vars[1]);  // 6
    double expected_dy = 2 * (vars[0] + vars[1]);  // 6

    ASSERT_NEAR(grad[0], expected_dx, 1e-10);
    ASSERT_NEAR(grad[1], expected_dy, 1e-10);

    std::cout << "   f(x,y) = x^2 + 2xy + y^2" << std::endl;
    std::cout << "   grad f at (1, 2) = (" << grad[0] << ", " << grad[1] << ")" << std::endl;
    std::cout << "   Expected: (" << expected_dx << ", " << expected_dy << ")" << std::endl;

    std::cout << "   [OK] Gradient computation passed" << std::endl;
}

//=============================================================================
// Test 8: Constraint Jacobian
//=============================================================================
void testConstraintJacobian() {
    std::cout << "\nTest 8: Constraint Jacobian for pendulum..." << std::endl;

    // Pendulum constraint: g(x, y) = x^2 + y^2 - L^2 = 0
    // For simplicity, set L = 1
    // grad g = (2x, 2y)

    using x = Var<0>;
    using y = Var<1>;
    using constraint = Sub<Add<Square<x>, Square<y>>, One>;

    std::array<double, 2> pos = {0.6, -0.8};  // Point on unit circle

    // Verify constraint is satisfied
    double g_val = eval<constraint>(pos);
    ASSERT_NEAR(g_val, 0.0, 1e-10);
    std::cout << "   Constraint value at (0.6, -0.8): " << g_val << " (should be 0)" << std::endl;

    // Compute Jacobian row
    auto jac_row = JacobianRow<constraint, 2>::eval(pos);
    double expected_dx = 2 * pos[0];  // 1.2
    double expected_dy = 2 * pos[1];  // -1.6

    ASSERT_NEAR(jac_row[0], expected_dx, 1e-10);
    ASSERT_NEAR(jac_row[1], expected_dy, 1e-10);

    std::cout << "   Jacobian row: [" << jac_row[0] << ", " << jac_row[1] << "]" << std::endl;
    std::cout << "   Expected: [" << expected_dx << ", " << expected_dy << "]" << std::endl;

    // Verify Jacobian is perpendicular to velocity constraint
    // For circular motion, velocity is tangent: v = (-y*omega, x*omega)
    // J dot v = 2x*(-y*omega) + 2y*(x*omega) = 0
    double omega = 1.0;
    double vx = -pos[1] * omega;
    double vy = pos[0] * omega;
    double J_dot_v = jac_row[0] * vx + jac_row[1] * vy;
    ASSERT_NEAR(J_dot_v, 0.0, 1e-10);
    std::cout << "   J dot v (velocity tangent check): " << J_dot_v << " (should be 0)" << std::endl;

    std::cout << "   [OK] Constraint Jacobian passed" << std::endl;
}

//=============================================================================
// Test 9: Second Derivatives (Hessian)
//=============================================================================
void testHessian() {
    std::cout << "\nTest 9: Hessian (second derivatives)..." << std::endl;

    using x = Var<0>;
    using y = Var<1>;

    // f(x, y) = x^2 * y + y^3
    // df/dx = 2xy, df/dy = x^2 + 3y^2
    // d2f/dx2 = 2y, d2f/dxdy = 2x, d2f/dy2 = 6y
    using f = Add<Mul<Square<x>, y>, Pow<y, 3>>;

    std::array<double, 2> vars = {2.0, 3.0};

    auto hess = Hessian<f, 2>::eval(vars);

    double expected_xx = 2 * vars[1];           // 6
    double expected_xy = 2 * vars[0];           // 4
    double expected_yy = 6 * vars[1];           // 18

    ASSERT_NEAR(hess[0][0], expected_xx, 1e-10);
    ASSERT_NEAR(hess[0][1], expected_xy, 1e-10);
    ASSERT_NEAR(hess[1][0], expected_xy, 1e-10);  // Symmetric
    ASSERT_NEAR(hess[1][1], expected_yy, 1e-10);

    std::cout << "   f(x,y) = x^2 * y + y^3" << std::endl;
    std::cout << "   Hessian at (2, 3):" << std::endl;
    std::cout << "   [" << hess[0][0] << "  " << hess[0][1] << "]" << std::endl;
    std::cout << "   [" << hess[1][0] << "  " << hess[1][1] << "]" << std::endl;
    std::cout << "   Expected:" << std::endl;
    std::cout << "   [" << expected_xx << "  " << expected_xy << "]" << std::endl;
    std::cout << "   [" << expected_xy << "  " << expected_yy << "]" << std::endl;

    std::cout << "   [OK] Hessian computation passed" << std::endl;
}

//=============================================================================
// Test 10: Compile-Time Type Verification
//=============================================================================
void testCompileTimeTypes() {
    std::cout << "\nTest 10: Compile-time type verification..." << std::endl;

    // Verify that d/dx(x) = 1 at compile time
    using x = Var<0>;
    using dx_dx = Diff_t<x, 0>;
    static_assert(std::is_same_v<dx_dx, One>, "d/dx(x) should be One");
    std::cout << "   d/dx(x) type = One (verified at compile time)" << std::endl;

    // Verify that d/dy(x) = 0 at compile time
    using dx_dy = Diff_t<x, 1>;
    static_assert(std::is_same_v<dx_dy, Zero>, "d/dy(x) should be Zero");
    std::cout << "   d/dy(x) type = Zero (verified at compile time)" << std::endl;

    // Verify that d/dx(constant) = 0
    using dc_dx = Diff_t<Const<42>, 0>;
    static_assert(std::is_same_v<dc_dx, Zero>, "d/dx(constant) should be Zero");
    std::cout << "   d/dx(42) type = Zero (verified at compile time)" << std::endl;

    // Verify simplification: 0 + x = x
    using sum_0_x = Add<Zero, x>;
    using simplified = Simplify_t<sum_0_x>;
    static_assert(std::is_same_v<simplified, x>, "0 + x should simplify to x");
    std::cout << "   Simplify(0 + x) = x (verified at compile time)" << std::endl;

    // Verify simplification: 1 * x = x
    using prod_1_x = Mul<One, x>;
    using simplified2 = Simplify_t<prod_1_x>;
    static_assert(std::is_same_v<simplified2, x>, "1 * x should simplify to x");
    std::cout << "   Simplify(1 * x) = x (verified at compile time)" << std::endl;

    std::cout << "   [OK] Compile-time type verification passed" << std::endl;
}

//=============================================================================
// Test 11: Double Pendulum Constraint Jacobian
//=============================================================================
void testDoublePendulumJacobian() {
    std::cout << "\nTest 11: Double pendulum constraint Jacobian..." << std::endl;

    // Variables: x1, y1, x2, y2
    using x1 = Var<0>;
    using y1 = Var<1>;
    using x2 = Var<2>;
    using y2 = Var<3>;

    // Constraint 1: g1 = x1^2 + y1^2 - L1^2
    // dg1/d[x1,y1,x2,y2] = [2x1, 2y1, 0, 0]
    using g1 = Sub<Add<Square<x1>, Square<y1>>, One>;

    // Constraint 2: g2 = (x2-x1)^2 + (y2-y1)^2 - L2^2
    // dg2/d[x1,y1,x2,y2] = [-2(x2-x1), -2(y2-y1), 2(x2-x1), 2(y2-y1)]
    using dx = Sub<x2, x1>;
    using dy = Sub<y2, y1>;
    using g2 = Sub<Add<Square<dx>, Square<dy>>, One>;

    // Test point: mass1 at (0.6, -0.8), mass2 at (1.4, -1.4)
    // This gives dx = 0.8, dy = -0.6
    std::array<double, 4> pos = {0.6, -0.8, 1.4, -1.4};

    // Verify constraints (approximately - depends on lengths)
    double g1_val = eval<g1>(pos);
    double g2_val = eval<g2>(pos);
    std::cout << "   g1 = " << g1_val << " (L1 = 1)" << std::endl;
    std::cout << "   g2 = " << g2_val << " (L2 = 1)" << std::endl;

    // Compute Jacobian using our symbolic system
    auto J = Jacobian<4, g1, g2>::eval(pos);

    std::cout << "   Jacobian:" << std::endl;
    std::cout << "   J1 = [" << J[0][0] << ", " << J[0][1] << ", " << J[0][2] << ", " << J[0][3] << "]" << std::endl;
    std::cout << "   J2 = [" << J[1][0] << ", " << J[1][1] << ", " << J[1][2] << ", " << J[1][3] << "]" << std::endl;

    // Verify expected values
    double dx_val = pos[2] - pos[0];  // 0.8
    double dy_val = pos[3] - pos[1];  // -0.6

    ASSERT_NEAR(J[0][0], 2 * pos[0], 1e-10);  // 1.2
    ASSERT_NEAR(J[0][1], 2 * pos[1], 1e-10);  // -1.6
    ASSERT_NEAR(J[0][2], 0.0, 1e-10);
    ASSERT_NEAR(J[0][3], 0.0, 1e-10);

    ASSERT_NEAR(J[1][0], -2 * dx_val, 1e-10);  // -1.6
    ASSERT_NEAR(J[1][1], -2 * dy_val, 1e-10);  // 1.2
    ASSERT_NEAR(J[1][2], 2 * dx_val, 1e-10);   // 1.6
    ASSERT_NEAR(J[1][3], 2 * dy_val, 1e-10);   // -1.2

    std::cout << "   [OK] Double pendulum constraint Jacobian passed" << std::endl;
}

//=============================================================================
// Main
//=============================================================================
int main() {
    std::cout << "=== Symbolic Differentiation Test Suite ===" << std::endl;
    std::cout << "Validating compile-time computer algebra system\n" << std::endl;

    testExpressionEvaluation();
    testTrigFunctions();
    testBasicDifferentiation();
    testProductQuotientRules();
    testPowerRule();
    testChainRule();
    testGradient();
    testConstraintJacobian();
    testHessian();
    testCompileTimeTypes();
    testDoublePendulumJacobian();

    std::cout << "\n=== ALL TESTS PASSED ===" << std::endl;
    std::cout << "Demonstrated:" << std::endl;
    std::cout << "  - Expression template evaluation" << std::endl;
    std::cout << "  - Compile-time symbolic differentiation" << std::endl;
    std::cout << "  - Product, quotient, power, and chain rules" << std::endl;
    std::cout << "  - Gradient and Jacobian computation" << std::endl;
    std::cout << "  - Hessian (second derivatives)" << std::endl;
    std::cout << "  - Compile-time type verification" << std::endl;
    std::cout << "  - Double pendulum constraint Jacobian" << std::endl;

    return 0;
}
