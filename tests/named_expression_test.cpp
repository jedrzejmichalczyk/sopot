/**
 * @file named_expression_test.cpp
 * @brief Tests for the named symbolic expression system
 *
 * Verifies that named expressions provide the same results as indexed
 * expressions while offering improved ergonomics.
 */

#include "physics/constraints/symbolic/named_expression.hpp"
#include "physics/constraints/symbolic/expression.hpp"
#include "physics/constraints/symbolic/differentiation.hpp"
#include "physics/pendulum/named_constraint_pendulum.hpp"
#include "physics/pendulum/symbolic_cartesian_pendulum.hpp"
#include "core/typed_component.hpp"
#include "core/solver.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <iomanip>

using namespace sopot::symbolic;

// ============================================================================
// Test utilities
// ============================================================================

constexpr double EPSILON = 1e-10;

bool approx_equal(double a, double b, double eps = EPSILON) {
    return std::abs(a - b) < eps;
}

void test_passed(const char* name) {
    std::cout << "  [PASS] " << name << std::endl;
}

// ============================================================================
// Test: Named symbols create correct types
// ============================================================================
void test_named_symbol_types() {
    std::cout << "Test: Named symbol types..." << std::endl;

    // Verify that NamedSymbol<I> maps to Var<I>
    static_assert(NamedSymbol<0>::index == 0);
    static_assert(NamedSymbol<1>::index == 1);
    static_assert(NamedSymbol<42>::index == 42);

    // Verify predefined symbols have correct indices
    namespace tb = cartesian::two_body_2d;
    static_assert(tb::x1.index == 0);
    static_assert(tb::y1.index == 1);
    static_assert(tb::x2.index == 2);
    static_assert(tb::y2.index == 3);

    test_passed("Named symbol types");
}

// ============================================================================
// Test: Named expressions evaluate correctly
// ============================================================================
void test_named_expression_evaluation() {
    std::cout << "Test: Named expression evaluation..." << std::endl;

    // Use namespace alias to avoid Bessel function collision with y0, y1
    namespace tb = cartesian::two_body_2d;

    // Test simple addition: x1 + y1
    auto sum = tb::x1 + tb::y1;
    std::array<double, 4> vars = {3.0, 4.0, 5.0, 6.0};

    double result = sum.eval(vars);
    assert(approx_equal(result, 7.0));

    // Test multiplication: x1 * y1
    auto prod = tb::x1 * tb::y1;
    assert(approx_equal(prod.eval(vars), 12.0));

    // Test subtraction: x2 - x1
    auto diff = tb::x2 - tb::x1;
    assert(approx_equal(diff.eval(vars), 2.0));

    // Test division: y2 / y1
    auto quot = tb::y2 / tb::y1;
    assert(approx_equal(quot.eval(vars), 1.5));

    // Test negation: -x1
    auto neg = -tb::x1;
    assert(approx_equal(neg.eval(vars), -3.0));

    test_passed("Named expression evaluation");
}

// ============================================================================
// Test: Square function
// ============================================================================
void test_square_function() {
    std::cout << "Test: Square function..." << std::endl;

    namespace tb = cartesian::two_body_2d;

    std::array<double, 4> vars = {3.0, 4.0, 5.0, 6.0};

    // sq(x1) should give x1^2 = 9
    auto x1_sq = sq(tb::x1);
    assert(approx_equal(x1_sq.eval(vars), 9.0));

    // sq(x2 - x1) should give (5-3)^2 = 4
    auto diff_sq = sq(tb::x2 - tb::x1);
    assert(approx_equal(diff_sq.eval(vars), 4.0));

    test_passed("Square function");
}

// ============================================================================
// Test: Complex constraint expression
// ============================================================================
void test_constraint_expression() {
    std::cout << "Test: Constraint expression..." << std::endl;

    namespace tb = cartesian::two_body_2d;

    // Double pendulum constraint: g1 = x1² + y1² (should equal L1² when satisfied)
    auto g1 = sq(tb::x1) + sq(tb::y1);

    // Test point on unit circle
    std::array<double, 4> on_circle = {0.6, -0.8, 0.0, 0.0};
    double g1_val = g1.eval(on_circle);
    assert(approx_equal(g1_val, 1.0));  // 0.36 + 0.64 = 1.0

    // Second constraint: g2 = (x2-x1)² + (y2-y1)²
    auto g2 = sq(tb::x2 - tb::x1) + sq(tb::y2 - tb::y1);

    std::array<double, 4> two_body = {0.6, -0.8, 1.4, -1.4};
    double g2_val = g2.eval(two_body);
    // (1.4-0.6)² + (-1.4-(-0.8))² = 0.64 + 0.36 = 1.0
    assert(approx_equal(g2_val, 1.0));

    test_passed("Constraint expression");
}

// ============================================================================
// Test: Named vs Indexed equivalence
// ============================================================================
void test_named_indexed_equivalence() {
    std::cout << "Test: Named vs Indexed equivalence..." << std::endl;

    namespace tb = cartesian::two_body_2d;

    // Named constraint
    auto g1_named = sq(tb::x1) + sq(tb::y1);

    // Indexed constraint (same as in symbolic_cartesian_pendulum.hpp)
    using g1_indexed = Add<Square<Var<0>>, Square<Var<1>>>;

    std::array<double, 4> vars = {0.6, -0.8, 1.4, -1.4};

    double named_result = g1_named.eval(vars);
    double indexed_result = eval<g1_indexed>(vars);

    assert(approx_equal(named_result, indexed_result));

    // Same for g2
    auto g2_named = sq(tb::x2 - tb::x1) + sq(tb::y2 - tb::y1);
    using dx = Sub<Var<2>, Var<0>>;
    using dy = Sub<Var<3>, Var<1>>;
    using g2_indexed = Add<Square<dx>, Square<dy>>;

    double g2_named_result = g2_named.eval(vars);
    double g2_indexed_result = eval<g2_indexed>(vars);

    assert(approx_equal(g2_named_result, g2_indexed_result));

    test_passed("Named vs Indexed equivalence");
}

// ============================================================================
// Test: Jacobian from named expressions
// ============================================================================
void test_named_jacobian() {
    std::cout << "Test: Jacobian from named expressions..." << std::endl;

    namespace tb = cartesian::two_body_2d;

    // Define constraints using named symbols
    auto g1 = sq(tb::x1) + sq(tb::y1);
    auto g2 = sq(tb::x2 - tb::x1) + sq(tb::y2 - tb::y1);

    // Get the underlying expression types
    using g1_type = decltype(g1)::type;
    using g2_type = decltype(g2)::type;

    // Build Jacobian type
    using J = Jacobian<4, g1_type, g2_type>;

    std::array<double, 4> pos = {0.6, -0.8, 1.4, -1.4};
    auto jacobian = J::eval(pos);

    // Expected Jacobian:
    // Row 0: [2*x1, 2*y1, 0, 0] = [1.2, -1.6, 0, 0]
    // Row 1: [-2*dx, -2*dy, 2*dx, 2*dy] where dx=0.8, dy=-0.6
    //        = [-1.6, 1.2, 1.6, -1.2]

    assert(approx_equal(jacobian[0][0], 1.2));
    assert(approx_equal(jacobian[0][1], -1.6));
    assert(approx_equal(jacobian[0][2], 0.0));
    assert(approx_equal(jacobian[0][3], 0.0));

    assert(approx_equal(jacobian[1][0], -1.6));
    assert(approx_equal(jacobian[1][1], 1.2));
    assert(approx_equal(jacobian[1][2], 1.6));
    assert(approx_equal(jacobian[1][3], -1.2));

    test_passed("Jacobian from named expressions");
}

// ============================================================================
// Test: Trigonometric functions with named symbols
// ============================================================================
void test_trig_functions() {
    std::cout << "Test: Trigonometric functions..." << std::endl;

    namespace pend = generalized::pendulum;

    // sin(theta1)
    auto s1 = sin(pend::theta1);
    auto c1 = cos(pend::theta1);

    std::array<double, 4> angles = {M_PI / 6, M_PI / 3, 0.0, 0.0};  // 30°, 60°

    assert(approx_equal(s1.eval(angles), 0.5, 1e-9));
    assert(approx_equal(c1.eval(angles), std::sqrt(3.0) / 2.0, 1e-9));

    // sin(theta2)
    auto s2 = sin(pend::theta2);
    assert(approx_equal(s2.eval(angles), std::sqrt(3.0) / 2.0, 1e-9));

    test_passed("Trigonometric functions");
}

// ============================================================================
// Test: make_symbols factory
// ============================================================================
void test_make_symbols() {
    std::cout << "Test: make_symbols factory..." << std::endl;

    // Create custom named symbols
    constexpr const char* names[] = {"mass", "pos", "vel", "acc"};
    auto [mass, pos, vel, acc] = make_symbols<4>(names);

    static_assert(mass.index == 0);
    static_assert(pos.index == 1);
    static_assert(vel.index == 2);
    static_assert(acc.index == 3);

    // Test expression building
    auto kinetic = sq(vel) * mass;  // 0.5 * m * v^2 (without the 0.5)

    std::array<double, 4> state = {2.0, 5.0, 3.0, 0.0};  // m=2, pos=5, v=3
    double ke = kinetic.eval(state);
    assert(approx_equal(ke, 18.0));  // 2 * 3^2 = 18

    test_passed("make_symbols factory");
}

// ============================================================================
// Test: NamedConstraintPendulum produces same results as SymbolicCartesianPendulum
// ============================================================================
void test_pendulum_equivalence() {
    std::cout << "Test: NamedConstraintPendulum equivalence..." << std::endl;

    using namespace sopot::pendulum;

    // Create both pendulum types with same parameters
    double m1 = 1.0, m2 = 1.0;
    double L1 = 1.0, L2 = 1.0;
    double g = 9.81;
    double theta1_0 = M_PI / 4;  // 45°
    double theta2_0 = M_PI / 6;  // 30°

    SymbolicCartesianPendulum<double> symbolic(m1, m2, L1, L2, g, theta1_0, theta2_0);
    NamedConstraintPendulum<double> named(m1, m2, L1, L2, g, theta1_0, theta2_0);

    // Get initial states
    auto state_sym = symbolic.getInitialLocalState();
    auto state_named = named.getInitialLocalState();

    // States should be identical
    for (size_t i = 0; i < 8; ++i) {
        assert(approx_equal(state_sym[i], state_named[i]));
    }

    // Energy should be identical
    std::vector<double> global_sym(state_sym.begin(), state_sym.end());
    std::vector<double> global_named(state_named.begin(), state_named.end());

    double E_sym = symbolic.compute(system::TotalEnergy{}, global_sym);
    double E_named = named.compute(system::TotalEnergy{}, global_named);
    assert(approx_equal(E_sym, E_named));

    // Constraint error should be identical
    double err_sym = symbolic.getConstraintError(global_sym);
    double err_named = named.getConstraintError(global_named);
    assert(approx_equal(err_sym, err_named));

    test_passed("NamedConstraintPendulum equivalence");
}

// ============================================================================
// Test: Gradient evaluation helper
// ============================================================================
void test_gradient_helper() {
    std::cout << "Test: Gradient evaluation helper..." << std::endl;

    // Use explicit namespace to avoid collision with Bessel function y1
    namespace tb = cartesian::two_body_2d;

    // f = x1² + y1²
    auto f = sq(tb::x1) + sq(tb::y1);

    std::array<double, 4> pos = {3.0, 4.0, 0.0, 0.0};

    // Gradient should be [2*x1, 2*y1, 0, 0] = [6, 8, 0, 0]
    auto grad = eval_gradient<4>(pos, f);

    assert(approx_equal(grad[0], 6.0));
    assert(approx_equal(grad[1], 8.0));
    assert(approx_equal(grad[2], 0.0));
    assert(approx_equal(grad[3], 0.0));

    test_passed("Gradient evaluation helper");
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Named Expression Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_named_symbol_types();
    test_named_expression_evaluation();
    test_square_function();
    test_constraint_expression();
    test_named_indexed_equivalence();
    test_named_jacobian();
    test_trig_functions();
    test_make_symbols();
    test_pendulum_equivalence();
    test_gradient_helper();

    std::cout << "========================================" << std::endl;
    std::cout << "All named expression tests passed!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
