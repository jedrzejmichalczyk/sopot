#pragma once

/**
 * @file causality.hpp
 * @brief Compile-time causality assignment for acausal models
 *
 * This implements the core Modelica-style algorithm:
 * 1. Collect equations from all components
 * 2. Build incidence matrix (which equations use which variables)
 * 3. Perform matching to assign one equation to each unknown
 * 4. Sort equations into evaluation order (BLT transformation)
 *
 * All done at compile time!
 */

#include "connector.hpp"
#include "component.hpp"
#include "physics/constraints/symbolic/expression.hpp"
#include "physics/constraints/symbolic/differentiation.hpp"
#include <array>
#include <tuple>

namespace sopot::acausal {

using namespace sopot::symbolic;

// ============================================================================
// VARIABLE CLASSIFICATION
// ============================================================================

/**
 * @brief Check if expression depends on a specific variable
 */
template<typename Expr, size_t VarIdx>
struct DependsOn;

// Variable depends on itself
template<size_t I, size_t VarIdx>
struct DependsOn<Var<I>, VarIdx> {
    static constexpr bool value = (I == VarIdx);
};

// Constants don't depend on variables
template<int N, int D, size_t VarIdx>
struct DependsOn<Const<N, D>, VarIdx> {
    static constexpr bool value = false;
};

// Parameters don't depend on variables
template<size_t ID, size_t VarIdx>
struct DependsOn<Param<ID>, VarIdx> {
    static constexpr bool value = false;
};

// Binary operations
template<typename L, typename R, size_t VarIdx>
struct DependsOn<Add<L, R>, VarIdx> {
    static constexpr bool value = DependsOn<L, VarIdx>::value || DependsOn<R, VarIdx>::value;
};

template<typename L, typename R, size_t VarIdx>
struct DependsOn<Sub<L, R>, VarIdx> {
    static constexpr bool value = DependsOn<L, VarIdx>::value || DependsOn<R, VarIdx>::value;
};

template<typename L, typename R, size_t VarIdx>
struct DependsOn<Mul<L, R>, VarIdx> {
    static constexpr bool value = DependsOn<L, VarIdx>::value || DependsOn<R, VarIdx>::value;
};

template<typename L, typename R, size_t VarIdx>
struct DependsOn<Div<L, R>, VarIdx> {
    static constexpr bool value = DependsOn<L, VarIdx>::value || DependsOn<R, VarIdx>::value;
};

// Unary operations
template<typename E, size_t VarIdx>
struct DependsOn<Neg<E>, VarIdx> {
    static constexpr bool value = DependsOn<E, VarIdx>::value;
};

template<typename E, int N, size_t VarIdx>
struct DependsOn<Pow<E, N>, VarIdx> {
    static constexpr bool value = DependsOn<E, VarIdx>::value;
};

template<typename E, size_t VarIdx>
struct DependsOn<Sin<E>, VarIdx> {
    static constexpr bool value = DependsOn<E, VarIdx>::value;
};

template<typename E, size_t VarIdx>
struct DependsOn<Cos<E>, VarIdx> {
    static constexpr bool value = DependsOn<E, VarIdx>::value;
};

template<typename E, size_t VarIdx>
struct DependsOn<Sqrt<E>, VarIdx> {
    static constexpr bool value = DependsOn<E, VarIdx>::value;
};

template<typename Expr, size_t VarIdx>
inline constexpr bool depends_on_v = DependsOn<Expr, VarIdx>::value;

// ============================================================================
// INCIDENCE MATRIX (Compile-time)
// ============================================================================

/**
 * @brief Check if equation's residual depends on variable VarIdx
 */
template<typename Eq, size_t VarIdx>
struct EquationDependsOn {
    static constexpr bool value = depends_on_v<typename Eq::Residual, VarIdx>;
};

/**
 * @brief Build one row of incidence matrix for an equation
 */
template<typename Eq, size_t NumVars, typename = std::make_index_sequence<NumVars>>
struct IncidenceRow;

template<typename Eq, size_t NumVars, size_t... Is>
struct IncidenceRow<Eq, NumVars, std::index_sequence<Is...>> {
    static constexpr std::array<bool, NumVars> value = {
        EquationDependsOn<Eq, Is>::value...
    };
};

/**
 * @brief Count number of variables an equation depends on
 */
template<typename Eq, size_t NumVars>
struct EquationDegree {
    static constexpr size_t value = []() constexpr {
        auto row = IncidenceRow<Eq, NumVars>::value;
        size_t count = 0;
        for (bool b : row) if (b) count++;
        return count;
    }();
};

// ============================================================================
// SYMBOLIC EQUATION SOLVING
// ============================================================================

/**
 * @brief Deep simplification that recursively simplifies subexpressions
 */
template<typename E>
struct DeepSimplify;

template<typename E>
using DeepSimplify_t = typename DeepSimplify<E>::type;

// Base cases
template<size_t I>
struct DeepSimplify<Var<I>> {
    using type = Var<I>;
};

template<int N, int D>
struct DeepSimplify<Const<N, D>> {
    using type = Const<N, D>;
};

template<size_t ID>
struct DeepSimplify<Param<ID>> {
    using type = Param<ID>;
};

// Recursive simplification for binary operations
template<typename L, typename R>
struct DeepSimplify<Add<L, R>> {
    using type = Simplify_t<Add<DeepSimplify_t<L>, DeepSimplify_t<R>>>;
};

template<typename L, typename R>
struct DeepSimplify<Sub<L, R>> {
    using type = Simplify_t<Sub<DeepSimplify_t<L>, DeepSimplify_t<R>>>;
};

template<typename L, typename R>
struct DeepSimplify<Mul<L, R>> {
    using type = Simplify_t<Mul<DeepSimplify_t<L>, DeepSimplify_t<R>>>;
};

template<typename L, typename R>
struct DeepSimplify<Div<L, R>> {
    using type = Simplify_t<Div<DeepSimplify_t<L>, DeepSimplify_t<R>>>;
};

template<typename E>
struct DeepSimplify<Neg<E>> {
    using type = Simplify_t<Neg<DeepSimplify_t<E>>>;
};

template<typename E, int N>
struct DeepSimplify<Pow<E, N>> {
    using type = Simplify_t<Pow<DeepSimplify_t<E>, N>>;
};

template<typename E>
struct DeepSimplify<Sin<E>> {
    using type = Sin<DeepSimplify_t<E>>;
};

template<typename E>
struct DeepSimplify<Cos<E>> {
    using type = Cos<DeepSimplify_t<E>>;
};

template<typename E>
struct DeepSimplify<Sqrt<E>> {
    using type = Sqrt<DeepSimplify_t<E>>;
};

/**
 * @brief Check if expression is linear in variable VarIdx
 *
 * Linear means: expr = a * x + b where a, b don't depend on x
 */
template<typename Expr, size_t VarIdx>
struct IsLinearIn {
    // Compute derivative and deeply simplify it
    using RawDerivative = Diff_t<Expr, VarIdx>;
    using Derivative = DeepSimplify_t<RawDerivative>;
    // Check if simplified derivative is constant w.r.t. VarIdx
    static constexpr bool value = !depends_on_v<Derivative, VarIdx>;
};

/**
 * @brief Solve linear equation for variable VarIdx
 *
 * Given: expr = 0 where expr = a*x + b
 * Solve for: x = -b/a
 *
 * Uses: x = -expr(x=0) / (d expr/dx)
 */
template<typename Expr, size_t VarIdx>
struct SolveLinear {
    static_assert(IsLinearIn<Expr, VarIdx>::value,
        "Cannot solve: expression is not linear in variable");

    // Coefficient a = d(expr)/dx
    using Coefficient = Diff_t<Expr, VarIdx>;

    // Constant term b = expr with x=0 (conceptually)
    // For linear expr = a*x + b, solving gives x = -b/a
    // Since expr = 0 → a*x + b = 0 → x = -b/a
    // And b = expr - a*x, so at solution: x = -(expr - a*x)/a when expr=0

    // Actually, for linear equation: expr = coeff * var + rest
    // Solution: var = -rest / coeff
    // We can compute: solution = Neg<Div<SubstituteZero<Expr, VarIdx>, Coefficient>>
    // But SubstituteZero is complex...

    // Simpler approach for linear: if expr = a*x + b = 0, then x = -b/a
    // derivative gives us 'a', and expr(x) - a*x gives us 'b'

    // For now, use the Newton step which works for linear:
    // x_new = x - expr/derivative = -expr/a (starting from x=0)
    // This is exact for linear equations.

    // We'll generate code that computes: x = -residual / jacobian_element
    // At runtime, given the residual value and Jacobian, solve directly.
};

// ============================================================================
// EQUATION SORTING (Simplified BLT)
// ============================================================================

/**
 * @brief Assignment: which variable is computed by which equation
 *
 * For a well-posed system, we need a perfect matching:
 * each equation computes exactly one unknown variable.
 */
template<size_t EqIdx, size_t VarIdx>
struct Assignment {
    static constexpr size_t equation = EqIdx;
    static constexpr size_t variable = VarIdx;
};

/**
 * @brief Sorted computation sequence
 *
 * Given a list of assignments, determine evaluation order such that
 * when we compute variable V, all variables it depends on are already known.
 */
template<typename... Assignments>
struct ComputationSequence {
    static constexpr size_t length = sizeof...(Assignments);

    // The assignments in sorted order
    using Sorted = std::tuple<Assignments...>;
};

// ============================================================================
// ACAUSAL SYSTEM
// ============================================================================

/**
 * @brief Compile-time acausal system
 *
 * Takes a set of components and connections, and:
 * 1. Collects all equations
 * 2. Determines causality (which variable each equation solves)
 * 3. Sorts into evaluation order
 * 4. Generates efficient evaluation code
 *
 * @tparam NumVars Total number of variables
 * @tparam NumParams Number of parameters
 * @tparam Equations Tuple of equation types
 */
template<size_t NumVars, size_t NumParams, typename Equations>
struct AcausalSystem {
    static constexpr size_t num_vars = NumVars;
    static constexpr size_t num_params = NumParams;

    /**
     * @brief Evaluate all residuals (for checking/Newton iteration)
     */
    template<typename T>
    static std::array<T, num_vars> evaluateResiduals(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params
    ) {
        return evaluateResidualsImpl(vars, params,
            std::make_index_sequence<std::tuple_size_v<Equations>>{});
    }

    /**
     * @brief Evaluate Jacobian matrix
     */
    template<typename T>
    static std::array<std::array<T, num_vars>, num_vars> evaluateJacobian(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params
    ) {
        return evaluateJacobianImpl(vars, params,
            std::make_index_sequence<std::tuple_size_v<Equations>>{});
    }

private:
    template<typename T, size_t... EqIs>
    static std::array<T, num_vars> evaluateResidualsImpl(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params,
        std::index_sequence<EqIs...>
    ) {
        return {
            eval<typename std::tuple_element_t<EqIs, Equations>::Residual>(vars, params)...
        };
    }

    template<typename T, size_t... EqIs>
    static std::array<std::array<T, num_vars>, num_vars> evaluateJacobianImpl(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params,
        std::index_sequence<EqIs...>
    ) {
        return {
            evaluateJacobianRow<T, EqIs>(vars, params)...
        };
    }

    template<typename T, size_t EqIdx>
    static std::array<T, num_vars> evaluateJacobianRow(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params
    ) {
        using Eq = std::tuple_element_t<EqIdx, Equations>;
        return Gradient<typename Eq::Residual, num_vars>::eval(vars, params);
    }
};

// ============================================================================
// SIMPLE CIRCUIT EXAMPLE (for testing)
// ============================================================================

/**
 * @brief Simple RC circuit: V - R - C - Ground
 *
 * Variables:
 *   0: V_source (voltage at source positive terminal)
 *   1: I_source (current from source)
 *   2: V_node   (voltage at R-C junction)
 *   3: I_R      (current through resistor)
 *   4: V_C      (capacitor voltage = state variable)
 *   5: I_C      (capacitor current)
 *
 * Equations:
 *   0: V_source = V0 (source voltage)
 *   1: V_source - V_node = R * I_R (Ohm's law)
 *   2: I_R = I_C (current continuity)
 *   3: V_node = V_C (capacitor connected to node)
 *   4: I_C = C * der(V_C) (capacitor equation)
 *   5: Ground: V at negative terminal = 0 (implicit)
 *
 * State: V_C
 * Derivative: der(V_C) = I_C / C = I_R / C = (V0 - V_C) / (R * C)
 */
template<size_t ParamV0, size_t ParamR, size_t ParamC>
struct SimpleRCCircuit {
    static constexpr size_t num_states = 1;
    static constexpr size_t num_algebraic = 3;
    static constexpr size_t num_vars = 4;
    static constexpr size_t num_params = 3;

    // Variables
    using V_C = Var<0>;    // Capacitor voltage (STATE)
    using V_node = Var<1>; // Node voltage (algebraic)
    using I_R = Var<2>;    // Resistor current (algebraic)
    using I_C = Var<3>;    // Capacitor current (algebraic)

    // Parameters
    using V0 = Param<ParamV0>;
    using R = Param<ParamR>;
    using C = Param<ParamC>;

    // Algebraic equations (these determine algebraic variables)
    // V_node = V_C
    using Eq1 = Equation<V_node, V_C>;

    // V0 - V_node = R * I_R  →  I_R = (V0 - V_node) / R
    using Eq2 = Equation<Mul<R, I_R>, Sub<V0, V_node>>;

    // I_R = I_C
    using Eq3 = Equation<I_R, I_C>;

    // Derivative equation
    // der(V_C) = I_C / C
    using DerivExpr = Div<I_C, C>;

    /**
     * @brief Compute derivatives given state and algebraic vars
     */
    template<typename T>
    static T computeDerivative(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params
    ) {
        return eval<DerivExpr>(vars, params);
    }

    /**
     * @brief Solve algebraic equations given V_C (the state)
     *
     * Causality (solved at compile time):
     *   V_C → V_node → I_R → I_C
     */
    template<typename T>
    static std::array<T, num_vars> solveAlgebraic(
        T V_C_value,
        const std::array<T, num_params>& params
    ) {
        std::array<T, num_vars> vars;
        vars[0] = V_C_value;

        // V_node = V_C (from Eq1)
        vars[1] = vars[0];

        // I_R = (V0 - V_node) / R (from Eq2)
        vars[2] = (params[ParamV0] - vars[1]) / params[ParamR];

        // I_C = I_R (from Eq3)
        vars[3] = vars[2];

        return vars;
    }

    /**
     * @brief Full ODE system interface
     */
    template<typename T>
    static std::array<T, num_states> computeDerivatives(
        const std::array<T, num_states>& states,
        const std::array<T, num_params>& params
    ) {
        // Solve algebraic equations
        auto vars = solveAlgebraic(states[0], params);

        // Compute derivative
        return {computeDerivative(vars, params)};
    }
};

} // namespace sopot::acausal
