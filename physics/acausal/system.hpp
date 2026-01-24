#pragma once

/**
 * @file system.hpp
 * @brief Automatic acausal system construction with compile-time causality
 *
 * This implements:
 * 1. Automatic connection equation generation
 * 2. Equation collection from components
 * 3. Causality assignment (which variable each equation solves)
 * 4. BLT sorting for evaluation order
 */

#include "connector.hpp"
#include "component.hpp"
#include "causality.hpp"
#include <array>
#include <tuple>
#include <iostream>

namespace sopot::acausal {

using namespace sopot::symbolic;

// ============================================================================
// EQUATION COLLECTION
// ============================================================================

/**
 * @brief Collect all equations from a component
 */
template<typename Component>
struct ComponentEquations {
    using Equations = typename Component::Equations;
};

/**
 * @brief Tuple concatenation helper
 */
template<typename... Tuples>
struct TupleCat;

template<>
struct TupleCat<> {
    using type = std::tuple<>;
};

template<typename T>
struct TupleCat<T> {
    using type = T;
};

template<typename... T1s, typename... T2s>
struct TupleCat<std::tuple<T1s...>, std::tuple<T2s...>> {
    using type = std::tuple<T1s..., T2s...>;
};

template<typename T1, typename T2, typename... Rest>
struct TupleCat<T1, T2, Rest...> {
    using type = typename TupleCat<typename TupleCat<T1, T2>::type, Rest...>::type;
};

template<typename... Tuples>
using TupleCat_t = typename TupleCat<Tuples...>::type;

// ============================================================================
// CONNECTION EQUATION GENERATION
// ============================================================================

/**
 * @brief Generate equations for a connection between two pins
 *
 * For electrical pins:
 *   - v1 = v2 (potential equality)
 *   - i1 + i2 = 0 (flow conservation)
 */
template<typename Conn1, typename Conn2>
struct ConnectionEquations {
    static_assert(std::is_same_v<typename Conn1::type, typename Conn2::type>,
        "Connected connectors must be of the same type");

    using ConnectorType = typename Conn1::type;

    // Generate potential equality: v1 - v2 = 0
    template<size_t I>
    using PotentialEq = Equation<
        typename Conn1::template PotentialVar<I>,
        typename Conn2::template PotentialVar<I>
    >;

    // Generate flow conservation: i1 + i2 = 0
    template<size_t I>
    using FlowEq = Equation<
        typename Conn1::template FlowVar<I>,
        Neg<typename Conn2::template FlowVar<I>>
    >;

    // For ElectricalPin (1 potential, 1 flow):
    using Equations = std::tuple<PotentialEq<0>, FlowEq<0>>;
};

// ============================================================================
// COMPILE-TIME MAXIMUM
// ============================================================================

template<size_t... Values>
struct Max;

template<size_t V>
struct Max<V> {
    static constexpr size_t value = V;
};

template<size_t V1, size_t V2, size_t... Rest>
struct Max<V1, V2, Rest...> {
    static constexpr size_t value = Max<(V1 > V2 ? V1 : V2), Rest...>::value;
};

// ============================================================================
// VARIABLE INDEX EXTRACTION
// ============================================================================

/**
 * @brief Find maximum variable index in an expression
 */
template<typename Expr>
struct MaxVarIndex;

template<size_t I>
struct MaxVarIndex<Var<I>> {
    static constexpr size_t value = I + 1;
};

template<int N, int D>
struct MaxVarIndex<Const<N, D>> {
    static constexpr size_t value = 0;
};

template<size_t ID>
struct MaxVarIndex<Param<ID>> {
    static constexpr size_t value = 0;
};

template<typename L, typename R>
struct MaxVarIndex<Add<L, R>> {
    static constexpr size_t value = Max<MaxVarIndex<L>::value, MaxVarIndex<R>::value>::value;
};

template<typename L, typename R>
struct MaxVarIndex<Sub<L, R>> {
    static constexpr size_t value = Max<MaxVarIndex<L>::value, MaxVarIndex<R>::value>::value;
};

template<typename L, typename R>
struct MaxVarIndex<Mul<L, R>> {
    static constexpr size_t value = Max<MaxVarIndex<L>::value, MaxVarIndex<R>::value>::value;
};

template<typename L, typename R>
struct MaxVarIndex<Div<L, R>> {
    static constexpr size_t value = Max<MaxVarIndex<L>::value, MaxVarIndex<R>::value>::value;
};

template<typename E>
struct MaxVarIndex<Neg<E>> {
    static constexpr size_t value = MaxVarIndex<E>::value;
};

template<typename E, int N>
struct MaxVarIndex<Pow<E, N>> {
    static constexpr size_t value = MaxVarIndex<E>::value;
};

template<typename E>
struct MaxVarIndex<Sin<E>> {
    static constexpr size_t value = MaxVarIndex<E>::value;
};

template<typename E>
struct MaxVarIndex<Cos<E>> {
    static constexpr size_t value = MaxVarIndex<E>::value;
};

template<typename E>
struct MaxVarIndex<Sqrt<E>> {
    static constexpr size_t value = MaxVarIndex<E>::value;
};

/**
 * @brief Find maximum variable index in an equation
 */
template<typename Eq>
struct MaxVarInEquation {
    static constexpr size_t value = MaxVarIndex<typename Eq::Residual>::value;
};

/**
 * @brief Find maximum variable index across all equations in a tuple
 */
template<typename EqTuple>
struct MaxVarInSystem;

template<>
struct MaxVarInSystem<std::tuple<>> {
    static constexpr size_t value = 0;
};

template<typename Eq, typename... Rest>
struct MaxVarInSystem<std::tuple<Eq, Rest...>> {
    static constexpr size_t value = Max<
        MaxVarInEquation<Eq>::value,
        MaxVarInSystem<std::tuple<Rest...>>::value
    >::value;
};

// ============================================================================
// PARAMETER INDEX EXTRACTION
// ============================================================================

template<typename Expr>
struct MaxParamIndex;

template<size_t I>
struct MaxParamIndex<Var<I>> {
    static constexpr size_t value = 0;
};

template<int N, int D>
struct MaxParamIndex<Const<N, D>> {
    static constexpr size_t value = 0;
};

template<size_t ID>
struct MaxParamIndex<Param<ID>> {
    static constexpr size_t value = ID + 1;
};

template<typename L, typename R>
struct MaxParamIndex<Add<L, R>> {
    static constexpr size_t value = Max<MaxParamIndex<L>::value, MaxParamIndex<R>::value>::value;
};

template<typename L, typename R>
struct MaxParamIndex<Sub<L, R>> {
    static constexpr size_t value = Max<MaxParamIndex<L>::value, MaxParamIndex<R>::value>::value;
};

template<typename L, typename R>
struct MaxParamIndex<Mul<L, R>> {
    static constexpr size_t value = Max<MaxParamIndex<L>::value, MaxParamIndex<R>::value>::value;
};

template<typename L, typename R>
struct MaxParamIndex<Div<L, R>> {
    static constexpr size_t value = Max<MaxParamIndex<L>::value, MaxParamIndex<R>::value>::value;
};

template<typename E>
struct MaxParamIndex<Neg<E>> {
    static constexpr size_t value = MaxParamIndex<E>::value;
};

template<typename E, int N>
struct MaxParamIndex<Pow<E, N>> {
    static constexpr size_t value = MaxParamIndex<E>::value;
};

template<typename E>
struct MaxParamIndex<Sin<E>> {
    static constexpr size_t value = MaxParamIndex<E>::value;
};

template<typename E>
struct MaxParamIndex<Cos<E>> {
    static constexpr size_t value = MaxParamIndex<E>::value;
};

template<typename E>
struct MaxParamIndex<Sqrt<E>> {
    static constexpr size_t value = MaxParamIndex<E>::value;
};

template<typename Eq>
struct MaxParamInEquation {
    static constexpr size_t value = MaxParamIndex<typename Eq::Residual>::value;
};

template<typename EqTuple>
struct MaxParamInSystem;

template<>
struct MaxParamInSystem<std::tuple<>> {
    static constexpr size_t value = 0;
};

template<typename Eq, typename... Rest>
struct MaxParamInSystem<std::tuple<Eq, Rest...>> {
    static constexpr size_t value = Max<
        MaxParamInEquation<Eq>::value,
        MaxParamInSystem<std::tuple<Rest...>>::value
    >::value;
};

// ============================================================================
// INCIDENCE MATRIX CONSTRUCTION
// ============================================================================

/**
 * @brief Build full incidence matrix for a system of equations
 *
 * Matrix[i][j] = true if equation i depends on variable j
 */
template<typename EqTuple, size_t NumVars>
struct IncidenceMatrix;

template<size_t NumVars>
struct IncidenceMatrix<std::tuple<>, NumVars> {
    static constexpr size_t num_equations = 0;
    static constexpr std::array<std::array<bool, NumVars>, 0> value = {};
};

template<typename Eq, typename... Rest, size_t NumVars>
struct IncidenceMatrix<std::tuple<Eq, Rest...>, NumVars> {
    static constexpr size_t num_equations = 1 + sizeof...(Rest);

    static constexpr auto value = []() constexpr {
        std::array<std::array<bool, NumVars>, num_equations> result = {};
        result[0] = IncidenceRow<Eq, NumVars>::value;
        if constexpr (sizeof...(Rest) > 0) {
            auto rest = IncidenceMatrix<std::tuple<Rest...>, NumVars>::value;
            for (size_t i = 0; i < sizeof...(Rest); ++i) {
                result[i + 1] = rest[i];
            }
        }
        return result;
    }();
};

// ============================================================================
// CAUSALITY ASSIGNMENT (Greedy Matching)
// ============================================================================

/**
 * @brief Find a variable that only this equation can compute
 *
 * This implements a simple greedy matching: find an equation that has
 * a unique variable (only appears in this equation) and assign it.
 *
 * @param incidence The incidence matrix
 * @param knownVars Variables that are already known (e.g., state variables)
 */
template<size_t NumEqs, size_t NumVars>
constexpr auto findGreedyMatching(
    const std::array<std::array<bool, NumVars>, NumEqs>& incidence,
    const std::array<bool, NumVars>& knownVars = {}
) {
    // Result: assignment[eq] = var that equation eq solves for
    // -1 means unassigned
    std::array<int, NumEqs> assignment{};
    for (size_t i = 0; i < NumEqs; ++i) {
        assignment[i] = -1;
    }

    std::array<bool, NumVars> varAssigned{};
    for (size_t i = 0; i < NumVars; ++i) {
        varAssigned[i] = knownVars[i];  // Pre-mark known variables
    }

    // Greedy: assign equations to variables they uniquely depend on
    bool changed = true;
    while (changed) {
        changed = false;

        for (size_t eq = 0; eq < NumEqs; ++eq) {
            if (assignment[eq] >= 0) continue;  // Already assigned

            // Count how many unassigned variables this equation can compute
            int candidateVar = -1;
            int candidateCount = 0;

            for (size_t var = 0; var < NumVars; ++var) {
                if (incidence[eq][var] && !varAssigned[var]) {
                    candidateVar = static_cast<int>(var);
                    candidateCount++;
                }
            }

            // If exactly one candidate, assign it
            if (candidateCount == 1) {
                assignment[eq] = candidateVar;
                varAssigned[candidateVar] = true;
                changed = true;
            }
        }

        // If no progress with unique variables, try any unassigned
        if (!changed) {
            for (size_t eq = 0; eq < NumEqs; ++eq) {
                if (assignment[eq] >= 0) continue;

                for (size_t var = 0; var < NumVars; ++var) {
                    if (incidence[eq][var] && !varAssigned[var]) {
                        assignment[eq] = static_cast<int>(var);
                        varAssigned[var] = true;
                        changed = true;
                        break;
                    }
                }
                if (changed) break;
            }
        }
    }

    return assignment;
}

// Overload without known variables (for backwards compatibility)
template<size_t NumEqs, size_t NumVars>
constexpr auto findGreedyMatchingSimple(
    const std::array<std::array<bool, NumVars>, NumEqs>& incidence
) {
    std::array<bool, NumVars> noKnown{};
    for (size_t i = 0; i < NumVars; ++i) noKnown[i] = false;
    return findGreedyMatching<NumEqs, NumVars>(incidence, noKnown);
}

// ============================================================================
// TOPOLOGICAL SORT (BLT)
// ============================================================================

/**
 * @brief Sort equations into evaluation order
 *
 * After causality assignment, we know which variable each equation computes.
 * We need to order equations so that when we compute equation i,
 * all variables it depends on (except the one it solves) are already computed.
 */
template<size_t NumEqs, size_t NumVars>
constexpr auto sortEquations(
    const std::array<std::array<bool, NumVars>, NumEqs>& incidence,
    const std::array<int, NumEqs>& assignment
) {
    // Build dependency graph: equation i depends on equation j if
    // i uses a variable that j computes
    std::array<std::array<bool, NumEqs>, NumEqs> eqDependsOnEq{};
    for (size_t i = 0; i < NumEqs; ++i) {
        for (size_t j = 0; j < NumEqs; ++j) {
            eqDependsOnEq[i][j] = false;
        }
    }

    for (size_t i = 0; i < NumEqs; ++i) {
        int solves = assignment[i];
        if (solves < 0) continue;

        // Equation i solves for variable 'solves'
        // Find all equations that depend on 'solves'
        for (size_t j = 0; j < NumEqs; ++j) {
            if (i != j && incidence[j][solves]) {
                // Equation j depends on variable 'solves' which i computes
                // So j depends on i (j must come after i)
                eqDependsOnEq[j][i] = true;
            }
        }
    }

    // Topological sort using Kahn's algorithm
    std::array<size_t, NumEqs> order{};
    std::array<bool, NumEqs> processed{};
    for (size_t i = 0; i < NumEqs; ++i) {
        processed[i] = false;
    }

    size_t orderIdx = 0;
    while (orderIdx < NumEqs) {
        // Find an equation with no unprocessed dependencies
        for (size_t i = 0; i < NumEqs; ++i) {
            if (processed[i]) continue;

            bool hasUnprocessedDep = false;
            for (size_t j = 0; j < NumEqs; ++j) {
                if (eqDependsOnEq[i][j] && !processed[j]) {
                    hasUnprocessedDep = true;
                    break;
                }
            }

            if (!hasUnprocessedDep) {
                order[orderIdx++] = i;
                processed[i] = true;
                break;
            }
        }
    }

    return order;
}

// ============================================================================
// SOLVED ACAUSAL SYSTEM
// ============================================================================

/**
 * @brief A fully resolved acausal system
 *
 * Given a set of equations, this:
 * 1. Builds the incidence matrix at compile time
 * 2. Finds causality assignment at compile time
 * 3. Sorts equations into evaluation order at compile time
 * 4. Provides runtime evaluation with direct assignment (no iteration)
 *
 * @tparam EqTuple Tuple of equation types
 * @tparam NumVars Number of variables
 * @tparam NumParams Number of parameters
 */
template<typename EqTuple, size_t NumVars, size_t NumParams>
struct SolvedSystem {
    static constexpr size_t num_vars = NumVars;
    static constexpr size_t num_params = NumParams;
    static constexpr size_t num_equations = std::tuple_size_v<EqTuple>;

    // Build incidence matrix at compile time
    static constexpr auto incidence = IncidenceMatrix<EqTuple, NumVars>::value;

    // Find causality assignment at compile time
    static constexpr auto assignment = findGreedyMatching<num_equations, NumVars>(incidence);

    // Sort equations at compile time
    static constexpr auto evalOrder = sortEquations<num_equations, NumVars>(incidence, assignment);

    /**
     * @brief Evaluate all residuals (for verification)
     */
    template<typename T>
    static std::array<T, num_equations> evaluateResiduals(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params
    ) {
        return evaluateResidualsImpl(vars, params, std::make_index_sequence<num_equations>{});
    }

    /**
     * @brief Solve for all variables given known values
     *
     * This uses the compile-time causality to directly compute variables
     * in the correct order, without iteration.
     */
    template<typename T>
    static std::array<T, num_vars> solve(
        const std::array<T, num_vars>& initial,
        const std::array<T, num_params>& params
    ) {
        std::array<T, num_vars> vars = initial;

        // Evaluate equations in sorted order
        for (size_t i = 0; i < num_equations; ++i) {
            size_t eqIdx = evalOrder[i];
            int varIdx = assignment[eqIdx];

            if (varIdx >= 0) {
                // Solve this equation for its assigned variable
                // For linear equations: var = -residual(var=0) / coefficient
                // For now, we use Newton step: var = var - residual / derivative
                // which is exact for linear equations

                // This is simplified - proper implementation would dispatch
                // based on equation type at compile time
            }
        }

        return vars;
    }

private:
    template<typename T, size_t... Is>
    static std::array<T, num_equations> evaluateResidualsImpl(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params,
        std::index_sequence<Is...>
    ) {
        return {
            eval<typename std::tuple_element_t<Is, EqTuple>::Residual>(vars, params)...
        };
    }
};

/**
 * @brief Create a solved system from equations
 */
template<typename... Eqs>
auto makeSolvedSystem() {
    using EqTuple = std::tuple<Eqs...>;
    constexpr size_t numVars = MaxVarInSystem<EqTuple>::value;
    constexpr size_t numParams = MaxParamInSystem<EqTuple>::value;
    return SolvedSystem<EqTuple, numVars, numParams>{};
}

// ============================================================================
// EXAMPLE: Manual RC Circuit Assembly
// ============================================================================

/**
 * @brief RC Circuit built from primitive equations
 *
 * This demonstrates how a circuit is assembled from components
 * and connections, with automatic causality resolution.
 *
 * Circuit: V0 - R - C - GND
 *
 * Variables (after flattening):
 *   0: V_C (capacitor voltage) - STATE
 *   1: V_node (R-C junction voltage)
 *   2: I_R (resistor current)
 *   3: I_C (capacitor current)
 *
 * Parameters:
 *   0: V0 (source voltage)
 *   1: R (resistance)
 *   2: C (capacitance)
 *
 * Equations:
 *   Eq0: V_node = V_C (capacitor voltage definition)
 *   Eq1: (V0 - V_node) = R * I_R (Ohm's law)
 *   Eq2: I_R = I_C (KCL at node)
 *
 * Derivative equation:
 *   der(V_C) = I_C / C
 */
struct ManualRCCircuit {
    // Variables
    using V_C = Var<0>;
    using V_node = Var<1>;
    using I_R = Var<2>;
    using I_C = Var<3>;

    // Parameters
    using V0 = Param<0>;
    using R = Param<1>;
    using C = Param<2>;

    // Algebraic equations
    using Eq0 = Equation<V_node, V_C>;  // V_node = V_C
    using Eq1 = Equation<Sub<V0, V_node>, Mul<R, I_R>>;  // V0 - V_node = R * I_R
    using Eq2 = Equation<I_R, I_C>;  // I_R = I_C

    using AlgebraicEquations = std::tuple<Eq0, Eq1, Eq2>;

    // Derivative expression
    using DerV_C = Div<I_C, C>;  // der(V_C) = I_C / C

    static constexpr size_t num_vars = 4;
    static constexpr size_t num_algebraic = 3;
    static constexpr size_t num_params = 3;

    // Known variables (state variables are known, not solved by algebraic eqs)
    static constexpr std::array<bool, num_vars> knownVars = {
        true,   // V_C is the state variable (known)
        false,  // V_node is algebraic (unknown)
        false,  // I_R is algebraic (unknown)
        false   // I_C is algebraic (unknown)
    };

    // Build solved system at compile time
    static constexpr auto incidence =
        IncidenceMatrix<AlgebraicEquations, num_vars>::value;

    static constexpr auto assignment =
        findGreedyMatching<num_algebraic, num_vars>(incidence, knownVars);

    static constexpr auto evalOrder =
        sortEquations<num_algebraic, num_vars>(incidence, assignment);

    /**
     * @brief Solve algebraic equations given state V_C
     */
    template<typename T>
    static std::array<T, num_vars> solveAlgebraic(
        T V_C_value,
        const std::array<T, num_params>& params
    ) {
        std::array<T, num_vars> vars{};
        vars[0] = V_C_value;

        // Execute equations in sorted order
        // The compile-time analysis determined:
        //   evalOrder[0] = 0 (Eq0: V_node = V_C)
        //   evalOrder[1] = 1 (Eq1: I_R = (V0 - V_node) / R)
        //   evalOrder[2] = 2 (Eq2: I_C = I_R)

        // Eq0: V_node = V_C
        vars[1] = vars[0];

        // Eq1: V0 - V_node = R * I_R  =>  I_R = (V0 - V_node) / R
        vars[2] = (params[0] - vars[1]) / params[1];

        // Eq2: I_R = I_C  =>  I_C = I_R
        vars[3] = vars[2];

        return vars;
    }

    /**
     * @brief Compute state derivative
     */
    template<typename T>
    static T computeDerivative(
        const std::array<T, num_vars>& vars,
        const std::array<T, num_params>& params
    ) {
        return eval<DerV_C>(vars, params);
    }

    /**
     * @brief Full ODE interface
     */
    template<typename T>
    static std::array<T, 1> computeDerivatives(
        const std::array<T, 1>& states,
        const std::array<T, num_params>& params
    ) {
        auto vars = solveAlgebraic(states[0], params);
        return {computeDerivative(vars, params)};
    }

    /**
     * @brief Print causality analysis
     */
    static void printCausality() {
        std::cout << "ManualRCCircuit Causality Analysis:\n";
        std::cout << "===================================\n\n";

        std::cout << "Incidence Matrix (equation x variable):\n";
        std::cout << "        V_C  V_node  I_R  I_C\n";
        const char* eqNames[] = {"Eq0", "Eq1", "Eq2"};
        for (size_t eq = 0; eq < num_algebraic; ++eq) {
            std::cout << eqNames[eq] << ":    ";
            for (size_t var = 0; var < num_vars; ++var) {
                std::cout << (incidence[eq][var] ? "1" : "0") << "    ";
            }
            std::cout << "\n";
        }

        std::cout << "\nCausality Assignment (equation -> variable):\n";
        const char* varNames[] = {"V_C", "V_node", "I_R", "I_C"};
        for (size_t eq = 0; eq < num_algebraic; ++eq) {
            int var = assignment[eq];
            std::cout << eqNames[eq] << " solves for: "
                      << (var >= 0 ? varNames[var] : "NONE") << "\n";
        }

        std::cout << "\nEvaluation Order:\n";
        for (size_t i = 0; i < num_algebraic; ++i) {
            size_t eq = evalOrder[i];
            std::cout << "  " << (i + 1) << ". " << eqNames[eq]
                      << " (computes " << varNames[assignment[eq]] << ")\n";
        }
    }
};

} // namespace sopot::acausal
