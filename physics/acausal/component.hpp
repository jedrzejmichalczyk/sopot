#pragma once

/**
 * @file component.hpp
 * @brief Acausal component definitions
 *
 * Components declare:
 *   - Connectors: interface points
 *   - Internal variables: state and algebraic
 *   - Equations: relationships between variables (NO assignment direction!)
 *
 * The system automatically determines causality.
 */

#include "connector.hpp"
#include "physics/constraints/symbolic/expression.hpp"
#include "physics/constraints/symbolic/differentiation.hpp"
#include <tuple>

namespace sopot::acausal {

using namespace sopot::symbolic;

// ============================================================================
// EQUATION DECLARATION
// ============================================================================

/**
 * @brief An acausal equation: LHS = RHS (or equivalently, LHS - RHS = 0)
 *
 * No direction is specified - the solver determines causality.
 */
template<typename LHS, typename RHS>
struct Equation {
    using Left = LHS;
    using Right = RHS;

    // Residual form: LHS - RHS = 0
    using Residual = Sub<LHS, RHS>;
};

/**
 * @brief Derivative equation: der(x) = expr
 *
 * Declares that the time derivative of variable x equals expr.
 */
template<typename StateVar, typename Expr>
struct DerEquation {
    using Variable = StateVar;
    using Derivative = Expr;
};

// ============================================================================
// ELECTRICAL COMPONENTS
// ============================================================================

/**
 * @brief Resistor component
 *
 * Equation: v = R * i  (Ohm's law)
 *
 * Modelica equivalent:
 *   model Resistor
 *     Pin p, n;
 *     parameter Real R;
 *   equation
 *     v = p.v - n.v;
 *     v = R * i;
 *     p.i + n.i = 0;
 *   end Resistor;
 */
template<size_t BaseIdx, size_t ParamR>
struct Resistor {
    // Connectors
    using PinP = ConnectorInstance<ElectricalPin, BaseIdx>;
    using PinN = ConnectorInstance<ElectricalPin, BaseIdx + 2>;

    // Internal variables
    using V = Var<BaseIdx + 4>;  // Voltage across resistor
    using I = Var<BaseIdx + 5>;  // Current through resistor

    // Parameters
    using R = Param<ParamR>;

    // Equations (all in residual form = 0)
    // 1. v = p.v - n.v
    using Eq1 = Equation<V, Sub<typename PinP::template PotentialVar<0>,
                                typename PinN::template PotentialVar<0>>>;

    // 2. v = R * i  →  v - R*i = 0
    using Eq2 = Equation<V, Mul<R, I>>;

    // 3. p.i + n.i = 0 (current conservation)
    using Eq3 = Equation<typename PinP::template FlowVar<0>,
                         Neg<typename PinN::template FlowVar<0>>>;

    // 4. i = p.i (current definition)
    using Eq4 = Equation<I, typename PinP::template FlowVar<0>>;

    static constexpr size_t num_vars = 6;  // 2 pins * 2 vars + 2 internal
    static constexpr size_t num_equations = 4;

    using Equations = std::tuple<Eq1, Eq2, Eq3, Eq4>;
};

/**
 * @brief Capacitor component
 *
 * Equations:
 *   v = p.v - n.v
 *   i = C * der(v)
 */
template<size_t BaseIdx, size_t ParamC>
struct Capacitor {
    using PinP = ConnectorInstance<ElectricalPin, BaseIdx>;
    using PinN = ConnectorInstance<ElectricalPin, BaseIdx + 2>;

    using V = Var<BaseIdx + 4>;  // Voltage (state variable!)
    using I = Var<BaseIdx + 5>;  // Current

    using C = Param<ParamC>;

    // v = p.v - n.v
    using Eq1 = Equation<V, Sub<typename PinP::template PotentialVar<0>,
                                typename PinN::template PotentialVar<0>>>;

    // i = C * der(v) - this declares V as a state variable
    // We represent this as: der(V) = i / C
    using DerEq = DerEquation<V, Div<I, C>>;

    // Current conservation
    using Eq2 = Equation<typename PinP::template FlowVar<0>,
                         Neg<typename PinN::template FlowVar<0>>>;

    using Eq3 = Equation<I, typename PinP::template FlowVar<0>>;

    static constexpr size_t num_state_vars = 1;  // V is a state
    static constexpr size_t num_vars = 6;
};

/**
 * @brief Inductor component
 *
 * Equations:
 *   v = p.v - n.v
 *   v = L * der(i)
 */
template<size_t BaseIdx, size_t ParamL>
struct Inductor {
    using PinP = ConnectorInstance<ElectricalPin, BaseIdx>;
    using PinN = ConnectorInstance<ElectricalPin, BaseIdx + 2>;

    using V = Var<BaseIdx + 4>;
    using I = Var<BaseIdx + 5>;  // Current (state variable!)

    using L = Param<ParamL>;

    using Eq1 = Equation<V, Sub<typename PinP::template PotentialVar<0>,
                                typename PinN::template PotentialVar<0>>>;

    // v = L * der(i)  →  der(i) = v / L
    using DerEq = DerEquation<I, Div<V, L>>;

    using Eq2 = Equation<typename PinP::template FlowVar<0>,
                         Neg<typename PinN::template FlowVar<0>>>;

    using Eq3 = Equation<I, typename PinP::template FlowVar<0>>;

    static constexpr size_t num_state_vars = 1;  // I is a state
};

/**
 * @brief Voltage source
 *
 * Equation: p.v - n.v = V0
 */
template<size_t BaseIdx, size_t ParamV0>
struct VoltageSource {
    using PinP = ConnectorInstance<ElectricalPin, BaseIdx>;
    using PinN = ConnectorInstance<ElectricalPin, BaseIdx + 2>;

    using V0 = Param<ParamV0>;

    // p.v - n.v = V0
    using Eq1 = Equation<Sub<typename PinP::template PotentialVar<0>,
                             typename PinN::template PotentialVar<0>>,
                         V0>;

    // Current conservation: p.i + n.i = 0
    using Eq2 = Equation<typename PinP::template FlowVar<0>,
                         Neg<typename PinN::template FlowVar<0>>>;

    static constexpr size_t num_vars = 4;
};

/**
 * @brief Ground (reference voltage = 0)
 */
template<size_t BaseIdx>
struct Ground {
    using Pin = ConnectorInstance<ElectricalPin, BaseIdx>;

    // v = 0
    using Eq1 = Equation<typename Pin::template PotentialVar<0>, Zero>;

    static constexpr size_t num_vars = 2;
};

// ============================================================================
// MECHANICAL COMPONENTS
// ============================================================================

/**
 * @brief Point mass
 *
 * Equations:
 *   m * der(v) = f
 *   der(s) = v
 */
template<size_t BaseIdx, size_t ParamM>
struct Mass {
    using Flange = ConnectorInstance<MechanicalFlange, BaseIdx>;

    using S = Var<BaseIdx>;      // Position (state)
    using V = Var<BaseIdx + 1>;  // Velocity (state)
    using F = Var<BaseIdx + 2>;  // Force

    using M = Param<ParamM>;

    // der(s) = v
    using DerEq1 = DerEquation<S, V>;

    // der(v) = f / m
    using DerEq2 = DerEquation<V, Div<F, M>>;

    static constexpr size_t num_state_vars = 2;
};

/**
 * @brief Spring (Hookean)
 *
 * Equation: f = k * (s1 - s2 - s_rel0)
 */
template<size_t BaseIdx, size_t ParamK, size_t ParamS0>
struct Spring {
    using FlangeA = ConnectorInstance<MechanicalFlange, BaseIdx>;
    using FlangeB = ConnectorInstance<MechanicalFlange, BaseIdx + 3>;

    using K = Param<ParamK>;
    using S0 = Param<ParamS0>;  // Unstretched length

    // f = k * (s_a - s_b - s0)
    using DeltaS = Sub<typename FlangeA::template PotentialVar<0>,
                       typename FlangeB::template PotentialVar<0>>;
    using Extension = Sub<DeltaS, S0>;

    using Eq1 = Equation<typename FlangeA::template FlowVar<0>,
                         Mul<K, Extension>>;

    // Force balance: f_a + f_b = 0
    using Eq2 = Equation<typename FlangeA::template FlowVar<0>,
                         Neg<typename FlangeB::template FlowVar<0>>>;
};

/**
 * @brief Damper (viscous friction)
 *
 * Equation: f = d * (v1 - v2)
 */
template<size_t BaseIdx, size_t ParamD>
struct Damper {
    using FlangeA = ConnectorInstance<MechanicalFlange, BaseIdx>;
    using FlangeB = ConnectorInstance<MechanicalFlange, BaseIdx + 3>;

    using D = Param<ParamD>;

    // f = d * (v_a - v_b)
    using DeltaV = Sub<typename FlangeA::template PotentialVar<1>,
                       typename FlangeB::template PotentialVar<1>>;

    using Eq1 = Equation<typename FlangeA::template FlowVar<0>,
                         Mul<D, DeltaV>>;

    using Eq2 = Equation<typename FlangeA::template FlowVar<0>,
                         Neg<typename FlangeB::template FlowVar<0>>>;
};

} // namespace sopot::acausal
