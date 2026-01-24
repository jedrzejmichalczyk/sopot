#pragma once

/**
 * @file connector.hpp
 * @brief Acausal connector definitions for Modelica-style modeling
 *
 * Connectors define the interface between components. Each connector has:
 *   - Potential (effort) variables: equalized when connected
 *   - Flow variables: sum to zero when connected (conservation)
 *
 * Examples:
 *   - Electrical: voltage (potential), current (flow)
 *   - Mechanical: velocity (potential), force (flow)
 *   - Hydraulic: pressure (potential), flow rate (flow)
 */

#include "physics/constraints/symbolic/expression.hpp"
#include <tuple>
#include <array>

namespace sopot::acausal {

using namespace sopot::symbolic;

// ============================================================================
// CONNECTOR TYPE TAGS
// ============================================================================

/**
 * @brief Tag for potential (effort/across) variables
 *
 * Potentials are equalized when connectors are connected.
 * Examples: voltage, velocity, pressure, temperature
 */
struct Potential {};

/**
 * @brief Tag for flow (through) variables
 *
 * Flows sum to zero at a connection (conservation law).
 * Examples: current, force, mass flow rate, heat flow
 */
struct Flow {};

// ============================================================================
// VARIABLE DECLARATION
// ============================================================================

/**
 * @brief A variable in a connector
 *
 * @tparam Kind Potential or Flow
 * @tparam ID Unique identifier within the component
 */
template<typename Kind, size_t ID>
struct ConnectorVar {
    using kind = Kind;
    static constexpr size_t id = ID;
};

// ============================================================================
// CONNECTOR DEFINITION
// ============================================================================

/**
 * @brief Electrical connector (Pin)
 *
 * Modelica equivalent:
 *   connector Pin
 *     Voltage v;
 *     flow Current i;
 *   end Pin;
 */
struct ElectricalPin {
    // Variable indices in the global state
    using Voltage = ConnectorVar<Potential, 0>;
    using Current = ConnectorVar<Flow, 1>;

    static constexpr size_t num_potentials = 1;
    static constexpr size_t num_flows = 1;
    static constexpr size_t num_vars = 2;

    // Variable names for debugging
    static constexpr const char* potential_names[] = {"v"};
    static constexpr const char* flow_names[] = {"i"};
};

/**
 * @brief Mechanical translational connector (Flange)
 *
 * Modelica equivalent:
 *   connector Flange
 *     Position s;
 *     flow Force f;
 *   end Flange;
 */
struct MechanicalFlange {
    using Position = ConnectorVar<Potential, 0>;
    using Velocity = ConnectorVar<Potential, 1>;
    using Force = ConnectorVar<Flow, 2>;

    static constexpr size_t num_potentials = 2;  // position and velocity
    static constexpr size_t num_flows = 1;
    static constexpr size_t num_vars = 3;

    static constexpr const char* potential_names[] = {"s", "v"};
    static constexpr const char* flow_names[] = {"f"};
};

/**
 * @brief Mechanical rotational connector
 */
struct RotationalFlange {
    using Angle = ConnectorVar<Potential, 0>;
    using AngularVelocity = ConnectorVar<Potential, 1>;
    using Torque = ConnectorVar<Flow, 2>;

    static constexpr size_t num_potentials = 2;
    static constexpr size_t num_flows = 1;
    static constexpr size_t num_vars = 3;
};

/**
 * @brief Thermal connector (HeatPort)
 */
struct HeatPort {
    using Temperature = ConnectorVar<Potential, 0>;
    using HeatFlow = ConnectorVar<Flow, 1>;

    static constexpr size_t num_potentials = 1;
    static constexpr size_t num_flows = 1;
    static constexpr size_t num_vars = 2;
};

// ============================================================================
// CONNECTOR INSTANCE
// ============================================================================

/**
 * @brief A connector instance on a component
 *
 * @tparam ConnectorType The connector definition (e.g., ElectricalPin)
 * @tparam BaseIndex Starting index in global variable array
 */
template<typename ConnectorType, size_t BaseIndex>
struct ConnectorInstance {
    using type = ConnectorType;
    static constexpr size_t base_index = BaseIndex;
    static constexpr size_t num_vars = ConnectorType::num_vars;

    // Get global variable index for a local variable
    template<typename LocalVar>
    static constexpr size_t globalIndex() {
        return BaseIndex + LocalVar::id;
    }

    // Get symbolic variable for potential at index I
    template<size_t I>
    using PotentialVar = Var<BaseIndex + I>;

    // Get symbolic variable for flow at index I (offset by num_potentials)
    template<size_t I>
    using FlowVar = Var<BaseIndex + ConnectorType::num_potentials + I>;
};

// ============================================================================
// CONNECTION SEMANTICS
// ============================================================================

/**
 * @brief A connection between two connector instances
 *
 * Generates equations:
 *   - For each potential: p1 = p2 (equality)
 *   - For each flow: f1 + f2 = 0 (conservation)
 */
template<typename Connector1, typename Connector2>
struct Connection {
    static_assert(std::is_same_v<typename Connector1::type, typename Connector2::type>,
        "Connected connectors must be of the same type");

    using ConnectorType = typename Connector1::type;

    // Number of equations generated
    static constexpr size_t num_equations =
        ConnectorType::num_potentials + ConnectorType::num_flows;

    /**
     * @brief Generate potential equality equations: v1 - v2 = 0
     */
    template<size_t I>
    using PotentialEquation = Sub<
        typename Connector1::template PotentialVar<I>,
        typename Connector2::template PotentialVar<I>
    >;

    /**
     * @brief Generate flow conservation equations: i1 + i2 = 0
     */
    template<size_t I>
    using FlowEquation = Add<
        typename Connector1::template FlowVar<I>,
        typename Connector2::template FlowVar<I>
    >;
};

/**
 * @brief A junction connecting multiple connectors (like a node in a circuit)
 *
 * For N connectors at a junction:
 *   - All potentials equal: v1 = v2 = v3 = ... = vN
 *   - Flows sum to zero: i1 + i2 + i3 + ... + iN = 0
 */
template<typename... Connectors>
struct Junction {
    static constexpr size_t num_connectors = sizeof...(Connectors);

    // Use first connector as reference for potential equalities
    // This generates (N-1) equality equations per potential
    // Plus 1 flow conservation equation per flow variable
};

} // namespace sopot::acausal
