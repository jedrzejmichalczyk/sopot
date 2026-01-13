/*
 * Simple Chemical Reactor Example
 * Demonstrates field-based component composition
 *
 * System: A + B -> C (exothermic reaction in a CSTR)
 *
 * Components are completely decoupled - they don't know what system they're in!
 */

#include "field_based_component.hpp"
#include <iostream>
#include <cmath>

using namespace sopot::experimental;

// ============================================================================
// DEFINE DOMAIN TAGS
// ============================================================================

namespace chemistry {
    struct Concentration { using ValueType = double; };
    struct ReactionRate { using ValueType = double; };
    struct HeatRelease { using ValueType = double; };
}

namespace thermodynamics {
    struct Temperature { using ValueType = double; };
    struct HeatCapacity { using ValueType = double; };
}

namespace environment {
    struct Pressure { using ValueType = double; };
}

// ============================================================================
// COMPONENT 1: Arrhenius Reaction A + B -> C
// ============================================================================

template<typename T = double>
class ArrheniusReaction {
public:
    static constexpr size_t StateSize = 0;  // Stateless provider

    // What this component NEEDS
    using Dependencies = FieldBundle<
        Field<chemistry::Concentration, "conc_A">,
        Field<chemistry::Concentration, "conc_B">,
        Field<thermodynamics::Temperature, "temp">
    >;

    // What this component PROVIDES
    using Provides = FieldBundle<
        Field<chemistry::ReactionRate, "rate_ABC">,
        Field<chemistry::HeatRelease, "heat_ABC">
    >;

    ArrheniusReaction(T A, T Ea, T dH)
        : m_A(A), m_Ea(Ea), m_dH(dH) {}

    // Compute: ONLY knows about declared dependencies!
    Provides compute(T t, const Dependencies& deps) const {
        // Extract dependencies (compile-time checked)
        T c_A = deps.template find<chemistry::Concentration, "conc_A">().value;
        T c_B = deps.template find<chemistry::Concentration, "conc_B">().value;
        T temp = deps.template find<thermodynamics::Temperature, "temp">().value;

        // Arrhenius equation: k = A * exp(-Ea / (R*T))
        T k = m_A * std::exp(-m_Ea / (R * temp));

        // Reaction rate: r = k * [A] * [B]
        T rate = k * c_A * c_B;

        // Heat release: Q = r * (-ΔH)
        T heat = rate * (-m_dH);  // Negative because exothermic

        // Return provisions
        return Provides{
            Field<chemistry::ReactionRate, "rate_ABC">{.value = rate},
            Field<chemistry::HeatRelease, "heat_ABC">{.value = heat}
        };
    }

private:
    T m_A;   // Pre-exponential factor [1/(mol*s)]
    T m_Ea;  // Activation energy [J/mol]
    T m_dH;  // Heat of reaction [J/mol]
    static constexpr T R = 8.314;  // Gas constant [J/(mol*K)]
};

// ============================================================================
// COMPONENT 2: Species Mass Balance for A
// ============================================================================

template<typename T = double>
class SpeciesMassBalance_A {
public:
    static constexpr size_t StateSize = 1;  // 1 state: concentration of A

    // What this component NEEDS
    using Dependencies = FieldBundle<
        Field<chemistry::ReactionRate, "rate_ABC">  // Reaction consumes A
    >;

    // What this component PROVIDES
    using Provides = FieldBundle<
        Field<chemistry::Concentration, "conc_A">
    >;

    SpeciesMassBalance_A(T initial_conc, T inlet_conc, T flow_rate, T volume)
        : m_initial(initial_conc),
          m_inlet_conc(inlet_conc),
          m_Q(flow_rate),
          m_V(volume) {}

    // Initial state
    T getInitialState() const {
        return m_initial;
    }

    // Provide current concentration from state
    Provides provideFromState(const T& local_state) const {
        return Provides{
            Field<chemistry::Concentration, "conc_A">{.value = local_state}
        };
    }

    // Compute derivative: dC_A/dt = (Q/V)*(C_in - C_A) - r_ABC
    T computeDerivative(T t, const T& local_state, const Dependencies& deps) const {
        T c_A = local_state;
        T rate = deps.template find<chemistry::ReactionRate, "rate_ABC">().value;

        T dilution = (m_Q / m_V) * (m_inlet_conc - c_A);
        T reaction = -rate;  // A is consumed

        return dilution + reaction;
    }

private:
    T m_initial;      // Initial concentration [mol/m³]
    T m_inlet_conc;   // Inlet concentration [mol/m³]
    T m_Q;            // Volumetric flow rate [m³/s]
    T m_V;            // Reactor volume [m³]
};

// ============================================================================
// COMPONENT 3: Species Mass Balance for B
// ============================================================================

template<typename T = double>
class SpeciesMassBalance_B {
public:
    static constexpr size_t StateSize = 1;

    using Dependencies = FieldBundle<
        Field<chemistry::ReactionRate, "rate_ABC">
    >;

    using Provides = FieldBundle<
        Field<chemistry::Concentration, "conc_B">
    >;

    SpeciesMassBalance_B(T initial_conc, T inlet_conc, T flow_rate, T volume)
        : m_initial(initial_conc),
          m_inlet_conc(inlet_conc),
          m_Q(flow_rate),
          m_V(volume) {}

    T getInitialState() const {
        return m_initial;
    }

    Provides provideFromState(const T& local_state) const {
        return Provides{
            Field<chemistry::Concentration, "conc_B">{.value = local_state}
        };
    }

    T computeDerivative(T t, const T& local_state, const Dependencies& deps) const {
        T c_B = local_state;
        T rate = deps.template find<chemistry::ReactionRate, "rate_ABC">().value;

        T dilution = (m_Q / m_V) * (m_inlet_conc - c_B);
        T reaction = -rate;  // B is consumed

        return dilution + reaction;
    }

private:
    T m_initial;
    T m_inlet_conc;
    T m_Q;
    T m_V;
};

// ============================================================================
// COMPONENT 4: Energy Balance (Temperature)
// ============================================================================

template<typename T = double>
class EnergyBalance {
public:
    static constexpr size_t StateSize = 1;  // 1 state: temperature

    using Dependencies = FieldBundle<
        Field<chemistry::HeatRelease, "heat_ABC">
    >;

    using Provides = FieldBundle<
        Field<thermodynamics::Temperature, "temp">
    >;

    EnergyBalance(T initial_temp, T inlet_temp, T flow_rate, T volume, T density, T cp)
        : m_initial(initial_temp),
          m_inlet_temp(inlet_temp),
          m_Q(flow_rate),
          m_V(volume),
          m_rho(density),
          m_cp(cp) {}

    T getInitialState() const {
        return m_initial;
    }

    Provides provideFromState(const T& local_state) const {
        return Provides{
            Field<thermodynamics::Temperature, "temp">{.value = local_state}
        };
    }

    // dT/dt = (Q/V)*(T_in - T) + Q_rxn / (ρ * V * cp)
    T computeDerivative(T t, const T& local_state, const Dependencies& deps) const {
        T temp = local_state;
        T Q_rxn = deps.template find<chemistry::HeatRelease, "heat_ABC">().value;

        T convection = (m_Q / m_V) * (m_inlet_temp - temp);
        T reaction_heat = Q_rxn / (m_rho * m_V * m_cp);

        return convection + reaction_heat;
    }

private:
    T m_initial;      // Initial temperature [K]
    T m_inlet_temp;   // Inlet temperature [K]
    T m_Q;            // Flow rate [m³/s]
    T m_V;            // Volume [m³]
    T m_rho;          // Density [kg/m³]
    T m_cp;           // Heat capacity [J/(kg*K)]
};

// ============================================================================
// MANUAL SYSTEM COMPOSITION (until we build the compiler)
// ============================================================================

int main() {
    std::cout << "=== Simple Chemical Reactor Example ===" << std::endl;
    std::cout << "Reaction: A + B -> C (exothermic)" << std::endl;
    std::cout << std::endl;

    // Component parameters
    constexpr double V = 1.0;       // 1 m³ reactor
    constexpr double Q = 0.1;       // 0.1 m³/s flow rate
    constexpr double rho = 1000.0;  // 1000 kg/m³ density
    constexpr double cp = 4184.0;   // 4184 J/(kg*K) heat capacity

    // Create components
    ArrheniusReaction<double> reaction(
        1.0e10,  // A = 1e10 [1/(mol*s)]
        50000.0, // Ea = 50 kJ/mol
        -80000.0 // ΔH = -80 kJ/mol (exothermic)
    );

    SpeciesMassBalance_A<double> balance_A(
        10.0,  // Initial: 10 mol/m³
        50.0,  // Inlet: 50 mol/m³
        Q, V
    );

    SpeciesMassBalance_B<double> balance_B(
        10.0,  // Initial: 10 mol/m³
        40.0,  // Inlet: 40 mol/m³
        Q, V
    );

    EnergyBalance<double> energy(
        300.0,  // Initial: 300 K
        300.0,  // Inlet: 300 K
        Q, V, rho, cp
    );

    // Initial state
    double c_A = balance_A.getInitialState();
    double c_B = balance_B.getInitialState();
    double T = energy.getInitialState();

    std::cout << "Initial conditions:" << std::endl;
    std::cout << "  [A] = " << c_A << " mol/m³" << std::endl;
    std::cout << "  [B] = " << c_B << " mol/m³" << std::endl;
    std::cout << "  T   = " << T << " K" << std::endl;
    std::cout << std::endl;

    // Simulate one time step manually (demonstrating the concept)
    double t = 0.0;
    double dt = 0.01;  // 0.01 s time step

    // Step 1: Components provide current values from state
    auto provides_A = balance_A.provideFromState(c_A);
    auto provides_B = balance_B.provideFromState(c_B);
    auto provides_T = energy.provideFromState(T);

    // Step 2: Build dependency bundles for each component
    // (In full system, this would be automatic based on dependency graph)

    // Reaction needs: conc_A, conc_B, temp
    ArrheniusReaction<double>::Dependencies deps_reaction{
        Field<chemistry::Concentration, "conc_A">{.value = c_A},
        Field<chemistry::Concentration, "conc_B">{.value = c_B},
        Field<thermodynamics::Temperature, "temp">{.value = T}
    };

    auto provides_reaction = reaction.compute(t, deps_reaction);

    std::cout << "Computed provisions from reaction:" << std::endl;
    std::cout << "  rate = " << provides_reaction.template find<chemistry::ReactionRate, "rate_ABC">().value
              << " mol/(m³*s)" << std::endl;
    std::cout << "  heat = " << provides_reaction.template find<chemistry::HeatRelease, "heat_ABC">().value
              << " W/m³" << std::endl;
    std::cout << std::endl;

    // Mass balance A needs: rate_ABC
    SpeciesMassBalance_A<double>::Dependencies deps_A{
        Field<chemistry::ReactionRate, "rate_ABC">{
            .value = provides_reaction.template find<chemistry::ReactionRate, "rate_ABC">().value
        }
    };

    double dc_A_dt = balance_A.computeDerivative(t, c_A, deps_A);

    // Mass balance B needs: rate_ABC
    SpeciesMassBalance_B<double>::Dependencies deps_B{
        Field<chemistry::ReactionRate, "rate_ABC">{
            .value = provides_reaction.template find<chemistry::ReactionRate, "rate_ABC">().value
        }
    };

    double dc_B_dt = balance_B.computeDerivative(t, c_B, deps_B);

    // Energy balance needs: heat_ABC
    EnergyBalance<double>::Dependencies deps_energy{
        Field<chemistry::HeatRelease, "heat_ABC">{
            .value = provides_reaction.template find<chemistry::HeatRelease, "heat_ABC">().value
        }
    };

    double dT_dt = energy.computeDerivative(t, T, deps_energy);

    std::cout << "Computed derivatives:" << std::endl;
    std::cout << "  d[A]/dt = " << dc_A_dt << " mol/(m³*s)" << std::endl;
    std::cout << "  d[B]/dt = " << dc_B_dt << " mol/(m³*s)" << std::endl;
    std::cout << "  dT/dt   = " << dT_dt << " K/s" << std::endl;
    std::cout << std::endl;

    // Euler step
    c_A += dc_A_dt * dt;
    c_B += dc_B_dt * dt;
    T += dT_dt * dt;

    std::cout << "After one time step (dt = " << dt << " s):" << std::endl;
    std::cout << "  [A] = " << c_A << " mol/m³" << std::endl;
    std::cout << "  [B] = " << c_B << " mol/m³" << std::endl;
    std::cout << "  T   = " << T << " K" << std::endl;
    std::cout << std::endl;

    std::cout << "=== Key Observations ===" << std::endl;
    std::cout << "1. Each component is COMPLETELY DECOUPLED" << std::endl;
    std::cout << "   - ArrheniusReaction doesn't know about mass balances" << std::endl;
    std::cout << "   - SpeciesMassBalance doesn't know about energy balance" << std::endl;
    std::cout << "   - Components only know their Dependencies and Provides interfaces" << std::endl;
    std::cout << std::endl;
    std::cout << "2. Type-safe dependency injection" << std::endl;
    std::cout << "   - Compile error if field type mismatches" << std::endl;
    std::cout << "   - Compile error if field name doesn't exist" << std::endl;
    std::cout << std::endl;
    std::cout << "3. Scalable to 100+ components" << std::endl;
    std::cout << "   - Just add more SpeciesMassBalance<> for more species" << std::endl;
    std::cout << "   - Add more ArrheniusReaction<> for more reactions" << std::endl;
    std::cout << "   - System builder auto-wires everything!" << std::endl;

    return 0;
}
