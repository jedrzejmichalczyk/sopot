/*
 * Simple Chemical Reactor Example - Simplified Proof of Concept
 * Demonstrates the field-based component composition concept
 *
 * System: A + B -> C (exothermic reaction in a CSTR)
 */

#include "field_based_component_simple.hpp"
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
}

// ============================================================================
// COMPONENT 1: Arrhenius Reaction A + B -> C
// ============================================================================

template<typename T = double>
class ArrheniusReaction {
public:
    static constexpr size_t StateSize = 0;  // Stateless provider

    // What this component NEEDS (declared, not queried!)
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
    Provides compute(T /* t */, T c_A, T c_B, T temp) const {
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
        Field<chemistry::ReactionRate, "rate_ABC">
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

    T getInitialState() const {
        return m_initial;
    }

    T provideConcentration(const T& local_state) const {
        return local_state;
    }

    // Compute derivative: dC_A/dt = (Q/V)*(C_in - C_A) - r_ABC
    T computeDerivative(T /* t */, const T& local_state, T rate) const {
        T c_A = local_state;

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

    T provideConcentration(const T& local_state) const {
        return local_state;
    }

    T computeDerivative(T /* t */, const T& local_state, T rate) const {
        T c_B = local_state;

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

    T provideTemperature(const T& local_state) const {
        return local_state;
    }

    // dT/dt = (Q/V)*(T_in - T) + Q_rxn / (ρ * V * cp)
    T computeDerivative(T /* t */, const T& local_state, T Q_rxn) const {
        T temp = local_state;

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
// DEMONSTRATION
// ============================================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "SOPOT V2: Compiler-Inspired Components" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "System: A + B -> C (exothermic reaction in CSTR)" << std::endl;
    std::cout << std::endl;

    // Component parameters
    constexpr double V = 1.0;       // 1 m³ reactor
    constexpr double Q = 0.1;       // 0.1 m³/s flow rate
    constexpr double rho = 1000.0;  // 1000 kg/m³ density
    constexpr double cp = 4184.0;   // 4184 J/(kg*K) heat capacity

    // Create components (completely independent!)
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

    // Simulate one time step (manual wiring - will be automatic in full system!)
    double t = 0.0;
    double dt = 0.01;  // 0.01 s time step

    // Step 1: Reaction component computes its provisions
    auto provisions_reaction = reaction.compute(t, c_A, c_B, T);
    double rate = provisions_reaction.get<0>().value;
    double heat = provisions_reaction.get<1>().value;

    std::cout << "Reaction provisions:" << std::endl;
    std::cout << "  rate = " << rate << " mol/(m³*s)" << std::endl;
    std::cout << "  heat = " << heat << " W/m³" << std::endl;
    std::cout << std::endl;

    // Step 2: Mass balances use reaction rate
    double dc_A_dt = balance_A.computeDerivative(t, c_A, rate);
    double dc_B_dt = balance_B.computeDerivative(t, c_B, rate);

    // Step 3: Energy balance uses heat release
    double dT_dt = energy.computeDerivative(t, T, heat);

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

    std::cout << "========================================" << std::endl;
    std::cout << "Key Architectural Insights" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "1. COMPLETE DECOUPLING" << std::endl;
    std::cout << "   - ArrheniusReaction doesn't know about mass balances" << std::endl;
    std::cout << "   - SpeciesMassBalance doesn't know about energy balance" << std::endl;
    std::cout << "   - Components only declare Dependencies and Provides" << std::endl;
    std::cout << std::endl;
    std::cout << "2. TYPE-SAFE INTERFACES" << std::endl;
    std::cout << "   - Field<Tag, Name> provides compile-time checking" << std::endl;
    std::cout << "   - Dependencies are explicit, not hidden queries" << std::endl;
    std::cout << std::endl;
    std::cout << "3. COMPILER-LIKE SYSTEM BUILDER (to be implemented)" << std::endl;
    std::cout << "   - Dependency resolution: Find providers for each dependency" << std::endl;
    std::cout << "   - Topological sort: Determine execution order" << std::endl;
    std::cout << "   - Code generation: Auto-wire components efficiently" << std::endl;
    std::cout << "   - Optimization: Eliminate redundant computations" << std::endl;
    std::cout << std::endl;
    std::cout << "4. SCALABILITY TO 100+ COMPONENTS" << std::endl;
    std::cout << "   - Just add more SpeciesMassBalance for more species" << std::endl;
    std::cout << "   - Add more reactions, each independent" << std::endl;
    std::cout << "   - System builder auto-wires everything at compile-time" << std::endl;
    std::cout << "   - Zero runtime overhead!" << std::endl;
    std::cout << std::endl;
    std::cout << "5. LIKE BUILDING A COMPILER" << std::endl;
    std::cout << "   - Components are \"modules\"" << std::endl;
    std::cout << "   - Dependencies are \"imports/requires\"" << std::endl;
    std::cout << "   - Provisions are \"exports/provides\"" << std::endl;
    std::cout << "   - System builder is the \"linker\"" << std::endl;
    std::cout << std::endl;

    return 0;
}
