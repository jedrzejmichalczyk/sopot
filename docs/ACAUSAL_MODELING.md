# Compile-Time Acausal Modeling in SOPOT

## Overview

SOPOT implements Modelica-style acausal (equation-based) modeling with compile-time causality resolution. This fills a gap in the ecosystem between hand-written equations and full Modelica toolchains.

## What is Acausal Modeling?

In **causal** modeling, you specify computation direction:
```cpp
// Causal: you decide what computes what
velocity = integrate(acceleration);
position = integrate(velocity);
```

In **acausal** modeling, you specify relationships only:
```cpp
// Acausal: equations, no direction
F = m * a          // Newton's second law
V = R * I          // Ohm's law
P = V * I          // Power definition
```

The system automatically determines which variable each equation solves for (**causality assignment**) and the order of evaluation (**BLT sorting**).

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    COMPILE TIME                                  │
├─────────────────────────────────────────────────────────────────┤
│  1. Equation Collection                                         │
│     - Components declare equations as types                     │
│     - Connections generate potential equality + flow conservation│
│                                                                 │
│  2. Incidence Matrix Construction                               │
│     - DependsOn<Expr, VarIdx> analyzes each equation            │
│     - IncidenceMatrix[eq][var] = true if equation uses variable │
│                                                                 │
│  3. Causality Assignment (Greedy Matching)                      │
│     - State variables marked as "known"                         │
│     - Each algebraic equation assigned to one unknown           │
│     - Result: assignment[eq] = var                              │
│                                                                 │
│  4. Topological Sort (BLT)                                      │
│     - Build equation dependency graph                           │
│     - Sort into evaluation order                                │
│     - Result: evalOrder[i] = equation index                     │
├─────────────────────────────────────────────────────────────────┤
│                    RUNTIME                                       │
├─────────────────────────────────────────────────────────────────┤
│  5. Direct Evaluation (no iteration!)                           │
│     - Execute equations in sorted order                         │
│     - Each equation directly computes its assigned variable     │
│     - For RC circuit: V_node=V_C; I_R=(V0-V_node)/R; I_C=I_R;  │
└─────────────────────────────────────────────────────────────────┘
```

## File Structure

```
physics/acausal/
├── connector.hpp    # Connector definitions
│   ├── Potential / Flow variable tags
│   ├── ElectricalPin (voltage, current)
│   ├── MechanicalFlange (position, velocity, force)
│   ├── RotationalFlange (angle, angular velocity, torque)
│   ├── HeatPort (temperature, heat flow)
│   └── Connection<A, B> equation generation
│
├── component.hpp    # Component library
│   ├── Equation<LHS, RHS> - acausal equation
│   ├── DerEquation<Var, Expr> - derivative equation
│   ├── Electrical: Resistor, Capacitor, Inductor, VoltageSource, Ground
│   └── Mechanical: Mass, Spring, Damper
│
├── causality.hpp    # Compile-time analysis
│   ├── DependsOn<Expr, VarIdx> - dependency analysis
│   ├── IncidenceRow<Eq, NumVars> - incidence matrix row
│   ├── IsLinearIn<Expr, VarIdx> - linearity check
│   └── DeepSimplify<Expr> - expression simplification
│
└── system.hpp       # System construction
    ├── IncidenceMatrix<Equations, NumVars>
    ├── findGreedyMatching() - causality assignment
    ├── sortEquations() - BLT topological sort
    └── ManualRCCircuit - working example
```

## Example: RC Circuit

### Equations (no direction specified)
```cpp
// Variables
using V_C = Var<0>;     // Capacitor voltage (STATE)
using V_node = Var<1>;  // Node voltage (algebraic)
using I_R = Var<2>;     // Resistor current (algebraic)
using I_C = Var<3>;     // Capacitor current (algebraic)

// Algebraic equations
using Eq0 = Equation<V_node, V_C>;                    // V_node = V_C
using Eq1 = Equation<Sub<V0, V_node>, Mul<R, I_R>>;  // V0 - V_node = R * I_R
using Eq2 = Equation<I_R, I_C>;                       // I_R = I_C

// Derivative equation
using DerV_C = Div<I_C, C>;  // der(V_C) = I_C / C
```

### Compile-Time Analysis Output
```
Incidence Matrix:
        V_C  V_node  I_R  I_C
Eq0:    1    1       0    0
Eq1:    0    1       1    0
Eq2:    0    0       1    1

Causality Assignment:
  Eq0 solves for: V_node
  Eq1 solves for: I_R
  Eq2 solves for: I_C

Evaluation Order:
  1. Eq0 (computes V_node)
  2. Eq1 (computes I_R)
  3. Eq2 (computes I_C)
```

### Generated Runtime Code
```cpp
// This is what actually runs - direct assignments, no iteration
vars[1] = vars[0];                              // V_node = V_C
vars[2] = (params[0] - vars[1]) / params[1];   // I_R = (V0 - V_node) / R
vars[3] = vars[2];                              // I_C = I_R
```

## Competitive Landscape

### Existing Tools

| Tool | Type | Causality | Platform | Overhead |
|------|------|-----------|----------|----------|
| **Modelica** (OpenModelica, Dymola) | Full acausal | Runtime | Desktop | Heavy toolchain |
| **ModelingToolkit.jl** | Full acausal | JIT | Julia | Runtime + GC |
| **SEMT** | Symbolic diff only | N/A | C++ | Compile-time |
| **libBondGraph** | Bond graphs | Runtime | C++ | XML, not header-only |
| **FMI/eFMI** | Model exchange | Generated | C export | Requires Modelica |
| **ViennaMath** | Symbolic math | N/A | C++ | No equations |
| **SOPOT Acausal** | Acausal modeling | **Compile-time** | **C++20** | **Zero** |

### Gap Analysis

```
                    Compile-time         Runtime
                    ────────────────────────────────
Header-only C++  │  SOPOT              (nothing)
                 │   ↑
                 │  UNIQUE POSITION
                 │
External tools   │  (nothing)          Modelica/FMI
                 │
Julia            │  (nothing)          ModelingToolkit.jl
```

**No existing tool provides:**
- Acausal/equation-based modeling
- Compile-time causality resolution
- Header-only C++
- Zero runtime overhead
- Embedded system targeting

## Performance Comparison vs ModelingToolkit.jl

| Aspect | ModelingToolkit.jl | SOPOT Acausal |
|--------|-------------------|---------------|
| Causality resolution | Runtime/JIT | Compile-time |
| First-run overhead | JIT warmup (~seconds) | Zero |
| Function dispatch | Some dynamic | All static, inlined |
| Memory allocation | GC managed | Stack only |
| Binary size | Julia runtime (~100MB) | Kilobytes |
| Bare metal MCU | Impossible | Native support |
| Deterministic timing | No (GC pauses) | Yes |

### Where SOPOT Wins
- **Embedded/real-time**: Deterministic timing, no GC pauses
- **Small systems (3-50 components)**: No JIT warmup, everything inlined
- **Bare metal**: No runtime needed, runs on Cortex-M

### Where ModelingToolkit.jl Wins
- **Large systems (1000+ equations)**: Better symbolic simplification
- **Nonlinear algebraic loops**: Has Newton solvers
- **Rapid prototyping**: No recompilation needed

## Use Cases

### Ideal for SOPOT Acausal
- Motor control (electrical + mechanical): 5-15 components
- Battery management (thermal + electrical): 10-20 components
- Hydraulic actuators (mechanical + fluid): 5-10 components
- Thrust vector control (servo + gimbal + body): 10-30 components
- HVAC systems (thermal + fluid + control): 15-30 components

### Better Suited for Modelica
- Full vehicle simulation: 1000+ components
- Building energy simulation: 500+ components
- Power grid simulation: Millions of nodes

## Future Directions

### 1. Integration with UnifiedGraphSystem
Combine the O(K) template graph machinery with acausal equations:
```cpp
// Define topology at runtime
auto system = UnifiedGraphSystem<...>();
system.addNode<ServoMotor>(...);
system.addNode<GimbalMechanism>(...);
system.addNode<RocketBody>(...);
system.connect(servo.shaft, gimbal.input);
system.connect(gimbal.output, body.thrustVector);

// Causality resolved at compile time per component type
// Topology flexibility at runtime
```

### 2. TVC (Thrust Vector Control) Example
Define entire TVC system with equations only:
```cpp
// Servo actuator
V = R*I + L*der(I) + Ke*omega
tau = Kt*I
J*der(omega) = tau - B*omega - tau_load

// Gimbal mechanism
tau_load = K_gimbal * (theta - theta_cmd)
der(theta) = omega

// Rocket body coupling
F_thrust = T * [sin(theta), 0, cos(theta)]
```

### 3. OpenModelica Component Translation
Most Modelica Standard Library components map directly:
```modelica
// Modelica
model Resistor
  Pin p, n;
  parameter Real R;
equation
  v = p.v - n.v;
  v = R * i;
  p.i + n.i = 0;
end Resistor;
```
```cpp
// SOPOT (nearly 1:1)
template<size_t BaseIdx, size_t ParamR>
struct Resistor {
    using Eq1 = Equation<V, Sub<PinP::V, PinN::V>>;
    using Eq2 = Equation<V, Mul<R, I>>;
    using Eq3 = Equation<PinP::I, Neg<PinN::I>>;
};
```

## Building and Testing

```bash
# Compile tests
g++ -std=c++20 -O3 -I. -o build/acausal_test tests/acausal_test.cpp
g++ -std=c++20 -O3 -I. -o build/acausal_system_test tests/acausal_system_test.cpp

# Run tests
./build/acausal_test
./build/acausal_system_test
```

## References

- Modelica Language Specification: https://modelica.org/
- Bond Graphs: Paynter, H.M. (1961) "Analysis and Design of Engineering Systems"
- BLT Transformation: Pantelides, C.C. (1988) "The Consistent Initialization of Differential-Algebraic Systems"
- SEMT (compile-time differentiation): https://github.com/st-gille/semt
- ModelingToolkit.jl: https://github.com/SciML/ModelingToolkit.jl
