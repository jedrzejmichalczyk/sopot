# SOPOT Roadmap: Toward System-Level Physical Design in Code

## The North Star

> **A complex physical system - a CubeSat, a drone, a medical device - defined entirely in C++ code. Structure, electronics, software, thermal, vibration, radiation - all in one place, all type-checked, all validated at compile time. Single source of truth. Change a component position once, every analysis updates. The code IS the design.**

```cpp
// The dream: cubesat.hpp IS the satellite
static_assert(ThermalAnalysis<MyCubeSat>::max_temp < 85_C);
static_assert(VibrationAnalysis<MyCubeSat>::first_mode > 100_Hz);
static_assert(RadiationAnalysis<MyCubeSat>::seu_rate < 1e-9);
static_assert(PowerBudget<MyCubeSat>::survives_eclipse);
// Compiles? Ready for flight.
```

---

## Current State (v0.1 - Foundation)

### What We Have

| Capability | Status | Notes |
|------------|--------|-------|
| Compile-time ODE systems | ✅ Done | TypedODESystem, RK4 integrator |
| Automatic differentiation | ✅ Done | Dual numbers, forward mode |
| Symbolic CAS | ✅ Done | Compile-time expressions, differentiation |
| Rocket 6-DOF simulation | ✅ Done | Full flight dynamics |
| Mass-spring systems | ✅ Done | 1D chains, 2D grids |
| Graph-based topology | ✅ Done | O(K) templates, runtime connectivity |
| Acausal modeling | ✅ Done | Modelica-style, compile-time causality |
| LQR control | ✅ Done | Inverted pendulum example |

### Proven Principles

1. **Zero runtime overhead** - All dispatch resolved at compile time
2. **Type-safe composition** - Components compose without runtime errors
3. **Compile-time validation** - Static assertions catch physics errors
4. **Header-only** - No build complexity, no dependencies
5. **Embedded-ready** - Runs on Cortex-M, deterministic timing

---

## Phase 1: Multi-Domain Acausal Modeling (v0.2)

**Goal:** Seamless electrical + mechanical + thermal modeling with automatic equation coupling

### Milestones

#### 1.1 Complete Electrical Domain
- [ ] Full Modelica Electrical.Analog equivalent
  - [ ] R, L, C, sources, ground
  - [ ] Diodes (piecewise linear)
  - [ ] Op-amps (ideal)
  - [ ] Transformers
- [ ] Connection semantics (Kirchhoff's laws automatic)
- [ ] Example: DC-DC converter model

#### 1.2 Complete Mechanical Domain
- [ ] Translational: mass, spring, damper, hard stops
- [ ] Rotational: inertia, spring, damper, gearbox
- [ ] Translational ↔ rotational coupling (rack-pinion, lead screw)
- [ ] Example: Servo actuator (electrical + mechanical)

#### 1.3 Thermal Domain
- [ ] Thermal mass (C_th)
- [ ] Thermal resistance (R_th)
- [ ] Convection, radiation boundaries
- [ ] Heat sources (power dissipation from electrical)
- [ ] Example: Motor thermal model

#### 1.4 Multi-Domain Coupling
- [ ] Electrical ↔ Mechanical: motors, generators
- [ ] Electrical ↔ Thermal: I²R losses
- [ ] Mechanical ↔ Thermal: friction losses
- [ ] Example: Complete brushed DC motor (electrical + mechanical + thermal)

### Deliverable
**TVC (Thrust Vector Control) system** - servo + gimbal + rocket body, all from equations only

---

## Phase 2: Flexible Body Dynamics (v0.3)

**Goal:** Structural dynamics integrated with control design

### Milestones

#### 2.1 Analytical Beam Modes
- [ ] Euler-Bernoulli beam theory
- [ ] Boundary conditions: free-free, clamped-free, etc.
- [ ] Mode shapes as compile-time expressions
- [ ] Natural frequencies from geometry/material

#### 2.2 Modal Dynamics Integration
- [ ] Modal coordinates as state variables
- [ ] Automatic modal participation at arbitrary positions
- [ ] Sensor placement analysis (what modes does IMU see?)
- [ ] Actuator placement analysis (what modes does force excite?)

#### 2.3 Flexible Body + Control
- [ ] Coupled rigid-body + flexible dynamics
- [ ] Compile-time notch filter frequency computation
- [ ] Example: Flexible rocket with TVC

#### 2.4 Extended Structures
- [ ] Plate modes (analytical for rectangular)
- [ ] Simple trusses / frame structures
- [ ] Craig-Bampton interface (import from FEM if needed)

### Deliverable
**Flexible launch vehicle control design** - IMU placement optimization, notch filter synthesis, all at compile time

---

## Phase 3: Thermal Networks (v0.4)

**Goal:** Thermal analysis integrated with system design

### Milestones

#### 3.1 Lumped Parameter Thermal
- [ ] Thermal node (temperature state)
- [ ] Conduction links (geometry-based R_th)
- [ ] Convection boundaries
- [ ] Radiation boundaries (view factors)

#### 3.2 Heat Sources Integration
- [ ] Automatic I²R from electrical domain
- [ ] Power dissipation tags on components
- [ ] Solar flux (orbit-dependent)

#### 3.3 Transient Thermal Analysis
- [ ] Orbital thermal cycling
- [ ] Eclipse entry/exit
- [ ] Compile-time worst-case temperature bounds

#### 3.4 Thermal ↔ Other Domains
- [ ] Temperature-dependent resistance
- [ ] Temperature-dependent battery capacity
- [ ] Thermal expansion (simple)

### Deliverable
**CubeSat thermal model** - electronics dissipation → structure → radiation to space, with orbital cycling

---

## Phase 4: Geometry Foundation (v0.5)

**Goal:** Spatial reasoning for physics coupling

### Milestones

#### 4.1 Compile-Time Primitives
- [ ] Points, vectors, transforms (already have Vector3)
- [ ] Simple shapes: box, cylinder, sphere
- [ ] CSG operations: union, intersection, difference
- [ ] Volume, surface area, centroid, inertia tensor

#### 4.2 Spatial Queries
- [ ] Point containment
- [ ] Ray casting (simple)
- [ ] Distance queries
- [ ] View factor computation (between surfaces)

#### 4.3 Mesh Generation (Runtime)
- [ ] Simple structured meshing
- [ ] Surface discretization for thermal radiation
- [ ] Interface for imported geometry

#### 4.4 Physics ↔ Geometry Coupling
- [ ] Component positions in 3D space
- [ ] Automatic inertia tensor from geometry
- [ ] Automatic thermal view factors
- [ ] Shielding thickness calculation (for radiation)

### Deliverable
**Parametric satellite structure** - geometry defines mass properties, thermal paths, shielding

---

## Phase 5: Power Systems (v0.6)

**Goal:** Complete power budget analysis

### Milestones

#### 5.1 Power Sources
- [ ] Solar panel model (area, efficiency, orientation)
- [ ] Battery model (capacity, SoC, temperature effects)
- [ ] Orbital illumination calculation

#### 5.2 Power Consumption
- [ ] Component power tags
- [ ] Operating modes (active, sleep, off)
- [ ] Duty cycling

#### 5.3 Power Budget Analysis
- [ ] Worst-case power balance
- [ ] Eclipse survival analysis
- [ ] Battery sizing
- [ ] Solar array sizing

#### 5.4 Power ↔ Thermal Coupling
- [ ] Dissipation = consumption - useful work
- [ ] Battery temperature limits
- [ ] Heater power during eclipse

### Deliverable
**CubeSat power system** - solar + battery + loads, with orbital and thermal coupling

---

## Phase 6: Radiation Environment (v0.7)

**Goal:** Space radiation effects analysis

### Milestones

#### 6.1 Radiation Environment Models
- [ ] Trapped radiation (Van Allen belts)
- [ ] Solar particle events
- [ ] Galactic cosmic rays
- [ ] Orbit-dependent flux (LEO, GEO, etc.)

#### 6.2 Shielding Analysis
- [ ] Material shielding effectiveness
- [ ] Geometry-based shielding thickness
- [ ] Sector analysis (shielding from different directions)

#### 6.3 Effects Analysis
- [ ] Total Ionizing Dose (TID) accumulation
- [ ] Single Event Upset (SEU) rate estimation
- [ ] Component radiation tolerance database

#### 6.4 Mitigation Strategies
- [ ] TMR (Triple Modular Redundancy) modeling
- [ ] EDAC effectiveness
- [ ] Radiation-hardened part selection

### Deliverable
**Radiation analysis** - orbit → flux → shielding → dose/SEU rate, compile-time bounds checking

---

## Phase 7: Electronics Integration (v0.8)

**Goal:** PCB and electronics in the unified model

### Milestones

#### 7.1 Component Library
- [ ] MCU models (power modes, computational capacity)
- [ ] Sensors (IMU, magnetometer, sun sensor, etc.)
- [ ] Actuators (magnetorquer, reaction wheel interface)
- [ ] Communication (radio power, data rates)

#### 7.2 PCB Representation
- [ ] Component placement (3D position)
- [ ] Power nets
- [ ] Signal integrity (optional, simplified)
- [ ] Thermal via to structure

#### 7.3 Electronics ↔ Physics
- [ ] Component mass in structure
- [ ] Component position for vibration analysis
- [ ] Power dissipation for thermal analysis
- [ ] Radiation exposure from position

#### 7.4 External Tool Interface
- [ ] JITX integration (programmatic PCB)
- [ ] KiCad netlist import
- [ ] BOM generation

### Deliverable
**Integrated avionics model** - PCB definition flows into thermal, vibration, radiation analyses

---

## Phase 8: Flight Software Integration (v0.9)

**Goal:** Software and physics in one model

### Milestones

#### 8.1 Control Systems
- [ ] Discrete-time controllers
- [ ] State machines
- [ ] Mode management

#### 8.2 Software ↔ Physics
- [ ] Sensor models (noise, quantization, delay)
- [ ] Actuator models (saturation, dynamics, delay)
- [ ] Computational timing (control loop rates)

#### 8.3 Simulation Modes
- [ ] Model-in-the-loop (MIL) - everything simulated
- [ ] Software-in-the-loop (SIL) - real control code
- [ ] Processor-in-the-loop (PIL) - real target timing

#### 8.4 Code Generation
- [ ] Generate embedded C from controller models
- [ ] Consistent with simulation model
- [ ] Verified by simulation

### Deliverable
**ADCS flight software** - generated from same model used for analysis, guaranteed consistent

---

## Phase 9: Full System Integration (v1.0)

**Goal:** The North Star realized

### Milestones

#### 9.1 System-Level Definition
- [ ] Single file/module defines entire system
- [ ] Hierarchical composition
- [ ] Variant management (different configurations)

#### 9.2 Compile-Time Validation Suite
- [ ] Thermal limits (all components)
- [ ] Vibration limits (launch survival)
- [ ] Radiation limits (mission lifetime)
- [ ] Power budget (all modes)
- [ ] Mass budget
- [ ] Data budget

#### 9.3 Design Exploration
- [ ] Parameter sweeps
- [ ] Trade studies
- [ ] Sensitivity analysis
- [ ] Optimization interface

#### 9.4 Documentation Generation
- [ ] Auto-generated specifications
- [ ] Analysis reports
- [ ] Compliance matrices

### Deliverable
**Complete CubeSat design** - structure, electronics, software, all analyses, all in code, all validated at compile time

---

## Technical Challenges

### Compile-Time Limits
- **Constexpr depth/operations** - May need to split compilation
- **Template instantiation explosion** - Keep O(K) discipline
- **Compiler memory** - GCC struggles with large constexpr, Clang better

### Numerical Precision
- **Floating point in constexpr** - C++20 allows, but determinism matters
- **Iterative solvers at compile time** - Need convergence guarantees

### Complexity Management
- **Readability** - Template heavy code can be hard to understand
- **Error messages** - C++ template errors are notoriously bad
- **Build times** - Compile-time computation is slow

### External Integration
- **Geometry import** - Can't do full CAD at compile time
- **FEM data** - May need runtime import for complex structures
- **Databases** - Component libraries can't all be constexpr

### Mitigation Strategies
1. **Hybrid approach** - Compile-time where beneficial, runtime where necessary
2. **Caching** - Pre-compute and cache complex analyses
3. **Layers** - Core physics compile-time, UI/viz runtime
4. **Tooling** - Custom error messages, IDE support

---

## Use Cases Along the Way

| Phase | Capability Unlocked | Example Use Case |
|-------|--------------------|--------------------|
| 0.1 | Dynamics + control | Rocket flight simulation, inverted pendulum |
| 0.2 | Multi-domain acausal | Motor control system, power electronics |
| 0.3 | Flexible bodies | Launch vehicle control, robotic arm vibration |
| 0.4 | Thermal networks | Electronics cooling, battery thermal management |
| 0.5 | Geometry | Parametric mechanism design, mass optimization |
| 0.6 | Power systems | Solar car, UAV endurance, satellite power |
| 0.7 | Radiation | Space mission planning, nuclear environments |
| 0.8 | Electronics | Integrated avionics, IoT device design |
| 0.9 | Flight software | Autopilot development, ADCS design |
| 1.0 | Full system | CubeSat, drone, medical device, rover |

---

## Philosophy

### Why Code?

1. **Version control** - Git tracks every change, enables collaboration
2. **Review** - Code review > document review for catching errors
3. **Testing** - Automated tests for design rules
4. **Reproducibility** - Same code = same design, always
5. **Composition** - Import, extend, customize like software libraries
6. **Automation** - CI/CD for physical systems

### Why Compile-Time?

1. **Early errors** - Catch mistakes before simulation runs
2. **Performance** - Zero overhead in final code
3. **Optimization** - Compiler can optimize across boundaries
4. **Embedded** - No runtime allocation, deterministic
5. **Trust** - If it compiles, physics constraints are satisfied

### Why C++?

1. **Performance** - When you need it, it's there
2. **Embedded** - Runs everywhere, bare metal to cloud
3. **Ecosystem** - Vast tooling, IDE support, debuggers
4. **Templates** - Powerful enough for compile-time physics
5. **Longevity** - C++ isn't going anywhere

---

## Call to Action

This roadmap is ambitious. It may take years. But the direction is clear:

> **Physical system design will become software engineering.**

The question is not *if*, but *who* and *when*.

SOPOT is positioned to be the foundation. Each phase delivers value independently. The North Star guides, but each step is useful on its own.

Let's build it.

---

*Document created: January 2026*
*Last updated: January 2026*
*Version: 0.1.0*
