# SOPOT Framework - Principles Compliance Audit Report

**Date:** 2026-01-17
**Auditor:** Claude (Sonnet 4.5)
**Scope:** Complete codebase audit for SOPOT principles adherence

## Executive Summary

This audit evaluated all components in the SOPOT framework for compliance with core principles:
- **Modularity** - Components should be self-contained with clear interfaces
- **Pure State Functions** - Query data through tags, not direct access
- **No Magic Indices** - Use abstraction layers, not hardcoded offsets
- **No Cross-Module Assumptions** - Components shouldn't assume other components' state layout

### Key Findings

✅ **Core Framework (core/):** EXCELLENT - Zero violations, clean abstraction
⚠️ **Rocket Components (rocket/):** VIOLATIONS FOUND - Magic indices in 5 files
⚠️ **Physics Components (physics/):** MIXED - Some violations, some exemplary code
🔴 **Critical Issue:** StandardAtmosphere has HARDCODED STATE LAYOUT ASSUMPTION

---

## Violations Found

### 🔴 CRITICAL SEVERITY

#### 1. StandardAtmosphere - Hardcoded State Layout Assumption

**File:** `rocket/standard_atmosphere.hpp`
**Lines:** 34, 77, 80, 83, 86
**Severity:** CRITICAL - Violates modularity principle

```cpp
// Line 34: UNACCEPTABLE ASSUMPTION
mutable size_t m_altitude_offset{3}; // Position Z is at offset 3 (after time + pos_xy)

// Lines 77-86: Direct state access based on assumption
T compute(environment::AtmosphericPressure, std::span<const T> state) const {
    return computePressure(state[m_altitude_offset]);  // ❌ VIOLATION
}
```

**Problem:**
- Assumes altitude is at index 3 in global state vector
- Comment reveals coupling: "after time + pos_xy" - assumes specific system composition
- Breaks when system changes (e.g., different component ordering)
- Violates fundamental SOPOT principle: components shouldn't know about other components' state layout

**Impact:** HIGH - System will break if component ordering changes

**Recommendation:**
```cpp
// CORRECT: Query altitude through registry
template<typename Registry>
T compute(environment::AtmosphericPressure, std::span<const T> state,
          const Registry& registry) const {
    T altitude = registry.template computeFunction<kinematics::Altitude>(state);
    return computePressure(altitude);
}
```

---

### ⚠️ MEDIUM SEVERITY

#### 2. Rocket Components - Magic Index Pattern

**Files with violations:**
1. `rocket/translation_kinematics.hpp` - Lines 61-62, 66
2. `rocket/translation_dynamics.hpp` - Line 47
3. `rocket/rotation_kinematics.hpp` - Line 51
4. `rocket/rotation_dynamics.hpp` - Line 52

**Pattern:**
```cpp
// ❌ VIOLATION: Magic indices with m_offset
Vector3<T> compute(kinematics::VelocityENU, std::span<const T> state) const {
    return {state[m_offset], state[m_offset + 1], state[m_offset + 2]};
}

// ✅ CORRECT: Use base class abstraction
Vector3<T> compute(kinematics::VelocityENU, std::span<const T> state) const {
    return {
        this->getGlobalState(state, 0),
        this->getGlobalState(state, 1),
        this->getGlobalState(state, 2)
    };
}
```

**Analysis:**
- Direct array indexing with `state[m_offset + X]`
- Hardcoded offsets (0, 1, 2, 3)
- Base class provides `getGlobalState()` helper, but not used
- Less severe than StandardAtmosphere since offsets are relative to component

**Impact:** MEDIUM - Violates abstraction principle, harder to maintain

---

#### 3. Physics Components - Inconsistent Abstraction Use

**Files with violations:**
1. `physics/coupled_oscillator/point_mass.hpp` - Lines 94, 98
2. `physics/harmonic_oscillator.hpp` - Lines 95, 99, 103-104, 109, 114

**Example:**
```cpp
// ❌ VIOLATION (coupled_oscillator/point_mass.hpp)
T compute(typename TagSet::Position, std::span<const T> state) const {
    return state[m_offset];  // Direct indexing
}
```

**Contrast with Good Examples:**
```cpp
// ✅ CORRECT (connected_masses/indexed_point_mass.hpp:102)
T compute(typename TagSet::Position, std::span<const T> state) const {
    return this->getGlobalState(state, 0);  // Uses abstraction
}
```

**Impact:** MEDIUM - Creates inconsistency in codebase

---

## Good Practices Observed

### ✅ Exemplary Components

#### 1. Core Framework (core/)
- **typed_component.hpp** - Clean abstraction with `getGlobalState()` helper
- **TypedRegistry** - Perfect compile-time dispatch, no runtime overhead
- **TypedODESystem** - Zero magic numbers, all offsets computed at compile time

#### 2. Stateless Components (Perfect Modularity)
- `rocket/force_aggregator.hpp` - Queries everything through registry
- `rocket/axisymmetric_aero.hpp` - Zero state, pure computation
- `physics/coupled_oscillator/spring.hpp` - Registry-only queries

#### 3. Recent Physics Components (Best Practices)
- `physics/connected_masses/indexed_point_mass.hpp` ✅
- `physics/connected_masses/indexed_point_mass_2d.hpp` ✅
- `physics/pendulum/double_pendulum.hpp` ✅
- `physics/pendulum/cartesian_pendulum.hpp` ✅
- `physics/control/cart_double_pendulum.hpp` ✅

**Common pattern in good components:**
```cpp
T compute(TagSet::Position, std::span<const T> state) const {
    return this->getGlobalState(state, 0);  // ✅ Uses base class helper
}
```

---

## Detailed File Analysis

### Files Using Magic Indices (❌ VIOLATIONS)

| File | Lines | Pattern | Severity |
|------|-------|---------|----------|
| `rocket/standard_atmosphere.hpp` | 34, 77, 80, 83, 86 | `state[m_altitude_offset]` where offset=3 | 🔴 CRITICAL |
| `rocket/translation_kinematics.hpp` | 61-62, 66 | `state[m_offset + X]` | ⚠️ MEDIUM |
| `rocket/translation_dynamics.hpp` | 47 | `state[m_offset + X]` | ⚠️ MEDIUM |
| `rocket/rotation_kinematics.hpp` | 51 | `state[m_offset + X]` | ⚠️ MEDIUM |
| `rocket/rotation_dynamics.hpp` | 52 | `state[m_offset + X]` | ⚠️ MEDIUM |
| `physics/coupled_oscillator/point_mass.hpp` | 94, 98 | `state[m_offset + X]` | ⚠️ MEDIUM |
| `physics/harmonic_oscillator.hpp` | 95, 99, 103-104, 109, 114 | `state[m_offset + X]` | ⚠️ MEDIUM |

### Files Using Proper Abstraction (✅ GOOD)

| File | Pattern | Status |
|------|---------|--------|
| `physics/connected_masses/indexed_point_mass.hpp` | `this->getGlobalState(state, idx)` | ✅ EXCELLENT |
| `physics/connected_masses/indexed_point_mass_2d.hpp` | `this->getGlobalState(state, idx)` | ✅ EXCELLENT |
| `physics/pendulum/double_pendulum.hpp` | `this->getGlobalState(state, idx)` | ✅ EXCELLENT |
| `physics/pendulum/cartesian_pendulum.hpp` | `this->getGlobalState(state, idx)` | ✅ EXCELLENT |
| `rocket/force_aggregator.hpp` | Registry queries only | ✅ EXCELLENT |
| `rocket/axisymmetric_aero.hpp` | Registry queries only | ✅ EXCELLENT |

---

## Recommendations

### Priority 1: Fix StandardAtmosphere (CRITICAL)

**Current violation:**
```cpp
// ❌ HARDCODED: Assumes altitude at index 3
mutable size_t m_altitude_offset{3};
return computePressure(state[m_altitude_offset]);
```

**Fix:**
```cpp
// ✅ Query through registry
template<typename Registry>
T compute(environment::AtmosphericPressure, std::span<const T> state,
          const Registry& registry) const {
    T altitude = registry.template computeFunction<kinematics::Altitude>(state);
    return computePressure(altitude);
}
```

### Priority 2: Standardize State Access Pattern

Replace all instances of:
```cpp
state[m_offset + X]  // ❌ Magic index
```

With:
```cpp
this->getGlobalState(state, X)  // ✅ Proper abstraction
```

**Files to update:**
1. All rocket kinematics/dynamics components (5 files)
2. `physics/coupled_oscillator/point_mass.hpp`
3. `physics/harmonic_oscillator.hpp`

### Priority 3: Add Codebase Guidelines

Create `CONTRIBUTING.md` with:
```markdown
## Component State Access Rules

✅ DO: Use base class helpers
   - `this->getGlobalState(state, index)` for own state
   - `registry.computeFunction<Tag>(state)` for other components

❌ DON'T: Direct array indexing
   - Never use `state[m_offset + X]`
   - Never assume global state layout
   - Never hardcode state offsets
```

---

## Code Quality Metrics

### Compliance Score by Directory

| Directory | Total Components | Violations | Compliance |
|-----------|-----------------|------------|------------|
| `core/` | 8 | 0 | 100% ✅ |
| `rocket/` | 13 | 5 | 62% ⚠️ |
| `physics/coupled_oscillator/` | 5 | 1 | 80% ⚠️ |
| `physics/connected_masses/` | 10 | 0 | 100% ✅ |
| `physics/pendulum/` | 5 | 0 | 100% ✅ |
| `physics/control/` | 4 | 0 | 100% ✅ |
| **Overall** | **45** | **7** | **84%** |

### Timeline Observation

**Older code** (rocket/, coupled_oscillator/):
- Uses `state[m_offset + X]` pattern
- Created before `getGlobalState()` was widely adopted

**Newer code** (connected_masses/, pendulum/, control/):
- Uses `this->getGlobalState()` consistently
- Better adherence to SOPOT principles

**Conclusion:** Coding standards improved over time, but legacy code needs refactoring.

---

## Testing Impact Analysis

### Current Risks

1. **StandardAtmosphere Brittleness**
   - If components are reordered, atmosphere calculations will break
   - No compile-time safety
   - Silent failure mode (wrong altitude used)

2. **Maintenance Burden**
   - Magic indices require manual tracking
   - Refactoring is error-prone
   - New developers may not understand offset calculations

3. **Composability Issues**
   - Cannot easily swap component implementations
   - Hard to create alternative system configurations

---

## Conclusion

The SOPOT framework has a **solid foundation** with excellent core abstractions. However, **7 components violate** the principles of modularity and pure state functions.

### Critical Action Items

1. 🔴 **IMMEDIATE:** Fix `StandardAtmosphere` hardcoded offset assumption
2. ⚠️ **HIGH:** Refactor 5 rocket components to use `getGlobalState()`
3. ⚠️ **MEDIUM:** Update 2 physics components for consistency
4. ✅ **LOW:** Document best practices for future contributors

### Positive Observations

- **Core framework is exemplary** - zero violations
- **Recent components follow best practices** - shows learning/evolution
- **Most stateless components are perfect** - good architectural choices
- **84% overall compliance** - better than many codebases

### Path Forward

The newer components (`connected_masses/`, `pendulum/`, `control/`) demonstrate that the team **knows the right patterns**. The violations are in **legacy code** that predates current standards. A focused refactoring sprint can bring compliance to **100%**.

---

## Appendix: Quick Reference

### ✅ Correct Patterns

```cpp
// 1. Query other components through registry
template<typename Registry>
LocalDerivative derivatives(..., const Registry& registry) const {
    T velocity = registry.template computeFunction<kinematics::Velocity>(global);
    return {velocity};
}

// 2. Access own state with abstraction
T compute(Tag, std::span<const T> state) const {
    return this->getGlobalState(state, 0);  // Not state[m_offset]
}

// 3. Stateless components - no assumptions
template<typename Registry>
T compute(Tag, std::span<const T> state, const Registry& registry) const {
    T value1 = registry.template computeFunction<Tag1>(state);
    T value2 = registry.template computeFunction<Tag2>(state);
    return computeSomething(value1, value2);
}
```

### ❌ Anti-Patterns to Avoid

```cpp
// 1. Hardcoded global state assumptions
mutable size_t m_some_offset{3};  // ❌ Assumes state layout
return state[m_some_offset];      // ❌ Breaks modularity

// 2. Magic indices
return state[m_offset + 1];       // ❌ Use getGlobalState(state, 1)

// 3. Direct array access without abstraction
Vector3<T> vel{state[5], state[6], state[7]};  // ❌ Magic numbers
```

---

**End of Report**

Generated by automated audit tool.
For questions, see `CLAUDE.md` for framework documentation.
