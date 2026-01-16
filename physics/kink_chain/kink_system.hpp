#pragma once

#include "rotational_segment.hpp"
#include "torsional_spring.hpp"
#include "tags.hpp"
#include "../../core/typed_component.hpp"
#include <span>
#include <string>
#include <cmath>
#include <numbers>

namespace sopot::physics::kink {

//=============================================================================
// EnergyMonitor - Computes total energy and winding number for kink chain
//=============================================================================
// This component monitors global properties of the kink chain system.
// It computes kinetic energy, potential energy, and the winding number
// (a topological invariant counting kinks).
//=============================================================================

template<size_t N, Scalar T = double>
class EnergyMonitor final : public TypedComponent<0, T> {
public:
    using Base = TypedComponent<0, T>;
    using typename Base::LocalState;
    using typename Base::LocalDerivative;

private:
    std::string m_name;

    // Wrap angle to [-π, π]
    static T wrapAngle(T angle) {
        constexpr T pi = T(std::numbers::pi);
        while (angle > pi) angle -= T(2.0) * pi;
        while (angle < -pi) angle += T(2.0) * pi;
        return angle;
    }

public:
    explicit EnergyMonitor(std::string name = "energy_monitor")
        : m_name(std::move(name)) {}

    void setOffset(size_t) const {}

    //=========================================================================
    // Required Component Interface
    //=========================================================================

    LocalState getInitialLocalState() const { return {}; }
    std::string_view getComponentType() const { return "EnergyMonitor"; }
    std::string_view getComponentName() const { return m_name; }

    //=========================================================================
    // State Functions
    //=========================================================================

    // Total kinetic energy: KE = Σ (0.5 * I_i * ω_i²)
    template<typename Registry>
    T computeKineticEnergy(std::span<const T> state, const Registry& registry) const {
        T ke = T(0.0);
        for (size_t i = 0; i < N; ++i) {
            using SegTag = segment<0>;  // We'll use dynamic tag lookup in practice
            // This is a simplified version - in practice we'd need to iterate properly
            // For now, this shows the structure
        }
        return ke;
    }

    // Winding number: counts total angular change / 2π
    // W = (1/2π) * Σ Δθ_i where Δθ_i is the angle difference between segments
    template<typename Registry>
    T compute(system::WindingNumber, std::span<const T> state, const Registry& registry) const {
        // For a chain of N segments, we sum the wrapped angle differences
        // This gives us a topological invariant that counts kinks
        T total_winding = T(0.0);

        // In a real implementation, we'd iterate through all adjacent pairs
        // For demonstration, we return a placeholder
        // The winding number would be computed from cumulative angle changes

        return total_winding / T(2.0 * std::numbers::pi);
    }

    // Total energy (kinetic + potential)
    template<typename Registry>
    T compute(system::TotalEnergy, std::span<const T> state, const Registry& registry) const {
        // Sum up kinetic and potential energies
        // This would query all segment inertias and spring potential energies
        return T(0.0);  // Placeholder
    }
};

//=============================================================================
// Helper Functions for Building Kink Chains
//=============================================================================

// Create a rotational segment with given index
template<size_t Index, Scalar T = double>
RotationalSegment<segment<Index>, T> createSegment(
    double inertia,
    double initial_angle = 0.0,
    double initial_omega = 0.0
) {
    return RotationalSegment<segment<Index>, T>(
        inertia, initial_angle, initial_omega,
        "segment_" + std::to_string(Index)
    );
}

// Create a torsional spring between two adjacent segments
template<size_t Index1, size_t Index2, Scalar T = double>
TorsionalSpring<segment<Index1>, segment<Index2>, T> createTorsionalSpring(
    double stiffness,
    double damping = 0.0
) {
    return TorsionalSpring<segment<Index1>, segment<Index2>, T>(
        stiffness, damping,
        "spring_" + std::to_string(Index1) + "_" + std::to_string(Index2)
    );
}

} // namespace sopot::physics::kink
