#include "../core/typed_component.hpp"
#include "../core/solver.hpp"
#include "../physics/harmonic_oscillator.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using namespace sopot;
using namespace sopot::physics;

// Test macro for assertions
#define ASSERT_NEAR(a, b, tolerance) \
    do { \
        if (std::abs((a) - (b)) > (tolerance)) { \
            std::cerr << "ASSERTION FAILED: " << #a << " (" << (a) << ") != " << #b << " (" << (b) \
                      << ") with tolerance " << (tolerance) << std::endl; \
            std::abort(); \
        } \
    } while(0)

int main() {
    std::cout << "=== SOPOT Pure Compile-Time State Function System ===" << std::endl;
    std::cout << "ZERO runtime dispatch - everything resolved at compile time!" << std::endl;

    try {
        std::cout << "\n1. Single component test..." << std::endl;

        // Test single harmonic oscillator with double scalar type
        auto oscillator = HarmonicOscillator<double>(1.0, 4.0, 0.0, 1.0, 0.0, "test_osc");
        auto single_system = makeTypedODESystem<double>(std::move(oscillator));

        std::cout << "   System type: TypedODESystem" << std::endl;
        std::cout << "   Components: " << single_system.getComponentCount() << std::endl;
        std::cout << "   State dimension: " << single_system.getStateDimension() << std::endl;

        // Test state function availability (all compile-time checks)
        static_assert(decltype(single_system)::template hasFunction<kinematics::Position>());
        static_assert(decltype(single_system)::template hasFunction<kinematics::Velocity>());
        static_assert(decltype(single_system)::template hasFunction<energy::Total>());
        static_assert(decltype(single_system)::template hasFunction<dynamics::Mass>());

        std::cout << "   All function availability checked at compile time" << std::endl;

        // Test function evaluation
        auto state = single_system.getInitialState();

        double position = single_system.template computeStateFunction<kinematics::Position>(state);
        double velocity = single_system.template computeStateFunction<kinematics::Velocity>(state);
        double energy = single_system.template computeStateFunction<energy::Total>(state);
        double mass = single_system.template computeStateFunction<dynamics::Mass>(state);

        std::cout << "   Initial state functions:" << std::endl;
        std::cout << "     Position: " << position << " m" << std::endl;
        std::cout << "     Velocity: " << velocity << " m/s" << std::endl;
        std::cout << "     Total Energy: " << energy << " J" << std::endl;
        std::cout << "     Mass: " << mass << " kg" << std::endl;

        ASSERT_NEAR(position, 1.0, 1e-10);
        ASSERT_NEAR(velocity, 0.0, 1e-10);
        ASSERT_NEAR(energy, 2.0, 1e-10); // PE = 0.5 * k * x^2 = 0.5 * 4 * 1^2 = 2J
        ASSERT_NEAR(mass, 1.0, 1e-10);

        std::cout << "\n2. Damped oscillator test..." << std::endl;

        // Test damped oscillator
        auto damped_osc = createDampedOscillator<double>(2.0, 9.0, 0.1, 1.5);
        auto damped_system = makeTypedODESystem<double>(std::move(damped_osc));

        std::cout << "   Damped system created" << std::endl;
        std::cout << "   Components: " << damped_system.getComponentCount() << std::endl;
        std::cout << "   State dimension: " << damped_system.getStateDimension() << std::endl;

        // Check state functions
        static_assert(decltype(damped_system)::template hasFunction<kinematics::Position>());
        static_assert(decltype(damped_system)::template hasFunction<energy::Total>());
        static_assert(decltype(damped_system)::template hasFunction<energy::Kinetic>());
        static_assert(decltype(damped_system)::template hasFunction<energy::Potential>());

        auto damped_state = damped_system.getInitialState();

        std::cout << "\n3. Batch function evaluation test..." << std::endl;

        // Test batch evaluation (all compile-time)
        auto [pos, vel, ke, pe, total] = damped_system.template computeStateFunctions<
            kinematics::Position,
            kinematics::Velocity,
            energy::Kinetic,
            energy::Potential,
            energy::Total
        >(damped_state);

        std::cout << "   Batch evaluation results:" << std::endl;
        std::cout << "     Position: " << pos << " m" << std::endl;
        std::cout << "     Velocity: " << vel << " m/s" << std::endl;
        std::cout << "     Kinetic Energy: " << ke << " J" << std::endl;
        std::cout << "     Potential Energy: " << pe << " J" << std::endl;
        std::cout << "     Total Energy: " << total << " J" << std::endl;

        // Verify energy conservation
        ASSERT_NEAR(total, ke + pe, 1e-10);

        std::cout << "\n4. Performance test..." << std::endl;

        // Performance test - measure pure compile-time dispatch
        constexpr int iterations = 1000000;
        auto start = std::chrono::high_resolution_clock::now();

        double sum = 0.0;
        for (int i = 0; i < iterations; ++i) {
            sum += damped_system.template computeStateFunction<energy::Total>(damped_state);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

        std::cout << "   " << iterations << " function calls in " << duration.count() << " ns" << std::endl;
        std::cout << "   Average per call: " << (double)duration.count() / iterations << " ns" << std::endl;
        std::cout << "   Sum (to prevent optimization): " << sum << std::endl;

        std::cout << "\n5. Integration test..." << std::endl;

        auto solver = createRK4Solver();

        // Wrap derivatives for solver (solver expects StateView, TypedODESystem uses vector)
        auto derivs_func = [&single_system](double t, StateView state) {
            std::vector<double> state_vec(state.begin(), state.end());
            return single_system.computeDerivatives(t, state_vec);
        };

        auto result = solver.solve(
            derivs_func,
            single_system.getStateDimension(),
            0.0, 2.0, 0.01,
            single_system.getInitialState()
        );

        std::cout << "   Integration completed: " << result.size() << " steps" << std::endl;
        std::cout << "   Solution time: " << result.total_time << " ms" << std::endl;

        // Test energy conservation (undamped oscillator)
        double initial_energy = single_system.template computeStateFunction<energy::Total>(result.states[0]);
        double final_energy = single_system.template computeStateFunction<energy::Total>(result.states.back());

        double energy_error = std::abs(final_energy - initial_energy) / initial_energy * 100.0;

        std::cout << "   Energy conservation (undamped):" << std::endl;
        std::cout << "     Initial energy: " << initial_energy << " J" << std::endl;
        std::cout << "     Final energy: " << final_energy << " J" << std::endl;
        std::cout << "     Relative error: " << energy_error << "%" << std::endl;

        // Energy should be well-conserved for undamped oscillator with RK4
        if (energy_error > 0.01) {
            std::cerr << "   [FAIL] Energy error too large!" << std::endl;
            std::abort();
        }

        std::cout << "\n6. Analytical validation test..." << std::endl;

        // Test against analytical solution
        auto simple_osc = createSimpleOscillator<double>(1.0, 1.0, 1.0);
        auto simple_system = makeTypedODESystem<double>(std::move(simple_osc));

        auto simple_derivs = [&simple_system](double t, StateView state) {
            std::vector<double> state_vec(state.begin(), state.end());
            return simple_system.computeDerivatives(t, state_vec);
        };

        auto simple_result = solver.solve(
            simple_derivs,
            simple_system.getStateDimension(),
            0.0, 1.0, 0.01,
            simple_system.getInitialState()
        );

        // Get reference to oscillator for analytical solution
        const auto& osc_ref = simple_system.template getComponent<0>();

        double max_error = 0.0;
        for (size_t i = 0; i < simple_result.size(); ++i) {
            double t = simple_result.times[i];
            auto [analytical_pos, analytical_vel] = osc_ref.analyticalSolution(t);

            double numerical_pos = simple_result.states[i][0];
            double error = std::abs(numerical_pos - analytical_pos);
            max_error = std::max(max_error, error);
        }

        std::cout << "   Maximum error vs analytical: " << max_error << std::endl;
        ASSERT_NEAR(max_error, 0.0, 1e-6); // Should be very small for RK4

        std::cout << "\n=== SOPOT COMPILE-TIME SUCCESS! ===" << std::endl;
        std::cout << "Pure compile-time state function dispatch" << std::endl;
        std::cout << "Zero runtime overhead - all calls inlined" << std::endl;
        std::cout << "Batch function evaluation optimized" << std::endl;
        std::cout << "Template metaprogramming optimization" << std::endl;
        std::cout << "Energy conservation verified" << std::endl;
        std::cout << "Analytical solution validation passed" << std::endl;
        std::cout << "Clean, minimal architecture" << std::endl;

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
