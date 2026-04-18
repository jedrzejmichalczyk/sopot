// Milestone 2: 2D planar missile-vs-interceptor engagement.
//
// This test proves three things about the VehicleTags<V> architecture:
//   1. Two vehicles coexist in a single TypedODESystem without tag collisions.
//   2. A guidance component on one vehicle can query the OTHER vehicle's state
//      through the shared registry (cross-vehicle tag access).
//   3. Proportional Navigation converges to intercept against a ballistic target.
//
// Geometry (east-up plane, no gravity, no thrust):
//   Missile:     starts at (0,    0, 1000) m, velocity (400, 0, 0)   m/s
//   Interceptor: starts at (6000, 0,    0) m, velocity (-300, 0, 250) m/s
// The two are on nearly-head-on closing geometry. Without guidance the miss
// distance is O(km); with PN N=3 the interceptor should come within < 20 m.
//
// Only the interceptor has PNGuidance<Interceptor, Missile, T>; the missile
// flies ballistic via NoGuidance. Gravity is zero for both so the scenario
// isolates the guidance law.

#include "../rocket/vector3.hpp"
#include "../rocket/vehicle_tags.hpp"
#include "../rocket/gravity.hpp"
#include "../rocket/planar_engagement.hpp"
#include "../core/typed_component.hpp"
#include "../core/solver.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

using namespace sopot;
using namespace sopot::rocket;
using namespace sopot::rocket::planar;

#define ASSERT_TRUE(cond) \
    do { \
        if (!(cond)) { \
            std::cerr << "ASSERTION FAILED at line " << __LINE__ << ": " << #cond << std::endl; \
            std::abort(); \
        } \
    } while(0)

#define ASSERT_LT(a, b) \
    do { \
        double _a = (a); double _b = (b); \
        if (!(_a < _b)) { \
            std::cerr << "ASSERTION FAILED at line " << __LINE__ << ": " \
                      << #a << " (" << _a << ") < " << #b << " (" << _b << ") FAILED" << std::endl; \
            std::abort(); \
        } \
    } while(0)

using T = double;
using MTags = VehicleTags<Missile>;
using ITags = VehicleTags<Interceptor>;

// Helper: build a system, run it, return (miss_distance, time_of_miss, final_state).
struct EngagementResult {
    double min_miss;
    double t_min;
    size_t n_steps;
    double cpu_ms;
};

template<typename System>
EngagementResult run_engagement(const System& system, double t_end, double dt) {
    auto derivs_func = [&system](double t, StateView state) {
        std::vector<double> state_vec(state.begin(), state.end());
        return system.computeDerivatives(t, state_vec);
    };
    auto solver = createRK4Solver();
    auto result = solver.solve(
        derivs_func,
        system.getStateDimension(),
        0.0, t_end, dt,
        system.getInitialState()
    );

    double min_miss = std::numeric_limits<double>::infinity();
    double t_min = 0.0;
    for (size_t i = 0; i < result.size(); ++i) {
        auto pM = system.template computeStateFunction<typename MTags::PositionENU>(result.states[i]);
        auto pI = system.template computeStateFunction<typename ITags::PositionENU>(result.states[i]);
        double d = (pM - pI).norm();
        if (d < min_miss) { min_miss = d; t_min = result.times[i]; }
    }
    return {min_miss, t_min, result.size(), result.total_time};
}

void test_ballistic_cross_vehicle_queries() {
    std::cout << "\n1. Ballistic baseline: no guidance, verify cross-vehicle tags work..." << std::endl;

    // Missile: level flight east at 400 m/s, altitude 1000 m.
    // Interceptor: starts 6 km east at ground, heading up-west. Ballistic too.
    // Components constructed as rvalues so makeTypedODESystem's concept check
    // sees value types, not references.
    auto system = makeTypedODESystem<T>(
        PointMassKinematics2D<Missile, T>(0.0, 1000.0, "missile_kin"),
        PointMassDynamics2D<Missile, T>(400.0, 0.0, "missile_dyn"),
        ConstantMass<Missile, T>(100.0, "missile_mass"),
        VelocityAlignedThrust2D<Missile, T>(0.0, "missile_thrust"),
        Gravity<Missile, T>(T(0), "missile_gravity"),
        NoGuidance<Missile, T>("missile_no_guidance"),
        PointMassForce2D<Missile, T>("missile_force"),
        PointMassKinematics2D<Interceptor, T>(6000.0, 0.0, "interceptor_kin"),
        PointMassDynamics2D<Interceptor, T>(-300.0, 250.0, "interceptor_dyn"),
        ConstantMass<Interceptor, T>(100.0, "interceptor_mass"),
        VelocityAlignedThrust2D<Interceptor, T>(0.0, "interceptor_thrust"),
        Gravity<Interceptor, T>(T(0), "interceptor_gravity"),
        NoGuidance<Interceptor, T>("interceptor_no_guidance"),  // ballistic baseline
        PointMassForce2D<Interceptor, T>("interceptor_force")
    );

    static_assert(decltype(system)::template hasFunction<MTags::PositionENU>());
    static_assert(decltype(system)::template hasFunction<ITags::PositionENU>());
    static_assert(decltype(system)::template hasFunction<MTags::VelocityENU>());
    static_assert(decltype(system)::template hasFunction<ITags::VelocityENU>());
    static_assert(decltype(system)::template hasFunction<MTags::TotalForceENU>());
    static_assert(decltype(system)::template hasFunction<ITags::TotalForceENU>());
    static_assert(decltype(system)::template hasFunction<MTags::GuidanceCommandENU>());
    static_assert(decltype(system)::template hasFunction<ITags::GuidanceCommandENU>());
    static_assert(decltype(system)::getStateDimension() == 8,
                  "Expected 4 states per vehicle x 2 vehicles");

    auto result = run_engagement(system, 15.0, 0.01);
    std::cout << "   Ballistic min miss: " << std::fixed << std::setprecision(2)
              << result.min_miss << " m at t = " << result.t_min << " s"
              << " (" << result.n_steps << " steps, " << result.cpu_ms << " ms)" << std::endl;

    // With the chosen geometry, ballistic miss should be hundreds of metres.
    ASSERT_TRUE(result.min_miss > 50.0);

    std::cout << "   \u2713 Ballistic baseline passed (cross-vehicle tags resolved at compile time)" << std::endl;
}

void test_pn_intercept() {
    std::cout << "\n2. PN guidance intercept..." << std::endl;

    auto system = makeTypedODESystem<T>(
        PointMassKinematics2D<Missile, T>(0.0, 1000.0, "missile_kin"),
        PointMassDynamics2D<Missile, T>(400.0, 0.0, "missile_dyn"),
        ConstantMass<Missile, T>(100.0, "missile_mass"),
        VelocityAlignedThrust2D<Missile, T>(0.0, "missile_thrust"),
        Gravity<Missile, T>(T(0), "missile_gravity"),
        NoGuidance<Missile, T>("missile_no_guidance"),
        PointMassForce2D<Missile, T>("missile_force"),
        PointMassKinematics2D<Interceptor, T>(6000.0, 0.0, "interceptor_kin"),
        PointMassDynamics2D<Interceptor, T>(-300.0, 250.0, "interceptor_dyn"),
        ConstantMass<Interceptor, T>(100.0, "interceptor_mass"),
        VelocityAlignedThrust2D<Interceptor, T>(0.0, "interceptor_thrust"),
        Gravity<Interceptor, T>(T(0), "interceptor_gravity"),
        PNGuidance<Interceptor, Missile, T>(T(3), "interceptor_pn"),
        PointMassForce2D<Interceptor, T>("interceptor_force")
    );

    auto result = run_engagement(system, 15.0, 0.005);
    std::cout << "   PN (N=3) min miss: " << std::fixed << std::setprecision(3)
              << result.min_miss << " m at t = " << result.t_min << " s"
              << " (" << result.n_steps << " steps, " << result.cpu_ms << " ms)" << std::endl;

    // PN should close to well under 20 m against a non-maneuvering target
    // with modest geometry; a fair numerical target is 10 m.
    ASSERT_LT(result.min_miss, 10.0);
    ASSERT_TRUE(result.t_min > 1.0 && result.t_min < 12.0);

    std::cout << "   \u2713 PN intercept achieved" << std::endl;
}

void test_pn_gain_sensitivity() {
    std::cout << "\n3. PN gain sensitivity..." << std::endl;

    auto build_and_run = [](T gain) -> double {
        auto system = makeTypedODESystem<T>(
            PointMassKinematics2D<Missile, T>(0.0, 1000.0),
            PointMassDynamics2D<Missile, T>(400.0, 0.0),
            ConstantMass<Missile, T>(100.0),
            VelocityAlignedThrust2D<Missile, T>(0.0),
            Gravity<Missile, T>(T(0)),
            NoGuidance<Missile, T>(),
            PointMassForce2D<Missile, T>(),
            PointMassKinematics2D<Interceptor, T>(6000.0, 0.0),
            PointMassDynamics2D<Interceptor, T>(-300.0, 250.0),
            ConstantMass<Interceptor, T>(100.0),
            VelocityAlignedThrust2D<Interceptor, T>(0.0),
            Gravity<Interceptor, T>(T(0)),
            PNGuidance<Interceptor, Missile, T>(gain),
            PointMassForce2D<Interceptor, T>()
        );
        return run_engagement(system, 15.0, 0.005).min_miss;
    };

    std::cout << "   Gain | Min miss [m]" << std::endl;
    std::cout << "   -----|-------------" << std::endl;
    double miss_n2 = build_and_run(2.0);
    double miss_n3 = build_and_run(3.0);
    double miss_n4 = build_and_run(4.0);
    double miss_n5 = build_and_run(5.0);
    std::cout << "    2.0 | " << std::fixed << std::setprecision(3) << miss_n2 << std::endl;
    std::cout << "    3.0 | " << std::fixed << std::setprecision(3) << miss_n3 << std::endl;
    std::cout << "    4.0 | " << std::fixed << std::setprecision(3) << miss_n4 << std::endl;
    std::cout << "    5.0 | " << std::fixed << std::setprecision(3) << miss_n5 << std::endl;

    // All gains in [3, 5] should achieve intercept against a non-maneuvering target.
    ASSERT_LT(miss_n3, 10.0);
    ASSERT_LT(miss_n4, 10.0);
    ASSERT_LT(miss_n5, 10.0);

    std::cout << "   \u2713 PN gain sensitivity passed" << std::endl;
}

int main() {
    std::cout << "=== 2D Planar Engagement Test (Missile vs Interceptor) ===" << std::endl;
    std::cout << "Demonstrates VehicleTags<V> architecture with cross-vehicle PN guidance." << std::endl;

    try {
        test_ballistic_cross_vehicle_queries();
        test_pn_intercept();
        test_pn_gain_sensitivity();

        std::cout << "\n=== ALL ENGAGEMENT TESTS PASSED ===" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
