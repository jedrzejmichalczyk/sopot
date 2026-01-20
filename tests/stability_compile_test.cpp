// Simple compile test for stability.hpp
#include "core/stability.hpp"
#include <iostream>

using namespace sopot;

int main() {
    // Test 1: Compile-time computation
    constexpr auto stability = GridStabilityInfo<3, 3, false>::analyze(10.0, 1.0, 0.5);

    std::cout << "Stability analysis result:\n";
    std::cout << "  Max connectivity: " << stability.max_connectivity << "\n";
    std::cout << "  Effective stiffness: " << stability.effective_stiffness << " N/m\n";
    std::cout << "  Max frequency: " << stability.max_frequency << " rad/s\n";
    std::cout << "  Recommended dt: " << stability.dt_recommended << " s\n";

    // Test 2: Static assertion example
    constexpr double my_dt = 0.05;
    static_assert(my_dt < stability.dt_recommended, "Time step too large!");

    std::cout << "\n✓ Compile-time stability checks work!\n";

    return 0;
}
