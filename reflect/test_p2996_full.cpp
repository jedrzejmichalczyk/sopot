// SOPOT Reflection-Based Simulation Framework Prototype
// Uses C++26 Reflection (P2996) via Bloomberg Clang
//
// Compile:
//   clang++ -std=c++26 -freflection-latest -stdlib=libc++ test_p2996_full.cpp -o test

#include <experimental/meta>
#include <iostream>
#include <vector>
#include <cmath>

//=============================================================================
// USER CODE - Just a plain struct with physics!
// No templates, no inheritance, no macros.
//=============================================================================

struct TwoMassSpring {
    // STATE VARIABLES
    double x1 = 0.0;    // Position of mass 1
    double v1 = 0.0;    // Velocity of mass 1
    double x2 = 1.0;    // Position of mass 2
    double v2 = 0.0;    // Velocity of mass 2

    // PARAMETERS
    double m1 = 1.0;    // Mass 1
    double m2 = 1.0;    // Mass 2
    double k = 10.0;    // Spring stiffness
    double L0 = 1.0;    // Rest length

    // DERIVATIVES
    double x1_dot() const { return v1; }
    double v1_dot() const { return -k * (x1 - x2 + L0) / m1; }
    double x2_dot() const { return v2; }
    double v2_dot() const { return k * (x1 - x2 + L0) / m2; }

    // COMPUTED
    double total_energy() const {
        double KE = 0.5*m1*v1*v1 + 0.5*m2*v2*v2;
        double ext = x2 - x1 - L0;
        double PE = 0.5*k*ext*ext;
        return KE + PE;
    }
};

//=============================================================================
// FRAMEWORK CODE
//=============================================================================

int main() {
    std::cout << "=== SOPOT Reflection-Based Framework ===\n\n";

    constexpr auto ctx = std::meta::access_context::current();

    //=========================================================================
    // 1. List all members via reflection
    //=========================================================================
    std::cout << "1. Reflecting on TwoMassSpring members:\n";

    template for (constexpr auto member :
                  std::define_static_array(
                      std::meta::nonstatic_data_members_of(^^TwoMassSpring, ctx))) {
        std::cout << "   " << std::meta::identifier_of(member)
                  << " : " << std::meta::display_string_of(std::meta::type_of(member))
                  << "\n";
    }

    //=========================================================================
    // 2. Create instance and show values via reflection
    //=========================================================================
    std::cout << "\n2. Instance with stretched spring:\n";

    TwoMassSpring sys;
    sys.x2 = 1.5;  // Stretch

    template for (constexpr auto member :
                  std::define_static_array(
                      std::meta::nonstatic_data_members_of(^^TwoMassSpring, ctx))) {
        std::cout << "   " << std::meta::identifier_of(member)
                  << " = " << sys.[:member:]
                  << "\n";
    }

    std::cout << "   Initial energy = " << sys.total_energy() << " J\n";

    //=========================================================================
    // 3. Simulate
    //=========================================================================
    std::cout << "\n3. Euler simulation (10 seconds, dt=0.001):\n";

    double dt = 0.001;
    for (int i = 0; i < 10000; i++) {
        double dx1 = sys.x1_dot();
        double dv1 = sys.v1_dot();
        double dx2 = sys.x2_dot();
        double dv2 = sys.v2_dot();

        sys.x1 += dx1 * dt;
        sys.v1 += dv1 * dt;
        sys.x2 += dx2 * dt;
        sys.v2 += dv2 * dt;
    }

    //=========================================================================
    // 4. Show final state via reflection
    //=========================================================================
    std::cout << "\n4. Final state (via reflection):\n";

    template for (constexpr auto member :
                  std::define_static_array(
                      std::meta::nonstatic_data_members_of(^^TwoMassSpring, ctx))) {
        std::cout << "   " << std::meta::identifier_of(member)
                  << " = " << sys.[:member:]
                  << "\n";
    }

    std::cout << "   Final energy = " << sys.total_energy() << " J\n";

    //=========================================================================
    // 5. Modify via reflection
    //=========================================================================
    std::cout << "\n5. Reset state via reflection:\n";

    // Reset first 4 members to 0 using reflection
    size_t idx = 0;
    template for (constexpr auto member :
                  std::define_static_array(
                      std::meta::nonstatic_data_members_of(^^TwoMassSpring, ctx))) {
        if (idx < 4) {  // Only state variables
            sys.[:member:] = 0.0;
        }
        idx++;
    }

    std::cout << "   After reset:\n";
    template for (constexpr auto member :
                  std::define_static_array(
                      std::meta::nonstatic_data_members_of(^^TwoMassSpring, ctx))) {
        std::cout << "   " << std::meta::identifier_of(member)
                  << " = " << sys.[:member:]
                  << "\n";
    }

    //=========================================================================
    // Summary
    //=========================================================================
    std::cout << "\n=== WHAT REFLECTION ENABLES ===\n";
    std::cout << "  - User writes plain struct (no TypedComponent<N,T>)\n";
    std::cout << "  - User writes plain methods (no registry.computeFunction<Tag>)\n";
    std::cout << "  - Framework reads member names, types, values at compile time\n";
    std::cout << "  - Framework modifies members by name at runtime\n";
    std::cout << "  - ZERO boilerplate, ZERO templates for the user!\n";
    std::cout << "\n=== SUCCESS ===\n";

    return 0;
}
