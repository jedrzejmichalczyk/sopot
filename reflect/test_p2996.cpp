// Test C++26 Reflection with Bloomberg Clang P2996
// Compile: clang++ -std=c++26 -freflection-latest -stdlib=libc++ test_p2996.cpp -o test_p2996

#include <experimental/meta>
#include <iostream>

//=============================================================================
// USER CODE - Plain struct with physics
//=============================================================================

struct TwoMassSpring {
    double x1 = 0.0;
    double v1 = 0.0;
    double x2 = 1.0;
    double v2 = 0.0;
    double m1 = 1.0;
    double m2 = 1.0;
    double k = 10.0;

    double x1_dot() const { return v1; }
    double v1_dot() const { return -k * (x1 - x2 + 1.0) / m1; }
    double x2_dot() const { return v2; }
    double v2_dot() const { return k * (x1 - x2 + 1.0) / m2; }
};

//=============================================================================
// FRAMEWORK - Use reflection to introspect
//=============================================================================

int main() {
    std::cout << "=== C++26 Reflection Test (P2996) ===\n\n";

    TwoMassSpring sys;
    sys.x2 = 1.5;

    std::cout << "Members of TwoMassSpring:\n";

    // Use define_static_array to avoid heap allocation issues
    template for (constexpr std::meta::info member :
                  std::define_static_array(
                      std::meta::nonstatic_data_members_of(
                          ^^TwoMassSpring,
                          std::meta::access_context::current()))) {
        std::cout << "  " << std::meta::identifier_of(member)
                  << " = " << sys.[:member:]
                  << "\n";
    }

    std::cout << "\n=== Reflection works! ===\n";
    return 0;
}
