// Explicit template instantiations for SOPOT core types
// This file precompiles common template instantiations to reduce compilation time
// across multiple translation units.
//
// NOTE: Since Dual<T, N> uses constexpr inline methods, we cannot explicitly
// instantiate individual methods. Instead, we use this compilation unit to
// force instantiation of commonly used templates.

#include "core/dual.hpp"
#include "core/typed_component.hpp"
#include <cmath>

namespace sopot {

// Force instantiation of Dual classes by using them
// This ensures the compiler generates code for these types in this TU

namespace {
    // Helper to force instantiation without optimization removing it
    template<typename T, size_t N>
    void forceDualInstantiation() {
        Dual<T, N> d1(T{1});
        Dual<T, N> d2(T{2});
        volatile auto sum = d1 + d2;
        volatile auto diff = d1 - d2;
        volatile auto prod = d1 * d2;
        volatile auto quot = d1 / d2;
        (void)sum; (void)diff; (void)prod; (void)quot;
    }

    // Force instantiation for common Dual types
    struct ForceInstantiation {
        ForceInstantiation() {
            forceDualInstantiation<double, 1>();
            forceDualInstantiation<double, 3>();
            forceDualInstantiation<double, 4>();
            forceDualInstantiation<double, 6>();
            forceDualInstantiation<double, 13>();
        }
    };

    // Static instance to trigger instantiation
    [[maybe_unused]] static ForceInstantiation force_inst;
}

} // namespace sopot
