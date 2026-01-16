// Explicit template instantiations for SOPOT core types
// This file precompiles common template instantiations to reduce compilation time
// across multiple translation units.
//
// NOTE: Since Dual<T, N> uses constexpr inline methods, explicit instantiation
// provides limited benefits. This file primarily serves to:
// 1. Ensure headers are parsed at least once during the build
// 2. Trigger template instantiation for common types
// 3. Help with incremental builds by creating compilation dependencies

#include "core/dual.hpp"
#include "core/typed_component.hpp"
#include <cmath>

namespace sopot {

// Force instantiation of Dual classes by using them in a non-optimizable way
// This ensures the compiler generates code for these types in this TU

namespace {
    // Helper to force instantiation - marked to prevent optimization removal
    template<typename T, size_t N>
    [[gnu::used, gnu::noinline]]
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
    // Using attribute to prevent dead code elimination
    struct [[gnu::used]] ForceInstantiation {
        ForceInstantiation() {
            forceDualInstantiation<double, 1>();
            forceDualInstantiation<double, 3>();
            forceDualInstantiation<double, 4>();
            forceDualInstantiation<double, 6>();
            forceDualInstantiation<double, 13>();
        }
    };

    // Static instance to trigger instantiation
    // Multiple attributes ensure it's not optimized away
    [[gnu::used, gnu::retain]]
    static ForceInstantiation force_inst;
}

} // namespace sopot
