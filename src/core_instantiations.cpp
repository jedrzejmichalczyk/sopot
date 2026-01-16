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

// Compiler-specific attributes to prevent dead code elimination
#if defined(__GNUC__) || defined(__clang__)
    #define SOPOT_USED [[gnu::used]]
    #define SOPOT_NOINLINE [[gnu::noinline]]
    #define SOPOT_RETAIN [[gnu::retain]]
#elif defined(_MSC_VER)
    #define SOPOT_USED
    #define SOPOT_NOINLINE __declspec(noinline)
    #define SOPOT_RETAIN
    // MSVC: Use #pragma to prevent inlining and ensure symbols are kept
    #pragma optimize("", off)
#else
    #define SOPOT_USED
    #define SOPOT_NOINLINE
    #define SOPOT_RETAIN
#endif

namespace sopot {

// Force instantiation of Dual classes by using them in a non-optimizable way
// This ensures the compiler generates code for these types in this TU

namespace {
    // Helper to force instantiation - marked to prevent optimization removal
    template<typename T, size_t N>
    SOPOT_USED SOPOT_NOINLINE
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
    // Using attributes to prevent dead code elimination
    struct SOPOT_USED ForceInstantiation {
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
    SOPOT_USED SOPOT_RETAIN
    static ForceInstantiation force_inst;
}

} // namespace sopot

#if defined(_MSC_VER)
    #pragma optimize("", on)
#endif

// Undefine macros to avoid polluting global namespace
#undef SOPOT_USED
#undef SOPOT_NOINLINE
#undef SOPOT_RETAIN
