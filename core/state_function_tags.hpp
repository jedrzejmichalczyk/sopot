#pragma once

#include <string_view>
#include <type_traits>

namespace sopot {

// Root tag for all state functions
struct StateFunction {
    static constexpr std::string_view category() { return "base"; }
    static constexpr std::string_view name() { return "unknown"; }
    static constexpr size_t type_id() { return 0; }
};

// Major physics categories
namespace categories {
    struct Kinematics : StateFunction {
        static constexpr std::string_view category() { return "kinematics"; }
    };
    
    struct Dynamics : StateFunction {
        static constexpr std::string_view category() { return "dynamics"; }
    };
    
    struct Energy : StateFunction {
        static constexpr std::string_view category() { return "energy"; }
    };
    
    struct Analysis : StateFunction {
        static constexpr std::string_view category() { return "analysis"; }
    };
}

// Core state function tags
namespace kinematics {
    struct Position : categories::Kinematics {
        static constexpr std::string_view name() { return "position"; }
        static constexpr size_t type_id() { return 1; }
    };
    
    struct Velocity : categories::Kinematics {
        static constexpr std::string_view name() { return "velocity"; }
        static constexpr size_t type_id() { return 2; }
    };
    
    struct Acceleration : categories::Kinematics {
        static constexpr std::string_view name() { return "acceleration"; }
        static constexpr size_t type_id() { return 3; }
    };
}

namespace energy {
    struct Kinetic : categories::Energy {
        static constexpr std::string_view name() { return "kinetic_energy"; }
        static constexpr size_t type_id() { return 101; }
    };
    
    struct Potential : categories::Energy {
        static constexpr std::string_view name() { return "potential_energy"; }
        static constexpr size_t type_id() { return 102; }
    };
    
    struct Total : categories::Energy {
        static constexpr std::string_view name() { return "total_energy"; }
        static constexpr size_t type_id() { return 103; }
    };
}

namespace dynamics {
    struct Force : categories::Dynamics {
        static constexpr std::string_view name() { return "force"; }
        static constexpr size_t type_id() { return 201; }
    };
    
    struct Mass : categories::Dynamics {
        static constexpr std::string_view name() { return "mass"; }
        static constexpr size_t type_id() { return 202; }
    };
}

// C++20 concept for state function tags
template<typename T>
concept StateTagConcept = std::is_base_of_v<StateFunction, T> && requires {
    T::name();
    T::category();
    T::type_id();
} && requires(T t) {
    { T::name() } -> std::convertible_to<std::string_view>;
    { T::category() } -> std::convertible_to<std::string_view>;
    { T::type_id() } -> std::convertible_to<size_t>;
};

} // namespace sopot