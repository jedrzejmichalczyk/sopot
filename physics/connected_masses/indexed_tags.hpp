#pragma once

#include "tags.hpp"
#include <string>
#include <string_view>

namespace sopot::connected_masses {

// Helper to create compile-time string for index
// Note: These create runtime strings, but that's acceptable for tag metadata
// The actual type identity is based on template parameters, not strings
template<size_t Index>
struct IndexedName {
    static std::string position() { return "Mass" + std::to_string(Index) + "::Position"; }
    static std::string velocity() { return "Mass" + std::to_string(Index) + "::Velocity"; }
    static std::string force() { return "Mass" + std::to_string(Index) + "::Force"; }
    static std::string mass() { return "Mass" + std::to_string(Index) + "::Mass"; }

    static std::string pos_symbol() { return "x" + std::to_string(Index); }
    static std::string vel_symbol() { return "v" + std::to_string(Index); }
    static std::string force_symbol() { return "F" + std::to_string(Index); }
    static std::string mass_symbol() { return "m" + std::to_string(Index); }

    static std::string mass_desc() { return "Mass " + std::to_string(Index); }
    static std::string pos_desc() { return "Position of mass " + std::to_string(Index); }
    static std::string vel_desc() { return "Velocity of mass " + std::to_string(Index); }
    static std::string force_desc() { return "Net force on mass " + std::to_string(Index); }
};

// Generate unique tag types for each mass index
// Each mass index gets its own complete namespace of tags
template<size_t Index>
struct MassTag {
    static constexpr size_t index = Index;

    struct Position : categories::Kinematics {
        static std::string name() { return IndexedName<Index>::position(); }
        static std::string symbol() { return IndexedName<Index>::pos_symbol(); }
        static std::string description() { return IndexedName<Index>::pos_desc(); }
    };

    struct Velocity : categories::Kinematics {
        static std::string name() { return IndexedName<Index>::velocity(); }
        static std::string symbol() { return IndexedName<Index>::vel_symbol(); }
        static std::string description() { return IndexedName<Index>::vel_desc(); }
    };

    struct Force : categories::Dynamics {
        static std::string name() { return IndexedName<Index>::force(); }
        static std::string symbol() { return IndexedName<Index>::force_symbol(); }
        static std::string description() { return IndexedName<Index>::force_desc(); }
    };

    struct Mass : categories::Dynamics {
        static std::string name() { return IndexedName<Index>::mass(); }
        static std::string symbol() { return IndexedName<Index>::mass_symbol(); }
        static std::string description() { return IndexedName<Index>::mass_desc(); }
    };
};

// Generate unique tag types for spring connections between two masses
template<size_t Index1, size_t Index2>
struct SpringTag {
    static constexpr size_t index1 = Index1;
    static constexpr size_t index2 = Index2;

    struct Extension : categories::ConnectionProperties {
        static std::string name() {
            return "Spring" + std::to_string(Index1) + std::to_string(Index2) + "::Extension";
        }
        static std::string symbol() {
            return "dx_" + std::to_string(Index1) + std::to_string(Index2);
        }
        static std::string description() {
            return "Extension of spring between mass " + std::to_string(Index1) +
                   " and mass " + std::to_string(Index2);
        }
    };

    struct PotentialEnergy : categories::ConnectionProperties {
        static std::string name() {
            return "Spring" + std::to_string(Index1) + std::to_string(Index2) + "::PotentialEnergy";
        }
        static std::string symbol() {
            return "PE_" + std::to_string(Index1) + std::to_string(Index2);
        }
        static std::string description() {
            return "Potential energy in spring between mass " + std::to_string(Index1) +
                   " and mass " + std::to_string(Index2);
        }
    };
};

} // namespace sopot::connected_masses
