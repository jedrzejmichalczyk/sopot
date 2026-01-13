#pragma once

#include <string_view>
#include <tuple>
#include <type_traits>
#include <concepts>

namespace sopot::experimental {

// ============================================================================
// FIXED STRING - For template non-type parameters
// ============================================================================

template<size_t N>
struct FixedString {
    char data[N];
    static constexpr size_t size = N;

    constexpr FixedString(const char (&str)[N]) {
        for (size_t i = 0; i < N; ++i) {
            data[i] = str[i];
        }
    }

    constexpr bool operator==(const FixedString<N>& other) const {
        for (size_t i = 0; i < N; ++i) {
            if (data[i] != other.data[i]) return false;
        }
        return true;
    }

    constexpr std::string_view view() const {
        return std::string_view(data, N - 1);  // Exclude null terminator
    }
};

// ============================================================================
// FIELD - Type + Name identifier
// ============================================================================

template<typename TagType, FixedString Name>
struct Field {
    using Tag = TagType;
    static constexpr auto name = Name;

    using ValueType = typename TagType::ValueType;
    ValueType value;

    // Implicit conversion for ergonomic access
    constexpr operator ValueType() const { return value; }
    constexpr ValueType operator()() const { return value; }
};

// Concept: Is this a Field?
template<typename T>
concept IsField = requires {
    typename T::Tag;
    { T::name } -> std::convertible_to<std::string_view>;
    typename T::ValueType;
};

// Field matching: same tag type and name
template<typename F1, typename F2>
concept MatchingFields = IsField<F1> && IsField<F2> &&
    std::same_as<typename F1::Tag, typename F2::Tag> &&
    (F1::name == F2::name);

// ============================================================================
// FIELD BUNDLE - Collection of fields
// ============================================================================

template<typename... Fields>
struct FieldBundle {
    std::tuple<Fields...> fields;

    constexpr FieldBundle() = default;
    constexpr FieldBundle(Fields... fs) : fields(fs...) {}

    // Access field by index
    template<size_t I>
    constexpr auto& get() {
        return std::get<I>(fields);
    }

    template<size_t I>
    constexpr const auto& get() const {
        return std::get<I>(fields);
    }

    // Access field by type
    template<typename Field>
    constexpr auto& get() {
        return std::get<Field>(fields);
    }

    template<typename Field>
    constexpr const auto& get() const {
        return std::get<Field>(fields);
    }

    // Find field by tag and name
    template<typename Tag, FixedString Name>
    constexpr auto& find() {
        return findHelper<Tag, Name>(std::make_index_sequence<sizeof...(Fields)>{});
    }

    template<typename Tag, FixedString Name>
    constexpr const auto& find() const {
        return findHelper<Tag, Name>(std::make_index_sequence<sizeof...(Fields)>{});
    }

    static constexpr size_t size() { return sizeof...(Fields); }

private:
    template<typename Tag, FixedString Name, size_t... Is>
    constexpr auto& findHelper(std::index_sequence<Is...>) {
        // Find first field with matching tag and name
        auto finder = [&]<size_t I>() -> auto& {
            using FieldType = std::tuple_element_t<I, decltype(fields)>;
            if constexpr (std::same_as<typename FieldType::Tag, Tag> &&
                         (FieldType::name == Name)) {
                return std::get<I>(fields);
            } else {
                if constexpr (I + 1 < sizeof...(Fields)) {
                    return finder.template operator()<I + 1>();
                } else {
                    // Use a dependent false to trigger compile error with better message
                    []<bool flag = false>() {
                        static_assert(flag, "Field not found in bundle - check Tag and Name");
                    }();
                }
            }
        };
        return finder.template operator()<0>();
    }

    template<typename Tag, FixedString Name, size_t... Is>
    constexpr const auto& findHelper(std::index_sequence<Is...>) const {
        // Same as above but const
        auto finder = [&]<size_t I>() -> const auto& {
            using FieldType = std::tuple_element_t<I, decltype(fields)>;
            if constexpr (std::same_as<typename FieldType::Tag, Tag> &&
                         (FieldType::name == Name)) {
                return std::get<I>(fields);
            } else {
                if constexpr (I + 1 < sizeof...(Fields)) {
                    return finder.template operator()<I + 1>();
                } else {
                    // Use a dependent false to trigger compile error with better message
                    []<bool flag = false>() {
                        static_assert(flag, "Field not found in bundle - check Tag and Name");
                    }();
                }
            }
        };
        return finder.template operator()<0>();
    }
};

// Deduction guide
template<typename... Fields>
FieldBundle(Fields...) -> FieldBundle<Fields...>;

// ============================================================================
// COMPONENT INTERFACE
// ============================================================================

template<typename T>
concept Component = requires(T comp) {
    // Component can declare dependencies (optional)
    // typename T::Dependencies;

    // Component can declare provisions (optional)
    // typename T::Provides;

    // Component has state size (can be 0)
    { T::StateSize } -> std::convertible_to<size_t>;

    // Component can compute given dependencies
    // { comp.compute(time, dependencies) } -> ...;
};

// Check if component has dependencies
template<typename T>
concept HasDependencies = Component<T> && requires {
    typename T::Dependencies;
};

// Check if component has provisions
template<typename T>
concept HasProvisions = Component<T> && requires {
    typename T::Provides;
};

// ============================================================================
// DEPENDENCY ANALYSIS
// ============================================================================

// Extract all dependency fields from a component list
template<typename... Components>
struct AllDependencies {
    // Concatenate all Dependencies bundles
    // TODO: Implement type-level concatenation
};

// Extract all provision fields from a component list
template<typename... Components>
struct AllProvisions {
    // Concatenate all Provides bundles
    // TODO: Implement type-level concatenation
};

// Check if all dependencies are satisfied
template<typename Dependencies, typename Provisions>
struct DependenciesSatisfied {
    // For each field in Dependencies, check if exists in Provisions
    static constexpr bool value = true;  // TODO: Implement
};

// ============================================================================
// EXAMPLE USAGE (in comments for now)
// ============================================================================

/*

// Define some tag types
struct TemperatureTag { using ValueType = double; };
struct PressureTag { using ValueType = double; };
struct DensityTag { using ValueType = double; };

// Define a component
template<typename T = double>
class IdealGasLaw {
public:
    static constexpr size_t StateSize = 0;  // Stateless

    // Dependencies
    using Dependencies = FieldBundle<
        Field<TemperatureTag, "temperature">,
        Field<PressureTag, "pressure">
    >;

    // Provisions
    using Provides = FieldBundle<
        Field<DensityTag, "density">
    >;

    // Compute
    template<typename Deps>
    auto compute(T t, const Deps& deps) const {
        T temp = deps.template find<TemperatureTag, "temperature">();
        T pres = deps.template find<PressureTag, "pressure">();

        T rho = pres / (R * temp);

        return Provides{Field<DensityTag, "density">{.value = rho}};
    }

private:
    static constexpr T R = 287.05;  // Gas constant for air
};

*/

}  // namespace sopot::experimental
