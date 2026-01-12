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
    constexpr ValueType get() const { return value; }
};

// ============================================================================
// SIMPLIFIED FIELD BUNDLE
// ============================================================================

template<typename... Fields>
struct FieldBundle {
    std::tuple<Fields...> fields;

    constexpr FieldBundle() : fields() {}

    // Constructor from individual fields
    constexpr explicit FieldBundle(Fields... fs) : fields(fs...) {}

    // Access by index
    template<size_t I>
    constexpr auto& get() {
        return std::get<I>(fields);
    }

    template<size_t I>
    constexpr const auto& get() const {
        return std::get<I>(fields);
    }

    static constexpr size_t size() { return sizeof...(Fields); }
};

// ============================================================================
// COMPONENT CONCEPTS
// ============================================================================

template<typename T>
concept Component = requires {
    { T::StateSize } -> std::convertible_to<size_t>;
};

template<typename T>
concept HasDependencies = Component<T> && requires {
    typename T::Dependencies;
};

template<typename T>
concept HasProvisions = Component<T> && requires {
    typename T::Provides;
};

}  // namespace sopot::experimental
