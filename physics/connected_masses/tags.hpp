#pragma once

#include "core/state_function_tags.hpp"

namespace sopot::connected_masses {

// Tag categories for connected mass systems
namespace categories {
    struct Kinematics : sopot::categories::Kinematics {};
    struct Dynamics : sopot::categories::Dynamics {};
    struct ConnectionProperties : sopot::categories::Analysis {};
}

} // namespace sopot::connected_masses
