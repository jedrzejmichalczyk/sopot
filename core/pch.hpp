#pragma once

// Precompiled Header for SOPOT Framework
// This file includes the most commonly used and compile-time expensive headers
// to reduce overall compilation time.

// Standard library headers
#include <array>
#include <cmath>
#include <concepts>
#include <iostream>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

// Core SOPOT framework headers (most expensive to compile)
#include "core/scalar.hpp"
#include "core/dual.hpp"
#include "core/typed_component.hpp"
#include "core/solver.hpp"
#include "core/state_function_tags.hpp"
