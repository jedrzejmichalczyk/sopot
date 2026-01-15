// Placeholder for physics instantiations
// This file forces compilation of commonly used physics headers in a single TU.
// Since most physics components use inline methods, explicit instantiation
// doesn't provide significant benefits. The main optimization comes from
// precompiled headers (PCH).

#include "physics/coupled_oscillator/point_mass.hpp"
#include "physics/coupled_oscillator/spring.hpp"
#include "physics/connected_masses/indexed_point_mass.hpp"
#include "physics/connected_masses/indexed_point_mass_2d.hpp"
#include "physics/connected_masses/indexed_spring.hpp"
#include "physics/connected_masses/indexed_spring_2d.hpp"
#include "physics/connected_masses/force_aggregator_2d.hpp"
#include "core/dual.hpp"

// This compilation unit ensures all physics headers are parsed at least once
// during the build, which can help with incremental builds.
