// Placeholder for rocket instantiations
// This file forces compilation of commonly used rocket headers in a single TU.
// Since most rocket components use inline methods, explicit instantiation
// doesn't provide significant benefits. The main optimization comes from
// precompiled headers (PCH).

#include "rocket/vector3.hpp"
#include "rocket/quaternion.hpp"
#include "rocket/translation_kinematics.hpp"
#include "rocket/translation_dynamics.hpp"
#include "rocket/rotation_kinematics.hpp"
#include "rocket/rotation_dynamics.hpp"
#include "rocket/gravity.hpp"
#include "rocket/standard_atmosphere.hpp"
#include "rocket/rocket_body.hpp"
#include "rocket/interpolated_engine.hpp"
#include "rocket/axisymmetric_aero.hpp"
#include "rocket/force_aggregator.hpp"
#include "rocket/simulation_time.hpp"
#include "core/dual.hpp"

// This compilation unit ensures all rocket headers are parsed at least once
// during the build, which can help with incremental builds.
