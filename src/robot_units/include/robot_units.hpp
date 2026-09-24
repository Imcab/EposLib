#pragma once

// ---------------------------------------------------------------------------
// robot_units - physical quantities for robot code.
//
// This package does not implement a units library. It vendors
// nholthaus/units v2.3.5, the same one WPILib's C++ bindings use, and makes
// it available to everything else on the robot:
//
//   #include <units.h>                 // or this header, which pulls it in
//   using namespace units::literals;
//
//   auto goal  = 45_deg;
//   auto speed = 1500_rpm;
//
//   units::angle::turn_t turns = goal;              // implicit, checked
//   double rad = units::angle::radian_t(goal).value();
//
//   auto v = 10_m / 2_s;   // deduced as meters_per_second_t
//
// Why vendored rather than fetched at build time: a rover that has to build
// in the field should not need the network, and pinning the header means
// every machine compiles the same code.
//
// Upstream: https://github.com/nholthaus/units  (MIT, see LICENSE-units.txt)
// ---------------------------------------------------------------------------

#include <units.h>
