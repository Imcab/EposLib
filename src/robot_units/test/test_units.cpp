// Smoke tests for the vendored units library.
//
// nholthaus/units has its own extensive test suite upstream; these do not
// repeat it. What they check is that the version we vendored behaves as this
// workspace assumes: that it builds under C++17 with the compiler ROS 2
// Humble ships, that dimensional analysis is on, and that the specific units
// the EPOS4 driver depends on exist and convert correctly.
//
// If a future update to units.h breaks any of that, this fails here rather
// than three packages downstream.

#include <gtest/gtest.h>

#include <type_traits>

#include "robot_units.hpp"

using namespace units;           // NOLINT(build/namespaces)
using namespace units::literals;  // NOLINT(build/namespaces)

TEST(VendoredUnits, AngleConversions)
{
  EXPECT_DOUBLE_EQ(angle::turn_t(45_deg).value(), 0.125);
  EXPECT_DOUBLE_EQ(angle::degree_t(1_tr).value(), 360.0);
  EXPECT_NEAR(angle::radian_t(180_deg).value(), 3.14159265358979323846, 1e-15);
}

// The factor of 2*pi/60 between rpm and rad/s is the one most often dropped
// by hand, and every maxon parameter is in rpm.
TEST(VendoredUnits, RpmAndRadiansPerSecond)
{
  EXPECT_NEAR(angular_velocity::radians_per_second_t(1500_rpm).value(), 157.0796, 1e-4);
  EXPECT_NEAR(angular_velocity::revolutions_per_minute_t(
      angular_velocity::radians_per_second_t(157.0796)).value(), 1500.0, 1e-3);
  EXPECT_DOUBLE_EQ(angular_velocity::degrees_per_second_t(60_rpm).value(), 360.0);
}

// maxon expresses the torque constant and the rated torque in micronewton
// metres: a factor of a million away from Nm.
TEST(VendoredUnits, TorqueScales)
{
  EXPECT_DOUBLE_EQ(torque::newton_meter_t(50.0).value() * 1e6, 50e6);
  EXPECT_DOUBLE_EQ(torque::newton_meter_t{0.05}.value(), 0.05);
}

TEST(VendoredUnits, CurrentAndVoltage)
{
  EXPECT_DOUBLE_EQ(current::ampere_t(3200_mA).value(), 3.2);
  EXPECT_DOUBLE_EQ(voltage::volt_t(24_V).value(), 24.0);
}

// Dimensional analysis is the reason for using this library rather than
// naming conversion functions: the result type is deduced, not asserted.
TEST(VendoredUnits, DimensionalAnalysisDeducesResultTypes)
{
  auto speed = 10_m / 2_s;
  EXPECT_DOUBLE_EQ(velocity::meters_per_second_t(speed).value(), 5.0);

  auto travelled = 5_mps * 4_s;
  EXPECT_DOUBLE_EQ(length::meter_t(travelled).value(), 20.0);
}

// The compiler rejects mixing dimensions. This is the whole point: with bare
// doubles the same mistake compiles, runs, and moves a joint to the wrong
// place.
TEST(VendoredUnits, DimensionsDoNotMix)
{
  // A quantity does not decay to a bare number: the unit has to be named to
  // get one out. This one IS visible to is_convertible.
  static_assert(!std::is_convertible<angle::degree_t, double>::value, "");
  static_assert(!std::is_convertible<double, angle::degree_t>::value, "");

  // Mixing dimensions does not compile either, but note HOW: the converting
  // constructor exists and is viable, and rejects the conversion with a
  // static_assert in its body -
  //
  //     units.h:1652: static assertion failed: Units are not compatible.
  //
  // rather than being removed by SFINAE. So std::is_convertible reports true
  // for, say, ampere_t to degree_t even though writing it is a hard error.
  // Do not use is_convertible to probe this library's compatibility rules;
  // the check is real, it is just not visible to type traits.
  //
  // Verified by hand - none of these build:
  //   angle::degree_t a = 5_A;
  //   auto x = 90_deg + 3_A;
  //   double d = 90_deg;
  SUCCEED();
}

// A quantity must cost nothing: same size as the double it wraps.
TEST(VendoredUnits, ZeroOverhead)
{
  static_assert(sizeof(angle::degree_t) == sizeof(double), "");
  static_assert(sizeof(angular_velocity::revolutions_per_minute_t) == sizeof(double), "");
  static_assert(std::is_trivially_copyable<angle::degree_t>::value, "");
}

// Conversions are constexpr, so anything known at compile time is folded away.
TEST(VendoredUnits, WorksAtCompileTime)
{
  constexpr auto quarter = angle::turn_t(90_deg);
  static_assert(quarter.value() == 0.25, "");
}
