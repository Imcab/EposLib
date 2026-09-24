// Tests for the conversion between the drive's raw integers and physical
// quantities.
//
// This is where factors get dropped: the four between encoder pulses and
// quadcounts, the 2*pi/60 between rpm and rad/s, and the gear ratio the drive
// knows nothing about. None of them fail loudly - they just move the joint to
// the wrong place.

#include <gtest/gtest.h>

#include <ros2units/units.h>

#include "epos4/controls/ControlRequests.hpp"
#include "epos4/core/Setpoint.hpp"
#include "epos4/core/UnitConversion.hpp"

using namespace epos4;            // NOLINT(build/namespaces)
using namespace units;            // NOLINT(build/namespaces)
using namespace units::literals;  // NOLINT(build/namespaces)

// A 500 CPR encoder: 500 pulses, 2000 quadcounts, direct drive.
constexpr MechanismScale kDirect{2000};

TEST(MechanismScale, QuadCountsToAngle)
{
  EXPECT_DOUBLE_EQ(kDirect.ToAngle(2000).value(), 1.0);            // turns
  EXPECT_DOUBLE_EQ(angle::degree_t(kDirect.ToAngle(1000)).value(), 180.0);
  EXPECT_DOUBLE_EQ(angle::degree_t(kDirect.ToAngle(500)).value(), 90.0);
  EXPECT_DOUBLE_EQ(kDirect.ToAngle(-2000).value(), -1.0);
}

TEST(MechanismScale, AngleToQuadCountsRoundTrips)
{
  EXPECT_EQ(kDirect.ToQuadCounts(1_tr), 2000);
  EXPECT_EQ(kDirect.ToQuadCounts(90_deg), 500);
  EXPECT_EQ(kDirect.ToQuadCounts(-180_deg), -1000);

  for (std::int32_t counts : {0, 137, -4021, 2000, 100000}) {
    EXPECT_EQ(kDirect.ToQuadCounts(kDirect.ToAngle(counts)), counts) << counts;
  }
}

// The factor of four. An encoder sold as "500 CPR" configured as 500 gives
// four times every angle, and says nothing about it.
TEST(MechanismScale, PulsesAreNotQuadCounts)
{
  constexpr MechanismScale wrong{500};
  constexpr MechanismScale right{2000};

  EXPECT_DOUBLE_EQ(right.ToAngle(2000).value(), 1.0);
  EXPECT_DOUBLE_EQ(wrong.ToAngle(2000).value(), 4.0)
    << "using pulses as the resolution multiplies every angle by four";
}

// The gear ratio is the driver's business: the drive reports motor-side
// counts and has no idea what is bolted to the shaft.
TEST(MechanismScale, GearRatioMovesTheAngleToTheOutputShaft)
{
  constexpr MechanismScale geared{2000, 1.0 / 100.0};  // 1:100 reduction

  EXPECT_DOUBLE_EQ(geared.ToAngle(2000).value(), 0.01);
  EXPECT_DOUBLE_EQ(angle::degree_t(geared.ToAngle(200000)).value(), 360.0);

  EXPECT_EQ(geared.ToQuadCounts(1_tr), 200000);
  EXPECT_EQ(geared.ToQuadCounts(90_deg), 50000);
}

TEST(MechanismScale, VelocityIsRpmAtTheMotorAndScaledByTheGear)
{
  EXPECT_DOUBLE_EQ(kDirect.ToAngularVelocity(1500).value(), 1500.0);
  EXPECT_NEAR(
    angular_velocity::radians_per_second_t(kDirect.ToAngularVelocity(1500)).value(),
    157.0796, 1e-4);
  EXPECT_EQ(kDirect.ToRpm(1500_rpm), 1500);

  constexpr MechanismScale geared{2000, 1.0 / 100.0};
  EXPECT_DOUBLE_EQ(geared.ToAngularVelocity(6000).value(), 60.0)
    << "6000 rpm at the motor is 60 rpm at a 1:100 output";
  EXPECT_EQ(geared.ToRpm(60_rpm), 6000);
}

// Rounding, not truncation. 60 rpm through a 1:100 gear arrives as
// 5999.9999..., and truncating would return 5999 - losing a unit on every
// conversion, which on position is a count of drift per command.
TEST(MechanismScale, RoundsToNearestInsteadOfTruncating)
{
  constexpr MechanismScale geared{2000, 1.0 / 100.0};
  EXPECT_EQ(geared.ToRpm(60_rpm), 6000) << "truncation would give 5999";

  auto angle = kDirect.ToAngle(1234);
  for (int i = 0; i < 10; ++i) {
    const std::int32_t counts = kDirect.ToQuadCounts(angle);
    EXPECT_EQ(counts, 1234) << "drifted on pass " << i;
    angle = kDirect.ToAngle(counts);
  }
  EXPECT_EQ(kDirect.ToQuadCounts(kDirect.ToAngle(-1234)), -1234);
}

// 0x60AA has one legal value, rpm/s, and nholthaus has no such unit. Kept as
// a plain number with the unit named rather than inventing one.
TEST(MechanismScale, AccelerationIsRpmPerSecond)
{
  EXPECT_DOUBLE_EQ(kDirect.ToRpmPerSecondAtOutput(10000), 10000.0);
  EXPECT_EQ(kDirect.FromRpmPerSecondAtOutput(10000.0), 10000u);

  constexpr MechanismScale geared{2000, 1.0 / 100.0};
  EXPECT_DOUBLE_EQ(geared.ToRpmPerSecondAtOutput(10000), 100.0);
}

// An unconfigured scale must not pretend: dividing by a resolution of zero
// would produce infinities that propagate into a position command.
TEST(MechanismScale, IsInvalidUntilTheMechanismIsKnown)
{
  constexpr MechanismScale none;
  EXPECT_FALSE(none.IsValid());
  EXPECT_TRUE(kDirect.IsValid());
  EXPECT_FALSE(MechanismScale(2000, 0.0).IsValid());
}

// --- torque ----------------------------------------------------------------

// 0x6071 and 0x6077 are in thousandths of «Motor rated torque» (0x6076),
// which maxon reports in micronewton metres.
TEST(TorqueConversion, PerThousandOfTheRatedTorque)
{
  constexpr std::uint32_t kRated = 100000;  // uNm, i.e. 0.1 Nm

  EXPECT_DOUBLE_EQ(ToTorque(1000, kRated).value(), 0.1);
  EXPECT_DOUBLE_EQ(ToTorque(500, kRated).value(), 0.05);
  EXPECT_DOUBLE_EQ(ToTorque(-250, kRated).value(), -0.025);

  EXPECT_EQ(ToPerThousand(torque::newton_meter_t{0.1}, kRated), 1000);
  EXPECT_EQ(ToPerThousand(torque::newton_meter_t{0.05}, kRated), 500);
  EXPECT_EQ(ToPerThousand(torque::newton_meter_t{-0.025}, kRated), -250);
}

// Rated torque is zero until the motor data has been configured. Returning a
// plausible number would command torque against an unknown scale.
TEST(TorqueConversion, ZeroRatedTorqueReturnsZero)
{
  EXPECT_EQ(ToPerThousand(torque::newton_meter_t{0.05}, 0), 0);
}

TEST(CurrentConversion, DriveReportsMilliamps)
{
  EXPECT_DOUBLE_EQ(ToCurrent(3200).value(), 3.2);
  EXPECT_EQ(ToMilliamps(current::ampere_t{3.2}), 3200);
  EXPECT_EQ(ToMilliamps(3200_mA), 3200);
}

// --- compile time ----------------------------------------------------------

TEST(MechanismScale, ConversionsAreConstexpr)
{
  constexpr MechanismScale scale{2000, 1.0 / 100.0};
  static_assert(scale.IsValid(), "");
  static_assert(scale.GetQuadCountsPerRevolution() == 2000, "");
  static_assert(scale.ToAngle(200000).value() == 1.0, "");
}

// ---------------------------------------------------------------------------
// Setpoints: raw drive units or physical quantities, same call
// ---------------------------------------------------------------------------

TEST(Setpoint, AcceptsRawCountsAndPassesThemThrough)
{
  constexpr MechanismScale scale{2000, 1.0 / 100.0};

  PositionSetpoint raw = 50000;
  EXPECT_TRUE(raw.IsRaw());

  std::int32_t counts{};
  ASSERT_TRUE(Resolve(raw, scale, counts));
  EXPECT_EQ(counts, 50000) << "a raw setpoint must not be rescaled";
}

TEST(Setpoint, AcceptsAnglesAndResolvesThemThroughTheMechanism)
{
  constexpr MechanismScale scale{2000, 1.0 / 100.0};  // 500 CPR, 1:100

  PositionSetpoint angle = units::angle::degree_t{90.0};
  EXPECT_FALSE(angle.IsRaw());

  std::int32_t counts{};
  ASSERT_TRUE(Resolve(angle, scale, counts));
  EXPECT_EQ(counts, 50000) << "90 deg at a 1:100 output is 50000 motor counts";
}

// A quantity has no defined answer without a resolution. Resolving it against
// an unconfigured mechanism must fail rather than produce a number.
TEST(Setpoint, AQuantityIsRejectedWhenTheMechanismIsUnknown)
{
  constexpr MechanismScale none;

  PositionSetpoint angle = units::angle::degree_t{90.0};
  std::int32_t counts = 12345;
  EXPECT_FALSE(Resolve(angle, none, counts));
  EXPECT_EQ(counts, 12345) << "the output must be left alone on failure";

  // Raw setpoints still work: they need no scale.
  PositionSetpoint raw = 50000;
  EXPECT_TRUE(Resolve(raw, none, counts));
  EXPECT_EQ(counts, 50000);
}

TEST(Setpoint, VelocityResolvesAtTheOutputShaft)
{
  constexpr MechanismScale scale{2000, 1.0 / 100.0};

  VelocitySetpoint fromUnits = 60_rpm;
  std::int32_t rpm{};
  ASSERT_TRUE(Resolve(fromUnits, scale, rpm));
  EXPECT_EQ(rpm, 6000) << "60 rpm at a 1:100 output is 6000 rpm at the motor";

  VelocitySetpoint fromRaw = 6000;
  ASSERT_TRUE(Resolve(fromRaw, scale, rpm));
  EXPECT_EQ(rpm, 6000) << "raw values are already motor-side";
}

// Profile velocity is a magnitude: direction comes from the target position,
// not from the sign of the speed. A negative speed would be rejected by the
// drive, so it is made positive here.
TEST(Setpoint, ProfileSpeedIsAMagnitude)
{
  constexpr MechanismScale scale{2000};

  SpeedSetpoint negative = units::angular_velocity::revolutions_per_minute_t{-1500.0};
  std::uint32_t rpm{};
  ASSERT_TRUE(Resolve(negative, scale, rpm));
  EXPECT_EQ(rpm, 1500u);
}

// Both forms build the same request, which is the point of the dual setpoint:
// existing code written in counts keeps working while new code uses units.
TEST(Setpoint, BothFormsProduceTheSameCommand)
{
  constexpr MechanismScale scale{2000, 1.0 / 100.0};

  auto byCounts = controls::ProfilePosition{}.WithPosition(50000).WithVelocity(6000u);
  auto byUnits = controls::ProfilePosition{}
  .WithPosition(units::angle::degree_t{90.0})
  .WithVelocity(60_rpm);

  std::int32_t countsA{}, countsB{};
  ASSERT_TRUE(Resolve(byCounts.position, scale, countsA));
  ASSERT_TRUE(Resolve(byUnits.position, scale, countsB));
  EXPECT_EQ(countsA, countsB);

  std::uint32_t speedA{}, speedB{};
  ASSERT_TRUE(Resolve(*byCounts.velocity, scale, speedA));
  ASSERT_TRUE(Resolve(*byUnits.velocity, scale, speedB));
  EXPECT_EQ(speedA, speedB);
}


// --- power and thermal ------------------------------------------------------

// 0x2200:01 is in tenths of a volt. 480 is a nominal 48 V pack.
TEST(PowerUnits, SupplyVoltageIsTenthsOfAVolt)
{
  EXPECT_DOUBLE_EQ(ToVoltage(480).value(), 48.0);
  EXPECT_DOUBLE_EQ(ToVoltage(0).value(), 0.0);
  EXPECT_DOUBLE_EQ(units::voltage::millivolt_t{ToVoltage(245)}.value(), 24500.0);
}

// 0x3201 is in tenths of a degree, and signed: a cold start is below zero.
TEST(PowerUnits, TemperatureIsSignedTenthsOfADegree)
{
  EXPECT_DOUBLE_EQ(ToTemperature(452).value(), 45.2);
  EXPECT_DOUBLE_EQ(ToTemperature(-155).value(), -15.5);

  // A real unit, not a number labelled celsius: it converts.
  EXPECT_NEAR(units::temperature::kelvin_t{ToTemperature(0)}.value(), 273.15, 1e-9);
}

TEST(PowerUnits, CurrentIsMilliamps)
{
  EXPECT_DOUBLE_EQ(ToCurrent(1500).value(), 1.5);
  EXPECT_DOUBLE_EQ(ToCurrent(-250).value(), -0.25);
}
