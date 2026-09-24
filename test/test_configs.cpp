// Tests for configuration serialisation.
//
// These check two properties that matter more than they look:
//
//  1. An unset field writes nothing. A configuration object with defaults
//     baked in would silently overwrite motor data and controller gains that
//     somebody spent a day tuning, the first time anyone applied a partially
//     filled struct.
//
//  2. Brake-related writes come out in an order that cannot damage hardware.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "epos4/configs/Configs.hpp"

using namespace epos4::configs;
using namespace epos4::signals;

namespace
{

std::vector<std::uint32_t>
Keys(const ConfigWrites & writes)
{
  std::vector<std::uint32_t> out;
  for (const auto & w : writes) {
    out.push_back((static_cast<std::uint32_t>(w.entry.index) << 8) | w.entry.subindex);
  }
  return out;
}

constexpr std::uint32_t Key(std::uint16_t index, std::uint8_t sub = 0)
{
  return (static_cast<std::uint32_t>(index) << 8) | sub;
}

std::ptrdiff_t
IndexOf(const ConfigWrites & writes, std::uint32_t key)
{
  const auto keys = Keys(writes);
  const auto it = std::find(keys.begin(), keys.end(), key);
  return it == keys.end() ? -1 : std::distance(keys.begin(), it);
}

}  // namespace

// The property the whole optional-based design exists for.
TEST(Configs, AnEmptyConfigurationWritesNothing)
{
  Epos4Configuration config;
  EXPECT_TRUE(config.ToWrites().empty());
}

TEST(Configs, OnlyExplicitlySetFieldsAreWritten)
{
  Epos4Configuration config;
  config.positionControl.p = 1500000;
  config.limits.maxMotorSpeed = 8000;

  const auto writes = config.ToWrites();
  ASSERT_EQ(writes.size(), 2u);
  EXPECT_NE(IndexOf(writes, Key(0x30A1, 1)), -1);  // position controller P gain
  EXPECT_NE(IndexOf(writes, Key(0x6080, 0)), -1);  // max motor speed

  // Nothing else, in particular no motor data and no other gains.
  EXPECT_EQ(IndexOf(writes, Key(0x3001, 1)), -1);
  EXPECT_EQ(IndexOf(writes, Key(0x30A1, 2)), -1);
}

TEST(Configs, ValuesAndObjectAddressesMatchTheManual)
{
  Epos4Configuration config;
  config.motor.nominalCurrent = 3000;              // 0x3001:01
  config.motor.motorType = MotorType::kBrushlessSinusoidal;  // 0x6402
  config.motionProfile.profileVelocity = 2000;     // 0x6081
  config.homing.method = HomingMethod::kActualPosition;      // 0x6098
  config.limits.minPositionLimit = -100000;        // 0x607D:01

  const auto writes = config.ToWrites();
  EXPECT_NE(IndexOf(writes, Key(0x3001, 1)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x6402, 0)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x6081, 0)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x6098, 0)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x607D, 1)), -1);

  // Motor type is UNSIGNED16 per Table 6-171, and 10 is the sinusoidal BLDC.
  for (const auto & w : writes) {
    if (w.entry.index == 0x6402) {
      ASSERT_TRUE(std::holds_alternative<std::uint16_t>(w.value));
      EXPECT_EQ(std::get<std::uint16_t>(w.value), 10u);
    }
  }
}

// ---------------------------------------------------------------------------
// Holding brake
// ---------------------------------------------------------------------------

// The manual: "The holding brake or the motor may be damaged if the holding
// brake will activate before the motor has reached full standstill." So the
// condition that defines standstill has to be in force before any output can
// be driving a brake coil.
TEST(BrakeConfigs, StandstillIsConfiguredBeforeTheBrakeAndTheOutput)
{
  Epos4Configuration config;
  config.standstill.window = 30;
  config.standstill.windowTimeMs = 2;
  config.holdingBrake.couplingTimeMs = 40;
  config.holdingBrake.openingTimeMs = 25;
  config.digitalOutputs.output1 = DigitalOutputFunction::kHoldingBrake;

  const auto writes = config.ToWrites();
  const auto standstill = IndexOf(writes, Key(0x30E0, 1));
  const auto brakeTiming = IndexOf(writes, Key(0x3158, 1));
  const auto outputAssign = IndexOf(writes, Key(0x3151, 1));

  ASSERT_NE(standstill, -1);
  ASSERT_NE(brakeTiming, -1);
  ASSERT_NE(outputAssign, -1);

  EXPECT_LT(standstill, brakeTiming) << "brake timings written before standstill";
  EXPECT_LT(brakeTiming, outputAssign) << "brake pin driven before its timings exist";
}

TEST(BrakeConfigs, TimingsMapToTheDocumentedSubIndices)
{
  HoldingBrakeConfigs brake;
  brake.couplingTimeMs = 40;   // 0x3158:01, reaction time closing
  brake.openingTimeMs = 25;    // 0x3158:02, reaction time opening

  ConfigWrites writes;
  brake.AppendTo(writes);
  ASSERT_EQ(writes.size(), 2u);
  EXPECT_EQ(writes[0].entry.index, 0x3158);
  EXPECT_EQ(writes[0].entry.subindex, 1);
  EXPECT_EQ(std::get<std::uint16_t>(writes[0].value), 40u);
  EXPECT_EQ(writes[1].entry.subindex, 2);
  EXPECT_EQ(std::get<std::uint16_t>(writes[1].value), 25u);
}

// Sub-indices 4 and 5 only exist on Disk 60/8, Disk 60/12 and
// Module/Compact 60/20. On anything else writing them aborts, so they must
// stay out of the write list unless asked for.
TEST(BrakeConfigs, VariantSpecificVoltagesAreOmittedUnlessSet)
{
  HoldingBrakeConfigs brake;
  brake.couplingTimeMs = 40;

  ConfigWrites writes;
  brake.AppendTo(writes);
  for (const auto & w : writes) {
    EXPECT_LT(w.entry.subindex, 4) << "wrote a sub-index that most variants lack";
  }

  brake.openingVoltageDeciVolt = 240;
  ConfigWrites withVoltage;
  brake.AppendTo(withVoltage);
  EXPECT_NE(IndexOf(withVoltage, Key(0x3158, 4)), -1);
}

TEST(BrakeConfigs, HoldingBrakeFunctionCodeIs24)
{
  DigitalOutputConfigs outputs;
  outputs.output1 = DigitalOutputFunction::kHoldingBrake;

  ConfigWrites writes;
  outputs.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(writes[0].entry.index, 0x3151);
  EXPECT_EQ(std::get<std::uint8_t>(writes[0].value), 24u);  // Table 6-134
}

TEST(BrakeConfigs, OutputPolarityIsCarriedSeparatelyFromTheFunction)
{
  DigitalOutputConfigs outputs;
  outputs.output1 = DigitalOutputFunction::kHoldingBrake;
  outputs.polarity = 0x0001;

  ConfigWrites writes;
  outputs.AppendTo(writes);
  EXPECT_NE(IndexOf(writes, Key(0x3151, 1)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x3150, 2)), -1);
}

TEST(StandstillConfigs, MapToTheDocumentedSubIndices)
{
  StandstillConfigs standstill;
  standstill.window = 30;            // 0x30E0:01
  standstill.windowTimeMs = 2;       // 0x30E0:02
  standstill.windowTimeoutMs = 500;  // 0x30E0:03

  ConfigWrites writes;
  standstill.AppendTo(writes);
  ASSERT_EQ(writes.size(), 3u);
  EXPECT_EQ(writes[0].entry.subindex, 1);
  EXPECT_EQ(std::get<std::uint32_t>(writes[0].value), 30u);
  EXPECT_EQ(writes[1].entry.subindex, 2);
  EXPECT_EQ(writes[2].entry.subindex, 3);
}

// ---------------------------------------------------------------------------
// Stop option codes
//
// These pin each enum to the value range its table gives. They exist because
// QuickStopOption originally carried two values the EPOS4 does not implement:
// writing them aborts with 0x06090030 "Value range error", and since Apply()
// stops at the first failure, everything after it silently never landed.
// ---------------------------------------------------------------------------

// Table 6-149: the manual gives the range as "6 to 6". One value, no choice.
TEST(StopOptions, QuickStopHasExactlyOneValue)
{
  EXPECT_EQ(
    static_cast<std::int16_t>(QuickStopOption::kSlowDownOnQuickStopRampAndStayInQuickStop),
    6);

  StopOptionConfigs stops;
  stops.quickStop = QuickStopOption::kSlowDownOnQuickStopRampAndStayInQuickStop;
  ConfigWrites writes;
  stops.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(writes[0].entry.index, 0x605A);
  EXPECT_EQ(std::get<std::int16_t>(writes[0].value), 6);
}

// Table 6-150 and Table 6-151: both are 0 or 1.
TEST(StopOptions, ShutdownAndDisableOperationAreZeroOrOne)
{
  EXPECT_EQ(static_cast<std::int16_t>(ShutdownOption::kDisableDrive), 0);
  EXPECT_EQ(static_cast<std::int16_t>(ShutdownOption::kSlowDownOnSlowDownRamp), 1);
  EXPECT_EQ(static_cast<std::int16_t>(DisableOperationOption::kDisableDrive), 0);
  EXPECT_EQ(static_cast<std::int16_t>(DisableOperationOption::kSlowDownOnSlowDownRamp), 1);
}

// Table 6-153: 0, 1 and 2. Value 1 was missing, and it is the useful middle
// one - ramp down normally on a fault instead of braking hard, which on a
// loaded arm is the difference between a controlled stop and a jolt.
TEST(StopOptions, FaultReactionHasAllThreeValuesIncludingTheMiddleOne)
{
  EXPECT_EQ(static_cast<std::int16_t>(FaultReactionOption::kDisableDrive), 0);
  EXPECT_EQ(static_cast<std::int16_t>(FaultReactionOption::kSlowDownOnSlowDownRamp), 1);
  EXPECT_EQ(static_cast<std::int16_t>(FaultReactionOption::kSlowDownOnQuickStopRamp), 2);
}

// Table 6-146: only 2 and 3. This is what the drive does when the bus goes
// away, so a value it rejects means no configured reaction at all.
TEST(StopOptions, AbortConnectionIsTwoOrThree)
{
  EXPECT_EQ(static_cast<std::int16_t>(AbortConnectionOption::kDisableVoltage), 2);
  EXPECT_EQ(static_cast<std::int16_t>(AbortConnectionOption::kSlowDownOnQuickStopRamp), 3);

  StopOptionConfigs stops;
  stops.abortConnectionOption = AbortConnectionOption::kSlowDownOnQuickStopRamp;
  ConfigWrites writes;
  stops.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(writes[0].entry.index, 0x6007);
  EXPECT_EQ(std::get<std::int16_t>(writes[0].value), 3);
}

TEST(StopOptions, AllFourCodesTargetTheirOwnObject)
{
  StopOptionConfigs stops;
  stops.quickStop = QuickStopOption::kSlowDownOnQuickStopRampAndStayInQuickStop;
  stops.shutdown = ShutdownOption::kSlowDownOnSlowDownRamp;
  stops.disableOperation = DisableOperationOption::kDisableDrive;
  stops.faultReaction = FaultReactionOption::kSlowDownOnSlowDownRamp;
  stops.abortConnectionOption = AbortConnectionOption::kDisableVoltage;

  ConfigWrites writes;
  stops.AppendTo(writes);
  ASSERT_EQ(writes.size(), 5u);
  EXPECT_NE(IndexOf(writes, Key(0x605A)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x605B)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x605C)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x605E)), -1);
  EXPECT_NE(IndexOf(writes, Key(0x6007)), -1);

  // 0x605D, the halt option code, is documented in the manual but absent from
  // the EPOS4 Module 50/15 EDS, so nothing here must ever write it.
  EXPECT_EQ(IndexOf(writes, Key(0x605D)), -1);
}

// ---------------------------------------------------------------------------
// Cyclic modes, 0x60C2
// ---------------------------------------------------------------------------

TEST(CyclicConfig, WritesOnlyThePeriodValueNotTheTimeIndex)
{
  CyclicConfigs cyclic;
  cyclic.interpolationTimePeriodMs = 10;

  ConfigWrites writes;
  cyclic.AppendTo(writes);

  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(writes[0].entry.index, 0x60C2);
  EXPECT_EQ(writes[0].entry.subindex, 1);
  EXPECT_EQ(std::get<std::uint8_t>(writes[0].value), 10u);

  // Sub-index 2 is fixed at -3 by the device (range -3 to -3), so writing it
  // can only ever be an opportunity to get it wrong.
  EXPECT_EQ(IndexOf(writes, Key(0x60C2, 2)), -1);
}

TEST(CyclicConfig, UnsetWritesNothing)
{
  CyclicConfigs cyclic;
  ConfigWrites writes;
  cyclic.AppendTo(writes);
  EXPECT_TRUE(writes.empty());
}

// The value has to match the master's SYNC period. bus.yml uses microseconds
// and 0x60C2 uses milliseconds, which is exactly the kind of mismatch that
// produces stepping motion and no error message.
TEST(CyclicConfig, TenMillisecondsMatchesASyncPeriodOfTenThousandMicroseconds)
{
  constexpr std::uint32_t kSyncPeriodMicroseconds = 10000;  // from bus.yml
  constexpr std::uint8_t kExpectedPeriodMs = kSyncPeriodMicroseconds / 1000;

  CyclicConfigs cyclic;
  cyclic.interpolationTimePeriodMs = kExpectedPeriodMs;

  ConfigWrites writes;
  cyclic.AppendTo(writes);
  EXPECT_EQ(std::get<std::uint8_t>(writes[0].value), 10u);
}

TEST(CyclicConfig, IsSeparateFromTheProfileGroup)
{
  // Setting the cyclic period must not emit any of the PPM/PVM profile
  // objects, and vice versa: they configure different modes.
  Epos4Configuration config;
  config.cyclic.interpolationTimePeriodMs = 10;

  const auto writes = config.ToWrites();
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(IndexOf(writes, Key(0x6081)), -1);
  EXPECT_EQ(IndexOf(writes, Key(0x6083)), -1);
}
