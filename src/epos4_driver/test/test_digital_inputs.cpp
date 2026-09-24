// Tests for digital input mapping.
//
// Limit switches are the one input a driver must not get wrong: a switch that
// reads inverted, or whose function landed on the wrong bit, turns an end stop
// into a surprise.

#include <gtest/gtest.h>

#include <cstdint>

#include "epos4/configs/Configs.hpp"

using namespace epos4::configs;
using namespace epos4::signals;

namespace
{
std::ptrdiff_t
IndexOf(const ConfigWrites & writes, std::uint16_t index, std::uint8_t sub)
{
  for (std::size_t i = 0; i < writes.size(); ++i) {
    if (writes[i].entry.index == index && writes[i].entry.subindex == sub) {
      return static_cast<std::ptrdiff_t>(i);
    }
  }
  return -1;
}
}  // namespace

// Table 6-131. These numbers are also the bit positions in 0x60FD, so a wrong
// value here breaks both the assignment and the readback.
TEST(DigitalInputFunctions, MatchTable6_131)
{
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kNegativeLimitSwitch), 0);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kPositiveLimitSwitch), 1);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kHomeSwitch), 2);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kGeneralPurposeA), 16);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kGeneralPurposeH), 23);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kNegativeLimitSwitchNoError), 24);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kPositiveLimitSwitchNoError), 25);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kTouchProbe), 26);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kDriveEnable), 27);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kQuickStop), 28);
  EXPECT_EQ(static_cast<std::uint8_t>(DigitalInputFunction::kNone), 255);
}

TEST(DigitalInputConfig, PinsMapToSubIndicesOneThroughEight)
{
  DigitalInputConfigs inputs;
  inputs.input1 = DigitalInputFunction::kNegativeLimitSwitch;
  inputs.input4 = DigitalInputFunction::kGeneralPurposeD;
  inputs.highSpeedInput4 = DigitalInputFunction::kTouchProbe;

  ConfigWrites writes;
  inputs.AppendTo(writes);

  ASSERT_NE(IndexOf(writes, 0x3142, 1), -1);
  ASSERT_NE(IndexOf(writes, 0x3142, 4), -1);
  ASSERT_NE(IndexOf(writes, 0x3142, 8), -1);
  EXPECT_EQ(IndexOf(writes, 0x3142, 2), -1) << "wrote a pin that was never set";

  EXPECT_EQ(std::get<std::uint8_t>(writes[IndexOf(writes, 0x3142, 1)].value), 0u);
  EXPECT_EQ(std::get<std::uint8_t>(writes[IndexOf(writes, 0x3142, 8)].value), 26u);
}

// The device defaults of Table 6-130, written out explicitly.
TEST(DigitalInputConfig, CanReproduceTheDocumentedDefaults)
{
  DigitalInputConfigs inputs;
  inputs.input1 = DigitalInputFunction::kNegativeLimitSwitch;  // 0
  inputs.input2 = DigitalInputFunction::kPositiveLimitSwitch;  // 1
  inputs.input3 = DigitalInputFunction::kHomeSwitch;           // 2
  inputs.input4 = DigitalInputFunction::kGeneralPurposeD;      // 19

  EXPECT_FALSE(inputs.Validate());

  ConfigWrites writes;
  inputs.AppendTo(writes);
  EXPECT_EQ(std::get<std::uint8_t>(writes[IndexOf(writes, 0x3142, 1)].value), 0u);
  EXPECT_EQ(std::get<std::uint8_t>(writes[IndexOf(writes, 0x3142, 2)].value), 1u);
  EXPECT_EQ(std::get<std::uint8_t>(writes[IndexOf(writes, 0x3142, 3)].value), 2u);
  EXPECT_EQ(std::get<std::uint8_t>(writes[IndexOf(writes, 0x3142, 4)].value), 19u);
}

// "Each function can only be mapped once." Catching it here beats having the
// drive reject one write in the middle and leave the rest applied.
TEST(DigitalInputConfig, RejectsTheSameFunctionOnTwoPins)
{
  DigitalInputConfigs inputs;
  inputs.input1 = DigitalInputFunction::kHomeSwitch;
  inputs.input3 = DigitalInputFunction::kHomeSwitch;

  EXPECT_TRUE(inputs.Validate()) << "duplicate function was accepted";
}

TEST(DigitalInputConfig, NoneMayRepeat)
{
  DigitalInputConfigs inputs;
  inputs.highSpeedInput1 = DigitalInputFunction::kNone;
  inputs.highSpeedInput2 = DigitalInputFunction::kNone;
  inputs.highSpeedInput3 = DigitalInputFunction::kNone;

  EXPECT_FALSE(inputs.Validate());
}

// Both limit switch variants are distinct functions, so mapping one of each
// is legal even though they report on related bits.
TEST(DigitalInputConfig, LimitSwitchVariantsAreDistinctFunctions)
{
  DigitalInputConfigs inputs;
  inputs.input1 = DigitalInputFunction::kNegativeLimitSwitch;
  inputs.input2 = DigitalInputFunction::kNegativeLimitSwitchNoError;

  EXPECT_FALSE(inputs.Validate());
}

// Polarity decides how a level is read, so it must be in force before any
// function starts acting on that pin.
TEST(DigitalInputConfig, PolarityIsWrittenBeforeTheFunctions)
{
  DigitalInputConfigs inputs;
  inputs.polarity = 0x0003;
  inputs.input1 = DigitalInputFunction::kNegativeLimitSwitch;

  ConfigWrites writes;
  inputs.AppendTo(writes);

  const auto polarity = IndexOf(writes, 0x3141, 2);
  const auto function = IndexOf(writes, 0x3142, 1);
  ASSERT_NE(polarity, -1);
  ASSERT_NE(function, -1);
  EXPECT_LT(polarity, function);
}

TEST(DigitalInputConfig, EmptyConfigWritesNothing)
{
  DigitalInputConfigs inputs;
  ConfigWrites writes;
  inputs.AppendTo(writes);
  EXPECT_TRUE(writes.empty());
}

// The numeric value of a function IS its bit in 0x60FD. This is what lets
// IsInputActive() work without a lookup table, so it is worth pinning.
TEST(DigitalInputs, FunctionValueIsTheBitPositionInObject60FD)
{
  auto bit = [](DigitalInputFunction f) {
      return 1u << static_cast<std::uint8_t>(f);
    };

  EXPECT_EQ(bit(DigitalInputFunction::kNegativeLimitSwitch), 0x00000001u);
  EXPECT_EQ(bit(DigitalInputFunction::kPositiveLimitSwitch), 0x00000002u);
  EXPECT_EQ(bit(DigitalInputFunction::kHomeSwitch), 0x00000004u);
  EXPECT_EQ(bit(DigitalInputFunction::kGeneralPurposeA), 0x00010000u);
  EXPECT_EQ(bit(DigitalInputFunction::kQuickStop), 0x10000000u);
}

// ---------------------------------------------------------------------------
// Homing methods and the switches they depend on
// ---------------------------------------------------------------------------

// Section 3.5.3. These fifteen are what the EPOS4 implements, and the four
// negative ones are easy to transpose: -1 and -3 are POSITIVE speed, -2 and
// -4 are negative. Getting them backwards homes the axis the wrong way.
TEST(HomingMethods, MatchSection3_5_3)
{
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kNegativeLimitSwitchAndIndex), 1);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kPositiveLimitSwitchAndIndex), 2);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kHomeSwitchPositiveSpeedAndIndex), 7);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kHomeSwitchNegativeSpeedAndIndex), 11);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kNegativeLimitSwitch), 17);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kPositiveLimitSwitch), 18);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kHomeSwitchPositiveSpeed), 23);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kHomeSwitchNegativeSpeed), 27);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kIndexNegativeSpeed), 33);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kIndexPositiveSpeed), 34);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kActualPosition), 37);

  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kCurrentThresholdPositiveSpeedAndIndex), -1);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kCurrentThresholdNegativeSpeedAndIndex), -2);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kCurrentThresholdPositiveSpeed), -3);
  EXPECT_EQ(static_cast<std::int8_t>(HomingMethod::kCurrentThresholdNegativeSpeed), -4);
}

TEST(HomingMethods, SwitchRequirementsMatchTheMethodNames)
{
  EXPECT_EQ(
    RequiredInput(HomingMethod::kNegativeLimitSwitch),
    DigitalInputFunction::kNegativeLimitSwitch);
  EXPECT_EQ(
    RequiredInput(HomingMethod::kPositiveLimitSwitchAndIndex),
    DigitalInputFunction::kPositiveLimitSwitch);
  EXPECT_EQ(
    RequiredInput(HomingMethod::kHomeSwitchNegativeSpeed),
    DigitalInputFunction::kHomeSwitch);
  EXPECT_EQ(
    RequiredInput(HomingMethod::kHomeSwitchPositiveSpeedAndIndex),
    DigitalInputFunction::kHomeSwitch);
}

// The methods that need no switch at all. Reporting one of these as needing
// a switch would block a perfectly valid homing run.
TEST(HomingMethods, SwitchlessMethodsRequireNoInput)
{
  EXPECT_FALSE(RequiredInput(HomingMethod::kActualPosition).has_value());
  EXPECT_FALSE(RequiredInput(HomingMethod::kIndexNegativeSpeed).has_value());
  EXPECT_FALSE(RequiredInput(HomingMethod::kIndexPositiveSpeed).has_value());
  EXPECT_FALSE(RequiredInput(HomingMethod::kCurrentThresholdPositiveSpeed).has_value());
  EXPECT_FALSE(RequiredInput(HomingMethod::kCurrentThresholdNegativeSpeedAndIndex).has_value());
}

// "& index" in the name means the run ends on the encoder index pulse, which
// needs a 3-channel encoder configured with IndexType::kWithIndex.
TEST(HomingMethods, IndexRequirementFollowsTheName)
{
  EXPECT_TRUE(RequiresEncoderIndex(HomingMethod::kNegativeLimitSwitchAndIndex));
  EXPECT_TRUE(RequiresEncoderIndex(HomingMethod::kHomeSwitchNegativeSpeedAndIndex));
  EXPECT_TRUE(RequiresEncoderIndex(HomingMethod::kIndexPositiveSpeed));
  EXPECT_TRUE(RequiresEncoderIndex(HomingMethod::kCurrentThresholdPositiveSpeedAndIndex));

  EXPECT_FALSE(RequiresEncoderIndex(HomingMethod::kNegativeLimitSwitch));
  EXPECT_FALSE(RequiresEncoderIndex(HomingMethod::kHomeSwitchNegativeSpeed));
  EXPECT_FALSE(RequiresEncoderIndex(HomingMethod::kActualPosition));
  EXPECT_FALSE(RequiresEncoderIndex(HomingMethod::kCurrentThresholdNegativeSpeed));
}

// The default input mapping happens to satisfy the most common methods.
TEST(HomingMethods, DefaultInputMappingCoversTheCommonMethods)
{
  DigitalInputConfigs defaults;
  defaults.input1 = DigitalInputFunction::kNegativeLimitSwitch;
  defaults.input2 = DigitalInputFunction::kPositiveLimitSwitch;
  defaults.input3 = DigitalInputFunction::kHomeSwitch;

  auto mapped = [&defaults](DigitalInputFunction f) {
      return defaults.input1 == f || defaults.input2 == f || defaults.input3 == f;
    };

  for (auto method : {HomingMethod::kNegativeLimitSwitch,
      HomingMethod::kPositiveLimitSwitch,
      HomingMethod::kHomeSwitchNegativeSpeed,
      HomingMethod::kHomeSwitchPositiveSpeedAndIndex})
  {
    const auto required = RequiredInput(method);
    ASSERT_TRUE(required.has_value());
    EXPECT_TRUE(mapped(*required)) << ToString(method);
  }
}
