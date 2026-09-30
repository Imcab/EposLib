// What a drive says it can do, decoded without a bus.

#include <gtest/gtest.h>

#include <string>

#include "epos4/signals/Enums.hpp"

using epos4::signals::OperationMode;
using epos4::signals::SupportsMode;

// Section 6.2.151: the EPOS4 reports 0x000003A5 - PPM, PVM, HMM, CSP, CSV
// and CST, bit (mode - 1) each.
TEST(Capabilities, TheEpos4DefaultListsTheSixModesItImplements)
{
  constexpr std::uint32_t kEpos4 = 0x000003A5;
  EXPECT_TRUE(SupportsMode(kEpos4, OperationMode::kProfilePosition));
  EXPECT_TRUE(SupportsMode(kEpos4, OperationMode::kProfileVelocity));
  EXPECT_TRUE(SupportsMode(kEpos4, OperationMode::kHoming));
  EXPECT_TRUE(SupportsMode(kEpos4, OperationMode::kCyclicSynchronousPosition));
  EXPECT_TRUE(SupportsMode(kEpos4, OperationMode::kCyclicSynchronousVelocity));
  EXPECT_TRUE(SupportsMode(kEpos4, OperationMode::kCyclicSynchronousTorque));
}

TEST(Capabilities, ModesTheEpos4LacksAreNotReported)
{
  constexpr std::uint32_t kEpos4 = 0x000003A5;
  EXPECT_FALSE(SupportsMode(kEpos4, static_cast<OperationMode>(2)));  // Velocity Mode
  EXPECT_FALSE(SupportsMode(kEpos4, static_cast<OperationMode>(4)));  // Torque Mode
  EXPECT_FALSE(SupportsMode(kEpos4, static_cast<OperationMode>(7)));  // Interpolated Position
  EXPECT_FALSE(SupportsMode(kEpos4, OperationMode::kNone));           // not a mode at all
}

TEST(Capabilities, BitRatesAndFieldbusesHaveNames)
{
  using epos4::signals::CanBitRate;
  using epos4::signals::Fieldbus;
  EXPECT_STREQ(epos4::signals::ToString(CanBitRate::k1Mbit), "1 Mbit/s");
  EXPECT_STREQ(epos4::signals::ToString(CanBitRate::kAutomatic), "automatic detection");
  EXPECT_EQ(static_cast<int>(CanBitRate::k50kbit), 6);   // 5 is reserved (Table 6-94)
  EXPECT_STREQ(epos4::signals::ToString(Fieldbus::kCanOpen), "CANopen");
}
