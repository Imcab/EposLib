// Tests for the chapter 7 error tables.
//
// These check the transcription against the manual, not the code: if a row
// was mistyped, a range dropped or a "Position Clear" flag missed, the drive
// would be diagnosed wrongly in exactly the situation where a wrong diagnosis
// costs the most.

#include <gtest/gtest.h>

#include <string>

#include "epos4/signals/Errors.hpp"

using namespace epos4::signals;

// Table 7-186 has 73 rows once the three ranges are counted as one each.
TEST(DeviceErrors, KnownCodesResolve)
{
  EXPECT_STREQ(FindDeviceError(0x0000)->name, "No Error");
  EXPECT_STREQ(FindDeviceError(0x1000)->name, "Generic error");
  EXPECT_STREQ(FindDeviceError(0x2310)->name, "Overcurrent error");
  EXPECT_STREQ(FindDeviceError(0x3210)->name, "Overvoltage error");
  EXPECT_STREQ(FindDeviceError(0x3220)->name, "Undervoltage error");
  EXPECT_STREQ(FindDeviceError(0x4210)->name, "Thermal overload error");
  EXPECT_STREQ(FindDeviceError(0x8611)->name, "Following error");
  EXPECT_STREQ(FindDeviceError(0x8A80)->name, "Negative limit switch error");
  EXPECT_STREQ(FindDeviceError(0x8A82)->name, "Software position limit error");
  EXPECT_STREQ(FindDeviceError(0xFF01)->name, "System overloaded error");
}

// The manual lists three codes as ranges. A driver that only matched the
// first value of each would report "unknown error" for most of them.
TEST(DeviceErrors, RangesCoverEveryCodeInside)
{
  // Generic initialization error, 0x1080 through 0x1088
  for (std::uint16_t c = 0x1080; c <= 0x1088; ++c) {
    ASSERT_NE(FindDeviceError(c), nullptr) << std::hex << c;
    EXPECT_STREQ(FindDeviceError(c)->name, "Generic initialization error") << std::hex << c;
  }
  // Hardware error, 0x5480 through 0x5483
  for (std::uint16_t c = 0x5480; c <= 0x5483; ++c) {
    ASSERT_NE(FindDeviceError(c), nullptr) << std::hex << c;
    EXPECT_STREQ(FindDeviceError(c)->name, "Hardware error") << std::hex << c;
  }
  // Internal software error, 0x6180 through 0x61F0
  EXPECT_STREQ(FindDeviceError(0x6180)->name, "Internal software error");
  EXPECT_STREQ(FindDeviceError(0x61A0)->name, "Internal software error");
  EXPECT_STREQ(FindDeviceError(0x61F0)->name, "Internal software error");
}

TEST(DeviceErrors, UnknownCodeIsReportedAsUnknownRatherThanGuessed)
{
  EXPECT_EQ(FindDeviceError(0x1234), nullptr);
  EXPECT_NE(DeviceErrorName(0x1234).find("unknown"), std::string::npos);
}

// Error register bits, Table 6-60. Wrong bits here mean a temperature fault
// reported as a communication fault.
TEST(DeviceErrors, ErrorRegisterMatchesTable6_60)
{
  EXPECT_EQ(FindDeviceError(0x2310)->errorRegister, error_register::kCurrent);
  EXPECT_EQ(FindDeviceError(0x3220)->errorRegister, error_register::kVoltage);
  EXPECT_EQ(FindDeviceError(0x4210)->errorRegister, error_register::kTemperature);
  EXPECT_EQ(FindDeviceError(0x8130)->errorRegister, error_register::kCommunication);
  EXPECT_EQ(FindDeviceError(0x8611)->errorRegister, error_register::kMotion);
  EXPECT_EQ(FindDeviceError(0x7320)->errorRegister, error_register::kDeviceProfile);
}

// 'w' in the fault reaction column: the drive keeps running. Treating one of
// these as a fault would stop an axis that never stopped.
TEST(DeviceErrors, SystemOverloadedIsAWarningNotAFault)
{
  EXPECT_TRUE(IsWarning(0xFF01));
  EXPECT_FALSE(IsWarning(0x8611));
  EXPECT_FALSE(IsWarning(0x2310));
}

// "Position Clear" in the manual: resetting these loses the reference, so the
// axis has to be homed again before its position means anything.
TEST(DeviceErrors, PositionClearingErrorsAreFlagged)
{
  EXPECT_TRUE(ClearsPosition(0x7380));   // Position sensor breach error
  EXPECT_TRUE(ClearsPosition(0x7388));   // Hall sensor error
  EXPECT_TRUE(ClearsPosition(0x7392));   // Main sensor direction error
  EXPECT_TRUE(ClearsPosition(0xFF02));   // Watchdog error
  EXPECT_FALSE(ClearsPosition(0x8611));  // Following error keeps the position
}

TEST(DeviceErrors, FaultReactionMatchesTheManualNotation)
{
  // 'f': use Fault reaction option code
  EXPECT_EQ(FindDeviceError(0x8611)->faultReaction, FaultReaction::kFaultReactionOption);
  // 'a': use Abort connection option code
  EXPECT_EQ(FindDeviceError(0x8250)->faultReaction, FaultReaction::kAbortConnectionOption);
  // 'd': secure movement no longer possible
  EXPECT_EQ(FindDeviceError(0x2310)->faultReaction, FaultReaction::kNoSecureMovement);
  // 'w': warning
  EXPECT_EQ(FindDeviceError(0xFF01)->faultReaction, FaultReaction::kWarning);
}

// The prose is the whole point: a bare code is not a diagnosis.
TEST(DeviceErrors, DescriptionCarriesCauseAndRecovery)
{
  const std::string text = DescribeDeviceError(0x8611);
  EXPECT_NE(text.find("Following error"), std::string::npos);
  EXPECT_NE(text.find("Position demand value"), std::string::npos);
  EXPECT_NE(text.find("Reset fault with Controlword"), std::string::npos);

  const std::string overcurrent = DescribeDeviceError(0x2310);
  EXPECT_NE(overcurrent.find("Short circuit in motor winding"), std::string::npos);
  EXPECT_NE(overcurrent.find("Controller gains too high"), std::string::npos);
}

TEST(DeviceErrors, PositionClearingErrorSaysSoInTheDescription)
{
  EXPECT_NE(DescribeDeviceError(0x7388).find("homed again"), std::string::npos);
  EXPECT_EQ(DescribeDeviceError(0x8611).find("homed again"), std::string::npos);
}

TEST(ErrorRegister, DecodesEveryFlag)
{
  EXPECT_EQ(DescribeErrorRegister(0), "no error flags");
  EXPECT_EQ(DescribeErrorRegister(error_register::kCurrent), "current");
  EXPECT_NE(
    DescribeErrorRegister(
      error_register::kGeneric | error_register::kTemperature).find("generic"),
    std::string::npos);
  EXPECT_NE(
    DescribeErrorRegister(
      error_register::kGeneric | error_register::kTemperature).find("temperature"),
    std::string::npos);
}

// Table 7-187. These are what come back when an SDO is refused, and the
// difference between "read only object" and "object does not exist" is the
// difference between a typo in an index and a wrong access type.
TEST(AbortCodes, KnownCodesResolve)
{
  EXPECT_STREQ(FindAbortCode(0x00000000)->name, "No abort");
  EXPECT_STREQ(FindAbortCode(0x05040000)->name, "SDO timeout");
  EXPECT_STREQ(FindAbortCode(0x06010002)->name, "Read only error");
  EXPECT_STREQ(FindAbortCode(0x06020000)->name, "Object does not exist error");
  EXPECT_STREQ(FindAbortCode(0x06040041)->name, "PDO mapping error");
  EXPECT_STREQ(FindAbortCode(0x06090030)->name, "Value range error");
  EXPECT_STREQ(FindAbortCode(0x08000022)->name, "Wrong device state error");
  // maxon specific, above 0x0F000000
  EXPECT_STREQ(FindAbortCode(0x0F00FFC0)->name, "Wrong NMT state error");
}

TEST(AbortCodes, UnknownIsNotGuessed)
{
  EXPECT_EQ(FindAbortCode(0x12345678), nullptr);
  EXPECT_NE(DescribeAbortCode(0x12345678).find("unknown"), std::string::npos);
}

TEST(AbortCodes, DescriptionCarriesTheCause)
{
  EXPECT_NE(
    DescribeAbortCode(0x06010002).find("Write command to a read only object"),
    std::string::npos);
}

TEST(Emergency, DescribesCodeAndRegisterTogether)
{
  EmergencyMessage message{};
  message.errorCode = 0x4210;
  message.errorRegister = error_register::kTemperature;

  const std::string text = Describe(message);
  EXPECT_NE(text.find("Thermal overload error"), std::string::npos);
  EXPECT_NE(text.find("temperature"), std::string::npos);
}
