// Identity object 0x1018, section 6.2.11, decoded without a bus.
//
// The reference values are the ones in the EDS exported from the drive this
// repository targets: VendorNumber 0xFB, ProductNumber 0x65520000,
// RevisionNumber 0x01700000.

#include <gtest/gtest.h>

#include <string>

#include "epos4/signals/Identity.hpp"

using epos4::signals::DeviceIdentity;
using epos4::signals::Describe;
using epos4::signals::FirmwareName;
using epos4::signals::HardwareName;

namespace
{

DeviceIdentity
TheDriveInTheEds()
{
  DeviceIdentity identity;
  identity.vendorId = 0x000000FB;
  identity.productCode = 0x65520000;
  identity.revisionNumber = 0x01700000;
  identity.serialNumber = 12345678;
  return identity;
}

}  // namespace

TEST(Identity, SplitsProductCodeAndRevisionIntoTheirWords)
{
  const DeviceIdentity identity = TheDriveInTheEds();
  EXPECT_EQ(identity.HardwareVersion(), 0x6552);
  EXPECT_EQ(identity.ApplicationNumber(), 0x0000);
  EXPECT_EQ(identity.SoftwareVersion(), 0x0170);
  EXPECT_EQ(identity.ApplicationVersion(), 0x0000);
  EXPECT_TRUE(identity.IsMaxon());
}

// Table 6-66. The two that matter for this arm: the EDS is from a 50/15,
// and the question was whether the physical drive is a 50/5.
TEST(Identity, NamesTheHardwareFromTable6_66)
{
  EXPECT_STREQ(HardwareName(0x6552), "EPOS4 Module/Compact 50/15");
  EXPECT_STREQ(HardwareName(0x6150), "EPOS4 Module/Compact 50/5");
  EXPECT_STREQ(HardwareName(0x6A50), "EPOS4 Disk 60/8");
}

TEST(Identity, DoesNotGuessUnknownHardware)
{
  EXPECT_EQ(HardwareName(0x1234), nullptr);

  DeviceIdentity identity = TheDriveInTheEds();
  identity.productCode = 0x12340000;
  EXPECT_NE(Describe(identity).find("unknown maxon hardware"), std::string::npos);
}

// The same name maxon gives the binary in chapter 8, so the log line can be
// checked straight against the version history.
TEST(Identity, FirmwareIsNamedLikeMaxonsReleaseFiles)
{
  EXPECT_EQ(FirmwareName(TheDriveInTheEds()), "EPOS4_0170h_6552h_0000h_0000h");
}

TEST(Identity, DescribeCarriesHardwareFirmwareAndSerial)
{
  const std::string text = Describe(TheDriveInTheEds());
  EXPECT_NE(text.find("EPOS4 Module/Compact 50/15"), std::string::npos);
  EXPECT_NE(text.find("EPOS4_0170h_6552h_0000h_0000h"), std::string::npos);
  EXPECT_NE(text.find("12345678"), std::string::npos);
}

TEST(Identity, FlagsADeviceFromAnotherVendor)
{
  DeviceIdentity identity = TheDriveInTheEds();
  identity.vendorId = 0x555;  // the canopen_fake_slaves mock
  EXPECT_FALSE(identity.IsMaxon());
  EXPECT_NE(Describe(identity).find("not a maxon device"), std::string::npos);
}

// Section 6.2.1: 0x00020192 is a CiA 402 (0x0192) servo drive (2).
TEST(Identity, DeviceTypeSplitsIntoProfileAndDriveType)
{
  DeviceIdentity identity = TheDriveInTheEds();
  identity.deviceType = 0x00020192;
  EXPECT_EQ(identity.DeviceProfile(), 402);
  EXPECT_EQ(identity.DriveType(), 2);
}

// Table 6-92: application in the high word, bootloader in the low one.
TEST(Identity, ProgramSoftwareSplitsIntoApplicationAndBootloader)
{
  DeviceIdentity identity = TheDriveInTheEds();
  identity.programSoftware = 0x01700012;
  EXPECT_EQ(identity.ApplicationProgram(), 0x0170);
  EXPECT_EQ(identity.BootloaderProgram(), 0x0012);
}

// Table 6-93: 0 is "valid program available"; a download in progress (bit
// 0) or any error code (bits 7..1) is not, and the description says so.
TEST(Identity, AnInvalidFlashStatusIsCalledOut)
{
  DeviceIdentity identity = TheDriveInTheEds();
  identity.flashStatus = 0x00000000;
  EXPECT_TRUE(identity.IsProgramValid());
  EXPECT_EQ(Describe(identity).find("NOT VALID"), std::string::npos);

  identity.flashStatus = 0x00000001;   // download in progress
  EXPECT_FALSE(identity.IsProgramValid());
  identity.flashStatus = 0x000000A2;   // 81 << 1: no valid program available
  EXPECT_FALSE(identity.IsProgramValid());
  EXPECT_NE(Describe(identity).find("NOT VALID"), std::string::npos);

  identity.flashStatus = 0x12340000;   // manufacturer bits only: still valid
  EXPECT_TRUE(identity.IsProgramValid());
}
