#pragma once

#include <cstdint>
#include <string>

namespace epos4::signals
{

// ---------------------------------------------------------------------------
// Identity object 0x1018, section 6.2.11 (p.6-74), decoded.
//
// Worth reading once per drive at start-up and putting in the log: "which
// firmware was on the elbow when it faulted" is otherwise a question nobody
// can answer from a rover in the field.
//
// Note what it can and cannot tell apart. The hardware version names the
// power stage (50/5, 50/15, ...) but the Module and the Compact share every
// code in Table 6-66: they are the same electronics in a different housing.
// The power stage is what matters, because it sets the current limits; and
// the master already refuses to boot a drive whose product code differs from
// the one in the DCF (0x1F86), so a wrong model fails loudly at boot rather
// than running with somebody else's limits.
// ---------------------------------------------------------------------------

// «maxon motor ag», assigned by CiA. The only vendor this library drives.
constexpr std::uint32_t kMaxonVendorId = 0x000000FB;

struct DeviceIdentity
{
  std::uint32_t vendorId{0};        // 0x1018:01
  std::uint32_t productCode{0};     // 0x1018:02
  std::uint32_t revisionNumber{0};  // 0x1018:03
  std::uint32_t serialNumber{0};    // 0x1018:04, last 8 digits

  // Beyond 0x1018. Read with it, but best effort: an object a firmware does
  // not have leaves its field at zero instead of failing the whole read.
  std::uint32_t deviceType{0};           // 0x1000
  std::string deviceName;                // 0x1008, "EPOS4"
  std::uint64_t serialNumberComplete{0};  // 0x2100:01, all 64 bits
  std::uint32_t programSoftware{0};      // 0x1F56:01, Table 6-92
  std::uint32_t flashStatus{0};          // 0x1F57:01, Table 6-93

  // Product code: hardware version in the high word, application number in
  // the low word.
  std::uint16_t HardwareVersion() const {return static_cast<std::uint16_t>(productCode >> 16);}
  std::uint16_t ApplicationNumber() const {return static_cast<std::uint16_t>(productCode);}

  // Revision number: software (firmware) version in the high word,
  // application version in the low word.
  std::uint16_t SoftwareVersion() const {return static_cast<std::uint16_t>(revisionNumber >> 16);}
  std::uint16_t ApplicationVersion() const {return static_cast<std::uint16_t>(revisionNumber);}

  bool IsMaxon() const {return vendorId == kMaxonVendorId;}

  // Device type (6.2.1): the profile in the low word - 402 for CiA 402 -
  // and the drive type in the high word, 2 for a servo drive.
  std::uint16_t DeviceProfile() const {return static_cast<std::uint16_t>(deviceType);}
  std::uint16_t DriveType() const {return static_cast<std::uint16_t>(deviceType >> 16);}

  // Program software identification (Table 6-92).
  std::uint16_t ApplicationProgram() const
  {
    return static_cast<std::uint16_t>(programSoftware >> 16);
  }
  std::uint16_t BootloaderProgram() const {return static_cast<std::uint16_t>(programSoftware);}

  // Flash status (Table 6-93): bit 0 is "download in progress", bits 7..1
  // an error code, 0 meaning "valid program available". Anything else and
  // the firmware on the drive is not to be trusted.
  bool IsProgramValid() const {return (flashStatus & 0xFFu) == 0;}
};

// The hardware named by Table 6-66, e.g. 0x6552 -> "EPOS4 Module/Compact
// 50/15". nullptr for a code the table does not list, which means hardware
// newer than this library - worth reporting, not guessing.
const char * HardwareName(std::uint16_t hardwareVersion);

// The firmware as maxon names its files and release notes (chapter 8):
// software, hardware, application number and application version, e.g.
// "EPOS4_0170h_6552h_0000h_0000h". The same string EPOS Studio shows, so it
// can be compared directly against the version history.
std::string FirmwareName(const DeviceIdentity & identity);

// One line for a log: hardware, firmware and serial number.
std::string Describe(const DeviceIdentity & identity);

}  // namespace epos4::signals
