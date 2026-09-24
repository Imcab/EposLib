#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace epos4::signals
{

// ---------------------------------------------------------------------------
// Device errors, chapter 7 of the EPOS4 Firmware Specification.
//
// The tables in Errors.cpp are transcribed from Table 7-186 (p.7-265) and the
// per-error sections 7.2.1 to 7.2.72, which is where the cause, the effect and
// the recovery for each code live. All 73 codes and all 26 SDO abort codes are
// present; every summary row has its detail section and vice versa.
//
// The point of carrying the prose and not just the names: on a rover, a fault
// reported as "0x8611" is a dead arm and a long drive. Reported as "Following
// error - difference between Position demand value and Position actual value
// higher than Following error window - reset fault with Controlword", it is a
// diagnosis someone can act on from the base station.
// ---------------------------------------------------------------------------


// Error register, object 0x1001, Table 6-60 (p.6-68). The device ORs these
// flags together, so one byte summarises what kind of thing went wrong even
// before the specific code is looked up.
namespace error_register
{
constexpr std::uint8_t kGeneric = 1u << 0;
constexpr std::uint8_t kCurrent = 1u << 1;
constexpr std::uint8_t kVoltage = 1u << 2;
constexpr std::uint8_t kTemperature = 1u << 3;
constexpr std::uint8_t kCommunication = 1u << 4;
constexpr std::uint8_t kDeviceProfile = 1u << 5;
// bit 6 is reserved, always 0
constexpr std::uint8_t kMotion = 1u << 7;
}  // namespace error_register


// How the drive reacts when this error fires. The notation is the manual's own
// (p.7-265): the letters a, f, d and w in the "Fault reaction code" column.
enum class FaultReaction
{
  kNone,                    // no reaction defined
  kAbortConnectionOption,   // 'a': use Abort connection option code (0x6007)
  kFaultReactionOption,     // 'f': use Fault reaction option code (0x605E)
  kNoSecureMovement,        // 'd': a secure movement is no longer possible
  kWarning                  // 'w': no effect on device status
};

const char * ToString(FaultReaction value);


// One row of Table 7-186 plus its detail section.
//
// Some entries cover a range (Generic initialization error is 0x1080 through
// 0x1088, Internal software error 0x6180 through 0x61F0). For a single code
// codeEnd equals code.
struct DeviceError
{
  std::uint16_t code;
  std::uint16_t codeEnd;
  const char * name;
  std::uint8_t errorRegister;
  FaultReaction faultReaction;

  // "Position Clear" in the manual: the position is cleared when the error is
  // reset, so an axis that was homed is no longer referenced and has to be
  // homed again before its position means anything.
  bool clearsPosition;

  const char * const * causes;
  std::size_t causeCount;
  const char * const * effects;
  std::size_t effectCount;
  const char * const * recovery;
  std::size_t recoveryCount;
};

// Returns nullptr for a code the manual does not list, which is itself worth
// reporting: it means the firmware is newer than this table.
const DeviceError * FindDeviceError(std::uint16_t code);

// Multi-line human readable form: name, register, cause, effect, recovery.
std::string DescribeDeviceError(std::uint16_t code);

// Single line, for logs that cannot take a paragraph.
std::string DeviceErrorName(std::uint16_t code);

// True when the manual marks the reaction 'w': the drive keeps running.
// Treating one of these as a fault would stop an axis that never stopped.
bool IsWarning(std::uint16_t code);

// True when resetting this error clears the position, i.e. homing is lost.
bool ClearsPosition(std::uint16_t code);

// Decodes the 0x1001 byte into names.
std::string DescribeErrorRegister(std::uint8_t value);


// ---------------------------------------------------------------------------
// SDO abort codes, Table 7-187 (p.7-285).
//
// Sent instead of a response when an SDO request fails. Codes above
// 0x0F000000 are maxon specific.
// ---------------------------------------------------------------------------
struct AbortCode
{
  std::uint32_t code;
  const char * name;
  const char * cause;
};

const AbortCode * FindAbortCode(std::uint32_t code);
std::string DescribeAbortCode(std::uint32_t code);


// ---------------------------------------------------------------------------
// Emergency message, Table 7-185 (p.7-265).
//
// Transmitted once per error event on COB-ID 0x80 + node-ID, without anybody
// asking. Bytes 3 to 7 are documented as always zero on the EPOS4, but are
// carried here because the frame has room for them and a future firmware may
// use them.
// ---------------------------------------------------------------------------
struct EmergencyMessage
{
  std::uint16_t errorCode;
  std::uint8_t errorRegister;
  std::uint8_t manufacturerSpecific[5];
};

std::string Describe(const EmergencyMessage & message);

}  // namespace epos4::signals
