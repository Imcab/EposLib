#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Library version, semantic versioning.
//
// Pre-1.0 on purpose: the API may still change, and nothing here has been
// exercised against a physical EPOS4 yet - only against the simulator in
// tools/epos4_sim.cpp. Until a real drive has moved under it, the major
// version stays at 0 and the minor version is allowed to break things.
//
// What 1.0.0 will mean: verified against hardware, with the cyclic path and
// homing exercised on a real axis.
//
// Compile-time comparison:
//   #if EPOS4_VERSION >= EPOS4_VERSION_ENCODE(0, 2, 0)
// ---------------------------------------------------------------------------

#define EPOS4_VERSION_MAJOR 0
#define EPOS4_VERSION_MINOR 1
#define EPOS4_VERSION_PATCH 0

#define EPOS4_VERSION_ENCODE(major, minor, patch) \
  ((major) * 10000 + (minor) * 100 + (patch))

#define EPOS4_VERSION \
  EPOS4_VERSION_ENCODE(EPOS4_VERSION_MAJOR, EPOS4_VERSION_MINOR, EPOS4_VERSION_PATCH)

#define EPOS4_VERSION_STRING "0.1.0"

namespace epos4
{

struct Version
{
  std::uint16_t major;
  std::uint16_t minor;
  std::uint16_t patch;
};

// Version of the library this binary was linked against, which is not
// necessarily the one the caller was compiled against. Worth logging at
// startup on a rover, where the arm software and the driver are not always
// rebuilt together.
Version GetVersion();

const char * GetVersionString();

// The EPOS4 firmware edition these tables were transcribed from. Object
// indices and error codes come from this document; a drive running a
// different firmware may not match.
constexpr const char * kFirmwareSpecificationEdition = "2026-07, rel13740";

}  // namespace epos4
