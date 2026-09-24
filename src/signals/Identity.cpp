#include "epos4/signals/Identity.hpp"

#include <cstdio>

namespace epos4::signals
{

const char *
HardwareName(std::uint16_t hardwareVersion)
{
  // Table 6-66, Definition of hardware version.
  switch (hardwareVersion) {
    case 0x6A50: return "EPOS4 Disk 60/8";
    case 0x6B50: return "EPOS4 Disk 60/12";
    case 0x6851: return "EPOS4 Micro 24/1.5 CAN";
    case 0x6951: return "EPOS4 Micro 24/1.5 EtherCAT";
    case 0x6850: return "EPOS4 Micro 24/5 CAN";
    case 0x6950: return "EPOS4 Micro 24/5 EtherCAT";
    case 0x6050: return "EPOS4 Module/Compact 24/1.5";
    case 0x6150: return "EPOS4 Module/Compact 50/5";
    case 0x6551: return "EPOS4 Module/Compact 50/8";
    case 0x6552: return "EPOS4 Module/Compact 50/15";
    case 0x6553: return "EPOS4 Module/Compact 60/20";
    case 0x6350: return "EPOS4 50/5";
    case 0x6450: return "EPOS4 70/15";
    default: return nullptr;
  }
}

std::string
FirmwareName(const DeviceIdentity & identity)
{
  char buffer[48];
  std::snprintf(
    buffer, sizeof(buffer), "EPOS4_%04Xh_%04Xh_%04Xh_%04Xh",
    identity.SoftwareVersion(), identity.HardwareVersion(),
    identity.ApplicationNumber(), identity.ApplicationVersion());
  return buffer;
}

std::string
Describe(const DeviceIdentity & identity)
{
  if (!identity.IsMaxon()) {
    char buffer[96];
    std::snprintf(
      buffer, sizeof(buffer),
      "not a maxon device: vendor 0x%08X, product code 0x%08X",
      static_cast<unsigned>(identity.vendorId),
      static_cast<unsigned>(identity.productCode));
    return buffer;
  }

  const char * hardware = HardwareName(identity.HardwareVersion());
  char buffer[160];
  std::snprintf(
    buffer, sizeof(buffer), "%s, firmware %s, serial %08u",
    hardware ? hardware : "unknown maxon hardware (not in Table 6-66)",
    FirmwareName(identity).c_str(), static_cast<unsigned>(identity.serialNumber));
  return buffer;
}

}  // namespace epos4::signals
