#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <system_error>

#include "epos4/configs/ConfigFields.hpp"
#include "epos4/core/ObjectDictionary.hpp"
#include "epos4/signals/Enums.hpp"

namespace epos4::configs
{

// ---------------------------------------------------------------------------
// The device around the axis: how it talks, what it protects itself from,
// and its analog I/O. Configs.hpp holds the motion side.
//
// Same rules as there: every field optional, unset means "do not touch", and
// each group's Visit() is the single list Apply() writes and Refresh() reads.
// ---------------------------------------------------------------------------


// One heartbeat this drive watches for, 0x1016:01/02 (6.2.9). Packed as
// Node-ID in bits 23..16 and the time in 15..0; a time of 0 disables the
// entry.
struct HeartbeatConsumer
{
  std::uint8_t nodeId{0};
  std::uint16_t timeMs{0};
};

struct HeartbeatConsumerCodec
{
  using Wire = std::uint32_t;
  static constexpr Wire Encode(const HeartbeatConsumer & c)
  {
    return (static_cast<Wire>(c.nodeId) << 16) | c.timeMs;
  }
  static constexpr HeartbeatConsumer Decode(Wire value)
  {
    return HeartbeatConsumer{
      static_cast<std::uint8_t>((value >> 16) & 0xFFu),
      static_cast<std::uint16_t>(value & 0xFFFFu)};
  }
};


// ---------------------------------------------------------------------------
// Communication, 0x1005-0x1029 and 0x2000-0x2006. Sections 6.2.4-6.2.44.
//
// Two warnings before using this group:
//
//  1. Node-ID and the bit rates "only come into effect after restart", and
//     only if saved first (Configurator::Save()). Change the Node-ID of a
//     drive the master's DCF describes and, after the restart, the master
//     no longer finds it. They are written last for the same reason: every
//     other write still reaches the drive under its current identity.
//
//  2. The master's DCF also configures the heartbeat at boot (Lely writes
//     the slave's 0x1016/0x1017 from it). What is set here holds until the
//     next boot reconfigures the node; the DCF is the place for the
//     network's permanent heartbeat.
// ---------------------------------------------------------------------------
struct CommunicationConfigs
{
  // 0x1017 [ms]. How often the drive announces itself; 0 = never.
  std::optional<std::uint16_t> producerHeartbeatMs;

  // 0x1016:01/02. The manual recommends a consumer time at least 20 ms above
  // the producer's period, so one late frame is not a lost master.
  std::optional<HeartbeatConsumer> heartbeatConsumer1;
  std::optional<HeartbeatConsumer> heartbeatConsumer2;

  // 0x1029:01. What the NMT state does on a heartbeat loss.
  std::optional<signals::CommunicationErrorBehavior> communicationErrorBehavior;

  // 0x2006 / 0x2005 [ms], 50..65535. Gap after which a partial frame on the
  // service interface is dropped. RS232 is absent on the EtherCAT variants.
  std::optional<std::uint16_t> usbFrameTimeoutMs;
  std::optional<std::uint16_t> rs232FrameTimeoutMs;
  std::optional<signals::Rs232BitRate> rs232BitRate;  // 0x2002, after restart

  // 0x2001 and 0x2000, after save and restart - see warning 1. Node-ID is
  // 1..127; 255 is the "unconfigured" factory value.
  std::optional<signals::CanBitRate> canBitRate;
  std::optional<std::uint8_t> nodeId;

  // 0x1005 / 0x1014. Read-only here: the EDS marks the SYNC COB-ID constant,
  // and the EMCY one follows the Node-ID. Useful to confirm what a drive
  // listens and talks on.
  std::optional<std::uint32_t> syncCobId;
  std::optional<std::uint32_t> emcyCobId;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::At(od::comm::kProducerHeartbeatTime), s.producerHeartbeatMs);
    v.template Packed<HeartbeatConsumerCodec>(
      od::comm::kConsumerHeartbeatTime_Consumer1HeartbeatTime, s.heartbeatConsumer1);
    v.template Packed<HeartbeatConsumerCodec>(
      od::comm::kConsumerHeartbeatTime_Consumer2HeartbeatTime, s.heartbeatConsumer2);
    v(od::comm::kErrorBehavior_CommunicationError, s.communicationErrorBehavior);
    v(od::At(od::maxon_comm::kUSBFrameTimeout), s.usbFrameTimeoutMs);
    v(
      od::At(od::maxon_comm::kRS232FrameTimeout), s.rs232FrameTimeoutMs,
      Presence::kHardwareDependent);
    v(od::At(od::maxon_comm::kRS232BitRate), s.rs232BitRate, Presence::kHardwareDependent);
    v(od::At(od::maxon_comm::kCANBitRate), s.canBitRate);
    v(od::At(od::maxon_comm::kNodeID), s.nodeId);
    v.ReadOnly(od::At(od::comm::kCOBIDSYNC), s.syncCobId);
    v.ReadOnly(od::At(od::comm::kCOBIDEMCY), s.emcyCobId);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Self-protection limits: supply voltage (0x2201, 6.2.51) and power stage
// temperature (0x3201, 6.2.89).
//
// The defaults depend on the hardware variant (Tables 6-99, 6-101, 6-141)
// and are already the safe ones. The reason to touch them is the supply: on a
// battery, raising the undervoltage limit to the pack's cut-off makes the
// drive fault cleanly before the battery's own protection drops the bus.
// ---------------------------------------------------------------------------
struct ProtectionConfigs
{
  // 0x2201:01 / :02 [mV]. Undervoltage is written first: "if the
  // undervoltage limit is set higher than the overvoltage limit, the
  // overvoltage limit will be adjusted accordingly", so that order is valid
  // from any starting point, where the reverse can be refused.
  std::optional<std::uint32_t> undervoltageLimitMv;
  std::optional<std::uint32_t> overvoltageLimitMv;

  // 0x3201:04 [0.1 degC]. Above it the drive faults with 0x4210. Note the
  // manual marks it Backup NO: Save() does not keep it, so it is back at the
  // default after every power cycle and must be applied each boot.
  std::optional<std::uint16_t> maxPowerStageTemperatureDeciC;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon_comm::kPowerSupplySupervision_PowerSupplyUndervoltageLimit, s.undervoltageLimitMv);
    v(od::maxon_comm::kPowerSupplySupervision_PowerSupplyOvervoltageLimit, s.overvoltageLimitMv);
    v(
      od::maxon::kThermalOverloadProtection_MaximalTemperaturePowerStage,
      s.maxPowerStageTemperatureDeciC);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Custom persistent memory, 0x210C (6.2.49). Four words the drive stores for
// the application and never interprets - a calibration, a joint ID, the
// date the arm was last zeroed. Kept across power cycles once Save()d.
// ---------------------------------------------------------------------------
struct CustomPersistentMemoryConfigs
{
  std::array<std::optional<std::uint32_t>, 4> words;  // 0x210C:01..04

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    for (std::uint8_t n = 0; n < s.words.size(); ++n) {
      v(
        od::At(
          od::maxon_comm::kCustomPersistentMemory,
          static_cast<std::uint8_t>(n + 1)), s.words[n]);
    }
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Analog inputs: function (0x3161), calibration (0x3163) and the scaling of
// an input used as a set value (0x3170, 0x3171). Sections 6.2.80-6.2.84.
//
// An input that commands current or velocity takes over from the bus: the
// manual notes that in CSV with a velocity input the velocity offset 0x60B1
// is forced to 0, and likewise the torque offset in CST.
//
// The scaling and calibration are written BEFORE the functions. Assigning
// kCurrentSetValue first would make the input command current through
// whatever slope the drive held until then.
// ---------------------------------------------------------------------------
struct AnalogInputConfigs
{
  // 0x3163 [mV] offset, -1000..1000, and gain [1/10000], 5000..20000:
  // corrections applied to the raw input before anything else reads it.
  std::optional<std::int16_t> input1OffsetMv;
  std::optional<std::uint16_t> input1Gain;
  std::optional<std::int16_t> input2OffsetMv;
  std::optional<std::uint16_t> input2Gain;

  // 0x3170 / 0x3171. Two points of a line from input voltage [mV] to the set
  // value: current in [mA], velocity in velocity units. Only used by the
  // input carrying that function.
  std::optional<std::int32_t> currentFirstVoltageMv;
  std::optional<std::int32_t> currentFirstMa;
  std::optional<std::int32_t> currentSecondVoltageMv;
  std::optional<std::int32_t> currentSecondMa;
  std::optional<std::int32_t> velocityFirstVoltageMv;
  std::optional<std::int32_t> velocityFirst;
  std::optional<std::int32_t> velocitySecondVoltageMv;
  std::optional<std::int32_t> velocitySecond;

  // 0x3161:01 / :02.
  std::optional<signals::AnalogInputFunction> input1;
  std::optional<signals::AnalogInputFunction> input2;

  // The manual: "each function can only be mapped once". kNone is exempt.
  // Apply() checks this before writing anything.
  std::error_code Validate() const;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kAnalogInputAdjustment_AnalogInput1AdjustmentOffset, s.input1OffsetMv);
    v(od::maxon::kAnalogInputAdjustment_AnalogInput1AdjustmentGainFactor, s.input1Gain);
    v(od::maxon::kAnalogInputAdjustment_AnalogInput2AdjustmentOffset, s.input2OffsetMv);
    v(od::maxon::kAnalogInputAdjustment_AnalogInput2AdjustmentGainFactor, s.input2Gain);

    constexpr auto kCurrent = od::maxon::kAnalogInputCurrentSetValueProperties;
    constexpr auto kVelocity = od::maxon::kAnalogInputVelocitySetValueProperties;
    v(od::At(kCurrent, 1), s.currentFirstVoltageMv);
    v(od::At(kCurrent, 2), s.currentFirstMa);
    v(od::At(kCurrent, 3), s.currentSecondVoltageMv);
    v(od::At(kCurrent, 4), s.currentSecondMa);
    v(od::At(kVelocity, 1), s.velocityFirstVoltageMv);
    v(od::At(kVelocity, 2), s.velocityFirst);
    v(od::At(kVelocity, 3), s.velocitySecondVoltageMv);
    v(od::At(kVelocity, 4), s.velocitySecond);

    v(od::maxon::kConfigurationOfAnalogInputs_AnalogInput1Configuration, s.input1);
    v(od::maxon::kConfigurationOfAnalogInputs_AnalogInput2Configuration, s.input2);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Analog outputs, 0x3181 (6.2.86). Output 2 does not exist on the Disk 60/8,
// Disk 60/12 and Micro variants.
struct AnalogOutputConfigs
{
  std::optional<signals::AnalogOutputFunction> output1;  // 0x3181:01
  std::optional<signals::AnalogOutputFunction> output2;  // 0x3181:02

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kConfigurationOfAnalogOutputs_AnalogOutput1Configuration, s.output1);
    v(
      od::maxon::kConfigurationOfAnalogOutputs_AnalogOutput2Configuration, s.output2,
      Presence::kHardwareDependent);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};

}  // namespace epos4::configs
