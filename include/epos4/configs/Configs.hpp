#pragma once

#include <cstdint>
#include <optional>
#include <system_error>
#include <variant>
#include <vector>

#include "epos4/configs/ConfigFields.hpp"
#include "epos4/configs/SystemConfigs.hpp"
#include "epos4/core/ObjectDictionary.hpp"
#include "epos4/signals/Enums.hpp"

namespace epos4::configs
{


// ---------------------------------------------------------------------------
// Every field is optional on purpose.
//
// Only fields that were explicitly set are written to the drive. A config
// object with hardcoded defaults would silently overwrite controller gains
// and motor data that somebody spent a day tuning, the first time anybody
// called Apply() with a partially filled struct. Leaving a field unset means
// "do not touch", not "reset to zero".
// ---------------------------------------------------------------------------

// Motor data, 0x3001 and 0x3002. Section 6.2.53 / 6.2.54.
struct MotorConfigs
{
  std::optional<signals::MotorType> motorType;        // 0x6402
  std::optional<std::uint32_t> nominalCurrent;        // 0x3001:01 [mA]
  std::optional<std::uint32_t> outputCurrentLimit;    // 0x3001:02 [mA]
  std::optional<std::uint8_t> numberOfPolePairs;      // 0x3001:03
  std::optional<std::uint16_t> thermalTimeConstant;   // 0x3001:04 [0.1 s]
  std::optional<std::uint32_t> torqueConstant;        // 0x3001:05 [uNm/A]
  std::optional<std::uint32_t> electricalResistance;  // 0x3002:01 [mOhm]
  std::optional<std::uint16_t> electricalInductance;  // 0x3002:02 [uH], UNSIGNED16
  std::optional<std::uint32_t> ratedTorque;            // 0x6076, read-only [uNm]

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::At(od::cia402::kMotorType), s.motorType);
    v(od::maxon::kMotorData_NominalCurrent, s.nominalCurrent);
    v(od::maxon::kMotorData_OutputCurrentLimit, s.outputCurrentLimit);
    v(od::maxon::kMotorData_NumberOfPolePairs, s.numberOfPolePairs);
    v(od::maxon::kMotorData_ThermalTimeConstantWinding, s.thermalTimeConstant);
    v(od::maxon::kMotorData_TorqueConstant, s.torqueConstant);
    v(od::maxon::kElectricalSystemParameters_ElectricalResistance, s.electricalResistance);
    v(od::maxon::kElectricalSystemParameters_ElectricalInductance, s.electricalInductance);
    // Computed by the drive as nominal current x torque constant; "changing
    // the value by write access is not permitted" (6.2.112) although the
    // EDS lists it RW. Every torque object is in per mille of it.
    v.ReadOnly(od::At(od::cia402::kMotorRatedTorque), s.ratedTorque);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Gear configuration, 0x3003. Section 6.2.55.
struct GearConfigs
{
  std::optional<std::uint32_t> reductionNumerator;    // 0x3003:01
  std::optional<std::uint32_t> reductionDenominator;  // 0x3003:02
  std::optional<std::uint32_t> maxGearInputSpeed;     // 0x3003:03 [rpm]

  // 0x3003:04 bit 0 (Table 6-116): the gear output turns opposite to its
  // input. Set it for a gearbox with an odd number of stages, so positions
  // and velocities keep the sign of the joint rather than of the motor.
  std::optional<bool> invertedDirection;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kGearConfiguration_GearReductionNumerator, s.reductionNumerator);
    v(od::maxon::kGearConfiguration_GearReductionDenominator, s.reductionDenominator);
    v(od::maxon::kGearConfiguration_MaxGearInputSpeed, s.maxGearInputSpeed);
    v.template Packed<FlagCodec<std::uint32_t, 0>>(
      od::maxon::kGearConfiguration_GearMiscellaneousConfiguration, s.invertedDirection);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Axis configuration, 0x3000. Section 6.2.52. Selects which sensors exist and
// how the control loops are stacked; wrong values here make every other
// setting meaningless.
struct AxisConfigs
{
  std::optional<std::uint32_t> sensorsConfiguration;   // 0x3000:01
  std::optional<std::uint32_t> controlStructure;       // 0x3000:02
  std::optional<std::uint32_t> commutationSensors;     // 0x3000:03
  std::optional<std::uint32_t> miscellaneous;          // 0x3000:04
  std::optional<std::uint32_t> mainSensorResolution;   // 0x3000:05, read-only [inc/rev]
  std::optional<std::uint32_t> maxSystemSpeed;         // 0x3000:06, read-only [velocity units]

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kAxisConfiguration_SensorsConfiguration, s.sensorsConfiguration);
    v(od::maxon::kAxisConfiguration_ControlStructure, s.controlStructure);
    v(od::maxon::kAxisConfiguration_CommutationSensors, s.commutationSensors);
    v(od::maxon::kAxisConfiguration_AxisConfigurationMiscellaneous, s.miscellaneous);
    // Sub-indices 5 and 6 are computed by the drive from the sensors and the
    // motor (6.2.52.5-6): read back, never written.
    v.ReadOnly(od::maxon::kAxisConfiguration_MainSensorResolution, s.mainSensorResolution);
    v.ReadOnly(od::maxon::kAxisConfiguration_MaxSystemSpeed, s.maxSystemSpeed);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Current controller gains, 0x30A0. Section 6.2.61.
struct CurrentControlConfigs
{
  std::optional<std::uint32_t> p;  // 0x30A0:01 [uV/A]
  std::optional<std::uint32_t> i;  // 0x30A0:02 [uV/(A*ms)]

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kCurrentControlParameterSet_CurrentControllerPGain, s.p);
    v(od::maxon::kCurrentControlParameterSet_CurrentControllerIGain, s.i);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Position controller gains, 0x30A1. Section 6.2.62.
// The Phoenix equivalent of Slot0Configs, for the position loop.
struct PositionControlConfigs
{
  std::optional<std::uint32_t> p;                   // 0x30A1:01
  std::optional<std::uint32_t> i;                   // 0x30A1:02
  std::optional<std::uint32_t> d;                   // 0x30A1:03
  std::optional<std::uint32_t> feedForwardVelocity;      // 0x30A1:04
  std::optional<std::uint32_t> feedForwardAcceleration;  // 0x30A1:05

  // 0x30A1:09, the unit `i` is in (Tables 6-126/6-127). Firmware 0x0170 made
  // it selectable; the default micro-units top out at about 4295 A/(rad*s),
  // which a stiff joint on a high-resolution encoder can need more than.
  // Changing it rescales what `i` means - set both together.
  enum class IGainUnit : std::uint32_t
  {
    kMicroAmpPerRadianSecond = 0xFA040300,  // default
    kMilliAmpPerRadianSecond = 0xFD040300,
  };
  std::optional<IGainUnit> iGainUnit;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    // The unit first: the gain written after it is read in that unit.
    v(
      od::maxon::kPositionControlParameterSet_SIUnitPositionControllerIGain, s.iGainUnit,
      Presence::kFirmwareDependent);
    v(od::maxon::kPositionControlParameterSet_PositionControllerPGain, s.p);
    v(od::maxon::kPositionControlParameterSet_PositionControllerIGain, s.i);
    v(od::maxon::kPositionControlParameterSet_PositionControllerDGain, s.d);
    v(
      od::maxon::kPositionControlParameterSet_PositionControllerFFVelocityGain,
      s.feedForwardVelocity);
    v(
      od::maxon::kPositionControlParameterSet_PositionControllerFFAccelerationGain,
      s.feedForwardAcceleration);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Velocity controller gains, 0x30A2. Section 6.2.63.
struct VelocityControlConfigs
{
  std::optional<std::uint32_t> p;                       // 0x30A2:01
  std::optional<std::uint32_t> i;                       // 0x30A2:02
  std::optional<std::uint32_t> feedForwardVelocity;      // 0x30A2:03
  std::optional<std::uint32_t> feedForwardAcceleration;  // 0x30A2:04
  std::optional<std::uint16_t> filterCutOffFrequency;    // 0x30A2:05 [Hz], UNSIGNED16

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kVelocityControlParameterSet_VelocityControllerPGain, s.p);
    v(od::maxon::kVelocityControlParameterSet_VelocityControllerIGain, s.i);
    v(
      od::maxon::kVelocityControlParameterSet_VelocityControllerFFVelocityGain,
      s.feedForwardVelocity);
    v(
      od::maxon::kVelocityControlParameterSet_VelocityControllerFFAccelerationGain,
      s.feedForwardAcceleration);
    v(
      od::maxon::kVelocityControlParameterSet_VelocityControllerFilterCutOffFrequency,
      s.filterCutOffFrequency);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Velocity observer, 0x30A3. Section 6.2.64.
//
// The velocity the velocity loop acts on is not differentiated position: it
// is the estimate of a disturbance observer, a model of the axis corrected
// by the encoder. Its model terms - load gain, friction, inertia - are what
// Motion Studio's auto tuning identifies, and a mechanism very different
// from the tuned one (a payload on the arm's end) shows up here first.
struct VelocityObserverConfigs
{
  std::optional<std::uint32_t> positionCorrectionGain;  // 0x30A3:01 [per mille]
  std::optional<std::uint32_t> velocityCorrectionGain;  // 0x30A3:02 [mHz]
  std::optional<std::uint32_t> loadCorrectionGain;      // 0x30A3:03 [uNm/rad]
  std::optional<std::uint32_t> friction;                // 0x30A3:04 [0.001 uNm/rpm]
  std::optional<std::uint32_t> inertia;                 // 0x30A3:05 [0.001 g*cm^2]

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(
      od::maxon::kVelocityObserverParameterSet_VelocityObserverPositionCorrectionGain,
      s.positionCorrectionGain);
    v(
      od::maxon::kVelocityObserverParameterSet_VelocityObserverVelocityCorrectionGain,
      s.velocityCorrectionGain);
    v(
      od::maxon::kVelocityObserverParameterSet_VelocityObserverLoadCorrectionGain,
      s.loadCorrectionGain);
    v(od::maxon::kVelocityObserverParameterSet_VelocityObserverFriction, s.friction);
    v(od::maxon::kVelocityObserverParameterSet_VelocityObserverInertia, s.inertia);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Dual loop position control, 0x30AE. Section 6.2.65.
//
// The structure for a load encoder behind a gearbox with play: a P main loop
// on the load sensor, a gain scheduler between a low and a high bandwidth,
// an optional filter, and a PI velocity auxiliary loop with its own
// observer on the motor sensor. Only used when the control structure in
// AxisConfigs selects dual loop (Table 6-107).
struct DualLoopConfigs
{
  // Main (outer) loop. Gains in [10^-3 / s]; the scheduling weight in
  // [10^-3], 5000..20000.
  std::optional<std::uint32_t> mainPGainLowBandwidth;   // 0x30AE:01
  std::optional<std::uint32_t> mainPGainHighBandwidth;  // 0x30AE:02
  std::optional<std::uint16_t> mainGainSchedulingWeight;  // 0x30AE:03

  // Main loop filter, coefficients a..e in [10^-3] (0x30AE:10..14).
  //
  // The drive keeps using the old coefficients until bit 0 of 0x30AE:40 is
  // written, and bit 1 of that same object switches the filter on or off.
  // So writing coefficients means writing 0x40, and 0x40 cannot be written
  // without deciding bit 1: set filterActive whenever a coefficient is set
  // (Validate() refuses otherwise, rather than silently turning the filter
  // off). The update bit is added for you, after the coefficients.
  std::optional<std::uint32_t> filterA;
  std::optional<std::uint32_t> filterB;
  std::optional<std::uint32_t> filterC;
  std::optional<std::uint32_t> filterD;
  std::optional<std::uint32_t> filterE;
  std::optional<bool> filterActive;  // 0x30AE:40 bit 1

  // Auxiliary (inner) velocity loop, same units as VelocityControlConfigs.
  std::optional<std::uint32_t> auxPGain;                     // 0x30AE:20 [uA*s/rad]
  std::optional<std::uint32_t> auxIGain;                     // 0x30AE:21 [uA/rad]
  std::optional<std::uint32_t> auxFeedForwardVelocity;       // 0x30AE:22 [uA*s/rad]
  std::optional<std::uint32_t> auxFeedForwardAcceleration;   // 0x30AE:23 [uA*s^2/rad]

  // Auxiliary loop observer, same units as VelocityObserverConfigs.
  std::optional<std::uint32_t> auxObserverPositionCorrectionGain;  // 0x30AE:30
  std::optional<std::uint32_t> auxObserverVelocityCorrectionGain;  // 0x30AE:31
  std::optional<std::uint32_t> auxObserverLoadCorrectionGain;      // 0x30AE:32
  std::optional<std::uint32_t> auxObserverFriction;                // 0x30AE:33
  std::optional<std::uint32_t> auxObserverInertia;                 // 0x30AE:34

  std::error_code Validate() const;

  bool AnyFilterCoefficient() const
  {
    return filterA || filterB || filterC || filterD || filterE;
  }

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(
      od::maxon::kDualLoopPositionControlParameterSet_MainLoopPGainLowBandwidth,
      s.mainPGainLowBandwidth);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_MainLoopPGainHighBandwidth,
      s.mainPGainHighBandwidth);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_MainLoopGainSchedulingWeight,
      s.mainGainSchedulingWeight);
    v(od::maxon::kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientA, s.filterA);
    v(od::maxon::kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientB, s.filterB);
    v(od::maxon::kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientC, s.filterC);
    v(od::maxon::kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientD, s.filterD);
    v(od::maxon::kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientE, s.filterE);
    // After the coefficients, so the update it triggers picks them up. Bit 0
    // always reads back 0 (6.2.65.18), so only bit 1 is decoded.
    v.template Composite<std::uint16_t>(
      od::maxon::kDualLoopPositionControlParameterSet_DualLoopConfigurationMiscellaneous,
      [&s]() -> std::optional<std::uint16_t> {
        if (!s.filterActive) {return std::nullopt;}
        const unsigned active = *s.filterActive ? 0x2u : 0x0u;
        const unsigned update = s.AnyFilterCoefficient() ? 0x1u : 0x0u;
        return static_cast<std::uint16_t>(active | update);
      },
      [&s](auto word) {s.filterActive = (word & 0x2u) != 0u;});
    v(od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopPGain, s.auxPGain);
    v(od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopIGain, s.auxIGain);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopFFVelocityGain,
      s.auxFeedForwardVelocity);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopFFAccelerationGain,
      s.auxFeedForwardAcceleration);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverPositionCorrectionGain,
      s.auxObserverPositionCorrectionGain);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverVelocityCorrectionGain,
      s.auxObserverVelocityCorrectionGain);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverLoadCorrectionGain,
      s.auxObserverLoadCorrectionGain);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverFriction,
      s.auxObserverFriction);
    v(
      od::maxon::kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverInertia,
      s.auxObserverInertia);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Trajectory shape for the profiled modes, PPM and PVM.
struct MotionProfileConfigs
{
  std::optional<std::uint32_t> profileVelocity;      // 0x6081
  std::optional<std::uint32_t> profileAcceleration;  // 0x6083
  std::optional<std::uint32_t> profileDeceleration;  // 0x6084
  std::optional<std::uint32_t> quickStopDeceleration;  // 0x6085
  std::optional<signals::MotionProfileType> motionProfileType;  // 0x6086
  std::optional<std::uint32_t> maxProfileVelocity;   // 0x607F
  std::optional<std::uint32_t> maxAcceleration;      // 0x60C5

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::At(od::cia402::kProfileVelocity), s.profileVelocity);
    v(od::At(od::cia402::kProfileAcceleration), s.profileAcceleration);
    v(od::At(od::cia402::kProfileDeceleration), s.profileDeceleration);
    v(od::At(od::cia402::kQuickStopDeceleration), s.quickStopDeceleration);
    v(od::At(od::cia402::kMotionProfileType), s.motionProfileType);
    v(od::At(od::cia402::kMaxProfileVelocity), s.maxProfileVelocity);
    v(od::At(od::cia402::kMaxAcceleration), s.maxAcceleration);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Cyclic mode configuration, object 0x60C2. Section 6.2.138 (p.6-240).
//
// Only relevant to CSP and CSV, where the trajectory is generated by the
// master and the drive interpolates between the setpoints it receives.
//
// Kept apart from MotionProfileConfigs on purpose: the profile objects
// (0x6081, 0x6083, 0x6084) shape the ramp the DRIVE generates in PPM and PVM.
// This shapes how the drive fills in the gaps between setpoints the MASTER
// generates. Different modes, different concern; mixing them in one struct
// invites applying one while meaning the other.
// ---------------------------------------------------------------------------
struct CyclicConfigs
{
  // 0x60C2:01, [ms], 0 to 100.
  //
  // MUST match the master's synchronised PDO cycle - the SYNC period in the
  // network description. With bus.yml at sync_period: 10000 (microseconds),
  // this is 10.
  //
  // The manual on leaving it at 0: the drive "immediately takes the new set
  // value and adapts the position [...] within the next control cycle (i.e.
  // 0.4 ms). Afterwards it holds this set value until the next set value of
  // the master is received. This results in an interrupted and noisy motion
  // if the master just provides new set values at cycle rates of 1 ms, 2 ms,
  // or even lower."
  //
  // In other words: a joint that steps instead of moving. Set it wrong in the
  // other direction - larger than the real cycle - and the drive interpolates
  // over a window that never completes, which lags the commanded trajectory.
  std::optional<std::uint8_t> interpolationTimePeriodMs;

  // Sub-index 2, the time index, is deliberately absent. The manual fixes its
  // range at -3 to -3, so the unit is always 10^-3 s and there is nothing to
  // choose. Writing it would only be a chance to get it wrong.

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(
      od::cia402::kInterpolationTimePeriod_InterpolationTimePeriodValue,
      s.interpolationTimePeriodMs);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Factor group, objects 0x60A8, 0x60A9 and 0x60AA. Section 2.3 (p.2-17).
//
// Almost nothing here is a choice, and that is worth knowing before anyone
// goes looking for a way to make the drive report degrees:
//
//   0x60A8  position      ONE legal value, 0x00B50000 = increments
//   0x60AA  acceleration  ONE legal value, 0x00C00300 = rpm/s
//   0x60A9  velocity      rpm, with a prefix from 10^0 down to 10^-6
//
// So position is always quadcounts and acceleration always rpm/s, whatever
// is written. Converting to anything else is the driver's job - see
// core/UnitConversion.hpp - because it needs the encoder resolution and the
// gear ratio, which the drive does not know.
//
// The one real choice is the velocity prefix, which trades range for
// resolution: at milli-rpm a velocity reads to three more decimal places but
// the INTEGER32 that carries it runs out a thousand times sooner.
//
// Write access is only permitted while the motor has no power.
// ---------------------------------------------------------------------------
// 0x60A9 packs the velocity unit as Table 6-160 lays it out: prefix in bits
// 31..24, numerator 23..16, denominator 15..8. Only the prefix is a choice;
// numerator 0xB4 (revolutions) over denominator 0x47 (minute) is fixed.
struct VelocityPrefixCodec
{
  using Wire = std::uint32_t;
  static constexpr std::uint32_t kRevPerMin = 0x00B44700u;
  static constexpr Wire Encode(signals::VelocityPrefix prefix)
  {
    return (static_cast<std::uint32_t>(prefix) << 24) | kRevPerMin;
  }
  static constexpr signals::VelocityPrefix Decode(Wire value)
  {
    return static_cast<signals::VelocityPrefix>(value >> 24);
  }
};

struct SiUnitConfigs
{
  // 0x60A9. Only the prefix is variable; the unit is always rev/min.
  //   0x00 (none, plain rpm, the default), 0xFF deci, 0xFE centi, 0xFD milli,
  //   0xFC 1e-4, 0xFB 1e-5, 0xFA micro.
  std::optional<signals::VelocityPrefix> velocityPrefix;

  // Position and acceleration are not offered: there is nothing to choose,
  // and a setter that can only be given one value is an invitation to try
  // the other values the manual lists for CiA 402 but the EPOS4 rejects.

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    // Table 6-160: prefix in bits 31..24, numerator 23..16, denominator 15..8.
    // The unit is rev/min, so numerator 0xB4 (revolutions) over denominator
    // 0x47 (minute) - and those two never change on this device.
    v.template Packed<VelocityPrefixCodec>(od::At(od::cia402::kSIUnitVelocity), s.velocityPrefix);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Protective limits. These are what stop a mechanical problem from becoming a
// broken arm, so they are their own group rather than scattered.
struct LimitConfigs
{
  std::optional<std::int32_t> minPositionLimit;   // 0x607D:01
  std::optional<std::int32_t> maxPositionLimit;   // 0x607D:02
  std::optional<std::uint32_t> maxMotorSpeed;     // 0x6080 [rpm]
  std::optional<std::uint32_t> followingErrorWindow;  // 0x6065
  // 0x6066 [ms], read-only here: the manual fixes its range at 0..0
  // (6.2.106), so any other value aborts and 0 is already what it holds.
  std::optional<std::uint16_t> followingErrorTimeout;
  std::optional<std::uint32_t> positionWindow;    // 0x6067
  std::optional<std::uint16_t> positionWindowTime;  // 0x6068 [ms]

  // 0x607B «Position range limit» is deliberately absent: the EPOS4 does not
  // implement it (6.2.115) and aborts any value but 0 with 0x06090030. The
  // software limits above are the ones that work.

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::cia402::kSoftwarePositionLimit_MinPositionLimit, s.minPositionLimit);
    v(od::cia402::kSoftwarePositionLimit_MaxPositionLimit, s.maxPositionLimit);
    v(od::At(od::cia402::kMaxMotorSpeed), s.maxMotorSpeed);
    v(od::At(od::cia402::kFollowingErrorWindow), s.followingErrorWindow);
    v.ReadOnly(od::At(od::cia402::kFollowingErrorTimeOut), s.followingErrorTimeout);
    v(od::At(od::cia402::kPositionWindow), s.positionWindow);
    v(od::At(od::cia402::kPositionWindowTime), s.positionWindowTime);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Homing, section 3.5. Without this an incremental encoder has no absolute
// zero, so an arm has no idea where it is.
struct HomingConfigs
{
  std::optional<signals::HomingMethod> method;       // 0x6098
  std::optional<std::uint32_t> speedForSwitchSearch;  // 0x6099:01 [rpm]
  std::optional<std::uint32_t> speedForZeroSearch;    // 0x6099:02 [rpm]
  std::optional<std::uint32_t> acceleration;          // 0x609A
  std::optional<std::int32_t> homePosition;           // 0x30B0
  std::optional<std::int32_t> homeOffsetMoveDistance;  // 0x30B1
  std::optional<std::uint16_t> currentThreshold;       // 0x30B2 [mA], UNSIGNED16

  // 0x607C «Home offset» (firmware 0x0180+) is the same value as 0x30B0 under
  // its CiA 402 name (6.2.116). homePosition covers it on every firmware;
  // writing both would only be two chances to disagree.

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::At(od::cia402::kHomingMethod), s.method);
    v(od::cia402::kHomingSpeeds_SpeedForSwitchSearch, s.speedForSwitchSearch);
    v(od::cia402::kHomingSpeeds_SpeedForZeroSearch, s.speedForZeroSearch);
    v(od::At(od::cia402::kHomingAcceleration), s.acceleration);
    v(od::At(od::maxon::kHomePosition), s.homePosition);
    v(od::At(od::maxon::kHomeOffsetMoveDistance), s.homeOffsetMoveDistance);
    v(od::At(od::maxon::kCurrentThresholdForHomingMode), s.currentThreshold);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Holding brake, object 0x3158. Section 6.2.78 (p.6-193).
//
// A holding brake stops an axis drifting at standstill; it is NOT a brake for
// stopping a moving load - the controller does that. On an arm this is what
// keeps a joint from dropping its payload the moment power goes away.
//
// Three things from the manual that are easy to get wrong and expensive:
//
//  1. "The holding brake or the motor may be damaged if the holding brake
//     will activate before the motor has reached full standstill. Thus, it is
//     of vital importance to configure the standstill conditions." See
//     StandstillConfigs below - it is not optional decoration.
//
//  2. "The holding brake function will only work properly if a main sensor is
//     configured." Without feedback the drive cannot tell standstill from
//     slow motion, so it cannot know when it is safe to clamp.
//
//  3. The function has to be assigned to a physical output through
//     DigitalOutputConfigs. Setting the timings alone drives nothing.
//
// Timings come from the brake's data sheet: for permanent magnet brakes, the
// coupling time is usually called "reaction time closing" and the opening
// time "reaction time opening".
// ---------------------------------------------------------------------------
struct HoldingBrakeConfigs
{
  // 0x3158:01 [ms], 0..5000. Time from power-off until the brake reaches its
  // holding torque. The drive waits this long before considering the axis
  // held. Too short and the axis drops before the brake bites.
  // (Called "Holding brake rise time" in older EDS files.)
  std::optional<std::uint16_t> couplingTimeMs;

  // 0x3158:02 [ms], 0..5000. Time from power-on until the brake has released.
  // The drive waits this long before moving. Too short and the motor pulls
  // against a brake that is still clamped.
  // (Called "Holding brake fall time" in older EDS files.)
  std::optional<std::uint16_t> openingTimeMs;

  // 0x3158:04 [0.1 V], 0..600. Maximum voltage while opening.
  // 0x3158:05 [0.1 V]. Reduced voltage to keep it open afterwards, which is
  // what stops the brake coil cooking itself on a long hold.
  //
  // ONLY on EPOS4 Disk 60/8, Disk 60/12 and Module/Compact 60/20. On other
  // variants 0x3158 stops at sub-index 3 and writing these aborts with
  // "Subindex error". Leave them unset unless the hardware has them.
  std::optional<std::uint16_t> openingVoltageDeciVolt;
  std::optional<std::uint16_t> retainingVoltageDeciVolt;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kHoldingBrakeParameters_HoldingBrakeRiseTime, s.couplingTimeMs);
    v(od::maxon::kHoldingBrakeParameters_HoldingBrakeFallTime, s.openingTimeMs);
    // Sub-indices 4 and 5 exist only on some hardware variants, so they are
    // addressed literally rather than through a generated constant the EDS of
    // other variants does not contain - and a Refresh() on a drive without
    // them leaves them unset instead of failing.
    v(
      od::At(od::maxon::kHoldingBrakeParameters, 4), s.openingVoltageDeciVolt,
      Presence::kHardwareDependent);
    v(
      od::At(od::maxon::kHoldingBrakeParameters, 5), s.retainingVoltageDeciVolt,
      Presence::kHardwareDependent);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Standstill detection, object 0x30E0. Section 6.2.73 (p.6-185).
//
// Decides when the drive considers the axis stopped. The manual lists the
// state machine transitions that WAIT for this condition:
//
//     Operation enabled -> Switch on disabled
//     Operation enabled -> Ready to switch on
//     Operation enabled -> Switched on
//     Fault reaction active -> Fault
//
// So this is not only a brake concern: it is why Disable() can take longer
// than the three cycles the state machine needs. A window that is too tight
// on a noisy encoder means standstill is never declared and those transitions
// hang; too loose and the brake clamps on a still-turning shaft.
// ---------------------------------------------------------------------------
struct StandstillConfigs
{
  // 0x30E0:01 [velocity units], default 30. Symmetric band around zero.
  // The value 0xFFFFFFFF switches standstill detection off entirely and
  // standstill is deemed reached at the end of the trajectory - which is
  // exactly the assumption that damages a brake on an axis still coasting.
  std::optional<std::uint32_t> window;

  // 0x30E0:02 [ms], default 2. How long the velocity must stay inside the
  // window before standstill counts.
  std::optional<std::uint16_t> windowTimeMs;

  // 0x30E0:03 [ms]. Give up waiting for standstill after this long.
  std::optional<std::uint16_t> windowTimeoutMs;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kStandstillWindowConfiguration_StandstillWindow, s.window);
    v(od::maxon::kStandstillWindowConfiguration_StandstillWindowTime, s.windowTimeMs);
    v(od::maxon::kStandstillWindowConfiguration_StandstillWindowTimeout, s.windowTimeoutMs);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Digital inputs, objects 0x3141 and 0x3142. Sections 6.2.74 / 6.2.75.
//
// This is what makes homing possible: most of the methods in Table 3-26 need
// a home switch or a limit switch, and a switch that is wired but not mapped
// does nothing at all.
//
// The manual states two rules, and Validate() below enforces both rather
// than letting the drive reject the writes one at a time:
//
//   "Each function can only be mapped once, each digital input can only hold
//    one function."
//
// Plus a note of Table 6-131: touch probe is not available on high-speed
// inputs 1 and 3.
//
// And one interaction that is easy to lose an afternoon to:
//
//   "If a sensor 2 is configured, the high-speed digital inputs 1 to 4 will
//    be disabled. This configuration cannot be overridden as long as sensor
//    2 is configured."
//
// So a second encoder and the high-speed inputs are mutually exclusive: they
// share the pins.
//
// Device defaults, Table 6-130:
//   DgIn1 negative limit switch, DgIn2 positive limit switch,
//   DgIn3 home switch, DgIn4 general purpose D, high-speed inputs none.
// ---------------------------------------------------------------------------
struct DigitalInputConfigs
{
  std::optional<signals::DigitalInputFunction> input1;           // 0x3142:01
  std::optional<signals::DigitalInputFunction> input2;           // 0x3142:02
  std::optional<signals::DigitalInputFunction> input3;           // 0x3142:03
  std::optional<signals::DigitalInputFunction> input4;           // 0x3142:04
  std::optional<signals::DigitalInputFunction> highSpeedInput1;  // 0x3142:05
  std::optional<signals::DigitalInputFunction> highSpeedInput2;  // 0x3142:06
  std::optional<signals::DigitalInputFunction> highSpeedInput3;  // 0x3142:07
  std::optional<signals::DigitalInputFunction> highSpeedInput4;  // 0x3142:08

  // 0x3141:02. A bit set to 0 means that pin is high active. Bit positions
  // follow Table 6-129, i.e. the PIN order, not the function order.
  //
  // Worth getting right before trusting a limit switch: an inverted normally
  // closed switch reads as "not triggered" exactly when the wire breaks,
  // which is the failure a limit switch exists to catch.
  std::optional<std::uint16_t> polarity;

  // Returns an error if the same function is mapped to two inputs, which the
  // manual forbids. kNone may of course repeat.
  std::error_code Validate() const;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    // Polarity first: it decides how a pin's level is interpreted, so it
    // should be in force before a function starts acting on that pin.
    v(od::maxon::kDigitalInputProperties_DigitalInputsPolarity, s.polarity);
    v(od::At(od::maxon::kConfigurationOfDigitalInputs, 1), s.input1);
    v(od::At(od::maxon::kConfigurationOfDigitalInputs, 2), s.input2);
    v(od::At(od::maxon::kConfigurationOfDigitalInputs, 3), s.input3);
    v(od::At(od::maxon::kConfigurationOfDigitalInputs, 4), s.input4);
    v(od::At(od::maxon::kConfigurationOfDigitalInputs, 5), s.highSpeedInput1);
    v(od::At(od::maxon::kConfigurationOfDigitalInputs, 6), s.highSpeedInput2);
    v(od::At(od::maxon::kConfigurationOfDigitalInputs, 7), s.highSpeedInput3);
    v(od::At(od::maxon::kConfigurationOfDigitalInputs, 8), s.highSpeedInput4);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// Digital outputs, objects 0x3150 and 0x3151. Sections 6.2.76 / 6.2.77.
//
// This is where the holding brake function gets attached to a physical pin.
// Check the controller's Hardware Reference for the output current limit
// before wiring a brake coil to one.
// ---------------------------------------------------------------------------
struct DigitalOutputConfigs
{
  std::optional<signals::DigitalOutputFunction> output1;           // 0x3151:01
  std::optional<signals::DigitalOutputFunction> output2;           // 0x3151:02
  std::optional<signals::DigitalOutputFunction> highSpeedOutput1;  // 0x3151:03

  // 0x3151:04, ONLY on EPOS4 Disk 60/8 and Disk 60/12, and only kHoldingBrake
  // or kNone (Table 6-135): that output exists to drive a brake.
  std::optional<signals::DigitalOutputFunction> highSpeedOutput2;

  // 0x3150:02. A bit set to 1 inverts that output, so a logical "1" drives
  // the pin low. Brake wiring is commonly inverted, and getting this backwards
  // means the brake releases exactly when it should clamp.
  std::optional<std::uint16_t> polarity;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kConfigurationOfDigitalOutputs_DigitalOutput1Configuration, s.output1);
    v(od::maxon::kConfigurationOfDigitalOutputs_DigitalOutput2Configuration, s.output2);
    v(
      od::maxon::kConfigurationOfDigitalOutputs_HighSpeedDigitalOutput1Configuration,
      s.highSpeedOutput1);
    v(
      od::At(od::maxon::kConfigurationOfDigitalOutputs, 4), s.highSpeedOutput2,
      Presence::kHardwareDependent);
    v(od::maxon::kDigitalOutputProperties_DigitalOutputsPolarity, s.polarity);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// How the drive comes to rest in each situation. On a loaded arm the
// difference between cutting power and ramping down is the difference between
// a controlled stop and a dropped payload.
struct StopOptionConfigs
{
  std::optional<signals::QuickStopOption> quickStop;              // 0x605A
  std::optional<signals::ShutdownOption> shutdown;                // 0x605B
  std::optional<signals::DisableOperationOption> disableOperation;  // 0x605C
  std::optional<signals::FaultReactionOption> faultReaction;      // 0x605E
  std::optional<signals::AbortConnectionOption> abortConnectionOption;  // 0x6007
  std::optional<signals::HaltOption> halt;                        // 0x605D, firmware >= 0x0180

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::At(od::cia402::kQuickStopOptionCode), s.quickStop);
    v(od::At(od::cia402::kShutdownOptionCode), s.shutdown);
    v(od::At(od::cia402::kDisableOperationOptionCode), s.disableOperation);
    v(od::At(od::cia402::kFaultReactionOptionCode), s.faultReaction);
    v(od::At(od::cia402::kAbortConnectionOptionCode), s.abortConnectionOption);
    v(od::At(od::cia402::kHaltOptionCode), s.halt, Presence::kFirmwareDependent);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// ---------------------------------------------------------------------------
// The aggregate, equivalent to TalonFXConfiguration.
// ---------------------------------------------------------------------------
// Whether the manual allows writing this object only in «Power Disable»:
// all of 0x3000 (6.2.52), pole pairs 0x3001:03 (6.2.53.3), 0x3003 except
// the max gear input speed (6.2.55), the encoder objects (6.2.56-6.2.60)
// and the SI units 0x60A8-0x60AA (6.2.128-130). The Configurator refuses a
// set of writes containing one while the motor is powered, before the
// first write, rather than letting the drive abort it halfway.
bool RequiresPowerDisabled(od::Entry entry);


struct Epos4Configuration
{
  MotorConfigs motor;
  GearConfigs gear;
  AxisConfigs axis;
  CurrentControlConfigs currentControl;
  PositionControlConfigs positionControl;
  VelocityControlConfigs velocityControl;
  VelocityObserverConfigs velocityObserver;
  DualLoopConfigs dualLoop;
  MotionProfileConfigs motionProfile;
  LimitConfigs limits;
  HomingConfigs homing;
  StopOptionConfigs stopOptions;
  HoldingBrakeConfigs holdingBrake;
  StandstillConfigs standstill;
  CyclicConfigs cyclic;
  SiUnitConfigs siUnits;
  DigitalInputConfigs digitalInputs;
  DigitalOutputConfigs digitalOutputs;
  AnalogInputConfigs analogInputs;
  AnalogOutputConfigs analogOutputs;
  ProtectionConfigs protection;
  CustomPersistentMemoryConfigs customMemory;
  CommunicationConfigs communication;

  ConfigWrites ToWrites() const;

  // The checks a group makes before its own Apply() - input mappings - run
  // over the whole configuration, so a bad mapping is refused before the
  // first write rather than after half the drive is configured.
  std::error_code Validate() const;

  // Fills every group from the drive - everything ToWrites() could write.
  // Keeps going past a failure and returns the first one; fields that could
  // not be read stay unset.
  std::error_code ReadFrom(const ConfigReader & read);
};

}  // namespace epos4::configs
