#pragma once

#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

#include "epos4/core/ObjectDictionary.hpp"
#include "epos4/signals/Enums.hpp"

namespace epos4::configs
{

// A single pending object write. The Configurator turns a configuration
// object into a list of these and pushes them over SDO.
struct ConfigWrite
{
  od::Entry entry;
  std::variant<std::int8_t, std::int16_t, std::int32_t,
    std::uint8_t, std::uint16_t, std::uint32_t> value;
};

using ConfigWrites = std::vector<ConfigWrite>;


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
  std::optional<std::uint32_t> electricalResistance;  // 0x3002:01 [uOhm]
  std::optional<std::uint32_t> electricalInductance;  // 0x3002:02 [uH]

  void AppendTo(ConfigWrites & out) const;
};


// Gear configuration, 0x3003. Section 6.2.55.
struct GearConfigs
{
  std::optional<std::uint32_t> reductionNumerator;    // 0x3003:01
  std::optional<std::uint32_t> reductionDenominator;  // 0x3003:02
  std::optional<std::uint32_t> maxGearInputSpeed;     // 0x3003:03 [rpm]

  void AppendTo(ConfigWrites & out) const;
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
  std::optional<std::uint32_t> mainSensorResolution;   // 0x3000:05
  std::optional<std::uint32_t> maxSystemSpeed;         // 0x3000:06 [rpm]

  void AppendTo(ConfigWrites & out) const;
};


// Current controller gains, 0x30A0. Section 6.2.61.
struct CurrentControlConfigs
{
  std::optional<std::uint32_t> p;  // 0x30A0:01 [uV/A]
  std::optional<std::uint32_t> i;  // 0x30A0:02 [uV/(A*ms)]

  void AppendTo(ConfigWrites & out) const;
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

  void AppendTo(ConfigWrites & out) const;
};


// Velocity controller gains, 0x30A2. Section 6.2.63.
struct VelocityControlConfigs
{
  std::optional<std::uint32_t> p;                       // 0x30A2:01
  std::optional<std::uint32_t> i;                       // 0x30A2:02
  std::optional<std::uint32_t> feedForwardVelocity;      // 0x30A2:03
  std::optional<std::uint32_t> feedForwardAcceleration;  // 0x30A2:04
  std::optional<std::uint32_t> filterCutOffFrequency;    // 0x30A2:05 [Hz]

  void AppendTo(ConfigWrites & out) const;
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

  void AppendTo(ConfigWrites & out) const;
};


// Protective limits. These are what stop a mechanical problem from becoming a
// broken arm, so they are their own group rather than scattered.
struct LimitConfigs
{
  std::optional<std::int32_t> minPositionLimit;   // 0x607D:01
  std::optional<std::int32_t> maxPositionLimit;   // 0x607D:02
  std::optional<std::uint32_t> maxMotorSpeed;     // 0x6080 [rpm]
  std::optional<std::uint32_t> followingErrorWindow;  // 0x6065
  std::optional<std::uint16_t> followingErrorTimeout;  // 0x6066 [ms]
  std::optional<std::uint32_t> positionWindow;    // 0x6067
  std::optional<std::uint16_t> positionWindowTime;  // 0x6068 [ms]

  void AppendTo(ConfigWrites & out) const;
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
  std::optional<std::int16_t> currentThreshold;        // 0x30B2 [mA]

  void AppendTo(ConfigWrites & out) const;
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

  void AppendTo(ConfigWrites & out) const;
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

  void AppendTo(ConfigWrites & out) const;
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

  // 0x3150:02. A bit set to 1 inverts that output, so a logical "1" drives
  // the pin low. Brake wiring is commonly inverted, and getting this backwards
  // means the brake releases exactly when it should clamp.
  std::optional<std::uint16_t> polarity;

  void AppendTo(ConfigWrites & out) const;
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
  std::optional<std::int16_t> abortConnectionOption;              // 0x6007

  void AppendTo(ConfigWrites & out) const;
};


// ---------------------------------------------------------------------------
// The aggregate, equivalent to TalonFXConfiguration.
// ---------------------------------------------------------------------------
struct Epos4Configuration
{
  MotorConfigs motor;
  GearConfigs gear;
  AxisConfigs axis;
  CurrentControlConfigs currentControl;
  PositionControlConfigs positionControl;
  VelocityControlConfigs velocityControl;
  MotionProfileConfigs motionProfile;
  LimitConfigs limits;
  HomingConfigs homing;
  StopOptionConfigs stopOptions;
  HoldingBrakeConfigs holdingBrake;
  StandstillConfigs standstill;
  DigitalOutputConfigs digitalOutputs;

  ConfigWrites ToWrites() const;
};

}  // namespace epos4::configs
