#pragma once

#include <cstdint>

namespace epos4::signals
{

// ---------------------------------------------------------------------------
// Drive state, Table 2-5 (p.2-15).
//
// The enumerator values are the Statusword patterns with kStateMask applied.
// Bits 7 (Warning) and 4 (Voltage enabled) are marked 'x' in every row of the
// table, so they are masked off before comparing: a drive that is energised
// and carrying a warning is still in Operation enabled.
// ---------------------------------------------------------------------------
enum class State : std::uint16_t
{
  kNotReadyToSwitchOn = 0x00,
  kSwitchOnDisabled = 0x40,
  kReadyToSwitchOn = 0x21,
  kSwitchedOn = 0x23,
  kOperationEnabled = 0x27,
  kQuickStopActive = 0x07,
  kFaultReactionActive = 0x0F,
  kFault = 0x08
};

const char * ToString(State value);


// ---------------------------------------------------------------------------
// Operating mode, object 0x6060 / 0x6061, Table 6-154 (p.6-220).
// ---------------------------------------------------------------------------
enum class OperationMode : std::int8_t
{
  kNone = 0,
  kProfilePosition = 1,           // PPM
  kProfileVelocity = 3,           // PVM
  kHoming = 6,                    // HMM
  kCyclicSynchronousPosition = 8,  // CSP
  kCyclicSynchronousVelocity = 9,  // CSV
  kCyclicSynchronousTorque = 10    // CST
};

const char * ToString(OperationMode value);


// ---------------------------------------------------------------------------
// Statusword bits, Table 6-148 (p.6-216).
//
// Bits 10, 12 and 13 are deliberately absent: their meaning depends on the
// active operating mode, so they are exposed per mode instead (see the
// mode-specific status accessors on the device). Bits 8 and 14 are reserved.
// ---------------------------------------------------------------------------
namespace status_bits
{
constexpr std::uint16_t kStateMask = 0x6F;

constexpr std::uint16_t kReadyToSwitchOn = 1u << 0;
constexpr std::uint16_t kSwitchedOn = 1u << 1;
constexpr std::uint16_t kOperationEnabled = 1u << 2;
constexpr std::uint16_t kFault = 1u << 3;
constexpr std::uint16_t kVoltageEnabled = 1u << 4;
constexpr std::uint16_t kQuickStop = 1u << 5;
constexpr std::uint16_t kSwitchOnDisabled = 1u << 6;
constexpr std::uint16_t kWarning = 1u << 7;
constexpr std::uint16_t kRemote = 1u << 9;          // NMT is Operational
constexpr std::uint16_t kInternalLimit = 1u << 11;  // I2t / current / speed
constexpr std::uint16_t kHomeRefValid = 1u << 15;   // referenced to home

// Mode specific, meaning depends on 0x6061.
constexpr std::uint16_t kTargetReached = 1u << 10;        // PPM, PVM, HMM
constexpr std::uint16_t kSetpointAcknowledge = 1u << 12;  // PPM
constexpr std::uint16_t kSpeed = 1u << 12;                // PVM
constexpr std::uint16_t kHomingAttained = 1u << 12;       // HMM
constexpr std::uint16_t kFollowsCommandValue = 1u << 12;  // CSP, CSV, CST
constexpr std::uint16_t kFollowingError = 1u << 13;       // PPM, CSP
constexpr std::uint16_t kHomingError = 1u << 13;          // HMM
}  // namespace status_bits


// ---------------------------------------------------------------------------
// Controlword bits, Table 6-147 (p.6-215).
// ---------------------------------------------------------------------------
namespace control_bits
{
constexpr std::uint16_t kSwitchOn = 1u << 0;
constexpr std::uint16_t kEnableVoltage = 1u << 1;
constexpr std::uint16_t kEnableOperation = 1u << 3;

// CAREFUL - bit 2 is active low: 1 during normal operation, driven to 0 to
// trigger a quick stop. Table 2-7: «Quick stop» = 0xxx x01x
constexpr std::uint16_t kQuickStop = 1u << 2;

// CAREFUL - bit 7 is EDGE triggered (0->1). Holding it high makes the next
// fault impossible to clear.
constexpr std::uint16_t kFaultReset = 1u << 7;

// Mode specific.
constexpr std::uint16_t kNewSetpoint = 1u << 4;          // PPM
constexpr std::uint16_t kHomingOperationStart = 1u << 4;  // HMM
constexpr std::uint16_t kChangeSetImmediately = 1u << 5;  // PPM
constexpr std::uint16_t kAbsoluteRelative = 1u << 6;      // PPM
constexpr std::uint16_t kHalt = 1u << 8;                  // PPM, PVM, HMM
constexpr std::uint16_t kEndlessMovement = 1u << 15;      // PPM

constexpr std::uint16_t kStateMachineBits =
  kSwitchOn | kEnableVoltage | kQuickStop | kEnableOperation | kFaultReset;

constexpr std::uint16_t kModeBits =
  kNewSetpoint | kChangeSetImmediately | kAbsoluteRelative | kHalt |
  kEndlessMovement;
}  // namespace control_bits


// ---------------------------------------------------------------------------
// Holding brake state, object 0x3158:03 (read only), Table 6-136.
//
// "Active" means the brake is clamped, i.e. holding the axis. "Inactive"
// means released and the motor is free to turn.
// ---------------------------------------------------------------------------
enum class BrakeState : std::uint8_t
{
  kInactive = 0,  // released, the axis is free
  kActive = 1     // clamped, the axis is held
};

const char * ToString(BrakeState value);


// ---------------------------------------------------------------------------
// What a digital output does, object 0x3151, Table 6-134.
// ---------------------------------------------------------------------------
enum class DigitalOutputFunction : std::uint8_t
{
  kSetBrakeGpio = 0,     // raw GPIO brake control, no timing handled for you
  kGeneralPurposeA = 16,
  kGeneralPurposeB = 17,
  kGeneralPurposeC = 18,
  kHoldingBrake = 24,    // the drive drives the brake with the 0x3158 timing
  kReadyFault = 25,      // active when ready, inactive on fault
  kNone = 255
};

const char * ToString(DigitalOutputFunction value);


// Which physical output a function is assigned to. Availability depends on
// the hardware variant; see the Hardware Reference for the controller.
enum class DigitalOutput : std::uint8_t
{
  kOutput1 = 1,           // 0x3151:01
  kOutput2 = 2,           // 0x3151:02
  kHighSpeedOutput1 = 3,  // 0x3151:03
  kHighSpeedOutput2 = 4   // 0x3151:04, only EPOS4 Disk 60/8 and 60/12
};


// ---------------------------------------------------------------------------
// Motor type, object 0x6402, Table 6-171 (p.6-256).
// Only changeable while the power stage is disabled.
// ---------------------------------------------------------------------------
enum class MotorType : std::uint16_t
{
  kBrushedDc = 1,          // phase-modulated DC, maxon DC motor
  kBrushlessSinusoidal = 10,  // BLDC sine commutated, maxon EC motor
  kBrushlessTrapezoidal = 11  // BLDC block commutated, maxon EC motor
};

const char * ToString(MotorType value);


// ---------------------------------------------------------------------------
// Homing method, object 0x6098. Section 3.5 (p.3-28).
//
// Negative values are maxon specific; the positive ones are CiA 402.
// ---------------------------------------------------------------------------
enum class HomingMethod : std::int8_t
{
  kActualPosition = 37,
  kIndexNegativeSpeed = 33,
  kIndexPositiveSpeed = 34,
  kHomeSwitchNegativeSpeed = 27,
  kHomeSwitchPositiveSpeed = 23,
  kNegativeLimitSwitch = 17,
  kPositiveLimitSwitch = 18,
  kCurrentThresholdNegative = -1,
  kCurrentThresholdPositive = -2,
  kCurrentThresholdNegativeIndex = -3,
  kCurrentThresholdPositiveIndex = -4
};


// ---------------------------------------------------------------------------
// Motion profile type, object 0x6086.
// ---------------------------------------------------------------------------
enum class MotionProfileType : std::int16_t
{
  kLinearRamp = 0  // trapezoidal, the only type the EPOS4 supports
};


// ---------------------------------------------------------------------------
// What the drive does when a given stop is commanded. Objects 0x605A, 0x605B,
// 0x605C, 0x605E. These decide how the motor comes to rest, which is why they
// are not a detail: on a loaded arm, cutting power and ramping down are very
// different outcomes.
// ---------------------------------------------------------------------------
enum class QuickStopOption : std::int16_t
{
  kDisableDrive = 0,
  kSlowDownOnQuickStopRamp = 2,
  kSlowDownOnQuickStopRampAndStayInQuickStop = 6
};

enum class ShutdownOption : std::int16_t
{
  kDisableDrive = 0,
  kSlowDownOnSlowDownRamp = 1
};

enum class DisableOperationOption : std::int16_t
{
  kDisableDrive = 0,
  kSlowDownOnSlowDownRamp = 1
};

enum class FaultReactionOption : std::int16_t
{
  kDisableDrive = 0,
  kSlowDownOnQuickStopRamp = 2
};

}  // namespace epos4::signals
