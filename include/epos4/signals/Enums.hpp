#pragma once

#include <cstdint>
#include <optional>

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
// What a digital input does, object 0x3142, Table 6-131.
//
// The numeric value is also the BIT POSITION of that function in «Digital
// inputs» (0x60FD): whichever pin carries the negative limit switch, its
// state shows up in bit 0. That is why the state is queried by function and
// not by pin - moving a switch to a different terminal does not change the
// code that reads it.
// ---------------------------------------------------------------------------
enum class DigitalInputFunction : std::uint8_t
{
  // Generates a limit error outside homing mode. This is the safe default
  // for a real end stop: hitting it while moving normally is a fault.
  kNegativeLimitSwitch = 0,
  kPositiveLimitSwitch = 1,

  kHomeSwitch = 2,

  kGeneralPurposeA = 16,
  kGeneralPurposeB = 17,
  kGeneralPurposeC = 18,
  kGeneralPurposeD = 19,
  kGeneralPurposeE = 20,
  kGeneralPurposeF = 21,
  kGeneralPurposeG = 22,
  kGeneralPurposeH = 23,

  // Same switches, but they do NOT raise a limit error. Only usable by the
  // homing methods that expect them; on a normal move the axis would drive
  // straight through the end stop without complaining.
  kNegativeLimitSwitchNoError = 24,
  kPositiveLimitSwitchNoError = 25,

  kTouchProbe = 26,   // samples the actual position on an edge
  kDriveEnable = 27,  // enables/disables the drive, or clears a fault
  kQuickStop = 28,    // stops and enters «Quick stop active»

  kNone = 255
};

const char * ToString(DigitalInputFunction value);


// Which physical terminal. 0x3142 sub-indices 1 to 8, and the bit positions
// of «Digital inputs logic state» (0x3141:01, Table 6-129).
//
// The high-speed inputs are absent on EPOS4 Disk 60/8, Disk 60/12,
// Micro 24/1.5 and Micro 24/5, and are ALSO disabled - not overridably -
// whenever sensor 2 is configured in 0x3000:01, because they share pins with
// the second encoder.
enum class DigitalInput : std::uint8_t
{
  kInput1 = 1,
  kInput2 = 2,
  kInput3 = 3,
  kInput4 = 4,
  kHighSpeedInput1 = 5,
  kHighSpeedInput2 = 6,
  kHighSpeedInput3 = 7,
  kHighSpeedInput4 = 8
};

const char * ToString(DigitalInput value);


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
// Velocity unit prefix, the high byte of «SI unit velocity» (0x60A9),
// Table 6-161. The unit itself is always rev/min; only the scale varies.
//
// Finer resolution costs range: the INTEGER32 carrying a velocity in
// micro-rpm overflows a million times sooner than one in rpm.
// ---------------------------------------------------------------------------
enum class VelocityPrefix : std::uint8_t
{
  kNone = 0x00,   // rpm            - the default
  kDeci = 0xFF,   // 0.1 rpm
  kCenti = 0xFE,  // 0.01 rpm
  kMilli = 0xFD,  // 0.001 rpm
  kE4 = 0xFC,     // 0.0001 rpm
  kE5 = 0xFB,     // 0.00001 rpm
  kMicro = 0xFA   // 0.000001 rpm
};

const char * ToString(VelocityPrefix value);

// The factor a raw velocity must be multiplied by to get rpm.
double VelocityPrefixScale(VelocityPrefix prefix);


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
// Homing method, object 0x6098. Section 3.5.3 (p.3-30 onwards).
//
// These fifteen are the ones the EPOS4 implements; the drive reports its own
// list in «Supported homing methods» (0x60E3). Negative values are maxon
// specific, the positive ones are CiA 402.
//
// Most of them need a switch, and a switch that is wired but not mapped in
// «Configuration of digital inputs» (0x3142) does nothing at all. Use
// RequiredInput() below to find out which one a method depends on.
// ---------------------------------------------------------------------------
enum class HomingMethod : std::int8_t
{
  // --- limit switch, then back off to the encoder index ---
  kNegativeLimitSwitchAndIndex = 1,
  kPositiveLimitSwitchAndIndex = 2,

  // --- home switch, then the index ---
  kHomeSwitchPositiveSpeedAndIndex = 7,
  kHomeSwitchNegativeSpeedAndIndex = 11,

  // --- limit switch alone, no index needed ---
  kNegativeLimitSwitch = 17,
  kPositiveLimitSwitch = 18,

  // --- home switch alone ---
  kHomeSwitchPositiveSpeed = 23,
  kHomeSwitchNegativeSpeed = 27,

  // --- the encoder index alone, no switch at all ---
  kIndexNegativeSpeed = 33,
  kIndexPositiveSpeed = 34,

  // --- no motion: take the current position as home ---
  kActualPosition = 37,

  // --- run into a hard stop and detect it by motor current ---
  // Needs «Current threshold for homing mode» (0x30B2) configured, and a
  // mechanical end that tolerates being pushed against.
  kCurrentThresholdPositiveSpeedAndIndex = -1,
  kCurrentThresholdNegativeSpeedAndIndex = -2,
  kCurrentThresholdPositiveSpeed = -3,
  kCurrentThresholdNegativeSpeed = -4
};

const char * ToString(HomingMethod value);

// Which digital input function this method depends on, or nullopt when it
// needs no switch at all (index-only, actual position, current threshold).
//
// A homing run whose switch is not mapped does not fail fast: the axis drives
// until it hits something or times out. Checking first turns that into an
// error before anything moves.
std::optional<DigitalInputFunction> RequiredInput(HomingMethod method);

// True when the method finishes on the encoder index pulse, which needs a
// 3-channel encoder configured with IndexType::kWithIndex.
bool RequiresEncoderIndex(HomingMethod method);

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
// Quick stop option code, 0x605A, Table 6-149.
//
// The EPOS4 offers exactly one value: the manual gives the range as "6 to 6".
// There is no choice to make here - the drive always decelerates on the quick
// stop ramp and stays in «Quick stop active». The enum exists so the object
// is written with a name rather than a bare 6, and so nobody is tempted to
// try the CiA 402 values the EPOS4 does not implement (writing 0 or 2 aborts
// with 0x06090030 "Value range error", and because Apply() stops at the first
// failure, the rest of the configuration silently never lands).
enum class QuickStopOption : std::int16_t
{
  kSlowDownOnQuickStopRampAndStayInQuickStop = 6
};

const char * ToString(QuickStopOption value);


// Shutdown option code, 0x605B, Table 6-150. Applies to the transition from
// «Operation enabled» to «Ready to switch on» or «Switch on disabled».
// Device default: 0.
enum class ShutdownOption : std::int16_t
{
  kDisableDrive = 0,          // switch off the power stage
  kSlowDownOnSlowDownRamp = 1  // decelerate first, then disable
};

const char * ToString(ShutdownOption value);


// Disable operation option code, 0x605C, Table 6-151. Applies to the
// transition from «Operation enabled» to «Switched on».
// Device default: 1, i.e. it ramps down rather than cutting power.
enum class DisableOperationOption : std::int16_t
{
  kDisableDrive = 0,
  kSlowDownOnSlowDownRamp = 1
};

const char * ToString(DisableOperationOption value);


// Fault reaction option code, 0x605E, Table 6-153. Applies to the errors
// chapter 7 labels "f". Device default: 2.
//
// Note that the errors labelled "d" - "a secure movement is no longer
// possible" - always disable the drive regardless of what is set here.
enum class FaultReactionOption : std::int16_t
{
  kDisableDrive = 0,            // cut power immediately
  kSlowDownOnSlowDownRamp = 1,  // the gentle one: normal deceleration ramp
  kSlowDownOnQuickStopRamp = 2  // the default: quick stop ramp, then disable
};

const char * ToString(FaultReactionOption value);


// Abort connection option code, 0x6007, Table 6-146. Applies to the errors
// chapter 7 labels "a", which are all the communication errors.
// Device default: 3.
//
// This is what the drive does when the bus goes away - a cable shaken loose
// on a moving rover, or a master that stopped answering. Only two values
// exist; 0 and 1 are not implemented on the EPOS4.
enum class AbortConnectionOption : std::int16_t
{
  kDisableVoltage = 2,          // «Disable voltage» command
  kSlowDownOnQuickStopRamp = 3  // decelerate on the quick stop ramp, then disable
};

const char * ToString(AbortConnectionOption value);

}  // namespace epos4::signals
