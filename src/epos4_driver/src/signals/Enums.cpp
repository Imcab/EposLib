#include "epos4/signals/Enums.hpp"

namespace epos4::signals
{

const char *
ToString(State value)
{
  switch (value) {
    case State::kNotReadyToSwitchOn: return "Not ready to switch on";
    case State::kSwitchOnDisabled: return "Switch on disabled";
    case State::kReadyToSwitchOn: return "Ready to switch on";
    case State::kSwitchedOn: return "Switched on";
    case State::kOperationEnabled: return "Operation enabled";
    case State::kQuickStopActive: return "Quick stop active";
    case State::kFaultReactionActive: return "Fault reaction active";
    case State::kFault: return "Fault";
  }
  return "unknown";
}

const char *
ToString(OperationMode value)
{
  switch (value) {
    case OperationMode::kNone: return "None";
    case OperationMode::kProfilePosition: return "Profile Position (PPM)";
    case OperationMode::kProfileVelocity: return "Profile Velocity (PVM)";
    case OperationMode::kHoming: return "Homing (HMM)";
    case OperationMode::kCyclicSynchronousPosition: return "Cyclic Sync Position (CSP)";
    case OperationMode::kCyclicSynchronousVelocity: return "Cyclic Sync Velocity (CSV)";
    case OperationMode::kCyclicSynchronousTorque: return "Cyclic Sync Torque (CST)";
  }
  return "unknown";
}

const char *
ToString(QuickStopOption)
{
  // Only one value exists on the EPOS4.
  return "Decelerate with quick stop ramp and stay in «Quick stop active»";
}

const char *
ToString(ShutdownOption value)
{
  switch (value) {
    case ShutdownOption::kDisableDrive: return "Disable drive function";
    case ShutdownOption::kSlowDownOnSlowDownRamp:
      return "Decelerate with slowdown ramp, then disable";
  }
  return "unknown";
}

const char *
ToString(DisableOperationOption value)
{
  switch (value) {
    case DisableOperationOption::kDisableDrive: return "Disable drive function";
    case DisableOperationOption::kSlowDownOnSlowDownRamp:
      return "Decelerate with slowdown ramp, then disable";
  }
  return "unknown";
}

const char *
ToString(FaultReactionOption value)
{
  switch (value) {
    case FaultReactionOption::kDisableDrive: return "Disable drive function";
    case FaultReactionOption::kSlowDownOnSlowDownRamp:
      return "Decelerate with slowdown ramp, then disable";
    case FaultReactionOption::kSlowDownOnQuickStopRamp:
      return "Decelerate with quick stop ramp, then disable";
  }
  return "unknown";
}

const char *
ToString(AbortConnectionOption value)
{
  switch (value) {
    case AbortConnectionOption::kDisableVoltage: return "«Disable voltage» command";
    case AbortConnectionOption::kSlowDownOnQuickStopRamp:
      return "Decelerate with quick stop ramp, then disable";
  }
  return "unknown";
}

const char *
ToString(DigitalInputFunction value)
{
  switch (value) {
    case DigitalInputFunction::kNegativeLimitSwitch: return "Negative limit switch";
    case DigitalInputFunction::kPositiveLimitSwitch: return "Positive limit switch";
    case DigitalInputFunction::kHomeSwitch: return "Home switch";
    case DigitalInputFunction::kGeneralPurposeA: return "General purpose A";
    case DigitalInputFunction::kGeneralPurposeB: return "General purpose B";
    case DigitalInputFunction::kGeneralPurposeC: return "General purpose C";
    case DigitalInputFunction::kGeneralPurposeD: return "General purpose D";
    case DigitalInputFunction::kGeneralPurposeE: return "General purpose E";
    case DigitalInputFunction::kGeneralPurposeF: return "General purpose F";
    case DigitalInputFunction::kGeneralPurposeG: return "General purpose G";
    case DigitalInputFunction::kGeneralPurposeH: return "General purpose H";
    case DigitalInputFunction::kNegativeLimitSwitchNoError:
      return "Negative limit switch without errors";
    case DigitalInputFunction::kPositiveLimitSwitchNoError:
      return "Positive limit switch without errors";
    case DigitalInputFunction::kTouchProbe: return "Touch probe";
    case DigitalInputFunction::kDriveEnable: return "Drive enable";
    case DigitalInputFunction::kQuickStop: return "Quick stop";
    case DigitalInputFunction::kNone: return "None";
  }
  return "unknown";
}

const char *
ToString(DigitalInput value)
{
  switch (value) {
    case DigitalInput::kInput1: return "Digital input 1";
    case DigitalInput::kInput2: return "Digital input 2";
    case DigitalInput::kInput3: return "Digital input 3";
    case DigitalInput::kInput4: return "Digital input 4";
    case DigitalInput::kHighSpeedInput1: return "High-speed digital input 1";
    case DigitalInput::kHighSpeedInput2: return "High-speed digital input 2";
    case DigitalInput::kHighSpeedInput3: return "High-speed digital input 3";
    case DigitalInput::kHighSpeedInput4: return "High-speed digital input 4";
  }
  return "unknown";
}

const char *
ToString(BrakeState value)
{
  switch (value) {
    case BrakeState::kInactive: return "inactive (released)";
    case BrakeState::kActive: return "active (holding)";
  }
  return "unknown";
}

const char *
ToString(DigitalOutputFunction value)
{
  switch (value) {
    case DigitalOutputFunction::kSetBrakeGpio: return "Set brake (GPIO)";
    case DigitalOutputFunction::kGeneralPurposeA: return "General purpose A";
    case DigitalOutputFunction::kGeneralPurposeB: return "General purpose B";
    case DigitalOutputFunction::kGeneralPurposeC: return "General purpose C";
    case DigitalOutputFunction::kHoldingBrake: return "Holding brake";
    case DigitalOutputFunction::kReadyFault: return "Ready/Fault";
    case DigitalOutputFunction::kNone: return "None";
  }
  return "unknown";
}

const char *
ToString(HomingMethod value)
{
  switch (value) {
    case HomingMethod::kNegativeLimitSwitchAndIndex:
      return "Negative limit switch & index";
    case HomingMethod::kPositiveLimitSwitchAndIndex:
      return "Positive limit switch & index";
    case HomingMethod::kHomeSwitchPositiveSpeedAndIndex:
      return "Home switch positive speed & index";
    case HomingMethod::kHomeSwitchNegativeSpeedAndIndex:
      return "Home switch negative speed & index";
    case HomingMethod::kNegativeLimitSwitch: return "Negative limit switch";
    case HomingMethod::kPositiveLimitSwitch: return "Positive limit switch";
    case HomingMethod::kHomeSwitchPositiveSpeed: return "Home switch positive speed";
    case HomingMethod::kHomeSwitchNegativeSpeed: return "Home switch negative speed";
    case HomingMethod::kIndexNegativeSpeed: return "Index negative speed";
    case HomingMethod::kIndexPositiveSpeed: return "Index positive speed";
    case HomingMethod::kActualPosition: return "Actual position";
    case HomingMethod::kCurrentThresholdPositiveSpeedAndIndex:
      return "Current threshold positive speed & index";
    case HomingMethod::kCurrentThresholdNegativeSpeedAndIndex:
      return "Current threshold negative speed & index";
    case HomingMethod::kCurrentThresholdPositiveSpeed:
      return "Current threshold positive speed";
    case HomingMethod::kCurrentThresholdNegativeSpeed:
      return "Current threshold negative speed";
  }
  return "unknown";
}

std::optional<DigitalInputFunction>
RequiredInput(HomingMethod method)
{
  switch (method) {
    case HomingMethod::kNegativeLimitSwitchAndIndex:
    case HomingMethod::kNegativeLimitSwitch:
      return DigitalInputFunction::kNegativeLimitSwitch;

    case HomingMethod::kPositiveLimitSwitchAndIndex:
    case HomingMethod::kPositiveLimitSwitch:
      return DigitalInputFunction::kPositiveLimitSwitch;

    case HomingMethod::kHomeSwitchPositiveSpeedAndIndex:
    case HomingMethod::kHomeSwitchNegativeSpeedAndIndex:
    case HomingMethod::kHomeSwitchPositiveSpeed:
    case HomingMethod::kHomeSwitchNegativeSpeed:
      return DigitalInputFunction::kHomeSwitch;

    // No switch: the index pulse, the current position, or motor current.
    case HomingMethod::kIndexNegativeSpeed:
    case HomingMethod::kIndexPositiveSpeed:
    case HomingMethod::kActualPosition:
    case HomingMethod::kCurrentThresholdPositiveSpeedAndIndex:
    case HomingMethod::kCurrentThresholdNegativeSpeedAndIndex:
    case HomingMethod::kCurrentThresholdPositiveSpeed:
    case HomingMethod::kCurrentThresholdNegativeSpeed:
      return std::nullopt;
  }
  return std::nullopt;
}

bool
RequiresEncoderIndex(HomingMethod method)
{
  switch (method) {
    case HomingMethod::kNegativeLimitSwitchAndIndex:
    case HomingMethod::kPositiveLimitSwitchAndIndex:
    case HomingMethod::kHomeSwitchPositiveSpeedAndIndex:
    case HomingMethod::kHomeSwitchNegativeSpeedAndIndex:
    case HomingMethod::kIndexNegativeSpeed:
    case HomingMethod::kIndexPositiveSpeed:
    case HomingMethod::kCurrentThresholdPositiveSpeedAndIndex:
    case HomingMethod::kCurrentThresholdNegativeSpeedAndIndex:
      return true;
    default:
      return false;
  }
}

const char *
ToString(VelocityPrefix value)
{
  switch (value) {
    case VelocityPrefix::kNone: return "rpm";
    case VelocityPrefix::kDeci: return "0.1 rpm";
    case VelocityPrefix::kCenti: return "0.01 rpm";
    case VelocityPrefix::kMilli: return "0.001 rpm";
    case VelocityPrefix::kE4: return "1e-4 rpm";
    case VelocityPrefix::kE5: return "1e-5 rpm";
    case VelocityPrefix::kMicro: return "1e-6 rpm";
  }
  return "unknown";
}

double
VelocityPrefixScale(VelocityPrefix prefix)
{
  switch (prefix) {
    case VelocityPrefix::kNone: return 1.0;
    case VelocityPrefix::kDeci: return 1e-1;
    case VelocityPrefix::kCenti: return 1e-2;
    case VelocityPrefix::kMilli: return 1e-3;
    case VelocityPrefix::kE4: return 1e-4;
    case VelocityPrefix::kE5: return 1e-5;
    case VelocityPrefix::kMicro: return 1e-6;
  }
  return 1.0;
}

const char *
ToString(MotorType value)
{
  switch (value) {
    case MotorType::kBrushedDc: return "Brushed DC";
    case MotorType::kBrushlessSinusoidal: return "Brushless, sinusoidal";
    case MotorType::kBrushlessTrapezoidal: return "Brushless, trapezoidal";
  }
  return "unknown";
}

}  // namespace epos4::signals
