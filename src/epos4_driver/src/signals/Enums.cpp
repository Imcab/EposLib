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
