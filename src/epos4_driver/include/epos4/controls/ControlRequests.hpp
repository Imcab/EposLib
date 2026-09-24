#pragma once

#include <cstdint>
#include <optional>

#include "epos4/core/Setpoint.hpp"
#include "epos4/signals/Enums.hpp"

namespace epos4::controls
{

// ---------------------------------------------------------------------------
// Control requests, one per CiA 402 operating mode.
//
// A request carries the target and whatever profile overrides come with it.
// Fields left unset fall back to whatever the drive is already configured
// with, so a caller who set up MotionProfileConfigs once does not have to
// repeat the velocity on every move.
//
// Each request knows which mode it needs. Epos4::SetControl switches the
// drive into that mode before commanding, and does nothing if it is already
// there, so the caller never has to think about 0x6060.
//
// Setpoints take either raw drive units or physical quantities:
//
//   .WithPosition(50000)     quadcounts, what the manual and EPOS Studio use
//   .WithPosition(90_deg)    an angle at the output shaft
//
// The quantity form needs the device's mechanism to have been configured
// through Epos4::SetMechanism(); without it SetControl returns
// std::errc::invalid_argument rather than guessing a resolution.
// ---------------------------------------------------------------------------


// Profile Position Mode, section 3.3 (p.3-21).
//
// The drive generates the trapezoidal ramp internally; the master only hands
// over the endpoint. Commanding this involves the setpoint handshake of Table
// 3-15: raise New setpoint (bit 4), wait for Setpoint acknowledge (bit 12),
// lower bit 4 again. SetControl does that for you - skipping it is the reason
// a second move silently never happens.
struct ProfilePosition
{
  PositionSetpoint position{0};  // 0x607A

  std::optional<SpeedSetpoint> velocity;      // 0x6081, overrides the config
  std::optional<std::uint32_t> acceleration;  // 0x6083, [rpm/s]
  std::optional<std::uint32_t> deceleration;  // 0x6084, [rpm/s]

  // Controlword bit 6. Absolute is almost always what an arm wants: relative
  // moves accumulate whatever error the previous move left behind.
  bool relative{false};

  // Controlword bit 5. false: finish the move in progress, then start this
  // one. true: abort the move in progress and start immediately.
  bool changeSetImmediately{false};

  static constexpr auto kMode = signals::OperationMode::kProfilePosition;

  ProfilePosition & WithPosition(PositionSetpoint v) {position = v; return *this;}
  ProfilePosition & WithVelocity(SpeedSetpoint v) {velocity = v; return *this;}
  ProfilePosition & WithAcceleration(std::uint32_t v) {acceleration = v; return *this;}
  ProfilePosition & WithDeceleration(std::uint32_t v) {deceleration = v; return *this;}
  ProfilePosition & WithRelative(bool v) {relative = v; return *this;}
  ProfilePosition & WithChangeSetImmediately(bool v) {changeSetImmediately = v; return *this;}
};


// Profile Velocity Mode, section 3.4 (p.3-25).
struct ProfileVelocity
{
  VelocitySetpoint velocity{0};  // 0x60FF

  std::optional<std::uint32_t> acceleration;  // 0x6083, [rpm/s]
  std::optional<std::uint32_t> deceleration;  // 0x6084, [rpm/s]

  static constexpr auto kMode = signals::OperationMode::kProfileVelocity;

  ProfileVelocity & WithVelocity(VelocitySetpoint v) {velocity = v; return *this;}
  ProfileVelocity & WithAcceleration(std::uint32_t v) {acceleration = v; return *this;}
  ProfileVelocity & WithDeceleration(std::uint32_t v) {deceleration = v; return *this;}
};


// Cyclic Synchronous Position Mode, section 3.6 (p.3-37).
//
// Here the trajectory generator is in the master, not the drive: a new target
// is expected every cycle and the drive interpolates between them using
// Interpolation time period (0x60C2). This is the mode that makes sense under
// a ros2_control JointTrajectoryController, and it is only meaningful over
// PDO - sending it by SDO would be far slower than the cycle it assumes.
struct CyclicPosition
{
  PositionSetpoint position{0};  // 0x607A

  std::optional<std::int32_t> positionOffset;  // 0x60B0, [quadcounts]
  std::optional<std::int16_t> torqueOffset;    // 0x60B2, feed forward

  static constexpr auto kMode = signals::OperationMode::kCyclicSynchronousPosition;

  CyclicPosition & WithPosition(PositionSetpoint v) {position = v; return *this;}
  CyclicPosition & WithPositionOffset(std::int32_t v) {positionOffset = v; return *this;}
  CyclicPosition & WithTorqueOffset(std::int16_t v) {torqueOffset = v; return *this;}
};


// Cyclic Synchronous Velocity Mode, section 3.7 (p.3-41).
struct CyclicVelocity
{
  VelocitySetpoint velocity{0};  // 0x60FF

  std::optional<std::int32_t> velocityOffset;  // 0x60B1
  std::optional<std::int16_t> torqueOffset;    // 0x60B2

  static constexpr auto kMode = signals::OperationMode::kCyclicSynchronousVelocity;

  CyclicVelocity & WithVelocity(VelocitySetpoint v) {velocity = v; return *this;}
  CyclicVelocity & WithVelocityOffset(std::int32_t v) {velocityOffset = v; return *this;}
  CyclicVelocity & WithTorqueOffset(std::int16_t v) {torqueOffset = v; return *this;}
};


// Cyclic Synchronous Torque Mode, section 3.8 (p.3-44).
struct CyclicTorque
{
  std::int16_t torque{0};  // 0x6071, per thousand of Motor rated torque

  std::optional<std::int16_t> torqueOffset;  // 0x60B2

  static constexpr auto kMode = signals::OperationMode::kCyclicSynchronousTorque;

  CyclicTorque & WithTorque(std::int16_t v) {torque = v; return *this;}
  CyclicTorque & WithTorqueOffset(std::int16_t v) {torqueOffset = v; return *this;}
};


// Homing Mode, section 3.5 (p.3-28).
//
// Homing takes time and can fail, so this is the one request that is not
// fire-and-forget: see Epos4::Home() and the homing status accessors.
struct Homing
{
  std::optional<signals::HomingMethod> method;  // 0x6098, else use the config

  static constexpr auto kMode = signals::OperationMode::kHoming;

  Homing & WithMethod(signals::HomingMethod v) {method = v; return *this;}
};


// Stop the motion in the current mode without leaving Operation enabled.
// Controlword bit 8, decelerating with Profile deceleration. Distinct from
// QuickStop, which leaves the state machine, and from Disable, which removes
// power entirely.
struct Halt
{
  static constexpr auto kMode = signals::OperationMode::kNone;
};

}  // namespace epos4::controls
