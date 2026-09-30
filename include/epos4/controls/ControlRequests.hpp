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
// a trajectory follower running on the master, and it is only meaningful over
// PDO - sending it by SDO would be far slower than the cycle it assumes.
struct CyclicPosition
{
  PositionSetpoint position{0};  // 0x607A

  std::optional<PositionSetpoint> positionOffset;  // 0x60B0, added to the target
  std::optional<TorqueSetpoint> torqueOffset;      // 0x60B2, feed forward

  static constexpr auto kMode = signals::OperationMode::kCyclicSynchronousPosition;

  CyclicPosition & WithPosition(PositionSetpoint v) {position = v; return *this;}
  CyclicPosition & WithPositionOffset(PositionSetpoint v) {positionOffset = v; return *this;}
  CyclicPosition & WithTorqueOffset(TorqueSetpoint v) {torqueOffset = v; return *this;}
};


// Cyclic Synchronous Velocity Mode, section 3.7 (p.3-41).
struct CyclicVelocity
{
  VelocitySetpoint velocity{0};  // 0x60FF

  std::optional<VelocitySetpoint> velocityOffset;  // 0x60B1, feed forward
  std::optional<TorqueSetpoint> torqueOffset;      // 0x60B2, feed forward

  static constexpr auto kMode = signals::OperationMode::kCyclicSynchronousVelocity;

  CyclicVelocity & WithVelocity(VelocitySetpoint v) {velocity = v; return *this;}
  CyclicVelocity & WithVelocityOffset(VelocitySetpoint v) {velocityOffset = v; return *this;}
  CyclicVelocity & WithTorqueOffset(TorqueSetpoint v) {torqueOffset = v; return *this;}
};


// Cyclic Synchronous Torque Mode, section 3.8 (p.3-44).
struct CyclicTorque
{
  // 0x6071. Raw: thousandths of «Motor rated torque» (0x6076). As a torque,
  // e.g. WithTorque(0.5_Nm), it is resolved against the motor's rated torque,
  // which the motor data (MotorConfigs) must have set.
  TorqueSetpoint torque{0};

  std::optional<TorqueSetpoint> torqueOffset;  // 0x60B2, added to the target

  static constexpr auto kMode = signals::OperationMode::kCyclicSynchronousTorque;

  CyclicTorque & WithTorque(TorqueSetpoint v) {torque = v; return *this;}
  CyclicTorque & WithTorqueOffset(TorqueSetpoint v) {torqueOffset = v; return *this;}
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


// Touch probe 1, sections 6.2.134-142: latches the actual position on an
// edge, in the drive, to the resolution of the encoder - far finer than
// anything the master could sample over the bus. For measuring where a
// sensor switches, or catching the index pulse.
//
// Not an operating mode: it runs alongside any of them, except Homing,
// which cannot be used at the same time and clears every latched value
// when it completes (6.2.134).
struct TouchProbe
{
  // Touch probe function bits 3..2 (Table 6-164).
  enum class Trigger : std::uint16_t
  {
    kInput = 0b00,        // the digital input mapped to «Touch probe» (0x3142 = 26)
    kIndexPulse = 0b01,   // the main encoder's index
    kSourceObject = 0b10, // whatever «Touch probe 1 source» (0x60D0:01) names
  };

  Trigger trigger{Trigger::kInput};
  bool continuous{false};    // bit 1: every edge, rather than the first only
  bool positiveEdge{true};   // bit 4
  bool negativeEdge{false};  // bit 5

  TouchProbe & WithTrigger(Trigger v) {trigger = v; return *this;}
  TouchProbe & WithContinuous(bool v) {continuous = v; return *this;}
  TouchProbe & WithPositiveEdge(bool v) {positiveEdge = v; return *this;}
  TouchProbe & WithNegativeEdge(bool v) {negativeEdge = v; return *this;}

  // Table 6-164 note [a]: on the index pulse, both edges at once is not
  // possible. Nor is arming with no edge at all, which would latch nothing.
  constexpr bool IsValid() const
  {
    return (positiveEdge || negativeEdge) &&
           !(trigger == Trigger::kIndexPulse && positiveEdge && negativeEdge);
  }

  // The «Touch probe function» word (0x60B8) that enables probe 1 so.
  constexpr std::uint16_t ToFunctionWord() const
  {
    return static_cast<std::uint16_t>(
      (1u << 0) |                                        // enable touch probe 1
      (continuous ? 1u << 1 : 0u) |
      (static_cast<std::uint16_t>(trigger) << 2) |
      (positiveEdge ? 1u << 4 : 0u) |
      (negativeEdge ? 1u << 5 : 0u));
  }
};

}  // namespace epos4::controls
