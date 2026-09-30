// A simulated maxon EPOS4 on the CAN bus.
//
// WHY THIS EXISTS
//
// canopen_fake_slaves ships a "cia402_slave" mock, but it does not implement
// the CiA 402 transitions: it answers 0x0040 (Switch on disabled) to every
// Controlword write and never advances. Verified by hand:
//
//     write 0x6040 = 0x06 (Shutdown)          -> statusword 0x0040
//     write 0x6040 = 0x07 (Switch on)         -> statusword 0x0040  (no change)
//     write 0x6040 = 0x0F (Enable operation)  -> statusword 0x0040  (no change)
//
// That makes the whole enable sequence untestable without hardware. This
// simulator implements Table 2-6 properly, and loads the real maxon EDS, so
// the master's identity check against 0x1F84/0x1F85 passes and the same
// config that will talk to the real drive can be used against it.
//
// Run, on the SAME network as the hardware (epos4_network), because this
// answers with the maxon identity:
//   CFG=install/eposlib/share/eposlib/config/epos4_network
//   ./build/eposlib/epos4_sim $CFG/epos4.eds 2 vcan0

#include <lely/coapp/slave.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/clock.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <string>

#include "epos4/core/Cia402StateMachine.hpp"

using namespace lely;
namespace e4 = epos4::core;
namespace sig = epos4::signals;

namespace
{

const char *
StateName(sig::State s)
{
  switch (s) {
    case sig::State::kNotReadyToSwitchOn: return "Not ready to switch on";
    case sig::State::kSwitchOnDisabled: return "Switch on disabled";
    case sig::State::kReadyToSwitchOn: return "Ready to switch on";
    case sig::State::kSwitchedOn: return "Switched on";
    case sig::State::kOperationEnabled: return "Operation enabled";
    case sig::State::kQuickStopActive: return "Quick stop active";
    case sig::State::kFaultReactionActive: return "Fault reaction active";
    case sig::State::kFault: return "Fault";
  }
  return "?";
}

// The drive side of Table 2-6: given the current state and a Controlword,
// which state do we move to? This is the mirror image of planStep(), which
// answers the master's question instead.
//
// The command patterns are checked most-specific first, because several of
// them overlap. Note in particular that «Switch on» (0xxx x111) and «Disable
// operation» (0xxx 0111) share bits: what they mean depends on where we are.
sig::State
NextState(sig::State current, std::uint16_t cw)
{
  using S = sig::State;

  // Disable voltage: bit 1 low. Drops power from anywhere it is legal.
  if ((cw & 0x0002) == 0) {
    switch (current) {
      case S::kReadyToSwitchOn:      // transition 7
      case S::kSwitchedOn:           // transition 10
      case S::kOperationEnabled:     // transition 9
      case S::kQuickStopActive:      // transition 12
        return S::kSwitchOnDisabled;
      default:
        return current;
    }
  }

  // Quick stop: bit 2 low with bit 1 high. Active low, as Table 2-7 shows.
  if ((cw & 0x0006) == 0x0002) {
    switch (current) {
      case S::kOperationEnabled:     // transition 11
        return S::kQuickStopActive;
      case S::kReadyToSwitchOn:
      case S::kSwitchedOn:
        return S::kSwitchOnDisabled;
      default:
        return current;
    }
  }

  // Shutdown: 0xxx x110
  if ((cw & 0x0087) == 0x0006) {
    switch (current) {
      case S::kSwitchOnDisabled:     // transition 2
      case S::kSwitchedOn:           // transition 6
      case S::kOperationEnabled:     // transition 8
        return S::kReadyToSwitchOn;
      default:
        return current;
    }
  }

  // Enable operation: 0xxx 1111
  if ((cw & 0x008F) == 0x000F) {
    switch (current) {
      case S::kSwitchedOn:           // transition 4
      case S::kQuickStopActive:      // transition 16
        return S::kOperationEnabled;
      case S::kReadyToSwitchOn:      // transitions 3 + 4 chained, note (*1)
        return S::kOperationEnabled;
      default:
        return current;
    }
  }

  // Switch on (from Ready) / Disable operation (from Operation enabled).
  // Same bit pattern, 0xxx 0111; both land on Switched on.
  if ((cw & 0x008F) == 0x0007) {
    switch (current) {
      case S::kReadyToSwitchOn:      // transition 3
      case S::kOperationEnabled:     // transition 5
        return S::kSwitchedOn;
      default:
        return current;
    }
  }

  return current;
}

}  // namespace


class Epos4Sim : public canopen::BasicSlave
{
public:
  using BasicSlave::BasicSlave;

  // The maxon EDS declares the identity in its [DeviceInfo] header, not as a
  // DefaultValue on object 0x1018, so a slave built from it starts up with an
  // identity of zero and the master rejects it:
  //
  //   es='D': Value of object 1018 sub-index 01 ... different to object 1F85
  //
  // dcfgen copies those header values into the master's expected-identity
  // objects, so the simulator has to answer with the same ones.
  void
  SetIdentity()
  {
    (*this)[0x1018][1] = std::uint32_t{0x000000FB};  // VendorNumber, maxon
    (*this)[0x1018][2] = std::uint32_t{0x65520000};  // ProductNumber
    (*this)[0x1018][3] = std::uint32_t{0x01700000};  // RevisionNumber
  }

  // Plausible readings for the objects the EDS leaves at zero, so that
  // reading them exercises the conversions instead of returning 0.0 for
  // everything: a 48 V pack (0x2200:01, tenths of a volt) and a power stage
  // at 35 degrees (0x3201:01, tenths of a degree). Both live outside the
  // communication area, so only a reset node reloads them.
  void
  SetMeasurements()
  {
    (*this)[0x2200][1] = std::uint16_t{480};
    (*this)[0x3201][1] = std::int16_t{350};
    SetCurrent(false);
  }

  // The timer that drives transition 1. Held by reference so the simulator
  // can re-arm it after every NMT reset.
  void
  UseInitTimer(io::Timer & t)
  {
    init_timer_ = &t;
    ArmInit();
  }

  void
  UseMotionTimer(io::Timer & t)
  {
    motionTimer_ = &t;
  }

  // Transition 1: initialisation finished. A real EPOS4 does this on its own
  // a few milliseconds after power-up, which is exactly what the mock in
  // canopen_fake_slaves fails to do.
  void
  FinishInit()
  {
    SetState(sig::State::kSwitchOnDisabled);
    printf("[sim] init complete -> %s\n", StateName(state_));
  }

private:
  // Both NMT resets reload the communication objects (0x1000-0x1FFF) from
  // the EDS, which wipes the identity written by SetIdentity() and makes the
  // master reject us with es='D' on the very next boot attempt, so both
  // re-apply it. Beyond that they differ, and the difference is the point:
  //
  //   reset node           the whole device restarts: back to «Not ready to
  //                        switch on», any fault gone, like a power cycle.
  //   reset communication  only the communication side restarts. The CiA 402
  //                        state machine is application, untouched - a drive
  //                        in «Fault» is still in «Fault» afterwards. What it
  //                        does do is satisfy the first half of the recovery
  //                        of a communication fault (7.2.35), after which the
  //                        Controlword fault reset is accepted.
  //
  // The master sends reset communication to every node when it starts, so a
  // drive faulted by a previous master that died stays faulted for the next
  // one until somebody clears it on purpose - as on the real hardware.
  void
  OnCommand(canopen::NmtCommand cs) noexcept override
  {
    if (cs == canopen::NmtCommand::RESET_NODE) {
      printf("[sim] NMT reset node: full restart\n");
      SetIdentity();
      SetMeasurements();
      commResetPending_ = false;
      positionReferenced_ = false;
      (*this)[0x603F][0] = std::uint16_t{0};
      SetState(sig::State::kNotReadyToSwitchOn);
      last_cw_ = 0;
      // Re-arm transition 1. Without this the simulator stays in «Not ready
      // to switch on» forever after the master's reset, because the one-shot
      // power-up timer has already fired.
      ArmInit();
    } else if (cs == canopen::NmtCommand::RESET_COMM) {
      printf(
        "[sim] NMT reset communication: state stays %s%s\n", StateName(state_),
        commResetPending_ ? ", communication fault now clearable" : "");
      SetIdentity();
      commResetPending_ = false;
    }
  }

  // Set by a communication fault whose recovery needs an NMT reset
  // communication first; until one arrives the Controlword fault reset is
  // ignored, exactly the behaviour ClearFault() has to cope with.
  bool commResetPending_{false};

  // The master's heartbeat stopped arriving within «Consumer heartbeat time»
  // (0x1016), which the master configured at boot. Lely's own consumer
  // detects it; what the drive does next is the EPOS4's part, section 7.2.35:
  // «CAN heartbeat error» 0x8130, a fault labelled "a", whose reaction is
  // «Abort connection option code» (0x6007). Its default, 3, is a quick stop
  // ramp and then disable; the ramp is not modelled, the axis just stops.
  //
  // Error() sends the EMCY and applies «Error behavior» (0x1029): NMT
  // pre-operational by default, which is what stops this node's PDOs.
  //
  // Nothing happens when the heartbeat comes back. On the real drive the
  // fault stays until an NMT reset communication plus a fault reset; a master
  // that restarts sends the NMT reset as part of booting the node anyway.
  void
  OnHeartbeat(std::uint8_t id, bool occurred) noexcept override
  {
    if (!occurred) {
      printf("[sim] heartbeat of node %u is back; fault stays until reset\n", id);
      return;
    }
    printf("[sim] heartbeat of node %u lost -> CAN heartbeat error 0x8130 -> Fault\n", id);
    (*this)[0x603F][0] = std::uint16_t{0x8130};
    commResetPending_ = true;
    SetState(sig::State::kFault);
    Error(0x8130, 0x10);  // 0x10: communication error, Table 6-60
  }

  sig::State state_{sig::State::kNotReadyToSwitchOn};

  // --- Cyclic Synchronous Position Mode ---
  //
  // The drive does not generate a trajectory here: the master sends a new
  // target every SYNC and the drive interpolates towards it over the
  // interpolation time period (0x60C2). Modelled the same way.
  bool cyclicFollowing_{false};

  // --- Cyclic Synchronous Velocity and Torque ---
  //
  // Not a motor model: enough that a commanded velocity turns into motion
  // and a commanded torque into acceleration, so the cyclic path can be
  // verified end to end. Velocity follows its command through a first-order
  // lag; under torque, velocity grows with the torque against a damping
  // term, so a constant torque settles at a constant speed.
  // --- Homing Mode, section 3.5 ---
  //
  // A virtual axis to home against: mechanical end stops at +-kEndStop, a
  // limit switch at each of them, a home switch across [kHomeSwitchLow,
  // kHomeSwitchHigh], and an encoder index once per revolution. Every method
  // of section 3.5.3 finds its event on it, travels the Home offset move
  // distance (0x30B1), and there sets Home position (0x30B0).
  //
  // A method whose switch is not mapped in 0x3142 never sees its edge: the
  // search runs into the end stop and ends in «Homing error», which is what
  // the drive would do.
  static constexpr std::int32_t kEndStop = 50000;
  static constexpr std::int32_t kHomeSwitchLow = 20000;
  static constexpr std::int32_t kHomeSwitchHigh = 22000;

  enum class HomingPhase {kIdle, kSearch, kOffset};
  HomingPhase homingPhase_{HomingPhase::kIdle};
  std::int32_t homingGoal_{0};
  std::int32_t homingOffsetGoal_{0};
  bool homingAttained_{false};
  bool homingError_{false};
  bool positionReferenced_{false};  // bit 15, survives leaving the mode
  bool lastHomingStart_{false};

  std::int32_t commandedRpm_{0};
  std::int16_t commandedTorque_{0};  // thousandths of rated torque
  double velocityRpm_{0.0};
  std::int16_t torqueActual_{0};

  // --- Profile Position Mode ---
  std::int32_t position_{0};
  double positionExact_{0.0};  // integrates fractions of a count at low speed
  std::int32_t target_{0};
  std::uint32_t profileVelocity_{1000};
  std::int32_t interpolationMs_{10};
  bool moving_{false};
  bool setpointAcknowledged_{false};
  bool targetReached_{true};
  io::Timer * motionTimer_{nullptr};
  bool motionArmed_{false};
  std::uint16_t last_cw_{0};
  io::Timer * init_timer_{nullptr};

  void
  ArmInit()
  {
    if (!init_timer_) {return;}
    init_timer_->submit_wait(
      [this](int /*overrun*/, ::std::error_code ec) {
        if (!ec) {FinishInit();}
      });
    init_timer_->settime(std::chrono::milliseconds(200));
  }

  // The PPM setpoint handshake from the drive's side, Table 3-15.
  //
  // The master raises New setpoint (bit 4); we answer with Setpoint
  // acknowledge (bit 12) and latch the target. When the master lowers bit 4
  // we lower bit 12, which is what frees the next move. A drive that never
  // lowered bit 12 would silently swallow every subsequent setpoint, which is
  // exactly the failure this simulator exists to be able to reproduce.
  void
  HandlePpm(std::uint16_t cw)
  {
    const bool newSetpoint = (cw & sig::control_bits::kNewSetpoint) != 0;

    if (newSetpoint && !setpointAcknowledged_) {
      target_ = (*this)[0x607A][0];
      const std::uint32_t pv = (*this)[0x6081][0];
      if (pv > 0) {profileVelocity_ = pv;}

      const bool relative = (cw & sig::control_bits::kAbsoluteRelative) != 0;
      if (relative) {target_ += position_;}

      setpointAcknowledged_ = true;
      moving_ = true;
      targetReached_ = false;
      printf(
        "[sim] PPM setpoint latched: %d -> %d at %u\n",
        position_, target_, profileVelocity_);
      PublishStatusword();
      ArmMotion();
    } else if (!newSetpoint && setpointAcknowledged_) {
      setpointAcknowledged_ = false;
      PublishStatusword();
    }
  }

  // CSP: take whatever the master last published and interpolate towards it.
  //
  // No setpoint handshake and no profile - that is the whole difference from
  // PPM. The drive is a follower here, and the trajectory lives in the
  // master.
  void
  HandleCsp()
  {
    const std::int32_t target = (*this)[0x607A][0];
    if (target == target_ && cyclicFollowing_) {
      return;
    }
    target_ = target;
    cyclicFollowing_ = true;
    moving_ = true;
    targetReached_ = false;

    // Interpolation time period (0x60C2:01) in milliseconds. Zero means the
    // master did not configure it, and the manual says the drive then jumps
    // to the new value within one control cycle - noisy, but modelled as
    // written rather than smoothed over.
    const std::uint8_t periodMs = (*this)[0x60C2][1];
    interpolationMs_ = (periodMs == 0) ? 1 : periodMs;

    PublishStatusword();
    ArmMotion();
  }

  // CSV and CST: latch the new command (target plus the optional feed
  // forward offset, 0x60B1 / 0x60B2, as sections 3.7 and 3.8 add them) and
  // keep the motion timer running for as long as the mode is followed.
  void
  HandleCsv()
  {
    const std::int32_t target = (*this)[0x60FF][0];
    const std::int32_t offset = (*this)[0x60B1][0];
    commandedRpm_ = target + offset;
    StartFollowing();
  }

  void
  HandleCst()
  {
    const std::int16_t target = (*this)[0x6071][0];
    const std::int16_t offset = (*this)[0x60B2][0];
    commandedTorque_ = static_cast<std::int16_t>(target + offset);
    StartFollowing();
  }

  void
  StartFollowing()
  {
    const bool wasFollowing = cyclicFollowing_;
    cyclicFollowing_ = true;
    moving_ = true;
    if (!wasFollowing) {PublishStatusword();}
    ArmMotion();
  }

  // Counts per motor revolution, to turn counts per tick into rpm. «Main
  // sensor resolution» (0x3000:05) once an encoder is configured; until then
  // 2000, a 500 CPR encoder - the example the demos use.
  double
  CountsPerRevolution()
  {
    const std::uint32_t resolution = (*this)[0x3000][5];
    return resolution == 0 ? 2000.0 : static_cast<double>(resolution);
  }

  // Whether a digital input is mapped to one of the given functions in
  // 0x3142 - the check the drive makes before it can see a switch.
  bool
  InputMapped(std::uint8_t function, std::uint8_t alternative)
  {
    for (std::uint8_t sub = 1; sub <= 8; ++sub) {
      const std::uint8_t mapped = (*this)[0x3142][sub];
      if (mapped == function || mapped == alternative) {return true;}
    }
    return false;
  }

  // The next index pulse from `from`, strictly in direction `dir`: pulses
  // sit at every multiple of a revolution. Floor division, so negative
  // positions round the same way as positive ones.
  std::int32_t
  NextIndex(std::int32_t from, int dir)
  {
    const auto rev = static_cast<std::int32_t>(CountsPerRevolution());
    auto floorDiv = [](std::int32_t a, std::int32_t b) {
        return a / b - ((a % b != 0) && ((a < 0) != (b < 0)) ? 1 : 0);
      };
    if (dir > 0) {
      return (floorDiv(from, rev) + 1) * rev;
    }
    return -(floorDiv(-from, rev) + 1) * rev;
  }

  // Homing operation start (Controlword bit 4), rising edge: plan the run.
  void
  StartHoming()
  {
    using M = sig::HomingMethod;
    const auto method = static_cast<M>(std::int8_t{(*this)[0x6098][0]});
    const std::int32_t offset = (*this)[0x30B1][0];
    const bool negLimit = InputMapped(0, 24);
    const bool posLimit = InputMapped(1, 25);
    const bool homeSwitch = InputMapped(2, 2);

    homingAttained_ = false;
    homingError_ = false;
    targetReached_ = false;

    // Event: where the method detects its reference. Away: the direction
    // the offset move travels, away from a stop or switch at the side.
    std::int32_t event = position_;
    int away = offset >= 0 ? 1 : -1;
    bool reachable = true;
    switch (method) {
      case M::kActualPosition: event = position_; break;
      case M::kCurrentThresholdPositiveSpeed: event = kEndStop; away = -1; break;
      case M::kCurrentThresholdNegativeSpeed: event = -kEndStop; away = 1; break;
      case M::kCurrentThresholdPositiveSpeedAndIndex:
        event = NextIndex(kEndStop, -1); away = -1; break;
      case M::kCurrentThresholdNegativeSpeedAndIndex:
        event = NextIndex(-kEndStop, 1); away = 1; break;
      case M::kNegativeLimitSwitch: event = -kEndStop; away = 1; reachable = negLimit; break;
      case M::kPositiveLimitSwitch: event = kEndStop; away = -1; reachable = posLimit; break;
      case M::kNegativeLimitSwitchAndIndex:
        event = NextIndex(-kEndStop, 1); away = 1; reachable = negLimit; break;
      case M::kPositiveLimitSwitchAndIndex:
        event = NextIndex(kEndStop, -1); away = -1; reachable = posLimit; break;
      case M::kHomeSwitchPositiveSpeed: event = kHomeSwitchLow; reachable = homeSwitch; break;
      case M::kHomeSwitchNegativeSpeed: event = kHomeSwitchHigh; reachable = homeSwitch; break;
      case M::kHomeSwitchPositiveSpeedAndIndex:
        event = NextIndex(kHomeSwitchLow, 1); reachable = homeSwitch; break;
      case M::kHomeSwitchNegativeSpeedAndIndex:
        event = NextIndex(kHomeSwitchHigh, -1); reachable = homeSwitch; break;
      case M::kIndexNegativeSpeed: event = NextIndex(position_, -1); break;
      case M::kIndexPositiveSpeed: event = NextIndex(position_, 1); break;
      default:
        printf(
          "[sim] homing method %d not supported -> homing error\n",
          static_cast<int>(method));
        homingError_ = true;
        PublishStatusword();
        return;
    }

    if (!reachable) {
      // The switch is never seen: the search drives into the end stop on
      // that side and stalls there.
      event = (method == M::kPositiveLimitSwitch || method == M::kPositiveLimitSwitchAndIndex ||
        method == M::kHomeSwitchPositiveSpeed || method == M::kHomeSwitchPositiveSpeedAndIndex) ?
        kEndStop : -kEndStop;
      printf(
        "[sim] homing method %d: its switch is not mapped in 0x3142, searching blind\n",
        static_cast<int>(method));
    }

    homingGoal_ = event;
    homingOffsetGoal_ = reachable ? event + away * (offset >= 0 ? offset : -offset) : event;
    homingPhase_ = HomingPhase::kSearch;
    stallOnArrival_ = !reachable;
    printf(
      "[sim] homing method %d: search to %d, offset move to %d\n",
      static_cast<int>(method), homingGoal_, homingOffsetGoal_);
    moving_ = true;
    PublishStatusword();
    ArmMotion();
  }

  bool stallOnArrival_{false};

  // One tick of a homing run: towards the event at «Speed for switch search»
  // (0x6099:01), then the offset move, then Home position.
  void
  StepHoming(std::int32_t tickMs)
  {
    const std::uint32_t rpm = (*this)[0x6099][1];
    std::int32_t step = static_cast<std::int32_t>(
      (rpm == 0 ? 100.0 : rpm) / 60.0 * CountsPerRevolution() * tickMs / 1000.0);
    if (step < 1) {step = 1;}

    const std::int32_t goal = homingPhase_ ==
      HomingPhase::kSearch ? homingGoal_ : homingOffsetGoal_;
    const std::int32_t remaining = goal - position_;
    position_ = std::abs(remaining) <= step ? goal : position_ + (remaining > 0 ? step : -step);

    // Pressing into an end stop is what the current threshold methods sense.
    SetCurrent(true);
    if (std::abs(position_) >= kEndStop) {
      const std::uint16_t threshold = (*this)[0x30B2][0];
      (*this)[0x30D1][1] = static_cast<std::int32_t>(threshold + 500);
    }
    PublishDigitalInputs();

    if (position_ != goal) {return;}

    if (stallOnArrival_) {
      printf("[sim] homing: stalled at the end stop without seeing the switch -> homing error\n");
      homingPhase_ = HomingPhase::kIdle;
      homingError_ = true;
      moving_ = false;
      stallOnArrival_ = false;
      return;
    }
    if (homingPhase_ == HomingPhase::kSearch) {
      homingPhase_ = HomingPhase::kOffset;
      return;
    }

    // Offset move done: this point is the Home position.
    const std::int32_t home = (*this)[0x30B0][0];
    printf("[sim] homing attained: position %d is now %d\n", position_, home);
    position_ = home;
    homingPhase_ = HomingPhase::kIdle;
    homingAttained_ = true;
    positionReferenced_ = true;
    targetReached_ = true;
    moving_ = false;
    SetCurrent(false);
  }

  // --- Touch probe 1, sections 6.2.134-142 ---
  //
  // The input mapped to «Touch probe» (0x3142 = 26) sees a sensor across the
  // same stretch as the home switch; the index trigger sees a pulse
  // kIndexWidth counts wide at every revolution. Edges are found exactly
  // between two positions, so the latched value is the edge position, not
  // wherever the tick happened to land.
  static constexpr std::int32_t kIndexWidth = 4;
  std::uint16_t touchProbeFunction_{0};
  std::uint16_t positiveEdges_{0};
  std::uint16_t negativeEdges_{0};

  void
  ConfigureTouchProbe()
  {
    touchProbeFunction_ = (*this)[0x60B8][0];
    positiveEdges_ = 0;
    negativeEdges_ = 0;
    std::uint16_t status = (touchProbeFunction_ & 1u) ? 1u : 0u;
    (*this)[0x60B9][0] = status;
    (*this)[0x60D5][0] = positiveEdges_;
    (*this)[0x60D6][0] = negativeEdges_;
  }

  // Every rising and falling edge of the probed signal between `from` and
  // `to`, in the order the axis meets them.
  void
  SampleTouchProbe(std::int32_t from, std::int32_t to)
  {
    if (!(touchProbeFunction_ & 1u) || from == to) {return;}
    const bool index = ((touchProbeFunction_ >> 2) & 3u) == 1u;
    if (!index && !InputMapped(26, 26)) {return;}

    const int dir = to > from ? 1 : -1;
    auto crossing = [&](std::int32_t low, std::int32_t high) {
        // Entering the active stretch is the rising edge, leaving it the
        // falling one - at `low` or `high` depending on the direction.
        const std::int32_t enter = dir > 0 ? low : high;
        const std::int32_t leave = dir > 0 ? high : low;
        auto passes = [&](std::int32_t p) {
            return dir > 0 ? (p > from && p <= to) : (p < from && p >= to);
          };
        if (passes(enter)) {Latch(true, enter);}
        if (passes(leave)) {Latch(false, leave);}
      };
    if (!index) {
      crossing(kHomeSwitchLow, kHomeSwitchHigh);
      return;
    }
    const auto rev = static_cast<std::int32_t>(CountsPerRevolution());
    const std::int32_t lo = std::min(from, to), hi = std::max(from, to);
    for (std::int32_t k = (lo / rev) - 1; k <= (hi / rev) + 1; ++k) {
      crossing(k * rev, k * rev + kIndexWidth);
    }
  }

  void
  Latch(bool positive, std::int32_t position)
  {
    const bool continuous = (touchProbeFunction_ & (1u << 1)) != 0;
    const bool wanted =
      positive ? (touchProbeFunction_ & (1u << 4)) : (touchProbeFunction_ & (1u << 5));
    std::uint16_t & count = positive ? positiveEdges_ : negativeEdges_;
    if (!wanted || (!continuous && count > 0)) {return;}
    ++count;
    std::uint16_t status = (*this)[0x60B9][0];
    if (positive) {
      (*this)[0x60BA][0] = position;
      (*this)[0x60D5][0] = count;
      status |= 1u << 1;
    } else {
      (*this)[0x60BB][0] = position;
      (*this)[0x60D6][0] = count;
      status |= 1u << 2;
    }
    (*this)[0x60B9][0] = status;
    printf(
      "[sim] touch probe: %s edge latched at %d (count %u)\n",
      positive ? "positive" : "negative", position, count);
  }

  // 0x3150:01, the pins: each output pin (0x3151:01..03) shows the state of
  // the function assigned to it in 0x60FE:01, inverted where its bit in
  // «Digital outputs polarity» (0x3150:02) is set - sections 6.2.76-77.
  void
  PublishDigitalOutputPins()
  {
    const std::uint32_t functions = (*this)[0x60FE][1];
    const std::uint16_t polarity = (*this)[0x3150][2];
    std::uint16_t pins = 0;
    for (std::uint8_t sub = 1; sub <= 3; ++sub) {
      const std::uint8_t function = (*this)[0x3151][sub];
      bool state = function != 255 && ((functions >> function) & 1u);
      if ((polarity >> (sub - 1)) & 1u) {state = !state;}
      if (state) {pins |= static_cast<std::uint16_t>(1u << (sub - 1));}
    }
    (*this)[0x3150][1] = pins;
  }

  // 0x60FD, by function: bit 0 negative limit, bit 1 positive limit, bit 2
  // home switch - each only if an input is mapped to it, as on the drive.
  void
  PublishDigitalInputs()
  {
    std::uint32_t bits = 0;
    if (position_ <= -kEndStop && InputMapped(0, 24)) {bits |= 1u << 0;}
    if (position_ >= kEndStop && InputMapped(1, 25)) {bits |= 1u << 1;}
    if (position_ >= kHomeSwitchLow && position_ <= kHomeSwitchHigh && InputMapped(2, 2)) {
      bits |= 1u << 2;
    }
    (*this)[0x60FD][0] = bits;
  }

  // A crude but honest trapezoid-free ramp: step towards the target at the
  // profile velocity. Enough to exercise Target reached and the handshake;
  // not a claim to model the drive's real trajectory generator.
  void
  StepMotion()
  {
    if (!moving_) {return;}

    constexpr std::int32_t kTickMs = 20;
    const std::int32_t before = position_;

    if (homingPhase_ != HomingPhase::kIdle) {
      StepHoming(kTickMs);
      // «After homing, all touch probe states and latched positions are
      // cleared» (6.2.134) - and the two cannot run at the same time.
      if (homingPhase_ == HomingPhase::kIdle && homingAttained_) {ConfigureTouchProbe();}
      positionExact_ = position_;
      (*this)[0x6064][0] = position_;
      (*this)[0x606C][0] = static_cast<std::int32_t>(
        (position_ - before) * 60.0 * 1000.0 / kTickMs / CountsPerRevolution());
      PublishStatusword();
      if (moving_) {ArmMotion();}
      return;
    }

    const auto mode = static_cast<sig::OperationMode>(std::int8_t{(*this)[0x6061][0]});
    if (mode == sig::OperationMode::kCyclicSynchronousVelocity ||
      mode == sig::OperationMode::kCyclicSynchronousTorque)
    {
      constexpr double dt = kTickMs / 1000.0;
      if (mode == sig::OperationMode::kCyclicSynchronousVelocity) {
        velocityRpm_ += (commandedRpm_ - velocityRpm_) * 0.4;   // first-order lag
        torqueActual_ = static_cast<std::int16_t>((commandedRpm_ - velocityRpm_) * 0.5);
      } else {
        torqueActual_ = commandedTorque_;
        velocityRpm_ += (torqueActual_ * 2.0 - velocityRpm_) * 0.2;  // torque vs damping
      }
      positionExact_ += velocityRpm_ / 60.0 * CountsPerRevolution() * dt;
      position_ = static_cast<std::int32_t>(positionExact_);
      SampleTouchProbe(before, position_);
      (*this)[0x6064][0] = position_;
      (*this)[0x606C][0] = static_cast<std::int32_t>(velocityRpm_);
      (*this)[0x6077][0] = torqueActual_;
      SetCurrent(velocityRpm_ != 0.0 || torqueActual_ != 0);
      ArmMotion();
      return;
    }

    // Under CSP the move is bounded by the interpolation period, not by a
    // profile velocity: the drive has to arrive by the time the next setpoint
    // is due.
    const std::int32_t stepSize =
      cyclicFollowing_ ?
      ((target_ - position_) * kTickMs) /
      (interpolationMs_ > kTickMs ? interpolationMs_ : kTickMs) :
      static_cast<std::int32_t>(profileVelocity_) * kTickMs / 1000;
    const std::int32_t remaining = target_ - position_;

    if (std::abs(remaining) <= std::abs(stepSize) || stepSize == 0) {
      position_ = target_;
      moving_ = false;
      targetReached_ = true;
      printf("[sim] PPM target reached at %d\n", position_);
    } else {
      position_ += (remaining > 0) ? stepSize : -stepSize;
    }

    SampleTouchProbe(before, position_);
    (*this)[0x6064][0] = position_;
    positionExact_ = position_;
    // Velocity actual from how far this tick moved; torque is not modelled
    // in the position modes.
    (*this)[0x606C][0] = static_cast<std::int32_t>(
      (position_ - before) * 60.0 * 1000.0 / kTickMs / CountsPerRevolution());
    SetCurrent(moving_);
    PublishStatusword();
    if (moving_) {ArmMotion();}
  }

  // Arms the next motion step unless one is already pending.
  //
  // Under CSP this is called on every new target, i.e. on every SYNC. With a
  // 10 ms SYNC and a 20 ms tick, re-arming each time pushed the expiry out
  // before it was ever reached: the timer never fired, the position never
  // left zero, and the trajectory controller tripped its tolerance while the
  // targets kept arriving perfectly. A step already pending will read the
  // latest target_ when it runs, so there is nothing to re-arm for.
  // Motor current, 0x30D1: sub 2 instantaneous, sub 1 averaged, in mA. Not a
  // model of the motor - just enough that moving and holding read
  // differently, so a caller watching current can see the axis work.
  void
  SetCurrent(bool moving)
  {
    const std::int32_t milliamps = moving ? 800 : 50;
    (*this)[0x30D1][2] = milliamps;
    (*this)[0x30D1][1] = milliamps;
  }

  void
  ArmMotion()
  {
    if (!motionTimer_ || motionArmed_) {return;}
    motionArmed_ = true;
    motionTimer_->submit_wait(
      [this](int, ::std::error_code ec) {
        motionArmed_ = false;
        if (!ec) {StepMotion();}
      });
    motionTimer_->settime(std::chrono::milliseconds(20));
  }

  void
  PublishStatusword()
  {
    std::uint16_t sw = static_cast<std::uint16_t>(state_);
    if (state_ == sig::State::kOperationEnabled ||
      state_ == sig::State::kSwitchedOn ||
      state_ == sig::State::kQuickStopActive)
    {
      sw |= sig::status_bits::kVoltageEnabled;
    }
    if (setpointAcknowledged_) {sw |= sig::status_bits::kSetpointAcknowledge;}
    if (targetReached_) {sw |= sig::status_bits::kTargetReached;}
    // Bit 12 under CSP is «Drive follows command value», not «Setpoint
    // acknowledge». Same bit, different meaning, decided by 0x6061.
    if (cyclicFollowing_) {sw |= sig::status_bits::kFollowsCommandValue;}
    // Bits 12 and 13 mean «Homing attained» and «Homing error» only in HMM.
    const auto mode = static_cast<sig::OperationMode>(std::int8_t{(*this)[0x6061][0]});
    if (mode == sig::OperationMode::kHoming) {
      if (homingAttained_) {sw |= sig::status_bits::kHomingAttained;}
      if (homingError_) {sw |= sig::status_bits::kHomingError;}
    }
    if (positionReferenced_) {sw |= sig::status_bits::kHomeRefValid;}
    (*this)[0x6041][0] = sw;
  }

  void
  SetState(sig::State s)
  {
    state_ = s;
    if (s != sig::State::kOperationEnabled) {
      // Losing power aborts whatever move was running, and a motor without
      // power neither turns nor produces torque.
      moving_ = false;
      setpointAcknowledged_ = false;
      cyclicFollowing_ = false;
      homingPhase_ = HomingPhase::kIdle;
      commandedRpm_ = 0;
      commandedTorque_ = 0;
      velocityRpm_ = 0.0;
      torqueActual_ = 0;
      (*this)[0x606C][0] = std::int32_t{0};
      (*this)[0x6077][0] = std::int16_t{0};
    }
    PublishStatusword();
  }

  // Evaluates the Controlword currently in the dictionary: the state machine
  // first, then whatever the active mode does with its mode bits.
  void
  HandleControlword()
  {
    {
      const std::uint16_t cw = (*this)[0x6040][0];

      // Fault reset is edge triggered: only a 0->1 transition of bit 7 counts.
      // A master that holds the bit high cannot clear a second fault.
      const bool reset_edge =
        (cw & sig::control_bits::kFaultReset) != 0 &&
        (last_cw_ & sig::control_bits::kFaultReset) == 0;
      last_cw_ = cw;

      if (reset_edge && state_ == sig::State::kFault) {
        if (commResetPending_) {
          printf(
            "[sim] cw=0x%04X  fault reset IGNORED: 0x8130 needs an NMT reset "
            "communication first (7.2.35)\n", cw);
          return;
        }
        printf("[sim] cw=0x%04X  fault reset edge -> Switch on disabled\n", cw);
        (*this)[0x603F][0] = std::uint16_t{0};
        SetState(sig::State::kSwitchOnDisabled);
        return;
      }

      const sig::State next = NextState(state_, cw);
      if (next != state_) {
        printf(
          "[sim] cw=0x%04X  %s -> %s\n", cw, StateName(state_), StateName(next));
        SetState(next);
      } else {
        printf("[sim] cw=0x%04X  %s (no transition)\n", cw, StateName(state_));
      }

      // Mode-specific handling, after the state machine has had its say.
      const std::int8_t mode = (*this)[0x6061][0];
      const auto activeMode = static_cast<sig::OperationMode>(mode);

      if (activeMode == sig::OperationMode::kProfilePosition &&
        state_ == sig::State::kOperationEnabled)
      {
        HandlePpm(cw);
      } else if (activeMode == sig::OperationMode::kHoming &&
        state_ == sig::State::kOperationEnabled)
      {
        HandleHoming(cw);
      } else if (state_ == sig::State::kOperationEnabled) {
        HandleCyclicTarget(activeMode);
      }
    }
  }

  // Controlword under HMM (Table 3-29): bit 4 starts the run on its rising
  // edge, bit 8 «Halt» stops it - Target reached without Homing attained,
  // which Table 3-33 reads as "interrupted".
  void
  HandleHoming(std::uint16_t cw)
  {
    const bool start = (cw & sig::control_bits::kHomingOperationStart) != 0;
    const bool halt = (cw & sig::control_bits::kHalt) != 0;
    if (halt && homingPhase_ != HomingPhase::kIdle) {
      printf("[sim] homing halted\n");
      homingPhase_ = HomingPhase::kIdle;
      moving_ = false;
      targetReached_ = true;
    } else if (start && !lastHomingStart_ && !halt) {
      StartHoming();
    }
    lastHomingStart_ = start;
    PublishStatusword();
  }

  // The cyclic modes have no handshake: the drive follows whatever target
  // the RPDO last carried. Called on a Controlword (entering the mode) and
  // on every write of a target or offset.
  void
  HandleCyclicTarget(sig::OperationMode mode)
  {
    switch (mode) {
      case sig::OperationMode::kCyclicSynchronousPosition: HandleCsp(); break;
      case sig::OperationMode::kCyclicSynchronousVelocity: HandleCsv(); break;
      case sig::OperationMode::kCyclicSynchronousTorque: HandleCst(); break;
      default: break;
    }
  }

  // Called whenever the master writes an object over SDO or RPDO.
  void
  OnWrite(std::uint16_t idx, std::uint8_t subidx) noexcept override
  {
    if (idx == 0x6040 && subidx == 0) {
      // Deferred, not handled inline. Lely writes the objects of a received
      // RPDO one at a time in mapping order, and calls this after each one.
      // RPDO1 carries the Controlword BEFORE Target position, so handling it
      // here would latch the PPM setpoint while 0x607A still held the previous
      // value - the master raised «New setpoint» and sent the new target in
      // the very same frame. A real drive applies the whole PDO before acting
      // on it; posting to the executor runs this once the frame is complete.
      GetExecutor().post([this]() {HandleControlword();});
    } else if (
      (idx == 0x607A || idx == 0x60FF || idx == 0x6071 || idx == 0x60B1 || idx == 0x60B2) &&
      subidx == 0)
    {
      // A new target or offset arriving by RPDO is what drives the cyclic
      // modes. In PPM, Target position is latched by the setpoint handshake
      // instead; HandleCyclicTarget ignores every mode but CSP/CSV/CST.
      // Deferred for the same reason as the Controlword: act on the whole
      // PDO, not on the first object of it.
      if (state_ == sig::State::kOperationEnabled) {
        GetExecutor().post(
          [this]() {
            const std::int8_t mode = (*this)[0x6061][0];
            HandleCyclicTarget(static_cast<sig::OperationMode>(mode));
          });
      }
    } else if (idx == 0x60B8 && subidx == 0) {
      ConfigureTouchProbe();
    } else if (idx == 0x60FE || idx == 0x3151 || (idx == 0x3150 && subidx == 2)) {
      PublishDigitalOutputPins();
    } else if (idx == 0x6060 && subidx == 0) {
      // Modes of operation: echo the request into Modes of operation display,
      // which is what a real drive does once it has actually switched.
      const std::int8_t mode = (*this)[0x6060][0];
      (*this)[0x6061][0] = mode;
      printf("[sim] mode of operation -> %d\n", static_cast<int>(mode));
    }
  }
};


int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, 0);

  const std::string eds = (argc > 1) ? argv[1] : "epos4.eds";
  const std::uint8_t node_id =
    static_cast<std::uint8_t>((argc > 2) ? std::atoi(argv[2]) : 2);
  const std::string iface = (argc > 3) ? argv[3] : "vcan0";

  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);

  io::CanController ctrl(iface.c_str());
  io::CanChannel chan(poll, exec);
  chan.open(ctrl);

  Epos4Sim sim(timer, chan, eds, "", node_id);

  printf(
    "[sim] EPOS4 simulator on %s, node %d, eds=%s\n",
    iface.c_str(), node_id, eds.c_str());

  // Transition 1 after a short delay, mimicking a real power-up. The
  // simulator re-arms this itself on every NMT reset.
  io::Timer init_timer(poll, exec, CLOCK_MONOTONIC);
  io::Timer motion_timer(poll, exec, CLOCK_MONOTONIC);

  io::SignalSet sigset(poll, exec);
  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);
  sigset.submit_wait([&](int) {sigset.clear(); ctx.shutdown();});

  sim.Reset();

  // After Reset(), not before: Reset() reloads the object dictionary from the
  // EDS defaults and would wipe anything written earlier.
  sim.SetIdentity();
  sim.SetMeasurements();
  sim.UseInitTimer(init_timer);
  sim.UseMotionTimer(motion_timer);

  loop.run();
  return 0;
}
