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
//   CFG=install/epos4_bringup/share/epos4_bringup/config/epos4_network
//   ./build/epos4_driver/epos4_sim $CFG/epos4.eds 2 vcan0

#include <lely/coapp/slave.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/clock.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

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
      commResetPending_ = false;
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
  bool cspFollowing_{false};

  // --- Profile Position Mode ---
  std::int32_t position_{0};
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
    if (target == target_ && cspFollowing_) {
      return;
    }
    target_ = target;
    cspFollowing_ = true;
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

  // A crude but honest trapezoid-free ramp: step towards the target at the
  // profile velocity. Enough to exercise Target reached and the handshake;
  // not a claim to model the drive's real trajectory generator.
  void
  StepMotion()
  {
    if (!moving_) {return;}

    constexpr std::int32_t kTickMs = 20;

    // Under CSP the move is bounded by the interpolation period, not by a
    // profile velocity: the drive has to arrive by the time the next setpoint
    // is due.
    const std::int32_t stepSize =
      cspFollowing_ ?
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

    (*this)[0x6064][0] = position_;
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
    if (cspFollowing_) {sw |= sig::status_bits::kFollowsCommandValue;}
    (*this)[0x6041][0] = sw;
  }

  void
  SetState(sig::State s)
  {
    state_ = s;
    if (s != sig::State::kOperationEnabled) {
      // Losing power aborts whatever move was running.
      moving_ = false;
      setpointAcknowledged_ = false;
      cspFollowing_ = false;
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
      } else if (activeMode == sig::OperationMode::kCyclicSynchronousPosition &&
        state_ == sig::State::kOperationEnabled)
      {
        HandleCsp();
      }
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
    } else if (idx == 0x607A && subidx == 0) {
      // A new target arriving by RPDO is what drives CSP. In PPM the same
      // object is latched by the setpoint handshake instead, so the mode
      // decides which of the two applies.
      const std::int8_t mode = (*this)[0x6061][0];
      if (static_cast<sig::OperationMode>(mode) ==
        sig::OperationMode::kCyclicSynchronousPosition &&
        state_ == sig::State::kOperationEnabled)
      {
        HandleCsp();
      }
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
  sim.UseInitTimer(init_timer);
  sim.UseMotionTimer(motion_timer);

  loop.run();
  return 0;
}
