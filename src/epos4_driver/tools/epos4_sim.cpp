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
// Run:
//   ./build/epos4_driver/epos4_sim \
//       install/epos4_bringup/share/epos4_bringup/config/epos4_network/epos4.eds 2

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
  // An NMT reset from the master reloads the object dictionary from the EDS,
  // which wipes the identity written by SetIdentity() and makes the master
  // reject us with es='D' on the very next boot attempt. Re-apply it, and go
  // back to «Not ready to switch on» like a drive that just powered up.
  void
  OnCommand(canopen::NmtCommand cs) noexcept override
  {
    if (cs == canopen::NmtCommand::RESET_NODE ||
      cs == canopen::NmtCommand::RESET_COMM)
    {
      printf("[sim] NMT reset received, restoring identity\n");
      SetIdentity();
      SetState(sig::State::kNotReadyToSwitchOn);
      last_cw_ = 0;
      // Re-arm transition 1. Without this the simulator stays in «Not ready
      // to switch on» forever after the master's reset, because the one-shot
      // power-up timer has already fired.
      ArmInit();
    }
  }

  sig::State state_{sig::State::kNotReadyToSwitchOn};

  // --- Profile Position Mode ---
  std::int32_t position_{0};
  std::int32_t target_{0};
  std::uint32_t profileVelocity_{1000};
  bool moving_{false};
  bool setpointAcknowledged_{false};
  bool targetReached_{true};
  io::Timer * motionTimer_{nullptr};
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

  // A crude but honest trapezoid-free ramp: step towards the target at the
  // profile velocity. Enough to exercise Target reached and the handshake;
  // not a claim to model the drive's real trajectory generator.
  void
  StepMotion()
  {
    if (!moving_) {return;}

    constexpr std::int32_t kTickMs = 20;
    const std::int32_t stepSize =
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

  void
  ArmMotion()
  {
    if (!motionTimer_) {return;}
    motionTimer_->submit_wait(
      [this](int, ::std::error_code ec) {
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
    }
    PublishStatusword();
  }

  // Called whenever the master writes an object over SDO or RPDO.
  void
  OnWrite(std::uint16_t idx, std::uint8_t subidx) noexcept override
  {
    if (idx == 0x6040 && subidx == 0) {
      const std::uint16_t cw = (*this)[0x6040][0];

      // Fault reset is edge triggered: only a 0->1 transition of bit 7 counts.
      // A master that holds the bit high cannot clear a second fault.
      const bool reset_edge =
        (cw & sig::control_bits::kFaultReset) != 0 &&
        (last_cw_ & sig::control_bits::kFaultReset) == 0;
      last_cw_ = cw;

      if (reset_edge && state_ == sig::State::kFault) {
        printf("[sim] cw=0x%04X  fault reset edge -> Switch on disabled\n", cw);
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
      if (static_cast<sig::OperationMode>(mode) ==
        sig::OperationMode::kProfilePosition &&
        state_ == sig::State::kOperationEnabled)
      {
        HandlePpm(cw);
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
