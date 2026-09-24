#include "epos4/hardware/Epos4.hpp"

#include <lely/coapp/loop_driver.hpp>
#include <lely/coapp/sdo_error.hpp>

#include <atomic>
#include <future>
#include <mutex>
#include <thread>

#include "epos4/core/BusImpl.hpp"
#include "epos4/core/Cia402StateMachine.hpp"
#include "epos4/hardware/Encoder.hpp"
#include "epos4/signals/Errors.hpp"

namespace epos4
{

namespace
{
constexpr auto kPollInterval = std::chrono::milliseconds{5};
}

// ---------------------------------------------------------------------------
// The Lely side of a device. Kept in the .cpp so that no Lely header reaches
// anything a user of this library includes.
//
// Every public call below is synchronous from the caller's thread: it posts
// onto the driver's executor and blocks on a future until the answer comes
// back.
//
// LoopDriver rather than FiberDriver, for two reasons:
//
//  1. FiberDriver documents that it "MUST be instantiated on the thread on
//     which its tasks are run", because its base member initialises fibers
//     for the *current* thread. A device constructed from application code
//     while the bus loop runs elsewhere trips
//     `fiber_resume_with: Assertion 'curr' failed` the first time it is used.
//     LoopDriver brings its own thread and initialises it itself.
//
//  2. Each axis gets its own loop, so a drive that is slow to answer holds up
//     only itself. On a six-axis arm sharing one bus that is the difference
//     between one stiff joint and a stalled arm.
//
// The one rule: these must not be called from the driver's own thread, which
// would deadlock. In practice that means not from inside a driver callback.
// ---------------------------------------------------------------------------
struct Epos4::Impl : public lely::canopen::LoopDriver
{
  Impl(lely::canopen::AsyncMaster & master, std::uint8_t id)
  : LoopDriver(master, id) {}

  template<typename T>
  std::error_code Read(od::Entry entry, T & out)
  {
    std::promise<std::error_code> promise;
    auto future = promise.get_future();
    Defer(
      [this, entry, &out, &promise]() {
        try {
          out = Wait(AsyncRead<T>(entry.index, entry.subindex));
          promise.set_value(std::error_code{});
        } catch (lely::canopen::SdoError & e) {
          promise.set_value(e.code());
        } catch (...) {
          promise.set_value(std::make_error_code(std::errc::io_error));
        }
      });
    return future.get();
  }

  template<typename T>
  std::error_code Write(od::Entry entry, T value)
  {
    std::promise<std::error_code> promise;
    auto future = promise.get_future();
    Defer(
      [this, entry, value, &promise]() {
        try {
          Wait(AsyncWrite<T>(entry.index, entry.subindex, T{value}));
          promise.set_value(std::error_code{});
        } catch (lely::canopen::SdoError & e) {
          promise.set_value(e.code());
        } catch (...) {
          promise.set_value(std::make_error_code(std::errc::io_error));
        }
      });
    return future.get();
  }

  // -------------------------------------------------------------------------
  // PDO
  //
  // Once the master has booted the node with the mapping from the DCF, values
  // arrive on every SYNC without anybody asking. Reading them costs nothing:
  // no frames, no round trip, no waiting. That is the difference that makes a
  // control loop at 100 Hz or more possible at all - an SDO read is two CAN
  // frames and a latency the loop cannot absorb, times every axis on the bus.
  //
  // pdoActive_ flips on the first inbound PDO rather than being configured,
  // so the same code works whether or not the DCF happens to map anything.
  // -------------------------------------------------------------------------

  void OnRpdoWrite(std::uint16_t idx, std::uint8_t subidx) noexcept override
  {
    pdoActive.store(true, std::memory_order_relaxed);
    lastPdo.store(
      std::chrono::steady_clock::now().time_since_epoch().count(),
      std::memory_order_relaxed);
    (void)idx;
    (void)subidx;
  }

  // Reads a value the drive publishes over TPDO. Posted onto the driver's
  // thread because the mapped-object accessors are not thread safe.
  template<typename T>
  std::error_code ReadMapped(od::Entry entry, T & out)
  {
    std::promise<std::error_code> promise;
    auto future = promise.get_future();
    Defer(
      [this, entry, &out, &promise]() {
        try {
          out = rpdo_mapped[entry.index][entry.subindex];
          promise.set_value(std::error_code{});
        } catch (...) {
          promise.set_value(std::make_error_code(std::errc::no_message_available));
        }
      });
    return future.get();
  }

  // Stages a value the master publishes over RPDO. It goes out on the next
  // SYNC, not immediately: that is the point of the cyclic modes.
  template<typename T>
  std::error_code WriteMapped(od::Entry entry, T value)
  {
    std::promise<std::error_code> promise;
    auto future = promise.get_future();
    Defer(
      [this, entry, value, &promise]() {
        try {
          tpdo_mapped[entry.index][entry.subindex] = value;
          promise.set_value(std::error_code{});
        } catch (...) {
          promise.set_value(std::make_error_code(std::errc::no_message_available));
        }
      });
    return future.get();
  }

  bool PdoActive() const {return pdoActive.load(std::memory_order_relaxed);}

  // Reads over PDO when it is available and falls back to SDO otherwise, so
  // callers never have to branch on the transport.
  template<typename T>
  std::error_code ReadPreferPdo(od::Entry entry, T & out)
  {
    if (PdoActive()) {
      if (!ReadMapped<T>(entry, out)) {return {};}
    }
    return Read<T>(entry, out);
  }

  std::error_code ReadStatusword(std::uint16_t & out)
  {
    return ReadPreferPdo<std::uint16_t>(od::At(od::cia402::kStatusword), out);
  }

  // Outbound mapping is independent of inbound: the master may be
  // transmitting the Controlword on every SYNC while the drive publishes
  // nothing back. Gating this on PdoActive(), which only knows about
  // *received* PDOs, meant writing the Controlword over SDO while the master
  // kept overwriting it with the RPDO's stale zero a few milliseconds later,
  // dropping the axis back to «Switch on disabled» after every command.
  //
  // So: always try the mapped write, and fall back to SDO only when the
  // object is not mapped at all.
  std::error_code WriteControlword()
  {
    const od::Entry cw = od::At(od::cia402::kControlword);
    if (!WriteMapped<std::uint16_t>(cw, controlword.Raw())) {return {};}
    return Write<std::uint16_t>(cw, controlword.Raw());
  }

  std::atomic<bool> pdoActive{false};
  std::atomic<long long> lastPdo{0};

  // Makes sure the drive is in the mode a control request needs. Writing
  // 0x6060 only asks; the manual recommends confirming with 0x6061, because a
  // mode change the drive refused would otherwise go unnoticed and every
  // subsequent command would be interpreted by the wrong mode.
  std::error_code EnsureMode(signals::OperationMode mode)
  {
    std::int8_t active{};
    auto ec = Read<std::int8_t>(od::At(od::cia402::kModesOfOperationDisplay), active);
    if (ec) {return ec;}
    if (static_cast<signals::OperationMode>(active) == mode) {return {};}

    ec = Write<std::int8_t>(
      od::At(od::cia402::kModesOfOperation),
      static_cast<std::int8_t>(mode));
    if (ec) {return ec;}

    for (int i = 0; i < 40; ++i) {
      ec = Read<std::int8_t>(od::At(od::cia402::kModesOfOperationDisplay), active);
      if (ec) {return ec;}
      if (static_cast<signals::OperationMode>(active) == mode) {return {};}
      std::this_thread::sleep_for(kPollInterval);
    }
    return std::make_error_code(std::errc::timed_out);
  }

  core::Controlword controlword;

  // Status signals live here so the device can hand out references to them.
  signals::StatusSignal<signals::State> state;
  signals::StatusSignal<std::uint16_t> statusword;
  signals::StatusSignal<signals::OperationMode> mode;
  signals::StatusSignal<std::int32_t> position;
  signals::StatusSignal<std::int32_t> positionDemand;
  signals::StatusSignal<std::int32_t> velocity;
  signals::StatusSignal<std::int32_t> velocityDemand;
  signals::StatusSignal<std::int16_t> torque;
  signals::StatusSignal<std::int32_t> followingError;
  signals::StatusSignal<std::int16_t> currentDemand;
  signals::StatusSignal<std::uint16_t> errorCode;
  signals::StatusSignal<std::uint8_t> errorRegister;
  signals::StatusSignal<bool> targetReached;
  signals::StatusSignal<bool> setpointAcknowledged;
  signals::StatusSignal<bool> followingErrorFlag;
  signals::StatusSignal<bool> homingAttained;
  signals::StatusSignal<bool> internalLimit;
  signals::StatusSignal<bool> warning;
  signals::StatusSignal<signals::BrakeState> brakeState;

  std::unique_ptr<Configurator> configurator;
  std::unique_ptr<Encoder> encoder;

  // Guarded because it is set from the application thread and read from the
  // CANopen thread inside OnEmcy.
  std::mutex emcyMutex;
  std::function<void(const signals::EmergencyMessage &)> emcyCallback;

  void OnEmcy(std::uint16_t eec, std::uint8_t er, std::uint8_t msef[5]) noexcept override
  {
    signals::EmergencyMessage message{};
    message.errorCode = eec;
    message.errorRegister = er;
    for (int i = 0; i < 5; ++i) {
      message.manufacturerSpecific[i] = msef[i];
    }

    std::function<void(const signals::EmergencyMessage &)> callback;
    {
      std::lock_guard<std::mutex> lock{emcyMutex};
      callback = emcyCallback;
    }
    if (callback) {
      callback(message);
    }
  }
};


// ---------------------------------------------------------------------------
// Configurator
// ---------------------------------------------------------------------------

Configurator::Configurator(Epos4 & device)
: device_(device) {}

std::error_code
Configurator::ApplyWrites(const configs::ConfigWrites & writes)
{
  for (const auto & w : writes) {
    const std::error_code ec = std::visit(
      [&](auto value) {
        return device_.impl_->Write(w.entry, value);
      },
      w.value);
    if (ec) {
      return ec;
    }
  }
  return {};
}

std::error_code
Configurator::Apply(const configs::Epos4Configuration & config)
{
  return ApplyWrites(config.ToWrites());
}

#define EPOS4_APPLY_GROUP(Type) \
  std::error_code Configurator::Apply(const configs::Type & config) \
  { \
    configs::ConfigWrites writes; \
    config.AppendTo(writes); \
    return ApplyWrites(writes); \
  }

EPOS4_APPLY_GROUP(MotorConfigs)
EPOS4_APPLY_GROUP(GearConfigs)
EPOS4_APPLY_GROUP(AxisConfigs)
EPOS4_APPLY_GROUP(CurrentControlConfigs)
EPOS4_APPLY_GROUP(PositionControlConfigs)
EPOS4_APPLY_GROUP(VelocityControlConfigs)
EPOS4_APPLY_GROUP(MotionProfileConfigs)
EPOS4_APPLY_GROUP(LimitConfigs)
EPOS4_APPLY_GROUP(HomingConfigs)
EPOS4_APPLY_GROUP(StopOptionConfigs)
EPOS4_APPLY_GROUP(HoldingBrakeConfigs)
EPOS4_APPLY_GROUP(StandstillConfigs)
EPOS4_APPLY_GROUP(DigitalOutputConfigs)

#undef EPOS4_APPLY_GROUP

std::error_code
Configurator::Save()
{
  // 0x1010:01, signature "save" as little-endian ASCII. Section 6.2.6.
  constexpr std::uint32_t kSaveSignature = 0x65766173;  // 's','a','v','e'
  return device_.impl_->Write<std::uint32_t>(od::At(0x1010, 1), kSaveSignature);
}

std::error_code
Configurator::RestoreDefaults()
{
  // 0x1011:01, signature "load". Section 6.2.7. Takes effect after a reset.
  constexpr std::uint32_t kLoadSignature = 0x64616F6C;  // 'l','o','a','d'
  return device_.impl_->Write<std::uint32_t>(od::At(0x1011, 1), kLoadSignature);
}

std::error_code
Configurator::Refresh(configs::Epos4Configuration & config)
{
  auto & impl = *device_.impl_;
  std::uint32_t u32{};
  std::uint16_t u16{};
  std::uint8_t u8{};
  std::int32_t i32{};

  auto ec = impl.Read<std::uint16_t>(od::At(od::cia402::kMotorType), u16);
  if (ec) {return ec;}
  config.motor.motorType = static_cast<signals::MotorType>(u16);

  if (!(ec = impl.Read(od::maxon::kMotorData_NominalCurrent, u32))) {
    config.motor.nominalCurrent = u32;
  } else {return ec;}
  if (!(ec = impl.Read(od::maxon::kMotorData_OutputCurrentLimit, u32))) {
    config.motor.outputCurrentLimit = u32;
  } else {return ec;}
  if (!(ec = impl.Read(od::maxon::kMotorData_NumberOfPolePairs, u8))) {
    config.motor.numberOfPolePairs = u8;
  } else {return ec;}
  if (!(ec = impl.Read(od::maxon::kMotorData_TorqueConstant, u32))) {
    config.motor.torqueConstant = u32;
  } else {return ec;}

  if (!(ec = impl.Read(od::maxon::kPositionControlParameterSet_PositionControllerPGain, u32))) {
    config.positionControl.p = u32;
  } else {return ec;}
  if (!(ec = impl.Read(od::maxon::kPositionControlParameterSet_PositionControllerIGain, u32))) {
    config.positionControl.i = u32;
  } else {return ec;}
  if (!(ec = impl.Read(od::maxon::kPositionControlParameterSet_PositionControllerDGain, u32))) {
    config.positionControl.d = u32;
  } else {return ec;}

  if (!(ec = impl.Read(od::At(od::cia402::kProfileVelocity), u32))) {
    config.motionProfile.profileVelocity = u32;
  } else {return ec;}
  if (!(ec = impl.Read(od::At(od::cia402::kProfileAcceleration), u32))) {
    config.motionProfile.profileAcceleration = u32;
  } else {return ec;}
  if (!(ec = impl.Read(od::At(od::cia402::kProfileDeceleration), u32))) {
    config.motionProfile.profileDeceleration = u32;
  } else {return ec;}

  if (!(ec = impl.Read(od::cia402::kSoftwarePositionLimit_MinPositionLimit, i32))) {
    config.limits.minPositionLimit = i32;
  } else {return ec;}
  if (!(ec = impl.Read(od::cia402::kSoftwarePositionLimit_MaxPositionLimit, i32))) {
    config.limits.maxPositionLimit = i32;
  } else {return ec;}
  if (!(ec = impl.Read(od::At(od::cia402::kMaxMotorSpeed), u32))) {
    config.limits.maxMotorSpeed = u32;
  } else {return ec;}

  return {};
}

}  // namespace epos4

namespace epos4
{

// ---------------------------------------------------------------------------
// Epos4
// ---------------------------------------------------------------------------

Epos4::Epos4(CanBus & bus, std::uint8_t nodeId)
: nodeId_(nodeId)
{
  // Nothing is created here: the master does not exist until CanBus::Start().
  // Registering now means Start() can attach this device before it resets the
  // network, which is what gets this node's PDOs routed to us.
  bus.Register(this);
}

void
Epos4::Attach(lely_master_t & master)
{
  if (impl_) {
    return;
  }
  impl_ = std::make_unique<Impl>(master, nodeId_);
  impl_->configurator = std::make_unique<Configurator>(*this);
  impl_->encoder = std::make_unique<Encoder>(*this);

  auto & d = *impl_;

  // Each signal knows how to fetch itself. Wiring them here keeps the
  // accessors below trivial and keeps every object index in one place.
  d.statusword = signals::StatusSignal<std::uint16_t>(
    [&d](std::uint16_t & out) {return d.ReadStatusword(out);});

  d.state = signals::StatusSignal<signals::State>(
    [&d](signals::State & out) -> std::error_code {
      std::uint16_t sw{};
      if (auto ec = d.ReadStatusword(sw)) {return ec;}
      const auto decoded = core::Decode(sw);
      if (!decoded) {
        // The masked pattern matched none of the eight legal states. Do not
        // invent one: report it and let the caller log the raw word.
        return std::make_error_code(std::errc::protocol_error);
      }
      out = *decoded;
      return {};
    });

  d.mode = signals::StatusSignal<signals::OperationMode>(
    [&d](signals::OperationMode & out) -> std::error_code {
      std::int8_t raw{};
      auto ec = d.Read<std::int8_t>(od::At(od::cia402::kModesOfOperationDisplay), raw);
      if (!ec) {out = static_cast<signals::OperationMode>(raw);}
      return ec;
    });

  auto i32 = [&d](std::uint16_t index) {
      return [&d, index](std::int32_t & out) {
               return d.ReadPreferPdo<std::int32_t>(od::At(index), out);
             };
    };
  auto i16 = [&d](std::uint16_t index) {
      return [&d, index](std::int16_t & out) {
               return d.ReadPreferPdo<std::int16_t>(od::At(index), out);
             };
    };

  d.position = signals::StatusSignal<std::int32_t>(i32(od::cia402::kPositionActualValue));
  d.positionDemand = signals::StatusSignal<std::int32_t>(i32(od::cia402::kPositionDemandValue));
  d.velocity = signals::StatusSignal<std::int32_t>(i32(od::cia402::kVelocityActualValue));
  d.velocityDemand = signals::StatusSignal<std::int32_t>(i32(od::cia402::kVelocityDemandValue));
  d.followingError =
    signals::StatusSignal<std::int32_t>(i32(od::cia402::kFollowingErrorActualValue));
  d.torque = signals::StatusSignal<std::int16_t>(i16(od::cia402::kTorqueActualValue));
  d.currentDemand = signals::StatusSignal<std::int16_t>(i16(od::maxon::kCurrentDemandValue));

  d.errorCode = signals::StatusSignal<std::uint16_t>(
    [&d](std::uint16_t & out) {
      return d.Read<std::uint16_t>(od::At(od::cia402::kErrorCode), out);
    });

  d.errorRegister = signals::StatusSignal<std::uint8_t>(
    [&d](std::uint8_t & out) {
      return d.Read<std::uint8_t>(od::At(od::comm::kErrorRegister), out);
    });

  // Mode-specific bits, exposed by meaning. Bit 12 alone means Setpoint
  // acknowledge under PPM, Speed under PVM, Homing attained under HMM and
  // Drive follows command value under the cyclic modes, which is exactly why
  // these are named accessors rather than a raw bit the caller has to
  // interpret against the active mode.
  auto bit = [&d](std::uint16_t mask) {
      return [&d, mask](bool & out) -> std::error_code {
               std::uint16_t sw{};
               if (auto ec = d.ReadStatusword(sw)) {return ec;}
               out = (sw & mask) != 0;
               return {};
             };
    };

  d.targetReached = signals::StatusSignal<bool>(bit(signals::status_bits::kTargetReached));
  d.setpointAcknowledged =
    signals::StatusSignal<bool>(bit(signals::status_bits::kSetpointAcknowledge));
  d.followingErrorFlag = signals::StatusSignal<bool>(bit(signals::status_bits::kFollowingError));
  d.homingAttained = signals::StatusSignal<bool>(bit(signals::status_bits::kHomingAttained));
  d.internalLimit = signals::StatusSignal<bool>(bit(signals::status_bits::kInternalLimit));
  d.warning = signals::StatusSignal<bool>(bit(signals::status_bits::kWarning));

  d.brakeState = signals::StatusSignal<signals::BrakeState>(
    [&d](signals::BrakeState & out) -> std::error_code {
      std::uint8_t raw{};
      auto ec = d.Read<std::uint8_t>(
        od::maxon::kHoldingBrakeParameters_HoldingBrakeState, raw);
      if (!ec) {out = static_cast<signals::BrakeState>(raw);}
      return ec;
    });
}

Epos4::~Epos4() = default;

std::uint8_t
Epos4::GetNodeId() const
{
  return nodeId_;
}

Configurator &
Epos4::GetConfigurator()
{
  return *impl_->configurator;
}

Encoder &
Epos4::GetEncoder()
{
  return *impl_->encoder;
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

namespace
{

// Drives the CiA 402 state machine towards a goal, one transition per round
// trip, giving up at the deadline. The transition table is not here: it is in
// core::PlanStep, which is pure logic and covered by tests that need no bus.
bool
RunToGoal(Epos4::Impl & d, core::Goal goal, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    std::uint16_t sw{};

    // Transient bus errors do not abort the attempt. The first SDO after the
    // master starts routinely fails with "SDO connection not available"
    // because the node has not finished booting, and a drive that answers
    // late is not a drive that refused. Retry until the deadline; the
    // deadline is what gives up, not the first hiccup.
    if (d.ReadStatusword(sw)) {
      std::this_thread::sleep_for(kPollInterval);
      continue;
    }
    const auto state = core::Decode(sw);
    if (!state) {
      std::this_thread::sleep_for(kPollInterval);
      continue;
    }

    const core::Step step = core::PlanStep(*state, goal);
    if (step.progress == core::Progress::kReached) {
      return true;
    }
    if (step.progress == core::Progress::kBlocked) {
      // Faulted. Never cleared implicitly: that is ClearFault()'s job, and it
      // is the caller's decision to make.
      return false;
    }
    if (step.command) {
      d.controlword.Apply(*step.command);
      d.WriteControlword();  // a dropped write is retried on the next pass
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  return false;
}

}  // namespace

bool
Epos4::WaitUntilReady(std::chrono::milliseconds timeout)
{
  if (!impl_) {
    return false;   // the bus never started, so this device was never attached
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    // Probe SDO explicitly rather than going through ReadStatusword(), which
    // prefers PDO: once the mapping is live the Statusword arrives on every
    // SYNC whether or not the node is reachable over SDO, and configuration
    // would then fail with "SDO connection not available" a moment later.
    //
    // Not IsReady() either. That reports whether the master's boot procedure
    // completed *cleanly*, and a perfectly usable node can report an advisory
    // like es='L' (slave was already operational) which leaves IsReady()
    // false forever. What callers actually need to know is whether the node
    // answers, so ask it.
    std::uint16_t statusword{};
    if (!impl_->Read<std::uint16_t>(od::At(od::cia402::kStatusword), statusword)) {
      return true;
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  return false;
}

bool
Epos4::Enable(std::chrono::milliseconds timeout)
{
  return RunToGoal(*impl_, core::Goal::kOperational, timeout);
}

bool
Epos4::Disable(std::chrono::milliseconds timeout)
{
  return RunToGoal(*impl_, core::Goal::kDisabled, timeout);
}

bool
Epos4::ClearFault(std::chrono::milliseconds timeout)
{
  auto & d = *impl_;

  std::uint16_t sw{};
  if (d.ReadStatusword(sw)) {return false;}
  auto state = core::Decode(sw);
  if (!state) {return false;}
  if (*state != signals::State::kFault) {
    return true;  // nothing to clear
  }

  // Table 2-7: Fault reset is the rising edge of bit 7, not its level.
  // Controlword::Apply lowers bit 7 first, so applying kFaultReset and then
  // any other command produces 0->1->0 without the caller tracking it.
  d.controlword.Apply(core::Command::kFaultReset);
  if (d.WriteControlword()) {return false;}

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(kPollInterval);
    if (d.ReadStatusword(sw)) {return false;}
    state = core::Decode(sw);
    if (!state) {return false;}
    if (*state != signals::State::kFault &&
      *state != signals::State::kFaultReactionActive)
    {
      // Complete the edge so the next fault can also be cleared.
      d.controlword.Apply(core::Command::kDisableVoltage);
      d.WriteControlword();
      return true;
    }
  }
  // Still faulted: the underlying cause has not gone away. Transition 15 only
  // succeeds "if no fault is present".
  return false;
}

std::error_code
Epos4::Halt()
{
  auto & d = *impl_;
  d.controlword.SetModeBits(signals::control_bits::kHalt);
  return d.WriteControlword();
}

std::error_code
Epos4::QuickStop()
{
  auto & d = *impl_;
  d.controlword.Apply(core::Command::kQuickStop);
  return d.WriteControlword();
}

bool
Epos4::IsEnabled()
{
  auto & signal = GetState();
  signal.Refresh();
  return !signal.GetStatus() &&
         signal.GetValue() == signals::State::kOperationEnabled;
}

bool
Epos4::IsFaulted()
{
  auto & signal = GetState();
  signal.Refresh();
  if (signal.GetStatus()) {return false;}
  const auto s = signal.GetValue();
  return s == signals::State::kFault || s == signals::State::kFaultReactionActive;
}

// ---------------------------------------------------------------------------
// Control
// ---------------------------------------------------------------------------

std::error_code
Epos4::SetControl(const controls::ProfilePosition & request)
{
  auto & d = *impl_;

  if (auto ec = d.EnsureMode(controls::ProfilePosition::kMode)) {return ec;}

  // Profile overrides, only when the request carries them. A caller who set
  // these once through MotionProfileConfigs does not pay for them per move.
  if (request.velocity) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileVelocity), *request.velocity)) {return ec;}
  }
  if (request.acceleration) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileAcceleration), *request.acceleration)) {return ec;}
  }
  if (request.deceleration) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileDeceleration), *request.deceleration)) {return ec;}
  }

  if (auto ec = d.Write<std::int32_t>(
      od::At(od::cia402::kTargetPosition), request.position)) {return ec;}

  // Mode bits, before the handshake so they are in force when the setpoint is
  // taken. These never touch the state machine bits: SetModeBits filters.
  if (request.relative) {
    d.controlword.SetModeBits(signals::control_bits::kAbsoluteRelative);
  } else {
    d.controlword.ClearModeBits(signals::control_bits::kAbsoluteRelative);
  }
  if (request.changeSetImmediately) {
    d.controlword.SetModeBits(signals::control_bits::kChangeSetImmediately);
  } else {
    d.controlword.ClearModeBits(signals::control_bits::kChangeSetImmediately);
  }

  // ---- the setpoint handshake, Table 3-15 ----
  //
  // Raise New setpoint (bit 4); the drive answers with Setpoint acknowledge
  // (bit 12); lower bit 4 again. Without the lowering step the drive keeps
  // bit 12 asserted and the NEXT move is silently ignored. That is the single
  // most common PPM bug, and the reason this is done here rather than left to
  // the caller.
  d.controlword.SetModeBits(signals::control_bits::kNewSetpoint);
  if (auto ec = d.WriteControlword()) {return ec;}

  bool acknowledged = false;
  for (int i = 0; i < 200; ++i) {
    std::uint16_t sw{};
    if (auto ec = d.ReadStatusword(sw)) {return ec;}
    if (sw & signals::status_bits::kSetpointAcknowledge) {
      acknowledged = true;
      break;
    }
    std::this_thread::sleep_for(kPollInterval);
  }

  d.controlword.ClearModeBits(signals::control_bits::kNewSetpoint);
  if (auto ec = d.WriteControlword()) {return ec;}

  if (!acknowledged) {
    return std::make_error_code(std::errc::timed_out);
  }
  return {};
}

std::error_code
Epos4::SetControl(const controls::ProfileVelocity & request)
{
  auto & d = *impl_;

  if (auto ec = d.EnsureMode(controls::ProfileVelocity::kMode)) {return ec;}

  if (request.acceleration) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileAcceleration), *request.acceleration)) {return ec;}
  }
  if (request.deceleration) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileDeceleration), *request.deceleration)) {return ec;}
  }

  // PVM has no setpoint handshake: the target velocity takes effect as soon
  // as it is written.
  // The cyclic modes exist to be commanded every cycle. Staging the setpoint
  // in the RPDO lets it ride the next SYNC instead of costing an SDO round
  // trip per update.
  const od::Entry target = od::At(od::cia402::kTargetVelocity);
  if (auto ec = d.WriteMapped<std::int32_t>(target, request.velocity); !ec) {return {};}
  return d.Write<std::int32_t>(target, request.velocity);
}

std::error_code
Epos4::SetControl(const controls::CyclicPosition & request)
{
  auto & d = *impl_;

  if (auto ec = d.EnsureMode(controls::CyclicPosition::kMode)) {return ec;}

  if (request.positionOffset) {
    if (auto ec = d.Write<std::int32_t>(
        od::At(od::cia402::kPositionOffset), *request.positionOffset)) {return ec;}
  }
  if (request.torqueOffset) {
    if (auto ec = d.Write<std::int16_t>(
        od::At(od::cia402::kTorqueOffset), *request.torqueOffset)) {return ec;}
  }
  // The cyclic modes exist to be commanded every cycle. Staging the setpoint
  // in the RPDO lets it ride the next SYNC instead of costing an SDO round
  // trip per update.
  const od::Entry target = od::At(od::cia402::kTargetPosition);
  if (auto ec = d.WriteMapped<std::int32_t>(target, request.position); !ec) {return {};}
  return d.Write<std::int32_t>(target, request.position);
}

std::error_code
Epos4::SetControl(const controls::CyclicVelocity & request)
{
  auto & d = *impl_;

  if (auto ec = d.EnsureMode(controls::CyclicVelocity::kMode)) {return ec;}

  if (request.velocityOffset) {
    if (auto ec = d.Write<std::int32_t>(
        od::At(od::cia402::kVelocityOffset), *request.velocityOffset)) {return ec;}
  }
  if (request.torqueOffset) {
    if (auto ec = d.Write<std::int16_t>(
        od::At(od::cia402::kTorqueOffset), *request.torqueOffset)) {return ec;}
  }
  return d.Write<std::int32_t>(od::At(od::cia402::kTargetVelocity), request.velocity);
}

std::error_code
Epos4::SetControl(const controls::CyclicTorque & request)
{
  auto & d = *impl_;

  if (auto ec = d.EnsureMode(controls::CyclicTorque::kMode)) {return ec;}

  if (request.torqueOffset) {
    if (auto ec = d.Write<std::int16_t>(
        od::At(od::cia402::kTorqueOffset), *request.torqueOffset)) {return ec;}
  }
  // The cyclic modes exist to be commanded every cycle. Staging the setpoint
  // in the RPDO lets it ride the next SYNC instead of costing an SDO round
  // trip per update.
  const od::Entry target = od::At(od::cia402::kTargetTorque);
  if (auto ec = d.WriteMapped<std::int16_t>(target, request.torque); !ec) {return {};}
  return d.Write<std::int16_t>(target, request.torque);
}

std::error_code
Epos4::SetControl(const controls::Halt &)
{
  return Halt();
}

bool
Epos4::Home(const controls::Homing & request, std::chrono::milliseconds timeout)
{
  auto & d = *impl_;

  if (d.EnsureMode(controls::Homing::kMode)) {return false;}

  if (request.method) {
    if (d.Write<std::int8_t>(
        od::At(od::cia402::kHomingMethod),
        static_cast<std::int8_t>(*request.method)))
    {
      return false;
    }
  }

  // Controlword bit 4 under HMM is «Homing operation start», not «New
  // setpoint». Same bit, different meaning, decided by 0x6061 - which is why
  // the mode is set first.
  d.controlword.SetModeBits(signals::control_bits::kHomingOperationStart);
  if (d.WriteControlword()) {return false;}

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  bool attained = false;

  while (std::chrono::steady_clock::now() < deadline) {
    std::uint16_t sw{};
    if (d.ReadStatusword(sw)) {break;}

    // Bit 13 under HMM is «Homing error»: a switch that never triggered, or a
    // move that ran out of travel.
    if (sw & signals::status_bits::kHomingError) {break;}

    // Bit 12 under HMM is «Homing attained», bit 10 is «Target reached».
    // Both together mean the run finished and the axis came to rest.
    if ((sw & signals::status_bits::kHomingAttained) &&
      (sw & signals::status_bits::kTargetReached))
    {
      attained = true;
      break;
    }
    std::this_thread::sleep_for(kPollInterval);
  }

  d.controlword.ClearModeBits(signals::control_bits::kHomingOperationStart);
  d.WriteControlword();
  return attained;
}

// ---------------------------------------------------------------------------
// Status signal accessors
// ---------------------------------------------------------------------------

signals::StatusSignal<signals::State> & Epos4::GetState() {return impl_->state;}
signals::StatusSignal<std::uint16_t> & Epos4::GetStatusword() {return impl_->statusword;}
signals::StatusSignal<signals::OperationMode> & Epos4::GetOperationMode() {return impl_->mode;}
signals::StatusSignal<std::int32_t> & Epos4::GetPosition() {return impl_->position;}
signals::StatusSignal<std::int32_t> & Epos4::GetPositionDemand() {return impl_->positionDemand;}
signals::StatusSignal<std::int32_t> & Epos4::GetVelocity() {return impl_->velocity;}
signals::StatusSignal<std::int32_t> & Epos4::GetVelocityDemand() {return impl_->velocityDemand;}
signals::StatusSignal<std::int16_t> & Epos4::GetTorque() {return impl_->torque;}
signals::StatusSignal<std::int32_t> & Epos4::GetFollowingError() {return impl_->followingError;}
signals::StatusSignal<std::int16_t> & Epos4::GetCurrentDemand() {return impl_->currentDemand;}
signals::StatusSignal<std::uint16_t> & Epos4::GetErrorCode() {return impl_->errorCode;}
signals::StatusSignal<std::uint8_t> & Epos4::GetErrorRegister() {return impl_->errorRegister;}
signals::StatusSignal<bool> & Epos4::IsTargetReached() {return impl_->targetReached;}
signals::StatusSignal<bool> & Epos4::IsSetpointAcknowledged() {return impl_->setpointAcknowledged;}
signals::StatusSignal<bool> & Epos4::HasFollowingError() {return impl_->followingErrorFlag;}
signals::StatusSignal<bool> & Epos4::IsHomingAttained() {return impl_->homingAttained;}
signals::StatusSignal<bool> & Epos4::IsInternalLimitActive() {return impl_->internalLimit;}
signals::StatusSignal<bool> & Epos4::HasWarning() {return impl_->warning;}
signals::StatusSignal<signals::BrakeState> & Epos4::GetBrakeState() {return impl_->brakeState;}

// ---------------------------------------------------------------------------
// Raw object access
// ---------------------------------------------------------------------------

template<typename T>
std::error_code
Epos4::ReadObject(od::Entry entry, T & value)
{
  return impl_->Read<T>(entry, value);
}

template<typename T>
std::error_code
Epos4::WriteObject(od::Entry entry, T value)
{
  return impl_->Write<T>(entry, value);
}

// The CANopen basic types. Anything outside this set is not a thing the
// object dictionary can hold.
#define EPOS4_INSTANTIATE_RAW(T) \
  template std::error_code Epos4::ReadObject<T>(od::Entry, T &); \
  template std::error_code Epos4::WriteObject<T>(od::Entry, T);

EPOS4_INSTANTIATE_RAW(std::int8_t)
EPOS4_INSTANTIATE_RAW(std::int16_t)
EPOS4_INSTANTIATE_RAW(std::int32_t)
EPOS4_INSTANTIATE_RAW(std::uint8_t)
EPOS4_INSTANTIATE_RAW(std::uint16_t)
EPOS4_INSTANTIATE_RAW(std::uint32_t)

#undef EPOS4_INSTANTIATE_RAW

}  // namespace epos4

namespace epos4
{

bool
Epos4::IsPdoActive() const
{
  return impl_->PdoActive();
}

std::chrono::steady_clock::duration
Epos4::GetTimeSinceLastPdo() const
{
  const auto last = impl_->lastPdo.load(std::memory_order_relaxed);
  if (last == 0) {
    return std::chrono::steady_clock::duration::max();
  }
  return std::chrono::steady_clock::now() -
         std::chrono::steady_clock::time_point{
           std::chrono::steady_clock::duration{last}};
}

}  // namespace epos4

namespace epos4
{

std::string
Epos4::DescribeLastError()
{
  if (!impl_) {
    return "device not attached to a running bus";
  }

  auto & code = GetErrorCode();
  code.Refresh();
  if (code.GetStatus()) {
    return "could not read the error code: " + code.GetStatus().message();
  }

  std::string out = signals::DescribeDeviceError(code.GetValue());

  auto & reg = GetErrorRegister();
  reg.Refresh();
  if (!reg.GetStatus()) {
    out += "\n  live reg : " + signals::DescribeErrorRegister(reg.GetValue());
  }
  return out;
}

std::error_code
Epos4::GetErrorHistory(std::vector<std::uint16_t> & out)
{
  out.clear();
  if (!impl_) {
    return std::make_error_code(std::errc::not_connected);
  }

  // Sub-index 0 holds how many entries are currently filled, 1..n the codes
  // themselves with the newest first.
  std::uint8_t count{};
  if (auto ec = impl_->Read<std::uint8_t>(od::At(od::comm::kErrorHistory, 0), count)) {
    return ec;
  }

  for (std::uint8_t i = 1; i <= count; ++i) {
    std::uint32_t entry{};
    if (auto ec = impl_->Read<std::uint32_t>(od::At(od::comm::kErrorHistory, i), entry)) {
      return ec;
    }
    // The low word is the error code; the high word is the error register and
    // manufacturer-specific information.
    out.push_back(static_cast<std::uint16_t>(entry & 0xFFFFu));
  }
  return {};
}

std::error_code
Epos4::ClearErrorHistory()
{
  if (!impl_) {
    return std::make_error_code(std::errc::not_connected);
  }
  return impl_->Write<std::uint8_t>(od::At(od::comm::kErrorHistory, 0), 0);
}

void
Epos4::SetEmergencyCallback(std::function<void(const signals::EmergencyMessage &)> callback)
{
  if (!impl_) {
    return;
  }
  std::lock_guard<std::mutex> lock{impl_->emcyMutex};
  impl_->emcyCallback = std::move(callback);
}

}  // namespace epos4
