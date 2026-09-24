#include "epos4/hardware/Epos4.hpp"

#include <lely/coapp/loop_driver.hpp>
#include <lely/coapp/sdo_error.hpp>

#include <atomic>
#include <future>
#include <mutex>
#include <thread>

#include "epos4/core/BusImpl.hpp"
#include "epos4/core/Cia402StateMachine.hpp"
#include "epos4/core/CyclicState.hpp"
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
  // NMT
  // -------------------------------------------------------------------------

  // Counts completed boots of this node by the master. ClearFault() compares
  // it before and after an NMT reset to know the node is back and has been
  // reconfigured, rather than guessing with a sleep.
  std::atomic<unsigned> bootCount{0};

  void OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept override
  {
    LoopDriver::OnBoot(st, es, what);  // keeps the default notifications
    bootCount.fetch_add(1, std::memory_order_release);
  }

  // NMT reset communication, to this node only. The node reloads its
  // communication objects and announces itself with a boot-up message, and
  // the master answers that by booting it again - downloading the concise
  // DCF, PDO mapping and heartbeat consumer included - which is what makes
  // OnBoot() run.
  void ResetCommunication()
  {
    std::promise<void> promise;
    auto future = promise.get_future();
    Defer(
      [this, &promise]() {
        master.Command(lely::canopen::NmtCommand::RESET_COMM, id());
        promise.set_value();
      });
    future.get();
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

  // Runs on the bus thread every time an inbound PDO updates a mapped object.
  // Everything it does is a relaxed atomic store, so the control thread can
  // read the values without locking, waiting or hopping threads.
  void OnRpdoWrite(std::uint16_t idx, std::uint8_t subidx) noexcept override
  {
    cyclic.RecordPdo(std::chrono::steady_clock::now());

    if (subidx != 0) {
      return;
    }
    // rpdo_mapped[idx][sub] converts implicitly on assignment, so the target
    // variable's type is what selects the width. Getting it wrong here would
    // silently truncate a position.
    try {
      switch (idx) {
        case od::cia402::kStatusword: {
            const std::uint16_t value = rpdo_mapped[idx][0];
            cyclic.RecordStatusword(value);
            break;
          }
        case od::cia402::kPositionActualValue: {
            const std::int32_t value = rpdo_mapped[idx][0];
            cyclic.RecordPosition(value);
            break;
          }
        case od::cia402::kVelocityActualValue: {
            const std::int32_t value = rpdo_mapped[idx][0];
            cyclic.RecordVelocity(value);
            break;
          }
        case od::cia402::kTorqueActualValue: {
            const std::int16_t value = rpdo_mapped[idx][0];
            cyclic.RecordTorque(value);
            break;
          }
        default:
          break;
      }
    } catch (...) {
      // A mapped object that is not really there. Leave the cache alone: a
      // stale value is visible through IsCyclicHealthy(), an invented one
      // would not be.
    }
  }

  // Runs on the bus thread when the master transmits SYNC, which is exactly
  // when the outbound PDO is about to go out. Publishing here means the
  // setpoint the control loop staged rides this cycle rather than the next.
  void OnSync(std::uint8_t, const time_point &) noexcept override
  {
    if (!cyclic.IsActive()) {
      return;
    }
    try {
      tpdo_mapped[od::cia402::kControlword][0] = cyclic.Controlword();
      tpdo_mapped[od::cia402::kTargetPosition][0] = cyclic.StagedTargetPosition();
    } catch (...) {
      // Not mapped: nothing to publish. The cyclic path is only meaningful
      // with a PDO mapping, and its absence shows up as IsCyclicHealthy()
      // being false rather than as an exception crossing the bus thread.
    }
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

  bool PdoActive() const {return cyclic.HasReceivedPdo();}

  // Reads over PDO when it is available and falls back to SDO otherwise, so
  // callers never have to branch on the transport.
  //
  // "Available" means a PDO arrived recently, not merely that one ever did.
  // A drive that loses the master's heartbeat drops to pre-operational and
  // stops sending PDOs, but keeps answering SDO; its last Statusword by PDO
  // still says «Operation enabled» while it sits in «Fault». Trusting that
  // made ClearFault() see nothing to clear and return true on a faulted
  // axis. A stale mapped value is only a reason to ask the drive directly.
  template<typename T>
  std::error_code ReadPreferPdo(od::Entry entry, T & out)
  {
    if (cyclic.TimeSinceLastPdo(std::chrono::steady_clock::now()) <= kPdoFreshness) {
      if (!ReadMapped<T>(entry, out)) {return {};}
    }
    return Read<T>(entry, out);
  }

  // Ten SYNC periods at the 10 ms of bus.yml. Generous enough not to fall
  // back to SDO over jitter; short enough that a node gone quiet is asked
  // directly almost at once. With event-driven PDOs a value may legitimately
  // be older than this - then the read simply costs an SDO, and is right.
  static constexpr std::chrono::milliseconds kPdoFreshness{100};

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
    // Into the copy OnSync republishes as well, or a Controlword written
    // while the cyclic path is active - QuickStop(), Halt() - would be
    // overwritten on the very next SYNC. See CyclicState::SetControlword.
    cyclic.SetControlword(controlword.Raw());
    return WriteOutput<std::uint16_t>(od::At(od::cia402::kControlword), controlword.Raw());
  }

  // The same rule for every object the master may transmit, not only the
  // Controlword. Any object mapped into an RPDO with transmission type 1 is
  // re-sent from the master's copy on EVERY SYNC, so an SDO write to it lasts
  // until the next SYNC and is then overwritten. For Target position under
  // PPM that meant the drive latched 0 on «New setpoint» instead of the value
  // just written, and the move never happened - with the handshake completing
  // perfectly, because nothing in it checks what was latched.
  template<typename T>
  std::error_code WriteOutput(od::Entry entry, T value)
  {
    if (!WriteMapped<T>(entry, value)) {return {};}
    return Write<T>(entry, value);
  }

  // Everything the cyclic path shares between the control thread and this
  // one: the last feedback received, the staged setpoint, and the rule for
  // trusting them. Lock-free throughout; see core::CyclicState.
  core::CyclicState cyclic;


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
  signals::StatusSignal<std::uint32_t> digitalInputs;
  signals::StatusSignal<std::uint32_t> motorRatedTorque;
  signals::StatusSignal<std::uint16_t> digitalInputPins;

  signals::StatusSignal<units::current::ampere_t> motorCurrent;
  signals::StatusSignal<units::current::ampere_t> motorCurrentAveraged;
  signals::StatusSignal<units::voltage::volt_t> supplyVoltage;
  signals::StatusSignal<units::temperature::celsius_t> powerStageTemperature;
  signals::StatusSignal<units::temperature::celsius_t> powerStageTemperatureLimit;


  // Guarded because it is set from the application thread and read from the
  // CANopen thread inside OnEmcy.
  std::mutex emcyMutex;
  std::function<void(const signals::EmergencyMessage &)> emcyCallback;

  // The code of the last EMCY, for readers that must not touch the bus. An
  // EMCY with code 0 is the drive announcing that its errors were reset, so
  // storing it as-is also clears this when the fault is cleared.
  std::atomic<std::uint16_t> lastEmcyCode{0};
  std::atomic<long long> lastEmcy{0};  // steady_clock ticks, 0 = never

  void OnEmcy(std::uint16_t eec, std::uint8_t er, std::uint8_t msef[5]) noexcept override
  {
    lastEmcyCode.store(eec, std::memory_order_relaxed);
    lastEmcy.store(
      std::chrono::steady_clock::now().time_since_epoch().count(),
      std::memory_order_relaxed);

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
EPOS4_APPLY_GROUP(CyclicConfigs)

#undef EPOS4_APPLY_GROUP

std::error_code
Configurator::Apply(const configs::DigitalInputConfigs & config)
{
  // Checked here rather than inside AppendTo so the caller learns the mapping
  // is wrong before a single write reaches the bus.
  if (auto ec = config.Validate()) {
    return ec;
  }
  configs::ConfigWrites writes;
  config.AppendTo(writes);
  return ApplyWrites(writes);
}

std::error_code
Configurator::Save()
{
  // 0x1010:01, signature "save" as little-endian ASCII. Section 6.2.6.
  constexpr std::uint32_t kSaveSignature = 0x65766173;  // 's','a','v','e'
  return device_.impl_->Write<std::uint32_t>(
    od::comm::kStoreParameters_SaveAllParameters,
    kSaveSignature);
}

std::error_code
Configurator::RestoreDefaults()
{
  // 0x1011:01, signature "load". Section 6.2.7. Takes effect after a reset.
  constexpr std::uint32_t kLoadSignature = 0x64616F6C;  // 'l','o','a','d'
  return device_.impl_->Write<std::uint32_t>(
    od::comm::kRestoreDefaultParameters_RestoreAllDefaultParameters, kLoadSignature);
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
: nodeId_(nodeId),
  configurator_(std::make_unique<Configurator>(*this)),
  encoder_(std::make_unique<Encoder>(*this))
{
  // Nothing that talks to the bus is created here: the master does not exist
  // until CanBus::Start().
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
  if (pendingEmcyCallback_) {
    // Locked like any other hand-over: constructing the driver has already
    // registered it with the master, so OnEmcy may in principle run now.
    std::lock_guard<std::mutex> lock{impl_->emcyMutex};
    impl_->emcyCallback = std::move(pendingEmcyCallback_);
    pendingEmcyCallback_ = nullptr;
  }

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

  // Converted where they are read, so no caller ever sees tenths of a volt.
  // Current may be PDO-mapped (TXPDO in 6.2.70); voltage and temperature
  // cannot be (PDO mapping: NO in 6.2.50 and 6.2.89) and always cost an SDO.
  auto amperes = [&d](od::Entry entry) {
      return [&d, entry](units::current::ampere_t & out) {
               std::int32_t milliamps{};
               auto ec = d.ReadPreferPdo<std::int32_t>(entry, milliamps);
               if (!ec) {out = ToCurrent(milliamps);}
               return ec;
             };
    };
  auto celsius = [&d](od::Entry entry) {
      return [&d, entry](units::temperature::celsius_t & out) {
               std::int16_t decidegrees{};
               auto ec = d.Read<std::int16_t>(entry, decidegrees);
               if (!ec) {out = ToTemperature(decidegrees);}
               return ec;
             };
    };

  d.motorCurrent = signals::StatusSignal<units::current::ampere_t>(
    amperes(od::maxon::kCurrentActualValues_CurrentActualValue));
  d.motorCurrentAveraged = signals::StatusSignal<units::current::ampere_t>(
    amperes(od::maxon::kCurrentActualValues_CurrentActualValueAveraged));
  d.supplyVoltage = signals::StatusSignal<units::voltage::volt_t>(
    [&d](units::voltage::volt_t & out) {
      std::uint16_t decivolts{};
      auto ec = d.Read<std::uint16_t>(od::maxon_comm::kPowerSupply_PowerSupplyVoltage, decivolts);
      if (!ec) {out = ToVoltage(decivolts);}
      return ec;
    });
  d.powerStageTemperature = signals::StatusSignal<units::temperature::celsius_t>(
    celsius(od::maxon::kThermalOverloadProtection_TemperaturePowerStage));
  // UNSIGNED16 on the drive, read into a signed one: the limit is a few
  // hundred tenths of a degree, far inside either range.
  d.powerStageTemperatureLimit = signals::StatusSignal<units::temperature::celsius_t>(
    [&d](units::temperature::celsius_t & out) {
      std::uint16_t decidegrees{};
      auto ec = d.Read<std::uint16_t>(
        od::maxon::kThermalOverloadProtection_MaximalTemperaturePowerStage, decidegrees);
      if (!ec) {out = ToTemperature(static_cast<std::int16_t>(decidegrees));}
      return ec;
    });

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

  d.motorRatedTorque = signals::StatusSignal<std::uint32_t>(
    [&d](std::uint32_t & out) {
      return d.Read<std::uint32_t>(od::At(od::cia402::kMotorRatedTorque), out);
    });

  d.digitalInputs = signals::StatusSignal<std::uint32_t>(
    [&d](std::uint32_t & out) {
      return d.ReadPreferPdo<std::uint32_t>(od::At(od::cia402::kDigitalInputs), out);
    });

  d.digitalInputPins = signals::StatusSignal<std::uint16_t>(
    [&d](std::uint16_t & out) {
      return d.Read<std::uint16_t>(
        od::maxon::kDigitalInputProperties_DigitalInputsLogicState, out);
    });

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
  return *configurator_;
}

Encoder &
Epos4::GetEncoder()
{
  return *encoder_;
}

void
Epos4::SetMechanism(std::uint32_t quadCountsPerRevolution, double gearRatio)
{
  encoder_->SetMechanism(quadCountsPerRevolution, gearRatio);
}

const MechanismScale &
Epos4::GetMechanism() const
{
  return encoder_->GetMechanism();
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
  if (!impl_) {
    return false;   // the bus never started, so there is no drive to ask
  }
  auto & d = *impl_;

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::uint16_t sw{};
  if (d.ReadStatusword(sw)) {return false;}
  auto state = core::Decode(sw);
  if (!state) {return false;}
  if (*state != signals::State::kFault) {
    return true;  // nothing to clear
  }

  // Some faults are not cleared by the Controlword alone. For a lost
  // heartbeat (0x8130) and CAN passive mode (0x8120) the recovery in
  // chapter 7 is "send NMT command reset communication, then reset fault
  // with Controlword"; without the first step the fault reset is ignored and
  // the axis stays in «Fault» however often it is repeated.
  std::uint16_t errorCode{};
  if (!d.Read<std::uint16_t>(od::At(od::cia402::kErrorCode), errorCode) &&
    signals::RequiresCommunicationReset(errorCode))
  {
    const unsigned bootsBefore = d.bootCount.load(std::memory_order_acquire);
    d.ResetCommunication();

    // Wait for the master to have booted the node again: until then its PDO
    // mapping and heartbeat consumer are the power-on defaults, and an SDO
    // to it fails with "SDO connection not available".
    while (d.bootCount.load(std::memory_order_acquire) == bootsBefore) {
      if (std::chrono::steady_clock::now() >= deadline) {return false;}
      std::this_thread::sleep_for(kPollInterval);
    }
  }

  // Table 2-7: Fault reset is the rising edge of bit 7, not its level.
  // Controlword::Apply lowers bit 7 first, so applying kFaultReset and then
  // any other command produces 0->1->0 without the caller tracking it.
  d.controlword.Apply(core::Command::kFaultReset);
  if (d.WriteControlword()) {return false;}

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

  // Resolve the setpoints first. A request expressed in real units against a
  // device whose mechanism was never configured is rejected here, before the
  // mode has been changed or anything has been written.
  std::int32_t targetCounts{};
  if (!Resolve(request.position, GetMechanism(), targetCounts)) {
    return std::make_error_code(std::errc::invalid_argument);
  }

  // Profile overrides, only when the request carries them. A caller who set
  // these once through MotionProfileConfigs does not pay for them per move.
  if (request.velocity) {
    std::uint32_t rpm{};
    if (!Resolve(*request.velocity, GetMechanism(), rpm)) {
      return std::make_error_code(std::errc::invalid_argument);
    }
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileVelocity), rpm)) {return ec;}
  }
  if (request.acceleration) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileAcceleration), *request.acceleration)) {return ec;}
  }
  if (request.deceleration) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileDeceleration), *request.deceleration)) {return ec;}
  }

  // Before raising «New setpoint», and through the RPDO when Target position
  // is mapped: see WriteOutput for why an SDO write here is lost.
  if (auto ec = d.WriteOutput<std::int32_t>(
      od::At(od::cia402::kTargetPosition), targetCounts)) {return ec;}

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

  std::int32_t targetRpm{};
  if (!Resolve(request.velocity, GetMechanism(), targetRpm)) {
    return std::make_error_code(std::errc::invalid_argument);
  }

  if (request.acceleration) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileAcceleration), *request.acceleration)) {return ec;}
  }
  if (request.deceleration) {
    if (auto ec = d.Write<std::uint32_t>(
        od::At(od::cia402::kProfileDeceleration), *request.deceleration)) {return ec;}
  }

  // PVM has no setpoint handshake: the target velocity takes effect as soon
  // as it arrives. It is commanded once and the drive generates the ramp, so
  // nothing here is cyclic - but if Target velocity is mapped into an RPDO the
  // write still has to go through it, or the next SYNC resets it to whatever
  // the master's copy holds. See WriteOutput.
  return d.WriteOutput<std::int32_t>(od::At(od::cia402::kTargetVelocity), targetRpm);
}

std::error_code
Epos4::SetControl(const controls::CyclicPosition & request)
{
  auto & d = *impl_;

  if (auto ec = d.EnsureMode(controls::CyclicPosition::kMode)) {return ec;}

  std::int32_t targetCounts{};
  if (!Resolve(request.position, GetMechanism(), targetCounts)) {
    return std::make_error_code(std::errc::invalid_argument);
  }

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
  return d.WriteOutput<std::int32_t>(od::At(od::cia402::kTargetPosition), targetCounts);
}

std::error_code
Epos4::SetControl(const controls::CyclicVelocity & request)
{
  auto & d = *impl_;

  if (auto ec = d.EnsureMode(controls::CyclicVelocity::kMode)) {return ec;}

  std::int32_t targetRpm{};
  if (!Resolve(request.velocity, GetMechanism(), targetRpm)) {
    return std::make_error_code(std::errc::invalid_argument);
  }

  if (request.velocityOffset) {
    if (auto ec = d.Write<std::int32_t>(
        od::At(od::cia402::kVelocityOffset), *request.velocityOffset)) {return ec;}
  }
  if (request.torqueOffset) {
    if (auto ec = d.Write<std::int16_t>(
        od::At(od::cia402::kTorqueOffset), *request.torqueOffset)) {return ec;}
  }
  // The cyclic modes exist to be commanded every cycle. Staging the setpoint
  // in the RPDO lets it ride the next SYNC instead of costing an SDO round
  // trip per update.
  return d.WriteOutput<std::int32_t>(od::At(od::cia402::kTargetVelocity), targetRpm);
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
  return d.WriteOutput<std::int16_t>(od::At(od::cia402::kTargetTorque), request.torque);
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
    // Check the switch BEFORE anything moves. A homing run whose switch is
    // not mapped in 0x3142 does not fail fast: the axis drives until it hits
    // something mechanical or the timeout expires, which on an arm means
    // finding the end stop with the payload.
    if (const auto required = signals::RequiredInput(*request.method)) {
      std::uint32_t mapped{};
      if (d.Read<std::uint32_t>(od::At(od::cia402::kDigitalInputs), mapped) == std::error_code{}) {
        // 0x60FD only reports functions that are actually assigned to a pin,
        // so an unmapped function can never read as asserted. Confirm the
        // mapping itself rather than the level.
        bool assigned = false;
        for (std::uint8_t sub = 1; sub <= 8 && !assigned; ++sub) {
          std::uint8_t function{};
          if (d.Read<std::uint8_t>(
              od::At(od::maxon::kConfigurationOfDigitalInputs, sub), function) ==
            std::error_code{})
          {
            assigned = (function == static_cast<std::uint8_t>(*required));
          }
        }
        if (!assigned) {
          return false;
        }
      }
    }

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

signals::StatusSignal<units::current::ampere_t> & Epos4::GetMotorCurrent()
{
  return impl_->motorCurrent;
}
signals::StatusSignal<units::current::ampere_t> & Epos4::GetMotorCurrentAveraged()
{
  return impl_->motorCurrentAveraged;
}
signals::StatusSignal<units::voltage::volt_t> & Epos4::GetSupplyVoltage()
{
  return impl_->supplyVoltage;
}
signals::StatusSignal<units::temperature::celsius_t> & Epos4::GetPowerStageTemperature()
{
  return impl_->powerStageTemperature;
}
signals::StatusSignal<units::temperature::celsius_t> & Epos4::GetPowerStageTemperatureLimit()
{
  return impl_->powerStageTemperatureLimit;
}
signals::StatusSignal<std::uint16_t> & Epos4::GetErrorCode() {return impl_->errorCode;}
signals::StatusSignal<std::uint8_t> & Epos4::GetErrorRegister() {return impl_->errorRegister;}
signals::StatusSignal<bool> & Epos4::IsTargetReached() {return impl_->targetReached;}
signals::StatusSignal<bool> & Epos4::IsSetpointAcknowledged() {return impl_->setpointAcknowledged;}
signals::StatusSignal<bool> & Epos4::HasFollowingError() {return impl_->followingErrorFlag;}
signals::StatusSignal<bool> & Epos4::IsHomingAttained() {return impl_->homingAttained;}
signals::StatusSignal<bool> & Epos4::IsInternalLimitActive() {return impl_->internalLimit;}
signals::StatusSignal<bool> & Epos4::HasWarning() {return impl_->warning;}
signals::StatusSignal<signals::BrakeState> & Epos4::GetBrakeState() {return impl_->brakeState;}
signals::StatusSignal<std::uint32_t> & Epos4::GetDigitalInputs() {return impl_->digitalInputs;}
signals::StatusSignal<std::uint32_t> & Epos4::GetMotorRatedTorque()
{
  return impl_->motorRatedTorque;
}
signals::StatusSignal<std::uint16_t> & Epos4::GetDigitalInputPins()
{
  return impl_->digitalInputPins;
}

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
  return impl_ && impl_->PdoActive();
}

std::chrono::steady_clock::duration
Epos4::GetTimeSinceLastPdo() const
{
  if (!impl_) {
    return std::chrono::steady_clock::duration::max();
  }
  return impl_->cyclic.TimeSinceLastPdo(std::chrono::steady_clock::now());
}

std::chrono::steady_clock::duration
Epos4::GetTimeSinceLastEmergency() const
{
  const long long last = impl_ ? impl_->lastEmcy.load(std::memory_order_relaxed) : 0;
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

std::error_code
Epos4::GetIdentity(signals::DeviceIdentity & out)
{
  if (!impl_) {
    return std::make_error_code(std::errc::not_connected);
  }
  auto & d = *impl_;
  signals::DeviceIdentity identity;
  if (auto ec = d.Read(od::comm::kIdentityObject_VendorID, identity.vendorId)) {return ec;}
  if (auto ec = d.Read(od::comm::kIdentityObject_ProductCode, identity.productCode)) {return ec;}
  if (auto ec = d.Read(od::comm::kIdentityObject_RevisionNumber, identity.revisionNumber)) {
    return ec;
  }
  if (auto ec = d.Read(od::comm::kIdentityObject_SerialNumber, identity.serialNumber)) {
    return ec;
  }
  out = identity;
  return {};
}

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
  // Before Start() there is no Impl yet, and devices are declared before
  // Start(), so this is exactly when a caller registers the callback. It used
  // to be dropped here without a word; keep it until Attach() instead.
  if (!impl_) {
    pendingEmcyCallback_ = std::move(callback);
    return;
  }
  std::lock_guard<std::mutex> lock{impl_->emcyMutex};
  impl_->emcyCallback = std::move(callback);
}

}  // namespace epos4

namespace epos4
{

bool
Epos4::IsInputActive(signals::DigitalInputFunction function)
{
  if (function == signals::DigitalInputFunction::kNone) {
    return false;
  }

  auto & signal = GetDigitalInputs();
  signal.Refresh();
  if (signal.GetStatus()) {
    return false;
  }

  // The function's numeric value is its bit position in 0x60FD, which is why
  // this works without a lookup table: the manual uses the same numbering for
  // "what is assigned to this pin" and "which bit reports it".
  const std::uint32_t mask = 1u << static_cast<std::uint8_t>(function);
  return (signal.GetValue() & mask) != 0;
}

bool
Epos4::IsNegativeLimitActive()
{
  // Both variants report on the same bit; the "without errors" one only
  // differs in whether hitting it raises a fault outside homing.
  return IsInputActive(signals::DigitalInputFunction::kNegativeLimitSwitch) ||
         IsInputActive(signals::DigitalInputFunction::kNegativeLimitSwitchNoError);
}

bool
Epos4::IsPositiveLimitActive()
{
  return IsInputActive(signals::DigitalInputFunction::kPositiveLimitSwitch) ||
         IsInputActive(signals::DigitalInputFunction::kPositiveLimitSwitchNoError);
}

bool
Epos4::IsHomeSwitchActive()
{
  return IsInputActive(signals::DigitalInputFunction::kHomeSwitch);
}

}  // namespace epos4

namespace epos4
{

std::optional<std::int16_t>
Epos4::TorqueToPerThousand(units::torque::newton_meter_t torque)
{
  auto & rated = GetMotorRatedTorque();
  rated.Refresh();
  if (rated.GetStatus() || rated.GetValue() == 0) {
    // Zero rated torque means the motor data has not been configured.
    // Returning 0 here would command no torque while looking like success.
    return std::nullopt;
  }

  const double scaled = torque.value() * 1e9 / static_cast<double>(rated.GetValue());

  // 0x6071 is an INTEGER16, so anything past the rails would wrap into a
  // torque in the opposite direction.
  if (scaled > 32767.0 || scaled < -32768.0) {
    return std::nullopt;
  }
  return epos4::ToPerThousand(torque, rated.GetValue());
}

std::optional<units::torque::newton_meter_t>
Epos4::PerThousandToTorque(std::int16_t perThousand)
{
  auto & rated = GetMotorRatedTorque();
  rated.Refresh();
  if (rated.GetStatus() || rated.GetValue() == 0) {
    return std::nullopt;
  }
  return epos4::ToTorque(perThousand, rated.GetValue());
}

}  // namespace epos4

namespace epos4
{

// ---------------------------------------------------------------------------
// Cyclic path
// ---------------------------------------------------------------------------

std::error_code
Epos4::EnterCyclicPositionMode()
{
  if (!impl_) {
    return std::make_error_code(std::errc::not_connected);
  }
  auto & d = *impl_;

  // The one SDO exchange of the whole cyclic path. Doing it here instead of
  // per command is the difference between a control loop and a bus flood.
  if (auto ec = d.EnsureMode(signals::OperationMode::kCyclicSynchronousPosition)) {
    return ec;
  }

  // Seed the setpoint with where the axis actually is. Publishing a zero on
  // the first SYNC would command a move to the origin, which on an arm is a
  // swing across its whole range.
  std::int32_t position{};
  if (auto ec = d.Read<std::int32_t>(od::At(od::cia402::kPositionActualValue), position)) {
    return ec;
  }
  // The Controlword published on every SYNC is whatever the state machine
  // last built, so an axis that was enabled stays enabled.
  d.cyclic.Activate(position, d.controlword.Raw());
  return {};
}

void
Epos4::ExitCyclicMode()
{
  if (impl_) {
    impl_->cyclic.Deactivate();
  }
}

bool
Epos4::IsCyclicModeActive() const
{
  return impl_ && impl_->cyclic.IsActive();
}

std::int32_t
Epos4::GetCachedPosition() const
{
  return impl_ ? impl_->cyclic.Position() : 0;
}

std::int32_t
Epos4::GetCachedVelocity() const
{
  return impl_ ? impl_->cyclic.Velocity() : 0;
}

std::int16_t
Epos4::GetCachedTorque() const
{
  return impl_ ? impl_->cyclic.Torque() : 0;
}

std::uint16_t
Epos4::GetCachedStatusword() const
{
  return impl_ ? impl_->cyclic.Statusword() : 0;
}

std::uint16_t
Epos4::GetCachedErrorCode() const
{
  return impl_ ? impl_->lastEmcyCode.load(std::memory_order_relaxed) : 0;
}

void
Epos4::StageTargetPosition(std::int32_t quadCounts)
{
  if (impl_) {
    impl_->cyclic.StageTargetPosition(quadCounts);
  }
}

bool
Epos4::IsCyclicHealthy(std::chrono::steady_clock::duration maxAge) const
{
  return impl_ && impl_->cyclic.IsHealthy(maxAge, std::chrono::steady_clock::now());
}

}  // namespace epos4
