#include "epos4/hardware/Epos4.hpp"

#include <algorithm>
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

  // The concise DCF the master downloads to this node at boot - read from
  // the master's own dictionary, on the bus thread like every access to it.
  std::error_code ReadConciseDcf(std::vector<std::uint8_t> & out)
  {
    std::promise<std::error_code> promise;
    auto future = promise.get_future();
    Defer(
      [this, &out, &promise]() {
        std::error_code ec;
        out = master.Read<std::vector<std::uint8_t>>(od::comm::kConciseDcf, id(), ec);
        promise.set_value(ec);
      });
    return future.get();
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
    syncCount.fetch_add(1, std::memory_order_release);
    lastSync.store(
      std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);

    if (!cyclic.IsActive()) {
      return;
    }
    try {
      tpdo_mapped[od::cia402::kControlword][0] = cyclic.Controlword();
      // Only the active mode's target. The others keep whatever they last
      // held in the RPDO, which the drive ignores outside their mode.
      switch (cyclic.Mode()) {
        case core::CyclicMode::kPosition:
          tpdo_mapped[od::cia402::kTargetPosition][0] = cyclic.StagedTargetPosition();
          break;
        case core::CyclicMode::kVelocity:
          tpdo_mapped[od::cia402::kTargetVelocity][0] = cyclic.StagedTargetVelocity();
          break;
        case core::CyclicMode::kTorque:
          tpdo_mapped[od::cia402::kTargetTorque][0] = cyclic.StagedTargetTorque();
          break;
      }
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

  // SYNCs seen, and when the last one was. See AwaitCommandApplied().
  std::atomic<std::uint32_t> syncCount{0};
  std::atomic<long long> lastSync{0};

  // Waits until a Controlword just written has reached the drive AND the
  // drive's answer to it has come back, so the next Statusword read
  // reflects the command rather than what came before it.
  //
  // With the Controlword in an RPDO of transmission type 1 it takes three
  // SYNCs, and CiA 301 is why: the write leaves with SYNC n; a synchronous
  // RPDO is not applied on reception but at the NEXT SYNC, n+1, whose TPDO
  // was already sampled; so the first Statusword that reflects the command
  // is the TPDO of SYNC n+2. Measured on the bus: at a 10 ms SYNC, bit 4
  // left at 0 ms and the drive's answer arrived at 20 ms.
  //
  // Reading earlier returns the previous state - for a handshake, the
  // previous handshake's answer. That made Home() report a run attained
  // (bits 12 and 10 still set from the last one) while the axis was still
  // searching, and it could do the same to a second PPM move.
  //
  // Without SYNC running, writes and reads go over SDO, which is already
  // in order: nothing to wait for.
  void AwaitCommandApplied()
  {
    const auto now = std::chrono::steady_clock::now();
    const auto last = std::chrono::steady_clock::time_point{
      std::chrono::steady_clock::duration{lastSync.load(std::memory_order_relaxed)}};
    if (lastSync.load(std::memory_order_relaxed) == 0 ||
      now - last > std::chrono::milliseconds{100})
    {
      return;
    }
    const std::uint32_t start = syncCount.load(std::memory_order_acquire);
    const auto deadline = now + std::chrono::milliseconds{250};

    // Three SYNCs, then a PDO received after the third: its TPDO arrives a
    // moment AFTER the master counts that SYNC, so the SYNC alone is not
    // enough.
    constexpr std::uint32_t kSyncsUntilAnswered = 3;
    while (syncCount.load(std::memory_order_acquire) - start < kSyncsUntilAnswered) {
      if (std::chrono::steady_clock::now() >= deadline) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    const auto answeringSync = std::chrono::steady_clock::time_point{
      std::chrono::steady_clock::duration{lastSync.load(std::memory_order_relaxed)}};
    while (std::chrono::steady_clock::now() < deadline) {
      const auto t = std::chrono::steady_clock::now();
      if (t - cyclic.TimeSinceLastPdo(t) >= answeringSync) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
  }

  // Everything the cyclic path shares between the control thread and this
  // one: the last feedback received, the staged setpoint, and the rule for
  // trusting them. Lock-free throughout; see core::CyclicState.
  core::CyclicState cyclic;

  // «Motor rated torque», read when torque mode is entered, so a torque in
  // N m can be staged from the control loop without an SDO read there.
  std::atomic<std::uint32_t> cyclicRatedTorque{0};


  // Makes sure the drive is in the mode a control request needs. Writing
  // 0x6060 only asks; the manual recommends confirming with 0x6061, because a
  // mode change the drive refused would otherwise go unnoticed and every
  // subsequent command would be interpreted by the wrong mode.
  std::error_code EnsureMode(signals::OperationMode mode)
  {
    // A mode the drive does not implement is refused here, by name, instead
    // of being written and then timing out waiting for 0x6061 to follow.
    if (static_cast<int>(mode) >= 1) {
      if (const auto modes = SupportedDriveModes();
        modes && !signals::SupportsMode(*modes, mode))
      {
        return std::make_error_code(std::errc::not_supported);
      }
    }
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
  signals::StatusSignal<bool> homingError;
  signals::StatusSignal<bool> atZeroSpeed;
  signals::StatusSignal<bool> followingCommand;
  signals::StatusSignal<bool> positionReferenced;
  signals::StatusSignal<bool> remote;
  signals::StatusSignal<bool> voltageEnabled;
  signals::StatusSignal<signals::BrakeState> brakeState;
  signals::StatusSignal<std::uint32_t> digitalInputs;
  signals::StatusSignal<std::uint32_t> motorRatedTorque;
  signals::StatusSignal<std::uint16_t> digitalInputPins;
  signals::StatusSignal<std::uint32_t> digitalOutputs;
  signals::StatusSignal<std::uint16_t> digitalOutputPins;

  signals::StatusSignal<std::int16_t> torqueAveraged;
  signals::StatusSignal<std::int32_t> velocityAveraged;
  signals::StatusSignal<signals::CanBitRate> canBitRate;
  signals::StatusSignal<signals::Fieldbus> activeFieldbus;
  signals::StatusSignal<std::uint32_t> supportedDriveModes;
  signals::StatusSignal<units::voltage::volt_t> analogInputVoltage[2];
  signals::StatusSignal<units::voltage::volt_t> analogInputGeneralPurpose[2];
  signals::StatusSignal<units::voltage::volt_t> analogOutputVoltage[2];

  // What the drive can do, read once: they are constants of the firmware.
  // Guarded because EnsureMode() and Home() may be called from several
  // application threads.
  std::mutex capabilitiesMutex;
  std::optional<std::uint32_t> knownDriveModes;
  std::optional<std::vector<std::int8_t>> knownHomingMethods;

  // «Supported drive modes» (0x6502), or nullopt when it cannot be read -
  // callers then skip the check rather than refuse a mode on a guess.
  std::optional<std::uint32_t> SupportedDriveModes()
  {
    std::lock_guard<std::mutex> lock{capabilitiesMutex};
    if (!knownDriveModes) {
      std::uint32_t modes{};
      // Zero is no information, not "supports nothing": no CiA 402 drive
      // implements no mode. Treated as unknown, so nothing gets refused on
      // it - the first run of this check against a simulator reporting 0
      // refused every mode and left the axis unable to move.
      if (!Read(od::At(od::cia402::kSupportedDriveModes), modes) && modes != 0) {
        knownDriveModes = modes;
      }
    }
    return knownDriveModes;
  }

  // «Supported homing methods» (0x60E3): subindex 0 holds how many follow.
  std::optional<std::vector<std::int8_t>> SupportedHomingMethods()
  {
    std::lock_guard<std::mutex> lock{capabilitiesMutex};
    if (!knownHomingMethods) {
      std::uint8_t count{};
      if (!Read(od::At(od::cia402::kSupportedHomingMethods, 0), count)) {
        std::vector<std::int8_t> methods;
        bool complete = true;
        for (std::uint8_t sub = 1; sub <= count && complete; ++sub) {
          std::int8_t method{};
          complete = !Read(od::At(od::cia402::kSupportedHomingMethods, sub), method);
          methods.push_back(method);
        }
        // All zeros is the same non-answer: 0 is not a homing method.
        bool informative = false;
        for (auto m : methods) {informative = informative || m != 0;}
        if (complete && informative) {knownHomingMethods = methods;}
      }
    }
    return knownHomingMethods;
  }

  signals::StatusSignal<units::current::ampere_t> motorCurrent;
  signals::StatusSignal<units::current::ampere_t> motorCurrentAveraged;
  signals::StatusSignal<units::voltage::volt_t> supplyVoltage;
  signals::StatusSignal<units::temperature::celsius_t> powerStageTemperature;
  signals::StatusSignal<units::temperature::celsius_t> powerStageTemperatureLimit;
  signals::StatusSignal<std::uint16_t> motorI2t;
  signals::StatusSignal<std::uint16_t> powerStageI2t;
  signals::StatusSignal<std::uint16_t> pwmDutyCycle;
  signals::StatusSignal<signals::StoInputStates> stoInputs;
  signals::StatusSignal<signals::StoCardStatus> stoCardStatus;


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
  const bool needsPowerOff = std::any_of(
    writes.begin(), writes.end(),
    [](const configs::ConfigWrite & w) {return configs::RequiresPowerDisabled(w.entry);});
  if (needsPowerOff) {
    if (auto ec = RequirePowerDisabled()) {
      return ec;
    }
  }
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
  if (auto ec = config.Validate()) {
    return ec;
  }
  return ApplyWrites(config.ToWrites());
}

configs::ConfigReader
Configurator::Reader()
{
  auto & d = *device_.impl_;
  return [&d](od::Entry entry, configs::ConfigValue & value) {
           return std::visit([&](auto & typed) {return d.Read(entry, typed);}, value);
         };
}

#define EPOS4_CONFIG_GROUP(Type) \
  std::error_code Configurator::Apply(const configs::Type & config) \
  { \
    configs::ConfigWrites writes; \
    config.AppendTo(writes); \
    return ApplyWrites(writes); \
  } \
  std::error_code Configurator::Refresh(configs::Type & config) \
  { \
    return config.ReadFrom(Reader()); \
  }

EPOS4_CONFIG_GROUP(MotorConfigs)
EPOS4_CONFIG_GROUP(GearConfigs)
EPOS4_CONFIG_GROUP(AxisConfigs)
EPOS4_CONFIG_GROUP(CurrentControlConfigs)
EPOS4_CONFIG_GROUP(PositionControlConfigs)
EPOS4_CONFIG_GROUP(VelocityControlConfigs)
EPOS4_CONFIG_GROUP(VelocityObserverConfigs)
EPOS4_CONFIG_GROUP(MotionProfileConfigs)
EPOS4_CONFIG_GROUP(LimitConfigs)
EPOS4_CONFIG_GROUP(HomingConfigs)
EPOS4_CONFIG_GROUP(StopOptionConfigs)
EPOS4_CONFIG_GROUP(HoldingBrakeConfigs)
EPOS4_CONFIG_GROUP(StandstillConfigs)
EPOS4_CONFIG_GROUP(DigitalOutputConfigs)
EPOS4_CONFIG_GROUP(CyclicConfigs)
EPOS4_CONFIG_GROUP(SiUnitConfigs)
EPOS4_CONFIG_GROUP(AnalogOutputConfigs)
EPOS4_CONFIG_GROUP(ProtectionConfigs)
EPOS4_CONFIG_GROUP(CustomPersistentMemoryConfigs)
EPOS4_CONFIG_GROUP(CommunicationConfigs)

#undef EPOS4_CONFIG_GROUP

std::error_code
Configurator::Refresh(configs::DigitalInputConfigs & config)
{
  return config.ReadFrom(Reader());
}

std::error_code
Configurator::Apply(const configs::AnalogInputConfigs & config)
{
  if (auto ec = config.Validate()) {
    return ec;
  }
  configs::ConfigWrites writes;
  config.AppendTo(writes);
  return ApplyWrites(writes);
}

std::error_code
Configurator::Refresh(configs::AnalogInputConfigs & config)
{
  return config.ReadFrom(Reader());
}

std::error_code
Configurator::Apply(const configs::DualLoopConfigs & config)
{
  if (auto ec = config.Validate()) {
    return ec;
  }
  configs::ConfigWrites writes;
  config.AppendTo(writes);
  return ApplyWrites(writes);
}

std::error_code
Configurator::Refresh(configs::DualLoopConfigs & config)
{
  return config.ReadFrom(Reader());
}

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
Configurator::RequirePowerDisabled()
{
  auto & state = device_.GetState();
  state.Refresh();
  if (state.GetStatus()) {
    return state.GetStatus();
  }
  return signals::IsPowerDisabled(state.GetValue()) ?
         std::error_code{} : std::make_error_code(std::errc::operation_not_permitted);
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
  // 0x1011:01, signature "load". Section 6.2.7. Takes effect after a reset,
  // and is "permitted in NMT state «Pre-Operational» and device state «Power
  // Disable», only". The power state is checked here; the NMT state is the
  // caller's (the drive aborts the write in «Operational»).
  if (auto ec = RequirePowerDisabled()) {
    return ec;
  }
  constexpr std::uint32_t kLoadSignature = 0x64616F6C;  // 'l','o','a','d'
  return device_.impl_->Write<std::uint32_t>(
    od::comm::kRestoreDefaultParameters_RestoreAllDefaultParameters, kLoadSignature);
}

std::error_code
Configurator::Refresh(configs::Epos4Configuration & config)
{
  return config.ReadFrom(Reader());
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
  d.torqueAveraged =
    signals::StatusSignal<std::int16_t>(
    [&d](std::int16_t & out) {
      return d.ReadPreferPdo<std::int16_t>(
        od::maxon::kTorqueActualValues_TorqueActualValueAveraged, out);
    });
  d.velocityAveraged =
    signals::StatusSignal<std::int32_t>(
    [&d](std::int32_t & out) {
      return d.ReadPreferPdo<std::int32_t>(
        od::maxon::kVelocityActualValues_VelocityActualValueAveraged, out);
    });
  d.canBitRate = signals::StatusSignal<signals::CanBitRate>(
    [&d](signals::CanBitRate & out) {
      std::uint8_t raw{};
      auto ec = d.Read(od::At(od::maxon_comm::kCANBitRateDisplay), raw);
      if (!ec) {out = static_cast<signals::CanBitRate>(raw);}
      return ec;
    });
  d.activeFieldbus = signals::StatusSignal<signals::Fieldbus>(
    [&d](signals::Fieldbus & out) {
      std::uint8_t raw{};
      auto ec = d.Read(od::At(od::maxon_comm::kActiveFieldbus), raw);
      if (!ec) {out = static_cast<signals::Fieldbus>(raw);}
      return ec;
    });
  d.supportedDriveModes = signals::StatusSignal<std::uint32_t>(
    [&d](std::uint32_t & out) {
      return d.Read(od::At(od::cia402::kSupportedDriveModes), out);
    });
  // The analog objects are all in mV, INTEGER16 (6.2.79, 6.2.81, 6.2.85).
  auto millivolts = [&d](od::Entry entry) {
      return [&d, entry](units::voltage::volt_t & out) {
               std::int16_t mv{};
               auto ec = d.ReadPreferPdo<std::int16_t>(entry, mv);
               if (!ec) {out = units::voltage::millivolt_t{static_cast<double>(mv)};}
               return ec;
             };
    };
  for (std::uint8_t i = 0; i < 2; ++i) {
    const std::uint8_t sub = i + 1;
    d.analogInputVoltage[i] = signals::StatusSignal<units::voltage::volt_t>(
      millivolts(od::At(od::maxon::kAnalogInputProperties, sub)));
    d.analogInputGeneralPurpose[i] = signals::StatusSignal<units::voltage::volt_t>(
      millivolts(od::At(od::maxon::kAnalogInputGeneralPurpose, sub)));
    d.analogOutputVoltage[i] = signals::StatusSignal<units::voltage::volt_t>(
      millivolts(od::At(od::maxon::kAnalogOutputProperties, sub)));
  }

  d.powerStageTemperatureLimit = signals::StatusSignal<units::temperature::celsius_t>(
    [&d](units::temperature::celsius_t & out) {
      std::uint16_t decidegrees{};
      auto ec = d.Read<std::uint16_t>(
        od::maxon::kThermalOverloadProtection_MaximalTemperaturePowerStage, decidegrees);
      if (!ec) {out = ToTemperature(static_cast<std::int16_t>(decidegrees));}
      return ec;
    });

  auto u16 = [&d](od::Entry entry) {
      return [&d, entry](std::uint16_t & out) {return d.ReadPreferPdo<std::uint16_t>(entry, out);};
    };
  d.motorI2t = signals::StatusSignal<std::uint16_t>(u16(od::maxon::kPowerLimitation_I2tLevelMotor));
  d.powerStageI2t = signals::StatusSignal<std::uint16_t>(
    u16(od::maxon::kPowerLimitation_I2tLevelPowerStage));
  d.pwmDutyCycle = signals::StatusSignal<std::uint16_t>(
    u16(od::maxon::kMotorControl_PWMDutyCycleActualValue));
  d.stoInputs = signals::StatusSignal<signals::StoInputStates>(
    [&d](signals::StoInputStates & out) {
      std::uint8_t raw{};
      auto ec = d.Read(od::maxon::kFunctionalSafety_STOInputStates, raw);
      if (!ec) {out = {(raw & 0x01u) != 0u, (raw & 0x02u) != 0u};}
      return ec;
    });
  d.stoCardStatus = signals::StatusSignal<signals::StoCardStatus>(
    [&d](signals::StoCardStatus & out) {
      // Sub-index 2 exists only on the 60/20, so it is not in the generated
      // table (built from a 50/15 EDS).
      std::uint8_t raw{};
      auto ec = d.Read(od::At(od::maxon::kFunctionalSafety, 2), raw);
      if (!ec) {
        out.state = static_cast<signals::StoCardState>((raw >> 4) & 0x3u);
        out.detection = static_cast<signals::StoCardDetection>(raw & 0x3u);
      }
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
  d.homingError = signals::StatusSignal<bool>(bit(signals::status_bits::kHomingError));
  d.atZeroSpeed = signals::StatusSignal<bool>(bit(signals::status_bits::kSpeed));
  d.followingCommand =
    signals::StatusSignal<bool>(bit(signals::status_bits::kFollowsCommandValue));
  d.positionReferenced = signals::StatusSignal<bool>(bit(signals::status_bits::kHomeRefValid));
  d.remote = signals::StatusSignal<bool>(bit(signals::status_bits::kRemote));
  d.voltageEnabled = signals::StatusSignal<bool>(bit(signals::status_bits::kVoltageEnabled));

  d.motorRatedTorque = signals::StatusSignal<std::uint32_t>(
    [&d](std::uint32_t & out) {
      return d.Read<std::uint32_t>(od::At(od::cia402::kMotorRatedTorque), out);
    });

  d.digitalInputs = signals::StatusSignal<std::uint32_t>(
    [&d](std::uint32_t & out) {
      return d.ReadPreferPdo<std::uint32_t>(od::At(od::cia402::kDigitalInputs), out);
    });

  d.digitalOutputs = signals::StatusSignal<std::uint32_t>(
    [&d](std::uint32_t & out) {
      return d.ReadPreferPdo<std::uint32_t>(od::cia402::kDigitalOutputs_PhysicalOutputs, out);
    });
  d.digitalOutputPins = signals::StatusSignal<std::uint16_t>(
    [&d](std::uint16_t & out) {
      return d.ReadPreferPdo<std::uint16_t>(
        od::maxon::kDigitalOutputProperties_DigitalOutputsLogicState, out);
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
  d.AwaitCommandApplied();

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
  // Let the drive see bit 4 low before anyone raises it again: the setpoint
  // is taken on its rising edge, and a next move that raised it within the
  // same SYNC period would produce no edge and no move.
  d.AwaitCommandApplied();

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

namespace
{

// A torque given as a torque needs the motor's rated torque to become the
// drive's unit (thousandths of 0x6076); a raw one does not, and then the
// rated torque is not read at all. Reads it once and keeps it in the signal.
std::error_code
ResolveTorque(
  signals::StatusSignal<std::uint32_t> & ratedTorque, const TorqueSetpoint & setpoint,
  std::int16_t & out)
{
  if (!setpoint.IsRaw() && !ratedTorque.HasValue()) {
    if (auto ec = ratedTorque.Refresh().GetStatus()) {return ec;}
  }
  return Resolve(setpoint, ratedTorque.GetValue(), out) ?
         std::error_code{} : std::make_error_code(std::errc::invalid_argument);
}

}  // namespace

std::error_code
Epos4::SetControl(const controls::CyclicPosition & request)
{
  auto & d = *impl_;

  // Everything resolved before anything is written: a request that cannot
  // be expressed in drive units must not leave half of itself behind.
  std::int32_t targetCounts{};
  if (!Resolve(request.position, GetMechanism(), targetCounts)) {
    return std::make_error_code(std::errc::invalid_argument);
  }
  std::int32_t positionOffset{};
  if (request.positionOffset &&
    !Resolve(*request.positionOffset, GetMechanism(), positionOffset))
  {
    return std::make_error_code(std::errc::invalid_argument);
  }
  std::int16_t torqueOffset{};
  if (request.torqueOffset) {
    if (auto ec = ResolveTorque(d.motorRatedTorque, *request.torqueOffset, torqueOffset)) {
      return ec;
    }
  }

  if (auto ec = d.EnsureMode(controls::CyclicPosition::kMode)) {return ec;}
  if (request.positionOffset) {
    if (auto ec = d.Write<std::int32_t>(
        od::At(od::cia402::kPositionOffset), positionOffset)) {return ec;}
  }
  if (request.torqueOffset) {
    if (auto ec = d.Write<std::int16_t>(
        od::At(od::cia402::kTorqueOffset), torqueOffset)) {return ec;}
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

  std::int32_t targetRpm{};
  if (!Resolve(request.velocity, GetMechanism(), targetRpm)) {
    return std::make_error_code(std::errc::invalid_argument);
  }
  std::int32_t velocityOffset{};
  if (request.velocityOffset &&
    !Resolve(*request.velocityOffset, GetMechanism(), velocityOffset))
  {
    return std::make_error_code(std::errc::invalid_argument);
  }
  std::int16_t torqueOffset{};
  if (request.torqueOffset) {
    if (auto ec = ResolveTorque(d.motorRatedTorque, *request.torqueOffset, torqueOffset)) {
      return ec;
    }
  }

  if (auto ec = d.EnsureMode(controls::CyclicVelocity::kMode)) {return ec;}
  if (request.velocityOffset) {
    if (auto ec = d.Write<std::int32_t>(
        od::At(od::cia402::kVelocityOffset), velocityOffset)) {return ec;}
  }
  if (request.torqueOffset) {
    if (auto ec = d.Write<std::int16_t>(
        od::At(od::cia402::kTorqueOffset), torqueOffset)) {return ec;}
  }
  return d.WriteOutput<std::int32_t>(od::At(od::cia402::kTargetVelocity), targetRpm);
}

std::error_code
Epos4::SetControl(const controls::CyclicTorque & request)
{
  auto & d = *impl_;

  std::int16_t target{};
  if (auto ec = ResolveTorque(d.motorRatedTorque, request.torque, target)) {return ec;}
  std::int16_t torqueOffset{};
  if (request.torqueOffset) {
    if (auto ec = ResolveTorque(d.motorRatedTorque, *request.torqueOffset, torqueOffset)) {
      return ec;
    }
  }

  if (auto ec = d.EnsureMode(controls::CyclicTorque::kMode)) {return ec;}
  if (request.torqueOffset) {
    if (auto ec = d.Write<std::int16_t>(
        od::At(od::cia402::kTorqueOffset), torqueOffset)) {return ec;}
  }
  return d.WriteOutput<std::int16_t>(od::At(od::cia402::kTargetTorque), target);
}

std::error_code
Epos4::SetControl(const controls::Halt &)
{
  return Halt();
}

std::error_code
Epos4::SetPosition(PositionSetpoint position, std::chrono::milliseconds timeout)
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  auto & d = *impl_;

  std::int32_t counts{};
  if (!Resolve(position, GetMechanism(), counts)) {
    return std::make_error_code(std::errc::invalid_argument);
  }
  // Homing only starts from «Operation enabled»; without this check the run
  // below would simply time out with nothing said about why.
  if (!IsEnabled()) {
    return std::make_error_code(std::errc::operation_not_permitted);
  }

  // What this borrows, to give it back afterwards.
  std::int8_t method{};
  std::int32_t homePosition{};
  std::int32_t offsetDistance{};
  std::int8_t mode{};
  if (auto ec = d.Read(od::At(od::cia402::kHomingMethod), method)) {return ec;}
  if (auto ec = d.Read(od::At(od::maxon::kHomePosition), homePosition)) {return ec;}
  if (auto ec = d.Read(od::At(od::maxon::kHomeOffsetMoveDistance), offsetDistance)) {return ec;}
  if (auto ec = d.Read(od::At(od::cia402::kModesOfOperationDisplay), mode)) {return ec;}

  std::error_code result;
  if (auto ec = d.Write<std::int32_t>(od::At(od::maxon::kHomePosition), counts)) {
    result = ec;
  } else if (auto ec2 = d.Write<std::int32_t>(od::At(od::maxon::kHomeOffsetMoveDistance), 0)) {
    result = ec2;
  } else if (!Home(
      controls::Homing{}.WithMethod(signals::HomingMethod::kActualPosition),
      timeout))
  {
    result = std::make_error_code(std::errc::timed_out);
  }

  // Restored whatever happened above: a failed SetPosition must not leave
  // somebody's homing configuration replaced by method 37 and a zero offset.
  // The first restore error is reported only if the run itself succeeded.
  auto restore = [&result](std::error_code ec) {
      if (ec && !result) {result = ec;}
    };
  restore(d.Write<std::int8_t>(od::At(od::cia402::kHomingMethod), method));
  restore(d.Write<std::int32_t>(od::At(od::maxon::kHomePosition), homePosition));
  restore(d.Write<std::int32_t>(od::At(od::maxon::kHomeOffsetMoveDistance), offsetDistance));
  if (mode != static_cast<std::int8_t>(signals::OperationMode::kHoming)) {
    restore(d.EnsureMode(static_cast<signals::OperationMode>(mode)));
  }
  return result;
}

bool
Epos4::Home(const controls::Homing & request, std::chrono::milliseconds timeout)
{
  auto & d = *impl_;

  if (d.EnsureMode(controls::Homing::kMode)) {return false;}

  if (request.method) {
    // A method the firmware does not list in 0x60E3 is refused up front.
    if (const auto methods = d.SupportedHomingMethods()) {
      bool listed = false;
      for (auto m : *methods) {
        listed = listed || m == static_cast<std::int8_t>(*request.method);
      }
      if (!listed) {return false;}
    }

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
            assigned = signals::SatisfiesHomingInput(
              static_cast<signals::DigitalInputFunction>(function), *required);
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
  d.AwaitCommandApplied();

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
  // As for PPM: the run starts on the rising edge of bit 4, so the low has
  // to reach the drive before a following Home() raises it again.
  d.AwaitCommandApplied();
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
signals::StatusSignal<std::uint16_t> & Epos4::GetMotorI2tPercent() {return impl_->motorI2t;}
signals::StatusSignal<std::uint16_t> & Epos4::GetPowerStageI2tPercent()
{
  return impl_->powerStageI2t;
}
signals::StatusSignal<std::uint16_t> & Epos4::GetPwmDutyCyclePerMille()
{
  return impl_->pwmDutyCycle;
}
signals::StatusSignal<signals::StoInputStates> & Epos4::GetStoInputs() {return impl_->stoInputs;}
signals::StatusSignal<signals::StoCardStatus> & Epos4::GetStoCardStatus()
{
  return impl_->stoCardStatus;
}
signals::StatusSignal<std::int16_t> & Epos4::GetTorqueAveraged() {return impl_->torqueAveraged;}
signals::StatusSignal<std::int32_t> & Epos4::GetVelocityAveraged() {return impl_->velocityAveraged;}
signals::StatusSignal<signals::CanBitRate> & Epos4::GetCanBitRate() {return impl_->canBitRate;}
signals::StatusSignal<signals::Fieldbus> & Epos4::GetActiveFieldbus()
{
  return impl_->activeFieldbus;
}
signals::StatusSignal<std::uint32_t> & Epos4::GetSupportedDriveModes()
{
  return impl_->supportedDriveModes;
}
signals::StatusSignal<units::voltage::volt_t> &
Epos4::GetAnalogInputVoltage(signals::AnalogInput input)
{
  return impl_->analogInputVoltage[static_cast<std::uint8_t>(input) - 1];
}
signals::StatusSignal<units::voltage::volt_t> &
Epos4::GetAnalogInputGeneralPurpose(signals::AnalogGeneralPurpose value)
{
  return impl_->analogInputGeneralPurpose[static_cast<std::uint8_t>(value) - 1];
}
signals::StatusSignal<units::voltage::volt_t> &
Epos4::GetAnalogOutputVoltage(signals::AnalogOutput output)
{
  return impl_->analogOutputVoltage[static_cast<std::uint8_t>(output) - 1];
}

std::error_code
Epos4::SetAnalogOutput(signals::AnalogGeneralPurpose output, units::voltage::volt_t voltage)
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  // 0x3182 is in mV, INTEGER32, and 6.2.87 gives the range as +-4000 mV.
  // Refused beyond it rather than clamped: an output that silently does
  // not reach what was asked for is worse than an error.
  const double mv = units::voltage::millivolt_t{voltage}.value();
  if (mv < -4000.0 || mv > 4000.0) {
    return std::make_error_code(std::errc::argument_out_of_domain);
  }
  const auto value = static_cast<std::int32_t>(mv < 0.0 ? mv - 0.5 : mv + 0.5);
  return impl_->WriteOutput<std::int32_t>(
    od::At(od::maxon::kAnalogOutputGeneralPurpose, static_cast<std::uint8_t>(output)), value);
}

bool
Epos4::SupportsMode(signals::OperationMode mode)
{
  if (!impl_) {return false;}
  const auto modes = impl_->SupportedDriveModes();
  return modes && signals::SupportsMode(*modes, mode);
}

std::error_code
Epos4::GetSupportedHomingMethods(std::vector<signals::HomingMethod> & out)
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  const auto methods = impl_->SupportedHomingMethods();
  if (!methods) {return std::make_error_code(std::errc::io_error);}
  out.clear();
  for (auto m : *methods) {
    out.push_back(static_cast<signals::HomingMethod>(m));
  }
  return {};
}
signals::StatusSignal<std::uint16_t> & Epos4::GetErrorCode() {return impl_->errorCode;}
signals::StatusSignal<std::uint8_t> & Epos4::GetErrorRegister() {return impl_->errorRegister;}
signals::StatusSignal<bool> & Epos4::IsTargetReached() {return impl_->targetReached;}
signals::StatusSignal<bool> & Epos4::IsSetpointAcknowledged() {return impl_->setpointAcknowledged;}
signals::StatusSignal<bool> & Epos4::HasFollowingError() {return impl_->followingErrorFlag;}
signals::StatusSignal<bool> & Epos4::IsHomingAttained() {return impl_->homingAttained;}
signals::StatusSignal<bool> & Epos4::IsInternalLimitActive() {return impl_->internalLimit;}
signals::StatusSignal<bool> & Epos4::HasWarning() {return impl_->warning;}
signals::StatusSignal<bool> & Epos4::HasHomingError() {return impl_->homingError;}
signals::StatusSignal<bool> & Epos4::IsAtZeroSpeed() {return impl_->atZeroSpeed;}
signals::StatusSignal<bool> & Epos4::IsFollowingCommand() {return impl_->followingCommand;}
signals::StatusSignal<bool> & Epos4::IsPositionReferenced() {return impl_->positionReferenced;}
signals::StatusSignal<bool> & Epos4::IsRemote() {return impl_->remote;}
signals::StatusSignal<bool> & Epos4::IsVoltageEnabled() {return impl_->voltageEnabled;}
signals::StatusSignal<signals::BrakeState> & Epos4::GetBrakeState() {return impl_->brakeState;}
signals::StatusSignal<std::uint32_t> & Epos4::GetDigitalInputs() {return impl_->digitalInputs;}
signals::StatusSignal<std::uint32_t> & Epos4::GetMotorRatedTorque()
{
  return impl_->motorRatedTorque;
}
signals::StatusSignal<std::uint32_t> & Epos4::GetDigitalOutputs()
{
  return impl_->digitalOutputs;
}

signals::StatusSignal<std::uint16_t> & Epos4::GetDigitalOutputPins()
{
  return impl_->digitalOutputPins;
}

std::error_code
Epos4::SetDigitalOutput(signals::DigitalOutputFunction function, bool active)
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  using F = signals::DigitalOutputFunction;
  // Bits 24..31 of 0x60FE are read-only (Table 6-170): the holding brake
  // and Ready/Fault belong to the drive, which sequences them itself.
  if (function == F::kHoldingBrake || function == F::kReadyFault || function == F::kNone) {
    return std::make_error_code(std::errc::invalid_argument);
  }
  auto & d = *impl_;

  // Read-modify-write: 0x60FE:01 carries every output at once, and the
  // others must keep whatever the host last set them to.
  std::uint32_t outputs{};
  if (auto ec = d.Read(od::cia402::kDigitalOutputs_PhysicalOutputs, outputs)) {return ec;}
  const std::uint32_t bit = 1u << static_cast<std::uint8_t>(function);
  outputs = active ? (outputs | bit) : (outputs & ~bit);
  // Through the RPDO when mapped (RXPDO-mappable, section 6.2.148), as with
  // every output - see WriteOutput.
  return d.WriteOutput<std::uint32_t>(od::cia402::kDigitalOutputs_PhysicalOutputs, outputs);
}

bool
Epos4::IsOutputActive(signals::DigitalOutputFunction function)
{
  if (function == signals::DigitalOutputFunction::kNone) {return false;}
  auto & signal = GetDigitalOutputs();
  if (signal.Refresh().GetStatus()) {return false;}
  // As with the inputs, the function's value is its bit position in 0x60FE.
  return (signal.GetValue() >> static_cast<std::uint8_t>(function)) & 1u;
}

std::error_code
Epos4::ArmTouchProbe(const controls::TouchProbe & probe)
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  if (!probe.IsValid()) {return std::make_error_code(std::errc::invalid_argument);}
  auto & d = *impl_;

  if (probe.trigger == controls::TouchProbe::Trigger::kInput) {
    bool mapped = false;
    for (std::uint8_t sub = 1; sub <= 8 && !mapped; ++sub) {
      std::uint8_t function{};
      if (!d.Read(od::At(od::maxon::kConfigurationOfDigitalInputs, sub), function)) {
        mapped = function == static_cast<std::uint8_t>(signals::DigitalInputFunction::kTouchProbe);
      }
    }
    if (!mapped) {return std::make_error_code(std::errc::invalid_argument);}
  }

  // A probe already enabled keeps its latched values; switching it off
  // first makes each Arm() a fresh start, counters included.
  if (auto ec = DisarmTouchProbe()) {return ec;}
  return d.WriteOutput<std::uint16_t>(
    od::At(od::cia402::kTouchProbeFunction), probe.ToFunctionWord());
}

std::error_code
Epos4::DisarmTouchProbe()
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  return impl_->WriteOutput<std::uint16_t>(od::At(od::cia402::kTouchProbeFunction), 0);
}

std::error_code
Epos4::GetTouchProbe(signals::TouchProbeState & out)
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  auto & d = *impl_;
  signals::TouchProbeState state;
  std::uint16_t status{};
  if (auto ec = d.Read(od::At(od::cia402::kTouchProbeStatus), status)) {return ec;}
  signals::DecodeTouchProbeStatus(status, state);
  if (auto ec = d.Read(od::At(od::cia402::kTouchProbe1PositiveEdge), state.positiveEdgePosition)) {
    return ec;
  }
  if (auto ec = d.Read(od::At(od::cia402::kTouchProbe1NegativeEdge), state.negativeEdgePosition)) {
    return ec;
  }
  if (auto ec = d.Read(
      od::At(od::cia402::kTouchProbe1PositiveEdgeCounter), state.positiveEdgeCount)) {return ec;}
  if (auto ec = d.Read(
      od::At(od::cia402::kTouchProbe1NegativeEdgeCounter), state.negativeEdgeCount)) {return ec;}
  out = state;
  return {};
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
  // Best effort beyond 0x1018: a firmware without one of these leaves that
  // field at its default rather than failing the identity as a whole.
  d.Read(od::At(od::comm::kDeviceType), identity.deviceType);
  d.Read(od::At(od::comm::kManufacturerDeviceName), identity.deviceName);
  d.Read(od::maxon_comm::kAdditionalIdentity_SerialNumberComplete, identity.serialNumberComplete);
  d.Read(od::comm::kProgramSoftwareIdentification_ProgramNumber1, identity.programSoftware);
  d.Read(od::comm::kFlashStatusIdentification_ProgramNumber1, identity.flashStatus);
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
Epos4::ReadPdoMapping(signals::PdoMapping & out)
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  auto & d = *impl_;
  signals::PdoMapping mapping;
  for (auto * channels : {&mapping.rpdo, &mapping.tpdo}) {
    for (auto & c : *channels) {
      const std::uint16_t param = c.ParameterIndex();
      if (auto ec = d.Read(od::At(param, 1), c.cobId)) {return ec;}
      if (auto ec = d.Read(od::At(param, 2), c.transmissionType)) {return ec;}
      if (c.direction == signals::PdoDirection::kTransmit) {
        std::uint16_t inhibit{};
        if (auto ec = d.Read(od::At(param, 3), inhibit)) {return ec;}
        c.inhibitTime100us = inhibit;
      }
      std::uint8_t count{};
      if (auto ec = d.Read(od::At(c.MappingIndex(), 0), count)) {return ec;}
      for (std::uint8_t sub = 1; sub <= count; ++sub) {
        std::uint32_t raw{};
        if (auto ec = d.Read(od::At(c.MappingIndex(), sub), raw)) {return ec;}
        c.objects.push_back(signals::PdoObject::Decode(raw));
      }
    }
  }
  out = std::move(mapping);
  return {};
}

std::error_code
Epos4::CheckPdoMapping(std::vector<signals::PdoMismatch> & mismatches)
{
  mismatches.clear();
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}

  std::vector<std::uint8_t> bytes;
  if (auto ec = impl_->ReadConciseDcf(bytes)) {return ec;}
  // An empty domain: the network description configures nothing on this
  // node, so there is no expectation to compare against.
  if (bytes.empty()) {return std::make_error_code(std::errc::no_such_file_or_directory);}
  const auto concise = signals::ParseConciseDcf(bytes);
  if (!concise) {return std::make_error_code(std::errc::bad_message);}

  signals::PdoMapping actual;
  if (auto ec = ReadPdoMapping(actual)) {return ec;}
  mismatches = signals::CompareWithConciseDcf(*concise, actual);
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

namespace
{

// Shared by the three Enter*Mode() calls: they differ only in the operating
// mode and in which setpoint OnSync publishes.
std::error_code
EnterCyclic(Epos4::Impl & d, signals::OperationMode mode, core::CyclicMode cyclicMode)
{
  // The one SDO exchange of the whole cyclic path. Doing it here instead of
  // per command is the difference between a control loop and a bus flood.
  if (auto ec = d.EnsureMode(mode)) {
    return ec;
  }

  // Seed every setpoint with where the axis actually is - see
  // CyclicState::Activate(). Read by SDO: the cached PDO value may be a
  // cycle old, and this is the value the first SYNC will command.
  std::int32_t position{};
  if (auto ec = d.Read<std::int32_t>(od::At(od::cia402::kPositionActualValue), position)) {
    return ec;
  }
  std::int16_t torque{};
  if (cyclicMode == core::CyclicMode::kTorque) {
    if (auto ec = d.Read<std::int16_t>(od::At(od::cia402::kTorqueActualValue), torque)) {
      return ec;
    }
    std::uint32_t rated{};
    if (auto ec = d.Read<std::uint32_t>(od::At(od::cia402::kMotorRatedTorque), rated)) {
      return ec;
    }
    d.cyclicRatedTorque.store(rated, std::memory_order_relaxed);
  }

  // The Controlword published on every SYNC is whatever the state machine
  // last built, so an axis that was enabled stays enabled.
  d.cyclic.Activate(cyclicMode, position, torque, d.controlword.Raw());
  return {};
}

}  // namespace

std::error_code
Epos4::EnterCyclicPositionMode()
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  return EnterCyclic(
    *impl_, signals::OperationMode::kCyclicSynchronousPosition, core::CyclicMode::kPosition);
}

std::error_code
Epos4::EnterCyclicVelocityMode()
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  return EnterCyclic(
    *impl_, signals::OperationMode::kCyclicSynchronousVelocity, core::CyclicMode::kVelocity);
}

std::error_code
Epos4::EnterCyclicTorqueMode()
{
  if (!impl_) {return std::make_error_code(std::errc::not_connected);}
  return EnterCyclic(
    *impl_, signals::OperationMode::kCyclicSynchronousTorque, core::CyclicMode::kTorque);
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

bool
Epos4::StageTargetPosition(PositionSetpoint position)
{
  std::int32_t counts{};
  if (!impl_ || !Resolve(position, GetMechanism(), counts)) {return false;}
  impl_->cyclic.StageTargetPosition(counts);
  return true;
}

bool
Epos4::StageTargetVelocity(VelocitySetpoint velocity)
{
  std::int32_t rpm{};
  if (!impl_ || !Resolve(velocity, GetMechanism(), rpm)) {return false;}
  impl_->cyclic.StageTargetVelocity(rpm);
  return true;
}

bool
Epos4::StageTargetTorque(TorqueSetpoint torque)
{
  std::int16_t perThousand{};
  if (!impl_ ||
    !Resolve(torque, impl_->cyclicRatedTorque.load(std::memory_order_relaxed), perThousand))
  {
    return false;
  }
  impl_->cyclic.StageTargetTorque(perThousand);
  return true;
}

bool
Epos4::IsCyclicHealthy(std::chrono::steady_clock::duration maxAge) const
{
  return impl_ && impl_->cyclic.IsHealthy(maxAge, std::chrono::steady_clock::now());
}

}  // namespace epos4
