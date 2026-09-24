#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

// Opaque stand-in for lely::canopen::AsyncMaster, so that no Lely header is
// pulled into anything a user of this library includes.
namespace lely::canopen {class AsyncMaster;}
using lely_master_t = lely::canopen::AsyncMaster;

#include "epos4/CanBus.hpp"
#include "epos4/configs/Configs.hpp"
#include "epos4/controls/ControlRequests.hpp"
#include "epos4/signals/Enums.hpp"
#include "epos4/signals/Errors.hpp"
#include "epos4/signals/StatusSignal.hpp"

namespace epos4
{

class Epos4;
class Encoder;

// ---------------------------------------------------------------------------
// Applies configuration objects to a device, and reads them back.
//
// Separate from the device for the same reason Phoenix separates it: setting
// up a drive and commanding it are different activities, happening at
// different times and usually from different code. Configuration goes over
// SDO and is slow; commanding is on the fast path.
// ---------------------------------------------------------------------------
class Configurator
{
public:
  explicit Configurator(Epos4 & device);

  // Writes every field that was explicitly set. Fields left unset are not
  // touched, so a partially filled configuration will not wipe tuned gains.
  std::error_code Apply(const configs::Epos4Configuration & config);

  // Individual groups, for when only one thing needs changing.
  std::error_code Apply(const configs::MotorConfigs & config);
  std::error_code Apply(const configs::GearConfigs & config);
  std::error_code Apply(const configs::AxisConfigs & config);
  std::error_code Apply(const configs::CurrentControlConfigs & config);
  std::error_code Apply(const configs::PositionControlConfigs & config);
  std::error_code Apply(const configs::VelocityControlConfigs & config);
  std::error_code Apply(const configs::MotionProfileConfigs & config);
  std::error_code Apply(const configs::LimitConfigs & config);
  std::error_code Apply(const configs::HomingConfigs & config);
  std::error_code Apply(const configs::StopOptionConfigs & config);
  std::error_code Apply(const configs::HoldingBrakeConfigs & config);
  std::error_code Apply(const configs::StandstillConfigs & config);
  std::error_code Apply(const configs::DigitalOutputConfigs & config);

  // Reads the current configuration back out of the device.
  std::error_code Refresh(configs::Epos4Configuration & config);

  // Persists the current parameters to non-volatile memory (0x1010).
  // Without this everything applied here is lost at the next power cycle.
  std::error_code Save();

  // Restores factory defaults (0x1011). Takes effect after a reset.
  std::error_code RestoreDefaults();

private:
  std::error_code ApplyWrites(const configs::ConfigWrites & writes);

  Epos4 & device_;
};


// ---------------------------------------------------------------------------
// A maxon EPOS4 positioning controller on a CAN bus.
//
//   epos4::CanBus bus{{.interface = "can0", .masterDcf = dcf}};
//   bus.Start();
//
//   epos4::Epos4 motor{bus, 1};
//
//   epos4::configs::Epos4Configuration cfg;
//   cfg.positionControl.p = 1500000;
//   cfg.limits.maxMotorSpeed = 8000;
//   motor.GetConfigurator().Apply(cfg);
//
//   motor.Enable();
//   motor.SetControl(epos4::controls::ProfilePosition{}
//                      .WithPosition(50000)
//                      .WithVelocity(2000));
//
//   while (!motor.IsTargetReached().GetValueRefreshed()) { ... }
//
// The device owns no CAN state of its own: the bus does. Several devices on
// one bus share the master, the loop and the socket.
// ---------------------------------------------------------------------------
class Epos4
{
public:
  Epos4(CanBus & bus, std::uint8_t nodeId);
  ~Epos4();

  Epos4(const Epos4 &) = delete;
  Epos4 & operator=(const Epos4 &) = delete;

  std::uint8_t GetNodeId() const;

  Configurator & GetConfigurator();

  // The feedback subsystem: sensor slots, the five encoder types and their
  // readings. Kept apart from the device because it is its own subject, with
  // its own rule that nothing can be configured while the motor has power.
  Encoder & GetEncoder();

  // -------------------------------------------------------------------------
  // State machine, section 2.2. The sequencing is bounded and non-blocking
  // internally: each call advances the drive by at most one transition per
  // round trip, so a drive that refuses to move gives up instead of spinning
  // on the bus.
  // -------------------------------------------------------------------------

  // Blocks until the node answers an SDO, i.e. until the master has finished
  // booting it. The first transfer after CanBus::Start() fails with "SDO
  // connection not available" until then, so call this rather than sleeping
  // and hoping.
  bool WaitUntilReady(std::chrono::milliseconds timeout = std::chrono::milliseconds{5000});

  // Walks the drive to «Operation enabled». Returns false on timeout, or if
  // the drive is in fault - faults are never cleared implicitly.
  bool Enable(std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

  // Walks the drive to «Switch on disabled», removing power. Completes even
  // when the axis is faulted: being unable to shut down is worse than an
  // inaccurate report of why.
  bool Disable(std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

  // Clears a fault with the rising edge on Controlword bit 7 that Table 2-7
  // requires. Deliberately explicit and never automatic: a fault is a
  // physical failure, and re-enabling on its own means pushing against
  // whatever caused it. Returns false if the fault condition is still present,
  // which the drive reports by faulting again immediately.
  bool ClearFault(std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

  // Stops motion while staying enabled (Controlword bit 8). Different from
  // QuickStop, which leaves the state machine, and from Disable, which cuts
  // power. How hard it stops is set by MotionProfileConfigs::profileDeceleration.
  std::error_code Halt();

  // Triggers the quick stop ramp (Controlword bit 2 driven low). The drive
  // ends up in «Quick stop active»; what it does on the way is set by
  // StopOptionConfigs::quickStop.
  std::error_code QuickStop();

  bool IsEnabled();
  bool IsFaulted();

  // True once the drive has sent at least one PDO, i.e. the mapping in the
  // DCF took effect. While it is true, status reads cost no bus traffic and
  // cyclic setpoints ride the SYNC instead of an SDO round trip.
  //
  // It is observed rather than configured: PDO mapping is declared in the
  // network description (bus.yml) and pushed by the master at boot, because
  // both ends have to agree on which bytes mean what. Remapping from here at
  // run time would configure the drive and leave the master's own dictionary
  // stale.
  bool IsPdoActive() const;

  // How long since the last PDO arrived. Growing without bound means the
  // cyclic path has stopped even though SDO may still answer.
  std::chrono::steady_clock::duration GetTimeSinceLastPdo() const;

  // -------------------------------------------------------------------------
  // Control
  // -------------------------------------------------------------------------

  // Switches into the mode the request needs, if not already there, then
  // commands it. For ProfilePosition this performs the setpoint handshake of
  // Table 3-15 on the caller's behalf.
  std::error_code SetControl(const controls::ProfilePosition & request);
  std::error_code SetControl(const controls::ProfileVelocity & request);
  std::error_code SetControl(const controls::CyclicPosition & request);
  std::error_code SetControl(const controls::CyclicVelocity & request);
  std::error_code SetControl(const controls::CyclicTorque & request);
  std::error_code SetControl(const controls::Halt & request);

  // Starts a homing run and waits for it to finish. Homing takes seconds and
  // can fail on a limit switch that never triggers, hence the timeout and the
  // boolean rather than fire-and-forget.
  bool Home(
    const controls::Homing & request,
    std::chrono::milliseconds timeout = std::chrono::milliseconds{30000});

  // -------------------------------------------------------------------------
  // Status signals. Each returns a cached signal; call Refresh() to go to the
  // bus. See StatusSignal for why that is explicit.
  // -------------------------------------------------------------------------

  signals::StatusSignal<signals::State> & GetState();
  signals::StatusSignal<std::uint16_t> & GetStatusword();
  signals::StatusSignal<signals::OperationMode> & GetOperationMode();

  signals::StatusSignal<std::int32_t> & GetPosition();        // 0x6064
  signals::StatusSignal<std::int32_t> & GetPositionDemand();  // 0x6062
  signals::StatusSignal<std::int32_t> & GetVelocity();        // 0x606C
  signals::StatusSignal<std::int32_t> & GetVelocityDemand();  // 0x606B
  signals::StatusSignal<std::int16_t> & GetTorque();          // 0x6077
  signals::StatusSignal<std::int32_t> & GetFollowingError();  // 0x60F4
  signals::StatusSignal<std::int16_t> & GetCurrentDemand();   // 0x30D0

  // Error code of the most recent fault, 0x603F. Decode it with
  // signals::DescribeDeviceError, or use DescribeLastError() below.
  signals::StatusSignal<std::uint16_t> & GetErrorCode();

  // Error register, 0x1001. A one-byte summary of what kind of thing is
  // wrong (current, voltage, temperature, communication, motion), available
  // without looking up the specific code.
  signals::StatusSignal<std::uint8_t> & GetErrorRegister();

  // Mode-specific statusword bits, exposed by meaning rather than by number
  // because bit 12 means four different things depending on 0x6061.
  signals::StatusSignal<bool> & IsTargetReached();      // bit 10, PPM/PVM/HMM
  signals::StatusSignal<bool> & IsSetpointAcknowledged();  // bit 12, PPM
  signals::StatusSignal<bool> & HasFollowingError();    // bit 13, PPM/CSP
  signals::StatusSignal<bool> & IsHomingAttained();     // bit 12, HMM
  signals::StatusSignal<bool> & IsInternalLimitActive();  // bit 11
  signals::StatusSignal<bool> & HasWarning();           // bit 7

  // Holding brake state, 0x3158:03. Active means clamped and holding the
  // axis; inactive means released.
  //
  // Read only: the drive drives the brake itself, applying the coupling and
  // opening times from HoldingBrakeConfigs around the state machine's power
  // transitions. There is deliberately no ReleaseBrake() here - releasing a
  // brake out of band on a loaded joint drops the load, and the drive already
  // sequences it correctly against standstill detection.
  //
  // For direct control the hardware supports DigitalOutputFunction::
  // kSetBrakeGpio, which hands the pin over raw, with no timing and no
  // standstill interlock. That is an explicit choice to opt into, not the
  // default.
  signals::StatusSignal<signals::BrakeState> & GetBrakeState();

  // -------------------------------------------------------------------------
  // Diagnostics
  // -------------------------------------------------------------------------

  // Reads 0x603F and 0x1001 and renders them with the chapter 7 tables:
  // name, cause, effect and how to recover. This is what belongs in a log
  // when an axis stops, rather than a bare hexadecimal number.
  std::string DescribeLastError();

  // The drive's own error history, object 0x1003, newest first. It survives
  // a fault reset, so it answers "what happened before the one I am looking
  // at" - which on an intermittent fault is the only useful question.
  std::error_code GetErrorHistory(std::vector<std::uint16_t> & out);

  // Clears the stored history by writing 0 to 0x1003:00.
  std::error_code ClearErrorHistory();

  // Called from the bus thread whenever the drive transmits an emergency
  // frame, without anybody polling. This is how a fault reaches the
  // application before the next status read would have noticed.
  //
  // The callback runs on the CANopen thread: keep it short, do not block,
  // and do not call back into this device from it.
  void SetEmergencyCallback(std::function<void(const signals::EmergencyMessage &)> callback);

  // -------------------------------------------------------------------------
  // Raw object access, for everything this API does not wrap yet. The object
  // dictionary has 164 entries and not all of them deserve a method.
  // -------------------------------------------------------------------------
  template<typename T>
  std::error_code ReadObject(od::Entry entry, T & value);

  template<typename T>
  std::error_code WriteObject(od::Entry entry, T value);

  // Internal. Public only so that free helpers in the implementation file can
  // reach it; the type itself is opaque outside that file.
  struct Impl;

private:
  friend class Configurator;
  friend class CanBus;

  // Creates the underlying CANopen driver and wires the status signals.
  // Called by CanBus::Start(), not by users: the master does not exist until
  // then, and the driver has to be registered before the node boots for its
  // PDOs to be routed here.
  void Attach(lely_master_t & master);

  std::uint8_t nodeId_{0};
  std::unique_ptr<Impl> impl_;
};

}  // namespace epos4
