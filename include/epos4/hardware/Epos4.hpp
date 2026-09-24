#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
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
#include "epos4/core/UnitConversion.hpp"
#include "epos4/signals/Errors.hpp"
#include "epos4/signals/Identity.hpp"
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

  // Validates the mapping before writing: the manual forbids the same
  // function on two inputs, and a rejected write halfway through would leave
  // limit switches half configured.
  std::error_code Apply(const configs::DigitalInputConfigs & config);
  std::error_code Apply(const configs::CyclicConfigs & config);

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

  // Tells the driver how many encoder counts make a turn and what the gearbox
  // does, which is what lets control requests and feedback be expressed in
  // real units.
  //
  //   motor.SetMechanism(2000, 1.0 / 100.0);   // 500 CPR, 1:100 reduction
  //   motor.SetControl(controls::ProfilePosition{}.WithPosition(90_deg));
  //
  // Without it, requests given in raw counts still work; requests given as
  // quantities are rejected with std::errc::invalid_argument rather than
  // being resolved against a guessed resolution.
  //
  // This is a shortcut to Encoder::SetMechanism(); the scale is shared.
  void SetMechanism(std::uint32_t quadCountsPerRevolution, double gearRatio = 1.0);
  const MechanismScale & GetMechanism() const;

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
  //
  // For the communication faults whose recovery in chapter 7 starts with an
  // NMT reset communication (signals::RequiresCommunicationReset: a lost
  // heartbeat, CAN passive mode) that reset is sent first and the node is
  // waited for until the master has booted it again. The timeout covers the
  // whole sequence, which is why it is longer than a fault reset needs.
  bool ClearFault(std::chrono::milliseconds timeout = std::chrono::milliseconds{3000});

  // Stops motion while staying enabled (Controlword bit 8). Different from
  // QuickStop, which leaves the state machine, and from Disable, which cuts
  // power. How hard it stops is set by MotionProfileConfigs::profileDeceleration.
  //
  // NOTE: what Halt does is formally decided by «Halt option code» (0x605D,
  // Table 6-152). That object is documented in the manual but is NOT present
  // in the EDS of the EPOS4 Module 50/15, so it is not exposed here - adding
  // a setter for an object the device does not implement would only produce
  // aborts at run time. The behaviour is whatever the firmware ships with.
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
  //
  // Before moving, the switch the method depends on is checked against the
  // digital input mapping (0x3142). A method whose switch is not assigned to
  // any pin returns false immediately instead of driving the axis into
  // whatever it finds first. See signals::RequiredInput().
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

  // Motor rated torque, 0x6076 [uNm]. READ ONLY: the drive computes it as
  // «Nominal current» x «Torque constant», both from MotorConfigs, and the
  // manual states that "changing the value by write access is not permitted".
  //
  // It matters because CyclicTorque commands 0x6071 in THOUSANDTHS of this
  // value. Without reading it, WithTorque(500) is a number with no physical
  // meaning - nobody can tell whether it is 50 mNm or 5 Nm. Use the two
  // converters below.
  signals::StatusSignal<std::uint32_t> & GetMotorRatedTorque();

  // Converts between real torque and the per-thousand units that 0x6071 and
  // 0x6077 use. Both refresh the rated torque signal, so they touch the bus
  // unless it has been read already.
  //
  // Return nullopt when the rated torque cannot be read or is zero, which
  // means the motor data has not been configured yet: silently returning 0
  // would command no torque and look like a working call.
  std::optional<std::int16_t> TorqueToPerThousand(units::torque::newton_meter_t torque);
  std::optional<units::torque::newton_meter_t> PerThousandToTorque(std::int16_t perThousand);

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

  // -------------------------------------------------------------------------
  // Power and thermal
  //
  // In physical units, unlike the motion signals above. Those stay in counts
  // and rpm because that is what the manual and EPOS Studio speak and what a
  // drive is tuned in; here the raw units (mA, tenths of a volt, tenths of a
  // degree) carry no meaning of their own and only invite a factor-of-ten
  // mistake. Phoenix makes the same choice for the same signals.
  // -------------------------------------------------------------------------

  // Motor current, 0x30D1 (section 6.2.70). The instantaneous value, and the
  // same through a 50 Hz first-order low-pass - the one to log or display,
  // the instantaneous one being mostly PWM ripple at that rate.
  signals::StatusSignal<units::current::ampere_t> & GetMotorCurrent();
  signals::StatusSignal<units::current::ampere_t> & GetMotorCurrentAveraged();

  // Power supply voltage, 0x2200:01. On a rover this is the battery, and a
  // sagging one shows up here before it shows up as an undervoltage fault
  // (0x3220). Not PDO-mappable: every Refresh() is an SDO read.
  signals::StatusSignal<units::voltage::volt_t> & GetSupplyVoltage();

  // Power stage temperature, 0x3201:01, and the limit above which the drive
  // raises «Thermal overload error» (0x4210) and stops: 0x3201:04. The gap
  // between the two is the warning a caller gets before an arm stops
  // mid-motion. Not PDO-mappable either.
  signals::StatusSignal<units::temperature::celsius_t> & GetPowerStageTemperature();
  signals::StatusSignal<units::temperature::celsius_t> & GetPowerStageTemperatureLimit();

  // -------------------------------------------------------------------------
  // Digital inputs
  //
  // Two views of the same pins, and the difference matters:
  //
  //   GetDigitalInputs()      0x60FD, indexed BY FUNCTION and after polarity
  //                           correction. Bit 0 is the negative limit switch
  //                           whichever terminal it happens to be wired to.
  //   GetDigitalInputPins()   0x3141:01, indexed BY PIN and before polarity
  //                           correction. This is the one to look at when
  //                           checking wiring, because it shows what the
  //                           terminal actually sees.
  // -------------------------------------------------------------------------
  signals::StatusSignal<std::uint32_t> & GetDigitalInputs();
  signals::StatusSignal<std::uint16_t> & GetDigitalInputPins();

  // Whether a given function is currently asserted, read from 0x60FD.
  // Refreshes the signal, so it costs a bus read unless PDOs are mapped.
  bool IsInputActive(signals::DigitalInputFunction function);

  // The three that homing depends on, by name. A limit switch that reads as
  // asserted before a homing run started usually means the polarity is
  // inverted, not that the axis is at the end stop.
  bool IsNegativeLimitActive();
  bool IsPositiveLimitActive();
  bool IsHomeSwitchActive();

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
  // Cyclic path
  //
  // Everything else on this class does a thread hop and a blocking wait: a
  // call posts onto the CANopen thread and waits on a future. That is fine
  // for configuration and for moving to poses, and unusable in a control
  // loop - at 500 Hz with six axes it is thousands of context switches a
  // second, each one blocking the thread that must not block.
  //
  // The accessors below do none of that. Inbound PDOs land in atomics on the
  // bus thread; setpoints are staged into atomics and published by the bus
  // thread on the next SYNC. Reading and writing from the control loop is a
  // relaxed atomic load or store, with no syscall and no waiting.
  //
  //     control thread                bus thread
  //     ──────────────                ──────────
  //     StageTargetPosition() ──────→ OnSync: atomics -> RPDO
  //     GetCachedPosition()   ←────── OnRpdoWrite: TPDO -> atomics
  // -------------------------------------------------------------------------

  // Switches the drive into Cyclic Synchronous Position and starts publishing
  // the staged setpoint on every SYNC.
  //
  // The mode is set ONCE here, not per command. SetControl() calls
  // EnsureMode() every time, which costs an SDO read - two CAN frames and a
  // round trip - and in a cyclic loop that is the whole budget.
  //
  // The drive must already be enabled: this does not walk the state machine.
  std::error_code EnterCyclicPositionMode();

  // Stops publishing. The drive keeps whatever mode it is in; use Disable()
  // to remove power.
  void ExitCyclicMode();

  bool IsCyclicModeActive() const;

  // Lock-free. Safe to call from a real-time loop.
  std::int32_t GetCachedPosition() const;    // 0x6064 [quadcounts]
  std::int32_t GetCachedVelocity() const;    // 0x606C [rpm]
  std::int16_t GetCachedTorque() const;      // 0x6077 [per thousand of rated]
  std::uint16_t GetCachedStatusword() const;  // 0x6041

  // The error code of the last EMCY the drive sent, 0 if none or if the
  // drive has since announced an error reset. Arrives unsolicited, so unlike
  // GetErrorCode() (0x603F, an SDO read) it costs nothing and still answers
  // when the node has gone silent - which is when it is most needed.
  std::uint16_t GetCachedErrorCode() const;

  // When that EMCY arrived; duration::max() if none ever did. A code without
  // its age is misleading: an EMCY from the boot sequence, minutes earlier,
  // would read as the cause of whatever just happened.
  std::chrono::steady_clock::duration GetTimeSinceLastEmergency() const;

  // Stages the next target position. It goes out on the following SYNC.
  // Lock-free, and safe to call from a different thread than the bus.
  void StageTargetPosition(std::int32_t quadCounts);

  // True while PDOs are arriving and the drive reports «Operation enabled».
  //
  // Worth checking every cycle: if the bus goes quiet the cached values stop
  // changing but keep reading back happily, so a controller would go on
  // believing a dead axis is tracking. maxAge is how long without a PDO
  // counts as dead; at a 10 ms SYNC a few tens of milliseconds is generous.
  bool IsCyclicHealthy(
    std::chrono::steady_clock::duration maxAge = std::chrono::milliseconds{50}) const;

  // -------------------------------------------------------------------------
  // Diagnostics
  // -------------------------------------------------------------------------

  // Reads 0x603F and 0x1001 and renders them with the chapter 7 tables:
  // name, cause, effect and how to recover. This is what belongs in a log
  // when an axis stops, rather than a bare hexadecimal number.
  std::string DescribeLastError();

  // The identity object, 0x1018: vendor, product code, firmware revision and
  // serial number. Four SDO reads; call it once, at start-up, and log
  // signals::Describe() of the result.
  std::error_code GetIdentity(signals::DeviceIdentity & out);

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
  //
  // May be called before CanBus::Start(); the callback is kept and installed
  // when the bus attaches the device.
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

  // Owned by the device, not by Impl, and created in the constructor rather
  // than in Attach(). Neither touches the bus when constructed, and both are
  // needed BEFORE CanBus::Start(): the mechanism in particular has to be set
  // on a device that must be declared before the bus starts. Kept in Impl
  // they did not exist yet, and SetMechanism() dereferenced a null pointer.
  std::unique_ptr<Configurator> configurator_;
  std::unique_ptr<Encoder> encoder_;

  // An emergency callback registered before Start(), handed to Impl in
  // Attach(). See SetEmergencyCallback().
  std::function<void(const signals::EmergencyMessage &)> pendingEmcyCallback_;

  std::unique_ptr<Impl> impl_;
};

}  // namespace epos4
