# Changelog

All notable changes to `eposlib` are recorded here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project uses [semantic versioning](https://semver.org/).

Pre-1.0 means the API may still change between minor versions. It will reach
1.0.0 once it has been verified against a physical EPOS4, with the cyclic path
and a homing run exercised on a real axis.

All object indices, bit layouts and error tables are transcribed from the
**EPOS4 Firmware Specification, edition 2026-07, rel13740** and the **EPOS4
Communication Guide, edition 2026-04, rel13604**.

---

## [Unreleased]

### Added

- **PDO mapping inspection** (read-only, `signals/PdoMapping.hpp`):
  `Epos4::ReadPdoMapping()` reads the four RPDOs and four TPDOs - COB-ID,
  transmission type, inhibit time and mapped objects (6.2.15-6.2.30) - and
  `signals::Describe()` prints them one line per channel.
  `Epos4::CheckPdoMapping()` compares them with what the master configured
  at boot, the concise DCF it holds for the node (0x1F22), and lists every
  difference. A drive whose mapping differs from the master's raises no
  error anywhere else - the master decodes its frames with the wrong layout
  and reads plausible values - so this turns that into an explicit report.
  It also reports channels valid on the drive that the network description
  does not configure. `api_demo` runs the check after boot.
  Verified on vcan0 against `epos4_sim`, including a TPDO remapped behind
  the master's back (Position demand in place of Position actual: same
  size, plausible values, caught).

- **`Refresh()` reads back everything `Apply()` can write.** Every
  configuration group now declares its fields once, as a list of field ->
  object (`configs/ConfigFields.hpp`); `Apply()` writes the set fields from
  that list and `Refresh()` fills every field from it, so a field can no
  longer be written to one object and read from another, or written and
  never read. `Configurator::Refresh(Epos4Configuration&)` used to read a
  dozen fields; it now reads all of them, and every group has its own
  `Refresh()` beside its `Apply()`, as do the encoder groups on `Encoder`.
  A refresh keeps going past an object it cannot read and returns the first
  error; objects only some variants or firmware have
  (`Presence::kHardwareDependent` / `kFirmwareDependent`) are simply left
  unset when absent.
- New configuration groups:
  - `CommunicationConfigs`: producer and consumer heartbeat (0x1016,
    0x1017), error behavior (0x1029), USB/RS232 timeouts and RS232 bit
    rate, CAN bit rate and Node-ID - written last, since they take effect
    only after save and restart - and the SYNC/EMCY COB-IDs, read-only.
  - `ProtectionConfigs`: supply under/overvoltage limits (0x2201), written
    in the order the manual makes always valid, and the power stage
    temperature limit (0x3201:04, not kept by Save()).
  - `CustomPersistentMemoryConfigs`: the four application words of 0x210C.
  - `AnalogInputConfigs` (functions, offset/gain calibration, and the
    current and velocity set-value lines of 0x3170/0x3171, written before
    the functions) and `AnalogOutputConfigs`.
  - `VelocityObserverConfigs` (0x30A3) and `DualLoopConfigs` (0x30AE),
    whose filter coefficients trigger the update bit of 0x30AE:40 after
    them; `Validate()` refuses coefficients without `filterActive`, because
    that same word switches the filter.
- Fields that were missing from existing groups: gear direction (0x3003:04),
  the position I gain unit (0x30A1:09), high-speed digital output 2
  (0x3151:04, Disk 60/8 and 60/12), the halt option (0x605D, firmware
  0x0180+), SSI commutation offset, position bits and additional delay
  (0x3012:0A, :0B, :0E), and, read-only, the encoders' index positions and
  the SSI refresh frequency.
- `Epos4Configuration::Validate()`, run by `Configurator::Apply()` before
  the first write: input mappings (digital and analog) and the dual loop
  filter.
- `MotorConfigs::ratedTorque` reads back «Motor rated torque» (0x6076), the
  value every torque object is a per mille of; read-only, since the drive
  computes it.
- Status signals: `GetMotorI2tPercent()` and `GetPowerStageI2tPercent()`
  (0x3200), `GetPwmDutyCyclePerMille()` (0x3203), `GetStoInputs()` and
  `GetStoCardStatus()` (0x3202).
- `test_config_types` checks the data type and access of every
  configuration field against the maxon EDS in `config/epos4_network`, so a
  field of the wrong width fails CI instead of a drive.

- More feedback: `GetTorqueAveraged()` and `GetVelocityAveraged()`
  (0x30D2, 0x30D3), and per-sensor position and velocity on the encoder
  subsystem - `GetSensorPosition()`, `GetSensorVelocity()`,
  `GetSensorVelocityAveraged()` (0x60E4, 0x60E5).
- Analog I/O: `GetAnalogInputVoltage()`, `GetAnalogInputGeneralPurpose()`,
  `GetAnalogOutputVoltage()` in volts, and `SetAnalogOutput()`, refused
  beyond the +-4 V of section 6.2.87 rather than clamped.
- Capabilities: `GetSupportedDriveModes()` / `SupportsMode()` (0x6502) and
  `GetSupportedHomingMethods()` (0x60E3). `SetControl()` now refuses a mode
  the drive does not implement with `not_supported`, and `Home()` a method
  it does not list, before anything is written. A drive answering 0 or an
  all-zero list is treated as unknown and nothing is refused on it.
- `GetCanBitRate()` (0x200A, the rate in use) and `GetActiveFieldbus()`.
- `DeviceIdentity` also carries the device type (0x1000), the device name
  (0x1008), the full 64-bit serial number (0x2100), the program software
  identification (0x1F56) and the flash status (0x1F57); `Describe()`
  calls out firmware whose flash status is not "valid program available".
- `epos4_sim` fills in the values a drive computes and the EDS leaves
  empty, from the specification's defaults, and models analog I/O and the
  per-sensor feedback.

- `SetPosition(position)`: declares the axis to be somewhere without moving
  it - homing method 37, «Actual position», with the Home offset move
  distance zeroed for the run. The homing method, 0x30B0, 0x30B1 and the
  operating mode are restored afterwards.
- Digital outputs: `SetDigitalOutput(function, active)`,
  `IsOutputActive()`, `GetDigitalOutputs()` (0x60FE, by function) and
  `GetDigitalOutputPins()` (0x3150:01, by pin, after polarity). The
  drive-owned holding brake and Ready/Fault bits are refused.
- Touch probe 1: `controls::TouchProbe` (trigger, edges, single or
  continuous, encoded per Table 6-164 and validated against its notes),
  `ArmTouchProbe()`, `DisarmTouchProbe()`, and `GetTouchProbe()` returning
  status, both latched positions and both edge counters together.
- Statusword signals that were missing: `IsPositionReferenced()` (bit 15),
  `HasHomingError()`, `IsAtZeroSpeed()` (PVM bit 12), `IsFollowingCommand()`
  (CSP/CSV/CST bit 12), `IsRemote()`, `IsVoltageEnabled()`.
- `epos4_sim` implements Homing Mode against a virtual axis - end stops,
  limit switches, a home switch and an index per revolution - covering
  every method of section 3.5.3; plus digital output pins and touch probe
  latching. `Home()` and `SetPosition()` had never been executed before.

- **The lock-free cyclic path for velocity and torque**, next to position:
  `EnterCyclicVelocityMode()` / `EnterCyclicTorqueMode()` and
  `StageTargetVelocity()` / `StageTargetTorque()`. Each SYNC publishes the
  active mode's target (0x607A, 0x60FF or 0x6071). Setpoints are seeded to
  hold the axis: velocity mode starts at zero, torque mode at the torque the
  axis is producing - zero would drop a load on a vertical joint.
  All three `StageTarget*()` take raw units or quantities (`90_deg`,
  `15_rpm`, `0.5_Nm`) and return false rather than guess when a quantity
  cannot be converted. Torque in N m uses the rated torque read once when
  entering the mode, so the loop never does an SDO read. Verified on vcan0:
  1500 rpm gives 50000 counts/s at 2000 counts/rev, and the RPDO carries
  the targets with no SDO traffic while cycling.
- `TorqueSetpoint`: the cyclic torque target and every torque offset
  (0x60B2) take N m as well as thousandths of rated torque; position and
  velocity offsets (0x60B0, 0x60B1) take quantities too.
- `epos4_sim` models Cyclic Synchronous Velocity and Torque, and publishes
  Velocity actual (0x606C) and Torque actual (0x6077) in every mode.

- `test_device_lifecycle`: what a device allows before the bus starts.
- `core::CyclicState` in `epos4_core`: the state the cyclic path shares
  between the control loop and the CANopen thread, and the rule for trusting
  it, moved out of the device so it can be tested without a bus. The clock
  is passed in, so "unhealthy once PDOs stop" is tested between two time
  points instead of by sleeping. Every member is statically asserted to be
  lock-free. `test_cyclic_state` covers it with 15 tests.
- `Epos4::GetCachedErrorCode()` and `GetTimeSinceLastEmergency()`: the last
  EMCY and its age, lock-free. They still answer when the node has gone
  silent, which is when an SDO read of 0x603F cannot.
- **Drives now watch the master's heartbeat.** `epos4_network/bus.yml` sets
  `heartbeat_consumer: true`, so `dcfgen` writes «Consumer heartbeat time»
  (0x1016) into each drive, and the master's heartbeat goes from 1000 ms to
  100 ms, which makes the timeout 300 ms instead of 3 s. When the process
  running the master dies, the drive raises «CAN heartbeat error» (0x8130),
  applies «Abort connection option code» (0x6007) and drops to NMT
  pre-operational. Before this, a drive in CSP kept obeying the last setpoint
  of a master that no longer existed. `epos4_sim` models the same reaction.
  Verified by `SIGKILL`ing the master process mid-operation: EMCY 0x8130
  300 ms after the last heartbeat, then no PDOs and NMT state 0x7F.
- `ClearFault()` recovers from communication faults. For a lost heartbeat
  (0x8130) and CAN passive mode (0x8120) chapter 7 requires an NMT reset
  communication before the fault reset, or the fault reset is ignored.
  `ClearFault()` now sends it, waits until the master has booted the node
  again, then resets the fault; its default timeout grows to 3 s to cover
  that. `signals::RequiresCommunicationReset()` decides, derived from the
  recovery text transcribed from the manual rather than a separate list.
  Verified by freezing the master for 1 s: cleared in about 540 ms, then
  re-enabled with PDOs flowing again.
- `StatusSignal::IsNear(target, tolerance)`, and the free functions
  `signals::RefreshAll(...)` and `signals::IsAllGood(...)`. `RefreshAll`
  refreshes concurrently: each drive's CANopen driver has its own thread, so
  SDO reads to different drives overlap instead of queueing. There is
  deliberately no `WaitForUpdate()`: signals here are pulled, not pushed at a
  configured rate as in Phoenix, and the cyclic data that is pushed already
  has `GetTimeSinceLastPdo()`. `test_status_signal` covers them.
- Power and thermal status signals, in physical units: `GetMotorCurrent()`
  and `GetMotorCurrentAveraged()` (0x30D1, amperes), `GetSupplyVoltage()`
  (0x2200:01, volts), `GetPowerStageTemperature()` and
  `GetPowerStageTemperatureLimit()` (0x3201:01 and :04, degrees Celsius).
  The motion signals stay in counts and rpm, the units a drive is tuned in;
  these have no meaningful raw unit. `ToVoltage()` and `ToTemperature()`
  join the conversions. `api_demo` prints them, and `epos4_sim` reports a
  48 V supply, 35 degC and a current that rises while moving.
- `Epos4::GetIdentity()` and `signals/Identity.hpp`: the identity object
  (0x1018) decoded - hardware from Table 6-66, and the firmware named the way
  maxon names its release files (`EPOS4_0170h_6552h_0000h_0000h`). Worth
  logging once per drive at start-up.

### Changed

- **One package, `eposlib`, at the root of the repository, built with plain
  CMake.** No ROS is needed to build or use it: `cmake -S . -B build` on
  any Linux with Lely CANopen, and it still builds under colcon through its
  package.xml (`build_type` cmake). A sourced ROS environment is searched
  for dependencies (its AMENT_PREFIX_PATH), but none is required.
  - `find_package(eposlib)` gives `eposlib::eposlib` (the device API) and
    `eposlib::core` (the pure CiA 402 logic), plus `eposlib_generate_dcf()`
    so a robot turns its own `bus.yml` into a master DCF, with or without
    ROS. The ament `generate_dcf()` from lely_core_libraries is no longer
    used.
  - Lely is found through pkg-config (`liblely-coapp`); ros2units through
    its header; tests use the system GoogleTest; uncrustify runs as a test
    when `ament_uncrustify` is on the PATH.
  - Libraries are `libeposlib.so` and `libeposlib_core.so`; tools and
    examples install to `lib/eposlib`; the example networks to
    `share/eposlib/config`. The C++ namespace stays `epos4`.
  - `epos4_driver` and `epos4_bringup` are gone as separate packages; their
    content is this one.
- Units come from **ros2units** (https://github.com/Imcab/ros2units), the
  robot's shared units package, instead of the `robot_units` copy that lived
  in this repository. Same nholthaus/units v2.3.5 underneath, so no type
  changes; the include is now `<ros2units/units.h>`. `epos.repos` at the
  workspace root pins it: `vcs import src < epos.repos`.

### Removed

- The `epos4_ros2_control` hardware plugin, the `epos4_interfaces` package and
  the ros2_control bring-up (URDF, xacro macros, controllers, launch file).
  The library is the product; how a robot integrates it with ROS belongs to
  that robot. Nothing in `epos4_driver` depended on them. `epos4_bringup`
  keeps the CANopen network description, which the library needs.

### Fixed

- `config/epos4_network/bus.yml` disables RPDO3 and RPDO4. The drive's
  defaults leave them valid (Controlword with target position / velocity on
  0x400/0x500 + node-ID), and dcfgen only touches the PDOs a network lists,
  so any other device sending on those IDs would have been commanding the
  axis. Found by the new mapping check.
- **Configuration fields of the wrong width or access, each of which aborts
  the SDO on a drive and stops `Apply()` halfway:**
  - «Electrical inductance» (0x3002:02) and «Velocity controller filter
    cut-off frequency» (0x30A2:05) are UNSIGNED16; they were written as 32
    bits.
  - «Main sensor resolution» and «Max system speed» (0x3000:05/06) are
    read-only; `AxisConfigs` and `SensorsConfigs` wrote them. They are now
    read back only.
  - «SSI refresh frequency» (0x3012:07) is read-only UNSIGNED32; it was
    written as 16 bits.
  - «Following error time out» (0x6066) accepts only 0 (6.2.106); it is
    now read back only.
  Found by reading the whole configuration from `epos4_sim` (which enforces
  the real EDS) and writing it back.
- **`Configurator::Apply()` wrote objects the manual only allows in «Power
  Disable»** - axis configuration, pole pairs, gear (except its max input
  speed), SI units and encoders - whatever the drive state, so applying one
  to an enabled drive aborted it partway. Such an Apply now returns
  `operation_not_permitted` before the first write while power is on, as
  `Encoder::Apply()` already did; `RestoreDefaults()` too (6.2.7). The rule
  lives in `configs::RequiresPowerDisabled()` and
  `signals::IsPowerDisabled()`.
- A half-set SinCos resolution (only `periodsPerTurn` or only
  `interpolationBits`) filled the other half with 8 periods; the default of
  0x3011:02 is 0x00080004, which is **2048** periods (Table 6-119). The
  resolution came out 256 times too coarse.
- `DigitalInputConfigs::Validate()` refuses touch probe on high-speed inputs
  1 and 3, which Table 6-131 excludes.
- `AnalogIncrementalEncoderType` and `SsiEncodingType` default to the
  drive's own defaults (with index; Gray code), like
  `IncrementalEncoderType` already did, so a partly filled type means what
  the drive would assume.
- Section numbers cited in comments that pointed at the wrong object.
- `Configurator::Apply(Epos4Configuration)` now validates the digital input
  mapping, as the per-group `Apply()` always did.
- «Electrical resistance» (0x3002:01) is documented in mOhm, not uOhm.

- **`Home()` could report a run attained that had not started, and a
  second PPM move could be taken as accepted without the drive seeing
  it.** With the Controlword in a synchronous RPDO, the drive applies it
  at the SYNC after it arrives and answers on the TPDO of the SYNC after
  that; reading the Statusword before then returned the previous run's
  bits. The handshakes now wait for a PDO after the third SYNC, both after
  raising bit 4 and after lowering it, so every edge reaches the drive.
  Measured on the bus: 20 ms at a 10 ms SYNC.
- `Home()` refused homing when the limit switch was mapped as the
  "without limit error" variant (24, 25), which section 6.2.75 reserves
  for exactly that use. `signals::SatisfiesHomingInput()` accepts both.
- `HomingConfigs::currentThreshold` was `int16_t`; 0x30B2 is UNSIGNED16.
- The cyclic `SetControl()` overloads switched the drive's mode before
  validating the request; a request that could not be converted then
  failed with the mode already changed. Everything is resolved first now.
- **`CanBus` could hang forever on destruction.** `Stop()` submitted the
  deconfigure-and-shutdown and then called `loop->stop()` at once, racing
  it. When the loop stopped first, `ctx->shutdown()` never ran, socket reads
  and timers stayed pending, and `~AsyncMaster` spun in `io_can_net_fini()`
  waiting for operations no loop would ever complete. Found by
  `RefreshAll`, which leaves more SDO transfers in flight: 9 hangs in 12
  runs. `Stop()` now lets the loop return by itself once shutdown has
  cancelled everything, forcing it only after a 2 s grace period: 0 in 24.
- The PDO mapping limitation listed under 0.1.0 does not exist. Lely's
  `BasicSlave` applies the mapping the master downloads at boot. What failed
  was the recipe: `epos4_sim` answers with the maxon identity (vendor `0xFB`),
  but was being run against the `sim_network` DCF, which expects the
  `canopen_fake_slaves` mock (vendor `0x555`). The master aborts the boot with
  `es='D'` before downloading the concise DCF, so the simulator kept its EDS
  default mapping and the master decoded those frames with its own. Run
  `epos4_sim` against `epos4_network`.
- **Profile Position and Profile Velocity never moved while PDOs were
  mapped.** Their targets were written over SDO only, but Target position and
  Target velocity sit in RPDOs with transmission type 1, so the master
  re-sent its own stale copy (0) on the next SYNC and the drive latched that
  on «New setpoint». The handshake still completed, so nothing reported it.
  Every mapped output now goes through the RPDO first, falling back to SDO
  only when the object is not mapped - the rule the Controlword already
  followed.
- `epos4_sim` evaluated the Controlword as soon as it was written, before
  the rest of the same RPDO. Target position follows the Controlword in
  RPDO1, so the simulator latched the previous target. It now defers the
  Controlword until the whole frame has been applied, as a real drive does.
- **`SetMechanism()` before `CanBus::Start()` crashed.** It went through
  the encoder, which was only created when the bus attached the device -
  but devices must be declared, and their mechanism set, before
  `CanBus::Start()`. The configurator and the encoder are now owned by
  `Epos4` and created in its constructor. `api_demo` never hit this
  because it sets the mechanism after starting the bus.

- `epos4_sim` never moved under Cyclic Synchronous Position. Every new
  target re-armed the motion timer, and with a 10 ms SYNC against a 20 ms
  tick the timer was pushed out before it could fire.
- `Epos4::SetEmergencyCallback()` called before `CanBus::Start()` was
  silently dropped. It is now kept and installed when the device attaches.
- `Epos4::GetTimeSinceLastPdo()` dereferenced a null pointer before
  `CanBus::Start()`.
- The cyclic path could publish a stale zero on its first SYNC. The seeded
  target and the «active» flag were both stored relaxed, so on a weakly
  ordered CPU (ARM) the bus thread could see the path active before it saw
  the seed - the very swing to the origin the seed exists to prevent. The
  flag is now stored with release and loaded with acquire.
- `QuickStop()` and `Halt()` issued during cyclic operation lasted one SYNC:
  the Controlword went into the RPDO, and `OnSync` then republished the copy
  taken when the cyclic path started. Every Controlword write now updates
  that copy.
- Reads that prefer PDO trusted any PDO ever received. A drive that drops to
  pre-operational stops sending PDOs but still answers SDO, so its last
  Statusword by PDO kept saying «Operation enabled» while it sat in «Fault»,
  and `ClearFault()` returned true on a faulted axis. Mapped values are now
  used only if a PDO arrived within 100 ms; otherwise the drive is asked.
- `epos4_sim` now tells the two NMT resets apart: reset node restarts
  everything, reset communication leaves the CiA 402 state alone, and a
  fault reset after a lost heartbeat is ignored until a reset communication
  arrives, as on the drive.
- `Epos4::IsPdoActive()` dereferenced a null pointer before
  `CanBus::Start()`.
- `Configurator::Save()` and `RestoreDefaults()` addressed 0x1010 / 0x1011
  with bare hexadecimal instead of the object dictionary constants.

---

## [0.1.0] - 2026-09-24

First working version. A device-oriented API for maxon EPOS4 controllers over
CANopen, built on Lely. The library core carries no ROS dependency.

### Added

**Object dictionary** (`epos4::od`)
- 556 index and sub-index constants generated from the device EDS
  (EPOS4 Module 50/15), split into `comm` (CiA 301), `maxon_comm`, `maxon` and
  `cia402` namespaces. Code written against `cia402` is portable to any CiA 402
  drive; the `maxon` namespaces are not. No other file in the library contains
  a bare hexadecimal object address.

**CiA 402 state machine** (`epos4::core`)
- `Decode()` maps a raw Statusword to one of the eight states of Table 2-5,
  masking the bits the manual marks as *don't care*. Returns `std::nullopt`
  for a pattern that matches none of them rather than inventing a state.
- `Controlword` builds the command words of Table 2-7 as a
  read-modify-write, preserving the operating-mode bits and generating the
  rising edge that Fault reset requires.
- `PlanStep()` answers "what do I send this cycle" from the current state and
  a goal. Stateless: the drive reports where it is, so no progress has to be
  tracked. Never emits a fault reset on its own.

**Device** (`epos4::Epos4`, `epos4::CanBus`)
- `CanBus` owns the Lely stack and runs the event loop on its own thread.
  Devices are declared against a bus and attached when it starts.
- `Enable()`, `Disable()`, `ClearFault()`, `Halt()`, `QuickStop()`,
  `Home()`, `WaitUntilReady()`.
- `SetControl()` for all six operating modes: Profile Position, Profile
  Velocity, Cyclic Synchronous Position / Velocity / Torque, and Homing.
  The PPM setpoint handshake of Table 3-15 is performed internally.
- 18 status signals with caching, timestamps and staleness, covering position,
  velocity, torque, following error, the mode-specific Statusword bits and the
  holding brake state.
- `ReadObject()` / `WriteObject()` for anything the API does not wrap.

**Configuration** (`epos4::configs`)
- Thirteen configuration groups: motor, gear, axis, current / position /
  velocity controllers, motion profile, limits, homing, stop options, holding
  brake, standstill detection and digital outputs.
- Every field is `std::optional`: only what is explicitly set gets written, so
  a partially filled configuration cannot wipe tuned gains.
- `Save()` (0x1010) and `RestoreDefaults()` (0x1011).

**Feedback** (`epos4::Encoder`, via `Epos4::GetEncoder()`)
- All five sensor types: digital incremental 1 and 2, analog SinCos, SSI
  absolute and digital Hall, plus the sensor-slot layout in 0x3000:01.
- Bitfield words are typed rather than raw. The Hall word's layout is the
  mirror of the incremental one (polarity on bit 0, method on bit 4), and a
  test asserts the two encodings differ.
- `Apply()` refuses while the motor is powered, as the manual requires, rather
  than letting the drive abort each write and leaving the configuration half
  applied.
- Unit helpers for the factor of four between encoder pulses and quadcounts.

**Diagnostics** (`epos4::signals`)
- All 73 device error codes of Table 7-186 with the cause, effect and recovery
  from sections 7.2.1 to 7.2.72, including the three ranges (0x1080-0x1088,
  0x5480-0x5483, 0x6180-0x61F0) that a point lookup would miss.
- All 26 SDO abort codes of Table 7-187.
- `IsWarning()` distinguishes the codes the drive keeps running through.
- `ClearsPosition()` flags the six errors whose reset loses the homing
  reference.
- `SetEmergencyCallback()` delivers EMCY frames without polling;
  `GetErrorHistory()` reads 0x1003, which survives a fault reset.

**PDO**
- Mapping is declared in the network description and generated by `dcfgen`
  into both the master's dictionary and the concise configuration pushed to
  the drive, so the two ends cannot drift apart.
- Status reads come from the mapped values with no bus traffic once PDOs are
  flowing; cyclic setpoints ride the next SYNC instead of an SDO round trip.
- `IsPdoActive()` and `GetTimeSinceLastPdo()`.

**Simulator** (`epos4_sim`)
- A CANopen slave that implements the CiA 402 transitions properly and loads
  the real maxon EDS, so the identity check passes and the production
  configuration can be used against it. Written because the mock in
  `canopen_fake_slaves` answers 0x0040 to every Controlword write and never
  advances, which makes the enable sequence untestable.

**Tests**
- 54 tests, none of which need a bus or hardware.

### Known limitations

- **Nothing has run against a physical EPOS4.** Everything is verified against
  the simulator.
- ~~Profile Position motion cannot be verified in simulation: Lely's
  `BasicSlave` does not remap its PDOs at run time.~~ Wrong diagnosis, see
  *Unreleased*.
- Homing and the cyclic modes are implemented but unexercised.
- `Configurator::Refresh()` reads back a subset of the configuration.
- Not yet wrapped: digital and analog inputs, analog outputs, power
  limitation, thermal protection, functional safety, dual-loop position
  control, the velocity observer and touch probe.
- The `epos4_ros2_control` and `epos4_interfaces` packages are empty.
