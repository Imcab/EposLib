# Changelog

All notable changes to `epos4_driver` are recorded here.
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

- `epos4_bringup`: `epos4.ros2_control.xacro` (`epos4_system` and
  `epos4_joint` macros), a minimal one-axis `epos4_arm.urdf.xacro`,
  `controllers.yaml` and `launch/arm.launch.py`. Verified end to end against
  `epos4_sim` on `vcan0`: `joint_trajectory_controller` drives a 0.5 rad
  trajectory, the RPDO carries a new Target position on every SYNC, and the
  axis tracks within about 3 mrad.
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
  Verified by `SIGKILL`ing `ros2_control_node` mid-operation: EMCY 0x8130
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
- `Epos4System::on_error`: when `read()` reports a dead axis, stop the cyclic
  path and command every axis to disable. The bus is kept up so the master
  keeps sending «Disable voltage» on every SYNC; an axis that reconnects is
  told to drop power, not to resume. Verified by killing `epos4_sim` with
  the controller active: detected in about 64 ms, logged once, and every
  RPDO after that carries 0x000D.

### Fixed

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
- **The ros2_control plugin crashed in `on_configure`.** `SetMechanism()`
  went through the encoder, which was only created when the bus attached
  the device - but devices must be declared, and their mechanism set,
  BEFORE `CanBus::Start()`. The configurator and the encoder are now owned
  by `Epos4` and created in its constructor. `api_demo` never hit this
  because it sets the mechanism after starting the bus.
- `epos4_sim` never moved under Cyclic Synchronous Position. Every new
  target re-armed the motion timer, and with a 10 ms SYNC against a 20 ms
  tick the timer was pushed out before it could fire.
- `Epos4System::read()` described a dead axis with `DescribeLastError()`,
  two SDO reads that each wait out the SDO timeout against a silent node -
  blocking the control loop of every other axis. It now reports the PDO
  age, the last cached state and the last EMCY with its age, all from
  memory.
- `Epos4System` destroyed its bus before its devices when the process
  exited without `on_cleanup` (Ctrl-C while active), because members are
  destroyed in reverse order of declaration. A destructor now releases them
  in the right order, and `on_configure` releases whatever `on_error` left.
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
