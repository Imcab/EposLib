# epos4_controller_ws

A C++ library for **maxon EPOS4** positioning controllers over **CANopen**,
built on [Lely CANopen](https://opensource.lely.com/canopen/). Written for the
robotic arm of the Quantum rover (URC), but the library itself knows nothing
about arms, joints or ROS.

```cpp
epos4::CanBus bus{{.interface = "can0", .masterDcf = dcf}};
epos4::Epos4  motor{bus, 2};
bus.Start();
motor.WaitUntilReady();

epos4::configs::Epos4Configuration cfg;
cfg.positionControl.p             = 1500000;
cfg.motionProfile.profileVelocity = 2000;
cfg.limits.maxMotorSpeed          = 8000;
motor.GetConfigurator().Apply(cfg);

motor.Enable();
motor.SetControl(epos4::controls::ProfilePosition{}
                   .WithPosition(50000)
                   .WithVelocity(1500));

while (!motor.IsTargetReached().Refresh().GetValue()) { /* ... */ }
```

> **Status: 0.1.0, pre-release.** Everything below has been verified against the
> included simulator. **Nothing has run against a physical EPOS4 yet.** See
> [Limitations](#limitations).

---

## Contents

- [Why a new library](#why-a-new-library)
- [Packages](#packages)
- [Getting started](#getting-started)
- [Running against the simulator](#running-against-the-simulator)
- [Using the API](#using-the-api)
- [Configuring the network](#configuring-the-network)
- [Diagnosing a fault](#diagnosing-a-fault)
- [Learning the stack](#learning-the-stack)
- [Limitations](#limitations)
- [Reference documents](#reference-documents)

---

## Why a new library

`epos_control`, the Python code this replaces, worked but was shaped around one
arm: node IDs, joint names and motion parameters were baked into it. This is a
library instead — a bus, devices addressed by node-ID, and configuration passed
in by whoever owns the machine.

Two boundaries make that real, and both are enforced by the build rather than
by convention:

```
epos4_core   pure CiA 402 logic. No Lely, no CAN, no ROS.
             Compiles with plain g++. 54 tests, none needing hardware.
                    ^
epos4        the device API. Adds the Lely transport on top.
             Still no ROS.
                    ^
epos4_ros2_control   the only package that knows ROS exists.
```

If `epos4_core` ever stops compiling without Lely, a dependency leaked in.

---

## Packages

| Package | What it holds |
|---|---|
| **`epos4_driver`** | The library, the simulator, the examples and the tests |
| **`epos4_bringup`** | CANopen network description (`bus.yml`), device EDS files, generated DCFs |
| **`epos4_interfaces`** | ROS messages and services — *empty, placeholder* |
| **`epos4_ros2_control`** | `hardware_interface::SystemInterface` plugin — *empty, placeholder* |

Inside `epos4_driver`:

```
include/epos4/
├── CanBus.hpp                one CAN network, shared by every device on it
├── Version.hpp
├── hardware/Epos4.hpp        the device
├── hardware/Encoder.hpp      the feedback subsystem
├── configs/                  typed configuration groups
├── controls/                 one request type per operating mode
├── signals/                  enums, status signals, the error tables
└── core/                     the CiA 402 state machine, the object dictionary

tools/epos4_sim.cpp           a simulated EPOS4 on the bus
examples/                     01..03 build up the Lely stack; api_demo uses the API
```

---

## Getting started

### Dependencies

ROS 2 Humble, plus:

```bash
sudo apt install \
  ros-humble-lely-core-libraries \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  can-utils
```

`ros-humble-lely-core-libraries` brings both the Lely libraries and `dcfgen`,
which generates the master DCF at build time.

### A virtual CAN bus

Everything runs on `vcan0` — a CAN bus in the kernel, with no hardware and no
motors to damage:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### Build

```bash
cd epos_controller_ws
source /opt/ros/humble/setup.bash
colcon build
colcon test --packages-select epos4_driver && colcon test-result --all
```

---

## Running against the simulator

`canopen_fake_slaves` ships a CiA 402 mock, but it answers `0x0040` to every
Controlword write and never changes state, so the enable sequence cannot be
tested against it. `epos4_sim` implements the transitions of Table 2-6
properly and loads the real maxon EDS, so the master's identity check passes
and the production configuration works unchanged.

Two terminals:

```bash
# terminal 1 — the simulated drive on node 2
CFG=install/epos4_bringup/share/epos4_bringup/config/epos4_network
./build/epos4_driver/epos4_sim $CFG/epos4.eds 2 vcan0
```

```bash
# terminal 2 — the API demo
CFG=install/epos4_bringup/share/epos4_bringup/config/epos4_network
./build/epos4_driver/api_demo $CFG/master.dcf vcan0 2
```

```
device on node 2, waiting for it to boot... ready
pdo      : active

configuration applied

state    : Switch on disabled
mode     : None
position : 0
statusword: 0x0440

enabling... ok, state = Operation enabled

commanding a profile position move
setpoint accepted (handshake completed)
```

A third terminal watching the wire is the fastest way to understand what is
happening — and to check the library against your own reading of the manual:

```bash
candump -td vcan0
```

---

## Using the API

### The bus and the devices

```cpp
epos4::CanBus bus{{.interface = "can0", .masterDcf = "/path/to/master.dcf"}};

epos4::Epos4 shoulder{bus, 1};
epos4::Epos4 elbow{bus, 2};

bus.Start();
```

**Devices are declared before `Start()`, not after.** A CANopen master boots
each slave once, at reset, and only routes a node's PDOs to a driver that was
registered at that moment. A device constructed later answers SDO perfectly and
never delivers a single PDO — which looks exactly like a PDO mapping that
failed to take.

### Configuration

Every field is `std::optional`, and only what you set is written. A partially
filled configuration cannot overwrite gains somebody spent a day tuning.

```cpp
epos4::configs::Epos4Configuration cfg;
cfg.motor.nominalCurrent      = 3000;    // mA
cfg.motor.motorType           = epos4::signals::MotorType::kBrushlessSinusoidal;
cfg.positionControl.p         = 1500000;
cfg.limits.minPositionLimit   = -100000;
cfg.limits.maxPositionLimit   =  100000;

motor.GetConfigurator().Apply(cfg);
motor.GetConfigurator().Save();          // 0x1010, or it is lost at power-off
```

### The holding brake

Three objects, and the order matters. The manual: *"The holding brake or the
motor may be damaged if the holding brake will activate before the motor has
reached full standstill."*

```cpp
cfg.standstill.window           = 30;    // what counts as stopped
cfg.standstill.windowTimeMs     = 2;
cfg.holdingBrake.couplingTimeMs = 40;    // from the brake's data sheet
cfg.holdingBrake.openingTimeMs  = 25;
cfg.digitalOutputs.output1 =
    epos4::signals::DigitalOutputFunction::kHoldingBrake;
```

`ToWrites()` emits them in that order, and a test fails if anyone reorders them.

There is deliberately no `ReleaseBrake()`: releasing a brake out of band on a
loaded joint drops the load, and the drive already sequences it against
standstill detection.

### Feedback

```cpp
auto & encoder = motor.GetEncoder();

encoder.Apply(epos4::configs::SensorsConfigs{
  .sensor1 = epos4::configs::Sensor1Type::kDigitalIncrementalEncoder1,
  .sensor3 = epos4::configs::Sensor3Type::kDigitalHallSensor});

epos4::configs::DigitalIncrementalEncoderConfigs enc;
enc.pulsesPerRevolution = 1024;
enc.type = epos4::configs::IncrementalEncoderType{
  .index = epos4::configs::IndexType::kWithIndex};
encoder.Apply(enc);
```

`Apply()` refuses while the motor has power, as the manual requires, instead of
letting the drive abort each write and leaving the configuration half applied.

**Changing the sensor layout clears the absolute position.** An axis that was
homed is no longer referenced afterwards.

Watch the units: `4 x pulses/rev = quadcounts/rev`. An encoder sold as "500 CPR"
is `pulsesPerRevolution = 500` and 2000 counts per turn.
`Encoder::QuadCountsPerRevolution()` exists so nobody has to remember.

### Status signals

A signal caches its value and only touches the bus when you ask it to — an SDO
read is two CAN frames, and a getter that silently did that on every call would
be a bandwidth problem hiding inside an innocent-looking accessor.

```cpp
motor.GetPosition().Refresh().GetValue();
motor.GetState().Refresh().GetValue();
motor.IsTargetReached().Refresh().GetValue();
motor.GetPosition().GetAge();               // how stale is this?
```

Once PDOs are flowing, `Refresh()` reads the value that arrived on the last
SYNC and costs nothing. The calling code does not change.

---

## Configuring the network

`src/epos4_bringup/config/epos4_network/bus.yml` describes what is on the bus.
`dcfgen` turns it, plus each node's EDS, into the master DCF and the concise
configuration the master pushes to each drive at boot.

```yaml
master:
  node_id: 1
  baudrate: 1000
  sync_period: 10000        # microseconds; 0 disables SYNC

node_2:
  node_id: 2
  dcf: "epos4.eds"
  heartbeat_producer: 500

  rpdo:                     # master -> drive
    1:
      cob_id: "auto"
      transmission: 1       # apply on every SYNC
      mapping:
        - {index: 0x6040}   # Controlword
        - {index: 0x607A}   # Target position

  tpdo:                     # drive -> master
    1:
      cob_id: "auto"
      transmission: 1
      mapping:
        - {index: 0x6041}   # Statusword
        - {index: 0x6064}   # Position actual
```

Add an axis by copying a block and changing the node-ID.

**PDO mapping belongs here, not in code.** Both ends have to agree on which
bytes mean what, and this is the one place that writes both sides from a single
description. Remapping at run time over SDO would configure the drive and leave
the master's own dictionary stale.

Every message's COB-ID is *base + node-ID*, so a lower node-ID wins bus
arbitration. Give the low numbers to whatever must not be starved.

There are two networks: `epos4_network` for hardware and `sim_network` for the
`canopen_fake_slaves` mock. They are separate because the master verifies slave
identity, and the mock reports vendor `0x555` where a real maxon drive reports
`0xFB`. That rejection is correct behaviour — it is what stops a miswired axis
from being driven — so simulation gets its own DCF rather than weakening it.

### Getting the EDS

maxon does **not** publish EDS files for download. Export one from **EPOS Studio**
with the controller connected over USB
([how to](https://support.maxongroup.com/hc/en-us/articles/360012778713-EPOS-IDX-Export-of-the-eds-resp-esi-file-Electronic-Data-Sheet)).
That way it matches the exact hardware and firmware.

The EDS committed here is from an **EPOS4 Module 50/15**.

---

## Diagnosing a fault

A bare `0x8611` from a rover 500 m away is a long walk. All 73 device error
codes carry their cause, effect and recovery from chapter 7 of the manual:

```cpp
printf("%s\n", motor.DescribeLastError().c_str());
```

```
0x8611 Following error
  register : motion
  reaction : Fault reaction option code (0x605E)
  cause :
      - Difference between Position demand value and Position actual value
        higher than Following error window
  effect :
      - Fault reaction defined in Fault reaction option code
  recovery :
      - Reset fault with Controlword
```

Two distinctions the table carries that a list of names does not:

```cpp
epos4::signals::IsWarning(0xFF01);       // true: the drive keeps running.
                                         // Treating it as a fault would stop
                                         // an axis that never stopped.

epos4::signals::ClearsPosition(0x7388);  // true: resetting this loses homing.
                                         // Moving to a stored pose afterwards
                                         // goes somewhere else.
```

Faults arrive without polling:

```cpp
motor.SetEmergencyCallback([](const auto & emcy) {
  RCLCPP_ERROR(logger, "%s", epos4::signals::Describe(emcy).c_str());
});
```

The callback runs on the CANopen thread: keep it short and do not call back
into the device from it.

`motor.GetErrorHistory(out)` reads the drive's own history (0x1003), which
survives a fault reset — the only useful question on an intermittent fault.

---

## Learning the stack

`examples/01..03` add one piece of the Lely stack at a time, each printing
something you can check with `candump`:

| | |
|---|---|
| `01_event_loop` | The event loop alone. `post()` queues, it does not execute; `run()` returns when nothing is pending. |
| `02_can_listen` | Plus the CAN socket. With a pending read the loop sleeps at 0% CPU until a frame arrives. |
| `03_master_boot` | Plus the timer, the CANopen master and a driver. Shows `OnBoot`, and reads objects over SDO. |
| `api_demo` | The library. |

Worth knowing before reading them: a CANopen message's identifier is
*function code + node-ID*, so `0x602` is an SDO **to** node 2 and `0x582` is its
reply. Object indices travel little-endian, so `0x6041` goes on the wire as
`41 60`.

---

## Limitations

- **Nothing has run against a physical EPOS4.** Everything is verified against
  `epos4_sim`.
- Profile Position **motion** cannot be verified in simulation: Lely's
  `BasicSlave` accepts the PDO mapping over SDO but does not remap its PDOs at
  run time, so it keeps its EDS defaults. The handshake and the state machine
  are verified; the movement is not.
- Homing and the cyclic modes are implemented but unexercised.
- `Configurator::Refresh()` reads back a subset of the configuration.
- Not yet wrapped: digital and analog inputs, analog outputs, power limitation,
  thermal protection, functional safety, dual-loop position control, the
  velocity observer, touch probe.
- `epos4_ros2_control` and `epos4_interfaces` are empty.

**1.0.0 will mean**: verified against hardware, with the cyclic path and a
homing run exercised on a real axis.

---

## Reference documents

Every object index, bit layout and error code is transcribed from, and cites,
these two. Both are free from
[maxongroup.com](https://www.maxongroup.com/); keep them in `resources/`.

| Document | Edition | What it answers |
|---|---|---|
| **EPOS4 Firmware Specification** | 2026-07, rel13740 | The object dictionary, the CiA 402 state machine, the operating modes, the error tables |
| **EPOS4 Communication Guide** | 2026-04, rel13604 | CANopen framing, the COB-ID allocation scheme, SDO and PDO on the wire |

Also useful: maxon's own
[ROS2 / CANopen / EPOS4 setup guide](https://support.maxongroup.com/hc/en-us/articles/23652318794780-ROS2-CANopen-EPOS4-Setup-Guide).

Comments in the source cite sections and page numbers directly — `Table 2-7
(p.2-16)`, `section 6.2.78` — so a reader can check any decision against the
document that justified it.
