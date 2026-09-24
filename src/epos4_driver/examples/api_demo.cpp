// The epos4 API, end to end.
//
// Run against the simulator:
//   CFG=install/epos4_bringup/share/epos4_bringup/config/epos4_network
//   ./build/epos4_driver/epos4_sim $CFG/epos4.eds 2 vcan0 &
//   ./build/epos4_driver/api_demo $CFG/master.dcf vcan0 2
//
// Against real hardware only the interface name and the DCF change.

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <units.h>

#include "epos4/hardware/Epos4.hpp"

using namespace std::chrono_literals;
using namespace units::literals;  // NOLINT(build/namespaces)

int
main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, 0);

  const std::string dcf = (argc > 1) ? argv[1] : "master.dcf";
  const std::string iface = (argc > 2) ? argv[2] : "vcan0";
  const std::uint8_t nodeId = static_cast<std::uint8_t>((argc > 3) ? std::atoi(argv[3]) : 2);

  // ---- the bus: one per CAN network, shared by every device on it ----
  // Positional: designated initializers are C++20 and this is C++17.
  epos4::CanBus bus{{iface, dcf, 1}};  // interface, master DCF, master node-ID

  // ---- the device, declared BEFORE the bus comes up ----
  // The master boots each slave once, at reset, and only routes a node's PDOs
  // to a driver registered at that moment.
  epos4::Epos4 motor{bus, nodeId};

  bus.Start();
  printf("device on node %u, waiting for it to boot... ", motor.GetNodeId());
  if (!motor.WaitUntilReady()) {
    printf("no answer\n");
    bus.Stop();
    return 1;
  }
  printf("ready\n");

  // PDO is not something this code turns on: the mapping lives in the network
  // description and the master pushes it at boot. All we do is notice.
  for (int i = 0; i < 40 && !motor.IsPdoActive(); ++i) {
    std::this_thread::sleep_for(50ms);
  }
  printf("pdo      : %s\n\n", motor.IsPdoActive() ? "active" : "not mapped (SDO only)");

  // ---- configuration ----
  //
  // Only the fields set here are written. Everything else on the drive is
  // left exactly as it was, so this cannot wipe tuned gains.
  // Tell the driver what the mechanism is, so commands and feedback can be
  // expressed in real units instead of encoder counts.
  // 500 CPR quadrature encoder (2000 counts per motor turn), 1:100 reduction.
  motor.SetMechanism(2000, 1.0 / 100.0);

  epos4::configs::Epos4Configuration config;
  config.motionProfile.profileVelocity = 2000;
  config.motionProfile.profileAcceleration = 10000;
  config.motionProfile.profileDeceleration = 10000;
  config.limits.maxMotorSpeed = 8000;

  if (auto ec = motor.GetConfigurator().Apply(config)) {
    printf("configuration failed: %s\n", ec.message().c_str());
  } else {
    printf("configuration applied\n");
  }

  // ---- status, before enabling ----
  printf(
    "\nstate    : %s\n",
    epos4::signals::ToString(motor.GetState().Refresh().GetValue()));
  printf(
    "mode     : %s\n",
    epos4::signals::ToString(motor.GetOperationMode().Refresh().GetValue()));
  printf("position : %d\n", motor.GetPosition().Refresh().GetValue());
  printf("statusword: 0x%04X\n", motor.GetStatusword().Refresh().GetValue());

  // ---- enable ----
  printf("\nenabling... ");
  if (!motor.Enable()) {
    printf("FAILED (faulted? %s)\n", motor.IsFaulted() ? "yes" : "no");
    if (motor.IsFaulted()) {
      printf("error code 0x%04X\n", motor.GetErrorCode().Refresh().GetValue());
      printf("clearing fault... %s\n", motor.ClearFault() ? "ok" : "still faulted");
    }
    bus.Stop();
    return 1;
  }
  printf(
    "ok, state = %s\n",
    epos4::signals::ToString(motor.GetState().Refresh().GetValue()));

  // ---- move ----
  printf("\ncommanding a profile position move\n");
  // Both forms work. This one is in real units at the joint:
  auto ec = motor.SetControl(
    epos4::controls::ProfilePosition{}
    .WithPosition(units::angle::degree_t{90.0})
    .WithVelocity(15_rpm)
    .WithAcceleration(8000));

  // The equivalent in raw drive units, which is what EPOS Studio and the
  // manual speak:
  //   .WithPosition(50000).WithVelocity(1500u)

  if (ec) {
    printf("move rejected: %s\n", ec.message().c_str());
  } else {
    printf("setpoint accepted (handshake completed)\n");
    printf(
      "mode now : %s\n",
      epos4::signals::ToString(motor.GetOperationMode().Refresh().GetValue()));

    // Wait for the drive to actually start moving before watching for the
    // end of it. Over PDO a status bit is only as fresh as the last SYNC, so
    // reading Target reached immediately after commanding a move can still
    // return the previous move's "yes" and end the wait before it began.
    for (int i = 0; i < 20; ++i) {
      if (!motor.IsTargetReached().Refresh().GetValue()) {break;}
      std::this_thread::sleep_for(20ms);
    }

    // Poll until the drive reports Target reached. On the real thing this is
    // where a caller would do something useful instead of spinning.
    for (int i = 0; i < 60; ++i) {
      if (motor.IsTargetReached().Refresh().GetValue()) {break;}
      printf("  moving... position = %d\n", motor.GetPosition().Refresh().GetValue());
      std::this_thread::sleep_for(100ms);
    }
    printf(
      "target reached, position = %d\n",
      motor.GetPosition().Refresh().GetValue());
  }

  // ---- disable and shut down ----
  // With PDO active this read touches no bus at all: the value arrived on
  // the last SYNC and is already in memory.
  if (motor.IsPdoActive()) {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      motor.GetTimeSinceLastPdo());
    printf("\nlast pdo was %ld ms ago\n", static_cast<long>(age.count()));
  }

  printf("\ndisabling... %s\n", motor.Disable() ? "ok" : "failed");
  bus.Stop();
  return 0;
}
