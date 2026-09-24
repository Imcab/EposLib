#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace epos4
{

class Epos4;

// ---------------------------------------------------------------------------
// One CAN network, and everything Lely needs to speak CANopen on it: the I/O
// context, the poller, the event loop, the timer, the socket and the CANopen
// master. The loop runs on its own thread so that device calls made from
// application threads do not have to own it.
//
// This is deliberately not per-axis. A bus carries up to 127 nodes; devices
// are constructed against it with their node-ID, the same way a TalonFX is
// constructed against a named CAN bus.
//
//   epos4::CanBus bus{{.interface = "can0", .masterDcf = ".../master.dcf"}};
//   epos4::Epos4 shoulder{bus, 1};
//   epos4::Epos4 elbow{bus, 2};
//   bus.Start();
//
// Devices are declared BEFORE Start(), not after. A CANopen master boots each
// slave once, at reset, and only routes a node's PDOs to a driver that was
// registered at that moment. Constructing a device after the network came up
// gives a node that answers SDO perfectly and never delivers a single PDO -
// which looks exactly like a PDO mapping that failed to take.
// ---------------------------------------------------------------------------
class CanBus
{
public:
  struct Options
  {
    // SocketCAN interface: "can0", "vcan0", ...
    std::string interface{"can0"};

    // Master DCF produced by dcfgen from the network description plus each
    // node's EDS. The master needs it to know what is supposed to be out
    // there; identity mismatches are rejected at boot, which is what stops a
    // miswired axis from being driven.
    //
    // NOTE: the DCF references each node's concise config (axis_N.bin) with a
    // path relative to the process working directory. CanBus resolves it
    // against the DCF's own directory so callers do not have to chdir.
    std::string masterDcf{};

    // Node-ID of the master itself, matching the DCF.
    std::uint8_t masterNodeId{1};
  };

  explicit CanBus(Options options);
  ~CanBus();

  CanBus(const CanBus &) = delete;
  CanBus & operator=(const CanBus &) = delete;

  // Opens the socket, attaches every registered device, boots the network and
  // starts the event loop thread. Devices constructed after this call will
  // work over SDO but receive no PDOs.
  //
  // Throws std::system_error if the interface or the DCF cannot be opened.
  void Start();

  // Stops the loop and shuts the network down cleanly, leaving slaves in a
  // safe state rather than powered and orphaned.
  void Stop();

  bool IsRunning() const;

  const Options & GetOptions() const;

private:
  friend class Epos4;

  // Called by Epos4's constructor. The device is only attached to the master
  // when Start() runs, because the master does not exist until then.
  void Register(Epos4 * device);

  struct Impl;
  std::unique_ptr<Impl> impl_;

  Impl & GetImpl();
};

}  // namespace epos4
