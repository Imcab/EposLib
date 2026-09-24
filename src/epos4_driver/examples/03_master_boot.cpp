// PIECE 3 - The CANopen master and our first driver.
//
// Build:  colcon build --packages-select epos4_driver
// Run:    ros2 launch canopen_fake_slaves cia402_slave.launch.py node_id:=2
//         ./build/epos4_driver/03_master_boot
//
// Adds three objects to pieces 1-2: the Timer, the AsyncMaster and a Driver.
// After this, run() never returns on its own: the master always has pending
// promises (heartbeats to watch, SDO timeouts to enforce).

#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/clock.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <cstdio>
#include <filesystem>
#include <string>
#include <unistd.h>

using namespace lely;

// ---------------------------------------------------------------------------
// Our driver. One instance per axis, bound to one node-ID.
//
// FiberDriver runs our code as a stackful coroutine on the master's thread.
// Wait() suspends the coroutine and hands control back to the loop, so the
// code below reads top-to-bottom while never blocking the event loop.
// ---------------------------------------------------------------------------
class AxisDriver : public canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;

private:
  // Called once the master has finished booting this slave: it detected the
  // node, checked its identity against the DCF and applied the configuration.
  //
  // 'es' is the error status character: '\0' means success. Anything else and
  // NOTHING will work afterwards, silently. This is always the first thing to
  // instrument.
  void
  OnBoot(canopen::NmtState /*st*/, char es, const std::string & what) noexcept override
  {
    // CiA 302-2 error status characters. Not all of them are fatal:
    //   'L' = the slave was already Operational when we booted it. Advisory:
    //         configuration still went through and SDOs work.
    //   'J' = configuration download failed. THIS one is fatal and silent -
    //         identity reads succeed and then nothing works.
    if (es == 0) {
      printf("[boot] node %d OK\n", id());
    } else if (es == 'L') {
      printf("[boot] node %d OK (advisory es='L': %s)\n", id(), what.c_str());
    } else {
      printf("[boot] node %d FAILED es='%c': %s\n", id(), es, what.c_str());
    }
  }

  // NMT state changes of the remote node. This is the layer that says
  // whether the node TALKS, not whether the motor has power - that is the
  // CiA 402 state machine in cia402_state_machine.hpp, a separate thing.
  void
  OnState(canopen::NmtState st) noexcept override
  {
    printf("[nmt ] node %d -> 0x%02X\n", id(), static_cast<int>(st));
  }

  // Fires when the slave's heartbeat stops arriving (true) or resumes
  // (false). This is the watchdog: cut the CAN cable and it fires.
  void
  OnHeartbeat(bool occurred) noexcept override
  {
    printf("[hb  ] node %d heartbeat %s\n", id(), occurred ? "LOST" : "back");
  }

  // The drive shouting that something broke, unsolicited. Chapter 7 of the
  // firmware spec lists every code that can show up here.
  void
  OnEmcy(uint16_t eec, uint8_t er, uint8_t /*msef*/[5]) noexcept override
  {
    printf("[emcy] node %d error 0x%04X register 0x%02X\n", id(), eec, er);
  }

  // Called after the slave reaches NMT 'Operational'. This runs inside the
  // fiber, which is why we can use the blocking-looking Wait() here.
  void
  OnConfig(std::function<void(std::error_code ec)> res) noexcept override
  {
    try {
      // An SDO upload, exactly like the 602/582 exchange we sent by hand with
      // cansend. Lely handles the timeout, the retries and the segmented
      // transfer for names that do not fit in four bytes.
      auto device_name = Wait(AsyncRead<std::string>(0x1008, 0x00));
      printf("[sdo ] 0x1008 device name   = \"%s\"\n", device_name.c_str());

      auto statusword = Wait(AsyncRead<uint16_t>(0x6041, 0x00));
      printf("[sdo ] 0x6041 statusword    = 0x%04X\n", statusword);

      auto mode = Wait(AsyncRead<int8_t>(0x6061, 0x00));
      printf("[sdo ] 0x6061 mode display  = %d\n", mode);

      res({});
    } catch (canopen::SdoError & e) {
      // This is the abort code we triggered by accident earlier, now typed.
      printf("[sdo ] ABORT: %s\n", e.what());
      res(e.code());
    }
  }
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, 0);

  // --- pieces 1 and 2, unchanged ---
  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();

  // (9) The clock. SDO timeouts, heartbeat deadlines and SYNC all come from
  //     here. Without it the master could not tell "slow" from "dead".
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);

  io::CanController ctrl("vcan0");
  io::CanChannel chan(poll, exec);
  chan.open(ctrl);

  // (10) The master, built from the DCF that dcfgen produced. Node-ID 1,
  //      matching bus.yml.
  //
  // GOTCHA: master.dcf refers to the per-node concise config as
  //   [1F22sub2] UploadFile=axis_1.bin
  // a RELATIVE path, which Lely resolves against the process working
  // directory, not against the DCF. Run from anywhere else and the boot
  // fails with es='J' "Configuration download failed" - the identity reads
  // succeed, then nothing happens. So: chdir into the config directory.
  const std::filesystem::path config_dir =
    (argc > 1) ?
    std::filesystem::path(argv[1]) :
    std::filesystem::path(
    "install/epos4_bringup/share/epos4_bringup/config/epos4_network");

  if (chdir(config_dir.c_str()) != 0) {
    printf("cannot enter %s\n", config_dir.c_str());
    return 1;
  }

  canopen::AsyncMaster master(timer, chan, "master.dcf", "", 1);

  // (11) Our driver, bound to node 2 (axis_1 in bus.yml).
  AxisDriver axis(master, 2);

  // Ctrl-C handling, so the master shuts the bus down cleanly instead of
  // leaving the slaves powered and orphaned.
  io::SignalSet sigset(poll, exec);
  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);
  sigset.submit_wait(
    [&](int /*signo*/) {
      sigset.clear();
      master.AsyncDeconfig().submit(exec, [&]() {ctx.shutdown();});
    });

  // (12) Start NMT: the master resets the network and boots every slave.
  master.Reset();

  printf("master running, waiting for node 2...\n");
  loop.run();
  printf("stopped\n");
  return 0;
}
