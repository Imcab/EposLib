// PIECE 2 - Listening to the bus. Same as piece 1 plus the CAN socket.
//
// Build:  colcon build --packages-select eposlib
// Run:    ./build/eposlib/02_can_listen
//
// Try it:
//   terminal 1:  ./build/eposlib/02_can_listen
//   terminal 2:  cansend vcan0 583#4F61600006000000
//
// What it shows:
//   - With a pending read the loop sleeps instead of returning (0% CPU).
//   - submit_read() is ONE promise: after the first frame the loop has
//     nothing left pending and the program exits. A program that listens
//     forever has to re-submit inside the callback.

#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>

#include <cstdio>
#include <system_error>

using namespace lely;

int main()
{
  setvbuf(stdout, nullptr, _IONBF, 0);

  // --- identical to piece 1 ---
  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();

  // --- new here ---

  // (6) The INTERFACE, i.e. "vcan0" in `ip link`. The hardware side: its
  //     state, its bitrate, whether it went bus-off.
  io::CanController ctrl("vcan0");

  // (7) The SOCKET: our window onto that interface. Several sockets can be
  //     opened on one interface, which is why they are separate objects.
  io::CanChannel chan(poll, exec);

  // (8) Connect the socket to the interface.
  chan.open(ctrl);

  printf("socket open on vcan0\n");

  // Buffer where Lely will leave the received frame.
  static can_msg msg = CAN_MSG_INIT;

  // THE PENDING PROMISE: "tell me when a frame arrives".
  // This does NOT read anything now. It registers the request and returns.
  chan.submit_read(
    &msg, nullptr, nullptr,
    [](int result, ::std::error_code ec) {
      if (ec) {
        printf("error: %s\n", ec.message().c_str());
      }
      if (result == 1) {
        printf("FRAME:  id=0x%03X  len=%u  data=", msg.id, msg.len);
        for (unsigned i = 0; i < msg.len; ++i) {
          printf("%02X ", msg.data[i]);
        }
        printf("\n");
      }
      // TODO (yours): re-submit the read here to keep listening forever.
    });

  printf("before run()  <- if it returns now, the promise did not count\n");

  loop.run();

  printf("after run()\n");
  return 0;
}
