// PIECE 1 - The event loop, no CAN yet.
//
// Build:  colcon build --packages-select epos4_driver
// Run:    ./build/epos4_driver/01_event_loop
//
// What it shows: exec.post() does not execute anything, it only QUEUES.
// The loop decides when. And loop.run() returns as soon as nothing is
// pending, which is why this program exits in ~3 ms instead of hanging.

#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>

#include <cstdio>

using namespace lely;

int main()
{
  setvbuf(stdout, nullptr, _IONBF, 0);

  // (1) Turns the Lely I/O subsystem on. Turns itself off when main returns.
  io::IoGuard io_guard;

  // (2) The registry of things to notify on shutdown.
  io::Context ctx;

  // (3) The epoll. This is what actually sleeps waiting for events.
  io::Poll poll(ctx);

  // (4) The loop, fed by that epoll.
  ev::Loop loop(poll.get_poll());

  // (5) The executor: the queue the loop pulls work from.
  auto exec = loop.get_executor();

  // NOTE on declaration order: Context -> Poll -> Loop -> Executor.
  // C++ destroys locals in reverse order, so Loop dies before Poll and Poll
  // before Context. Declaring them the other way round would leave a Loop
  // pointing at a destroyed Poll. The order is mandatory, not style.

  printf("before run()\n");

  exec.post([]() {printf("  task A executed by the loop\n");});
  exec.post([]() {printf("  task B executed by the loop\n");});

  // (6) Process everything pending. Returns once nothing is left to do.
  loop.run();

  printf("after run()\n");
  return 0;
}
