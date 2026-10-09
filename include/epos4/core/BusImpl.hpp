#pragma once

// Internal. Not part of the public API: it exists so that CanBus.cpp and
// Epos4.cpp can share the Lely objects without leaking Lely headers into
// anything a user of this library includes.

#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "epos4/CanBus.hpp"

namespace epos4
{

// Declaration order is the construction order and, reversed, the destruction
// order. Loop depends on Poll depends on Context; getting this wrong gives a
// loop pointing at a destroyed poller.
struct CanBus::Impl
{
  Options options;

  std::unique_ptr<lely::io::IoGuard> ioGuard;
  std::unique_ptr<lely::io::Context> ctx;
  std::unique_ptr<lely::io::Poll> poll;
  std::unique_ptr<lely::ev::Loop> loop;
  std::unique_ptr<lely::io::Timer> timer;
  std::unique_ptr<lely::io::CanController> ctrl;
  std::unique_ptr<lely::io::CanChannel> chan;
  std::unique_ptr<lely::canopen::AsyncMaster> master;

  // Devices registered before Start(), attached to the master when it runs.
  // Guarded because devices are constructed and destroyed on whichever
  // thread the application likes.
  std::mutex devicesMutex;
  std::vector<Epos4 *> devices;

  std::thread thread;
  std::atomic<bool> running{false};

  // Set by the loop thread when loop->run() returns. Stop() waits on it for
  // the loop to finish by itself, and only forces it after a grace period.
  std::promise<void> loopExited;
  std::future<void> loopExitedFuture;

  // Runs `task` on the loop's thread and waits for it, or runs it right here
  // when no loop is running. Registering a driver with the master and
  // removing one must not race with the loop dispatching frames to it.
  void RunOnLoop(const std::function<void()> & task);
};

}  // namespace epos4
