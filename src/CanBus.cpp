#include "epos4/CanBus.hpp"

#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/clock.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <stdexcept>
#include <thread>

#include "epos4/core/BusImpl.hpp"
#include "epos4/hardware/Epos4.hpp"

namespace epos4
{

namespace detail
{

// Copies the master DCF next to the original with every UploadFile= entry
// turned into an absolute path, and returns the copy's path. Writing the copy
// into the same directory keeps everything the DCF references reachable.
std::string
RewriteDcfPaths(const std::filesystem::path & dcf, const std::filesystem::path & dir)
{
  std::ifstream in{dcf};
  if (!in) {
    throw std::system_error(
            errno, std::generic_category(), "CanBus: cannot read " + dcf.string());
  }

  const auto out_path = dir / (dcf.stem().string() + ".resolved.dcf");
  std::ofstream out{out_path};
  if (!out) {
    // Not writable: fall back to the original and hope the caller's working
    // directory happens to be right.
    return dcf.string();
  }

  const std::string key = "UploadFile=";
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind(key, 0) == 0) {
      const std::string value = line.substr(key.size());
      if (!value.empty() && value.front() != '/') {
        line = key + (dir / value).string();
      }
    }
    out << line << '\n';
  }
  return out_path.string();
}

}  // namespace detail

CanBus::CanBus(Options options)
: impl_(std::make_unique<Impl>())
{
  impl_->options = std::move(options);
}

CanBus::~CanBus()
{
  Stop();
}

const CanBus::Options &
CanBus::GetOptions() const
{
  return impl_->options;
}

bool
CanBus::IsRunning() const
{
  return impl_->running.load();
}

CanBus::Impl &
CanBus::GetImpl()
{
  return *impl_;
}

void
CanBus::Register(Epos4 * device)
{
  impl_->devices.push_back(device);
  // If the bus is already up, attach immediately. The device will talk SDO
  // but will not receive PDOs until the next Start().
  if (impl_->running.load() && impl_->master) {
    device->Attach(*impl_->master);
  }
}

void
CanBus::Start()
{
  if (impl_->running.load()) {
    return;
  }
  if (impl_->options.masterDcf.empty()) {
    throw std::invalid_argument("CanBus: masterDcf is required");
  }

  auto & d = *impl_;

  d.ioGuard = std::make_unique<lely::io::IoGuard>();
  d.ctx = std::make_unique<lely::io::Context>();
  d.poll = std::make_unique<lely::io::Poll>(*d.ctx);
  d.loop = std::make_unique<lely::ev::Loop>(d.poll->get_poll());
  d.timer = std::make_unique<lely::io::Timer>(
    *d.poll, d.loop->get_executor(), CLOCK_MONOTONIC);
  d.ctrl = std::make_unique<lely::io::CanController>(d.options.interface.c_str());
  d.chan = std::make_unique<lely::io::CanChannel>(*d.poll, d.loop->get_executor());
  d.chan->open(*d.ctrl);

  // The master DCF names each node's concise configuration with a path
  // relative to the process working directory:
  //
  //     [1F22sub2]
  //     UploadFile=node_2.bin
  //
  // Lely resolves that path when it BOOTS the node, not when it loads the
  // DCF. A library cannot chdir for the lifetime of the process and cannot
  // chdir back either, because by then the boot has not happened yet; the
  // symptom is a node that answers SDO perfectly, never delivers a PDO, and
  // reports es='J' Configuration download failed in a callback nobody read.
  //
  // So: rewrite those paths to absolute ones in a copy, and hand Lely the
  // copy. The process working directory is left alone.
  const std::filesystem::path dcf{d.options.masterDcf};
  const auto dir = std::filesystem::absolute(dcf).parent_path();
  const std::string resolvedDcf = detail::RewriteDcfPaths(dcf, dir);

  d.master = std::make_unique<lely::canopen::AsyncMaster>(
    *d.timer, *d.chan, resolvedDcf, "", d.options.masterNodeId);

  // Attach every registered device BEFORE resetting: the master routes a
  // node's PDOs to the driver that was registered when it booted.
  for (auto * device : d.devices) {
    device->Attach(*d.master);
  }

  d.running.store(true);
  d.master->Reset();

  d.loopExited = std::promise<void>{};
  d.loopExitedFuture = d.loopExited.get_future();
  d.thread = std::thread(
    [&d]() {
      d.loop->run();
      d.running.store(false);
      d.loopExited.set_value();
    });
}

void
CanBus::Stop()
{
  auto & d = *impl_;
  if (!d.running.load() && !d.thread.joinable()) {
    return;
  }

  // Deconfigure, then shut the I/O context down, and let the loop RETURN BY
  // ITSELF: ctx->shutdown() cancels every pending socket read and timer, and
  // once nothing is outstanding loop->run() comes back. That is the shutdown
  // Lely's own examples use.
  //
  // This used to call loop->stop() straight after submitting the deconfig,
  // which races with it. When the loop lost - more often the more SDO
  // transfers were in flight, e.g. after signals::RefreshAll() - shutdown
  // never ran, operations stayed pending with no loop left to complete them,
  // and ~AsyncMaster spun forever in io_can_net_fini(). A hang at exit, in
  // 9 runs out of 12 in the test that found it.
  //
  // Posted rather than called here: everything touching the master belongs
  // on the loop's thread.
  if (d.loop && d.thread.joinable()) {
    d.loop->get_executor().post(
      [&d]() {
        if (!d.master) {
          d.ctx->shutdown();
          return;
        }
        try {
          d.master->AsyncDeconfig().submit(
            d.loop->get_executor(), [&d]() {d.ctx->shutdown();});
        } catch (...) {
          d.ctx->shutdown();
        }
      });

    // Bounded, so a deconfig that never completes cannot turn this into a
    // different hang. Forcing is what the old code always did; now it is the
    // exception, and it still cancels the I/O first.
    constexpr std::chrono::seconds kShutdownGrace{2};
    if (d.loopExitedFuture.valid() &&
      d.loopExitedFuture.wait_for(kShutdownGrace) != std::future_status::ready)
    {
      d.ctx->shutdown();
      d.loop->stop();
    }
  }
  if (d.thread.joinable()) {
    d.thread.join();
  }
  d.running.store(false);

  // Deliberately does NOT destroy the master here. Devices hold a reference to
  // it for as long as they exist, and tearing it down under a live Epos4 is a
  // use-after-free that shows up as a bus error at shutdown. Teardown happens
  // in ~Impl, which runs after every device that was declared inside the bus's
  // scope has already gone.
  //
  // Ownership rule for callers: an Epos4 must not outlive its CanBus. Declare
  // devices after the bus and C++ destruction order takes care of it.
}

}  // namespace epos4
