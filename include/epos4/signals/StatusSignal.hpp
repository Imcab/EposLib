#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <future>
#include <system_error>
#include <utility>

namespace epos4::signals
{

// ---------------------------------------------------------------------------
// A single value read from the drive, with the metadata needed to decide
// whether to trust it.
//
// Holding a cached value rather than reading on every call matters here more
// than it does on a local bus: an SDO round trip costs two CAN frames, and at
// 1 Mbit with six axes sharing the wire, a getter that silently talked to the
// device on every call would be a bandwidth problem hiding inside an
// innocent-looking accessor.
//
// Refresh() is explicit for the same reason. Once the cyclic path is mapped to
// PDO the underlying value is updated by the bus and Refresh() becomes free,
// but the calling code does not have to change.
//
// There is deliberately no WaitForUpdate(). In Phoenix a device pushes each
// signal at a configured rate and waiting for the next frame is meaningful;
// here a signal is pulled, and Refresh() already returns a value read at
// that moment. The one thing that is pushed is the cyclic PDO data, and for
// that Epos4::GetTimeSinceLastPdo() and IsCyclicHealthy() answer the real
// question - is it still arriving. A WaitForUpdate() that merely retried
// would promise something it could not deliver.
//
// Not thread safe per signal: refresh a given signal from one thread at a
// time. Different signals may be refreshed concurrently; see RefreshAll().
// ---------------------------------------------------------------------------
template<typename T>
class StatusSignal
{
public:
  using Refresher = std::function<std::error_code(T &)>;

  StatusSignal() = default;

  explicit StatusSignal(Refresher refresher)
  : refresher_(std::move(refresher)) {}

  // The last known value. Does not touch the bus.
  T GetValue() const {return value_;}

  // Re-read from the drive. Returns *this so calls can be chained.
  StatusSignal & Refresh()
  {
    if (refresher_) {
      T fresh{};
      status_ = refresher_(fresh);
      if (!status_) {
        value_ = fresh;
        timestamp_ = std::chrono::steady_clock::now();
      }
    }
    return *this;
  }

  // Error from the most recent Refresh(). Falsy when the value is good.
  std::error_code GetStatus() const {return status_;}

  // When the value was last successfully updated.
  std::chrono::steady_clock::time_point GetTimestamp() const {return timestamp_;}

  // How stale the value is. Worth checking before acting on a position.
  std::chrono::steady_clock::duration GetAge() const
  {
    return std::chrono::steady_clock::now() - timestamp_;
  }

  bool HasValue() const {return timestamp_.time_since_epoch().count() != 0;}

  // Whether the last known value is within tolerance of target, inclusive.
  //
  // Compares the cached value: it does not refresh, and it does not look at
  // GetStatus(). Where a failed read must not count as "near", check that
  // too - or use IsAllGood() on the way in.
  //
  //   if (motor.GetPosition().Refresh().IsNear(target, 50)) { ... }
  //
  // Written without abs() so it also works for unsigned values, where
  // value - target would wrap, and for unit types.
  bool IsNear(const T & target, const T & tolerance) const
  {
    const T difference = value_ < target ?
      static_cast<T>(target - value_) :
      static_cast<T>(value_ - target);
    return !(tolerance < difference);
  }

  // Convenience: refresh and hand back the value in one expression.
  T GetValueRefreshed() {return Refresh().GetValue();}

private:
  T value_{};
  std::error_code status_{};
  std::chrono::steady_clock::time_point timestamp_{};
  Refresher refresher_{};
};


// Refreshes every signal given, concurrently, and waits for all of them.
// Returns the first error in argument order, or success.
//
// The point is several drives. Each device's CANopen driver runs on its own
// thread, so SDO reads to different nodes proceed in parallel: the supply
// voltage of six axes costs about one round trip instead of six. Signals of
// the same device still queue on that device's SDO channel, which is correct
// - one SDO transfer per node at a time is how CANopen works.
//
//   signals::RefreshAll(shoulder.GetPowerStageTemperature(),
//                       elbow.GetPowerStageTemperature(),
//                       wrist.GetPowerStageTemperature());
//
// Each call starts a thread per signal. That is nothing at the few hertz
// diagnostics are polled at, and far too much for a control loop - which
// has the lock-free cyclic accessors of Epos4 for that.
//
// Pass each signal once: two threads refreshing the same signal race on it.
template<typename ... Signals>
std::error_code RefreshAll(Signals & ... signals)
{
  static_assert(sizeof...(Signals) > 0, "RefreshAll needs at least one signal");

  std::array<std::future<void>, sizeof...(Signals)> pending{
    std::async(std::launch::async, [&signals]() {signals.Refresh();}) ...};
  for (auto & refresh : pending) {
    refresh.get();
  }

  std::error_code first{};
  ((first = first ? first : signals.GetStatus()), ...);
  return first;
}

// True when every signal holds a value and its last refresh succeeded.
template<typename ... Signals>
bool IsAllGood(const Signals & ... signals)
{
  // Through a variable, not `return (... && ...);`: a fold expression needs
  // its outer parentheses, and ament_uncrustify removes parentheses around a
  // returned expression - which turns this into a syntax error.
  const bool good = ((signals.HasValue() && !signals.GetStatus()) && ...);
  return good;
}

}  // namespace epos4::signals
