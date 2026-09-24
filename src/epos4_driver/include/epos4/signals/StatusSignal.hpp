#pragma once

#include <chrono>
#include <functional>
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

  // Convenience: refresh and hand back the value in one expression.
  T GetValueRefreshed() {return Refresh().GetValue();}

private:
  T value_{};
  std::error_code status_{};
  std::chrono::steady_clock::time_point timestamp_{};
  Refresher refresher_{};
};

}  // namespace epos4::signals
