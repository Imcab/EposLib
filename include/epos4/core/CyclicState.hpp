#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

namespace epos4::core
{

// Which setpoint the cyclic path publishes on every SYNC. One per cyclic
// operating mode of CiA 402: Cyclic Synchronous Position (Target position,
// 0x607A), Velocity (Target velocity, 0x60FF) and Torque (Target torque,
// 0x6071).
enum class CyclicMode : std::uint8_t
{
  kPosition,
  kVelocity,
  kTorque,
};

// ---------------------------------------------------------------------------
// The state shared between a control loop and the CANopen thread on the
// cyclic path, and the rules for trusting it.
//
//     control thread                     bus thread
//     ──────────────                     ──────────
//     StageTargetPosition()  ─────────→  OnSync: StagedTargetPosition()
//     Position(), IsHealthy()  ←───────  OnRpdoWrite: Record*()
//
// Every member is a lock-free atomic, so neither side ever waits on the
// other: no mutex, no syscall, no thread hop. That is the whole point of the
// cyclic path, and the static_asserts below make it a compile-time fact
// rather than a hope about the platform.
//
// Kept apart from the device, in epos4_core, because none of this needs a
// bus: the device feeds it from Lely's callbacks, and tests feed it by hand.
// The clock is passed in for the same reason - "unhealthy once PDOs stop" is
// then a test about two time points rather than about sleeping.
// ---------------------------------------------------------------------------
class CyclicState
{
public:
  using Clock = std::chrono::steady_clock;

  // ---- inbound, from the bus thread ------------------------------------

  // Any PDO from the drive, whatever it carried. Its time is what staleness
  // is measured against.
  void RecordPdo(Clock::time_point now)
  {
    lastPdo_.store(now.time_since_epoch().count(), std::memory_order_relaxed);
    pdoReceived_.store(true, std::memory_order_relaxed);
  }

  void RecordStatusword(std::uint16_t value) {statusword_.store(value, kRelaxed);}
  void RecordPosition(std::int32_t value) {position_.store(value, kRelaxed);}
  void RecordVelocity(std::int32_t value) {velocity_.store(value, kRelaxed);}
  void RecordTorque(std::int16_t value) {torque_.store(value, kRelaxed);}

  // ---- the last values received -----------------------------------------

  std::uint16_t Statusword() const {return statusword_.load(kRelaxed);}
  std::int32_t Position() const {return position_.load(kRelaxed);}   // quadcounts
  std::int32_t Velocity() const {return velocity_.load(kRelaxed);}   // rpm
  std::int16_t Torque() const {return torque_.load(kRelaxed);}       // per thousand

  bool HasReceivedPdo() const {return pdoReceived_.load(kRelaxed);}

  // duration::max() if no PDO has ever arrived.
  Clock::duration TimeSinceLastPdo(Clock::time_point now) const;

  // ---- outbound, from the control thread --------------------------------

  // Starts publishing, in the given mode. Every target is seeded so that the
  // first SYNC holds the axis where it is instead of commanding zero:
  //
  //   position  the current position. Zero would be a move to the origin -
  //             on an arm, a swing across its whole range.
  //   velocity  zero, i.e. stand still.
  //   torque    the torque the axis is producing right now. Zero would let
  //             go of whatever it holds: on a vertical axis, the load drops.
  //
  // The position cache is seeded too, so a read before the next PDO does not
  // report zero either.
  void Activate(
    CyclicMode mode, std::int32_t currentPosition, std::int16_t currentTorque,
    std::uint16_t controlword);

  // Cyclic Synchronous Position, the common case.
  void Activate(std::int32_t currentPosition, std::uint16_t controlword)
  {
    Activate(CyclicMode::kPosition, currentPosition, 0, controlword);
  }

  void Deactivate() {active_.store(false, std::memory_order_release);}

  // Acquire, pairing with the release in Activate(): a reader that sees the
  // path active is guaranteed to also see the seeded target and Controlword.
  // With relaxed ordering it could see «active» first and publish a stale
  // zero on the first SYNC, on exactly the weakly ordered CPUs (ARM) a rover
  // is likely to run on.
  bool IsActive() const {return active_.load(std::memory_order_acquire);}

  // Meaningful while IsActive(); written before the release in Activate().
  CyclicMode Mode() const {return static_cast<CyclicMode>(mode_.load(kRelaxed));}

  void StageTargetPosition(std::int32_t quadCounts) {target_.store(quadCounts, kRelaxed);}
  std::int32_t StagedTargetPosition() const {return target_.load(kRelaxed);}

  void StageTargetVelocity(std::int32_t rpm) {targetVelocity_.store(rpm, kRelaxed);}
  std::int32_t StagedTargetVelocity() const {return targetVelocity_.load(kRelaxed);}

  // Thousandths of «Motor rated torque» (0x6076), the unit of 0x6071.
  void StageTargetTorque(std::int16_t perThousand) {targetTorque_.store(perThousand, kRelaxed);}
  std::int16_t StagedTargetTorque() const {return targetTorque_.load(kRelaxed);}

  // The Controlword published on every SYNC while active. Every Controlword
  // write has to land here too, not only in the RPDO: otherwise the next
  // SYNC republishes this copy and a QuickStop() or Halt() issued during
  // cyclic operation lasts a single cycle.
  void SetControlword(std::uint16_t value) {controlword_.store(value, kRelaxed);}
  std::uint16_t Controlword() const {return controlword_.load(kRelaxed);}

  // ---- the rule for trusting the rest -----------------------------------

  // True while the path is active, a PDO arrived within maxAge, and the last
  // Statusword says «Operation enabled».
  //
  // The age check is the one that matters. When the bus goes quiet the
  // cached values stop changing but keep reading back happily, and a
  // controller without it would go on believing a dead axis is tracking.
  bool IsHealthy(Clock::duration maxAge, Clock::time_point now) const;

private:
  static constexpr auto kRelaxed = std::memory_order_relaxed;

  std::atomic<bool> pdoReceived_{false};
  std::atomic<Clock::rep> lastPdo_{0};

  std::atomic<std::uint16_t> statusword_{0};
  std::atomic<std::int32_t> position_{0};
  std::atomic<std::int32_t> velocity_{0};
  std::atomic<std::int16_t> torque_{0};

  std::atomic<bool> active_{false};
  std::atomic<std::int32_t> target_{0};
  std::atomic<std::int32_t> targetVelocity_{0};
  std::atomic<std::int16_t> targetTorque_{0};
  std::atomic<std::uint8_t> mode_{0};
  std::atomic<std::uint16_t> controlword_{0};

  static_assert(std::atomic<bool>::is_always_lock_free);
  static_assert(std::atomic<Clock::rep>::is_always_lock_free);
  static_assert(std::atomic<std::uint16_t>::is_always_lock_free);
  static_assert(std::atomic<std::int32_t>::is_always_lock_free);
  static_assert(std::atomic<std::int16_t>::is_always_lock_free);
  static_assert(std::atomic<std::uint8_t>::is_always_lock_free);
};

}  // namespace epos4::core
