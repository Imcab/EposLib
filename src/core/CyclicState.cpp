#include "epos4/core/CyclicState.hpp"

#include "epos4/core/Cia402StateMachine.hpp"

namespace epos4::core
{

CyclicState::Clock::duration
CyclicState::TimeSinceLastPdo(Clock::time_point now) const
{
  if (!HasReceivedPdo()) {
    return Clock::duration::max();
  }
  const Clock::time_point last{Clock::duration{lastPdo_.load(kRelaxed)}};
  return now - last;
}

void
CyclicState::Activate(
  CyclicMode mode, std::int32_t currentPosition, std::int16_t currentTorque,
  std::uint16_t controlword)
{
  mode_.store(static_cast<std::uint8_t>(mode), kRelaxed);
  target_.store(currentPosition, kRelaxed);
  targetVelocity_.store(0, kRelaxed);
  targetTorque_.store(currentTorque, kRelaxed);
  position_.store(currentPosition, kRelaxed);
  controlword_.store(controlword, kRelaxed);
  // Last, and with release: see IsActive().
  active_.store(true, std::memory_order_release);
}

bool
CyclicState::IsHealthy(Clock::duration maxAge, Clock::time_point now) const
{
  if (!IsActive()) {
    return false;
  }
  if (TimeSinceLastPdo(now) > maxAge) {
    return false;
  }
  const auto state = Decode(Statusword());
  return state && *state == signals::State::kOperationEnabled;
}

}  // namespace epos4::core
