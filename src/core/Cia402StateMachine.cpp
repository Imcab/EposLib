#include "epos4/core/Cia402StateMachine.hpp"

namespace epos4::core
{

// The cases follow the order of Table 2-5 (p.2-15) so they can be read side
// by side with the manual.
std::optional<State> Decode(std::uint16_t statusword)
{
  switch (static_cast<State>(statusword & signals::status_bits::kStateMask)) {
    case State::kNotReadyToSwitchOn: return State::kNotReadyToSwitchOn;
    case State::kSwitchOnDisabled: return State::kSwitchOnDisabled;
    case State::kReadyToSwitchOn: return State::kReadyToSwitchOn;
    case State::kSwitchedOn: return State::kSwitchedOn;
    case State::kOperationEnabled: return State::kOperationEnabled;
    case State::kQuickStopActive: return State::kQuickStopActive;
    case State::kFaultReactionActive: return State::kFaultReactionActive;
    case State::kFault: return State::kFault;
  }

  // No 'default' on purpose: that way -Wswitch warns if a state is added to
  // the enum and its case is forgotten. Anything that matches nothing lands
  // here.
  return std::nullopt;
}

namespace
{

constexpr std::uint16_t with(std::uint16_t v, std::uint16_t bits)
{
  return static_cast<std::uint16_t>(v | bits);
}

constexpr std::uint16_t without(std::uint16_t v, std::uint16_t bits)
{
  return static_cast<std::uint16_t>(v & ~bits);
}

}  // namespace


// Patterns from Table 2-7 (p.2-16). A bit the manual marks as 'x' is NOT
// touched: that is not "set it to zero", it is "it is not mine".
void Controlword::Apply(Command command)
{
  using namespace signals::control_bits;

  // Bit 7 always goes down first. That way apply(kFaultReset) raises it and
  // the next apply() lowers it again: a 0->1->0 edge for free.
  value_ = without(value_, kFaultReset);

  switch (command) {
    case Command::kShutdown:  // 0xxx x110  (bit 3 is 'x')
      value_ = without(with(value_, kEnableVoltage | kQuickStop), kSwitchOn);
      break;

    case Command::kSwitchOn:  // 0xxx x111  (bit 3 is 'x')
      value_ = with(value_, kSwitchOn | kEnableVoltage | kQuickStop);
      break;

    case Command::kSwitchOnAndEnableOperation:  // 0xxx 1111
    case Command::kEnableOperation:             // 0xxx 1111
      value_ = with(
        value_,
        kSwitchOn | kEnableVoltage | kQuickStop | kEnableOperation);
      break;

    case Command::kDisableVoltage:  // 0xxx xx0x  (bit 1 only)
      value_ = without(value_, kEnableVoltage);
      break;

    case Command::kQuickStop:  // 0xxx x01x
      // kQuickStop low triggers the stop: active-low, 1 is the idle value.
      value_ = without(with(value_, kEnableVoltage), kQuickStop);
      break;

    case Command::kDisableOperation:  // 0xxx 0111
      value_ = without(
        with(value_, kSwitchOn | kEnableVoltage | kQuickStop),
        kEnableOperation);
      break;

    case Command::kFaultReset:  // 0xxx xxxx -> 1xxx xxxx
      value_ = with(value_, kFaultReset);
      break;
  }
}


void Controlword::SetModeBits(std::uint16_t bits)
{
  value_ = with(value_, static_cast<std::uint16_t>(bits & signals::control_bits::kModeBits));
}


void Controlword::ClearModeBits(std::uint16_t bits)
{
  value_ = without(value_, static_cast<std::uint16_t>(bits & signals::control_bits::kModeBits));
}


namespace
{

// Path towards «Operation enabled» (Table 2-6).
Step PlanTowardOperational(State current)
{
  switch (current) {
    case State::kNotReadyToSwitchOn:
      // The drive is still booting. No command speeds that up.
      return {Progress::kInProgress, std::nullopt};

    case State::kSwitchOnDisabled:
      return {Progress::kInProgress, Command::kShutdown};  // transition 2

    case State::kReadyToSwitchOn:
      return {Progress::kInProgress, Command::kSwitchOn};  // transition 3

    case State::kSwitchedOn:
      return {Progress::kInProgress, Command::kEnableOperation};  // transition 4

    case State::kOperationEnabled:
      return {Progress::kReached, std::nullopt};

    case State::kQuickStopActive:
      return {Progress::kInProgress, Command::kEnableOperation};  // transition 16

    case State::kFaultReactionActive:
      // The drive is braking after a failure. Wait: it lands in kFault on its
      // own.
      return {Progress::kInProgress, std::nullopt};

    case State::kFault:
      return {Progress::kBlocked, std::nullopt};
  }

  // Unreachable: decode() only ever yields the eight states above. Should
  // anything else arrive some day, not moving is the safe answer.
  return {Progress::kBlocked, std::nullopt};
}


// Path towards «Switch on disabled» (Table 2-6).
Step PlanTowardDisabled(State current)
{
  switch (current) {
    case State::kNotReadyToSwitchOn:
      // Transition 1: the drive moves to «Switch on disabled» by itself once
      // it finishes booting.
      return {Progress::kInProgress, std::nullopt};

    case State::kSwitchOnDisabled:
      return {Progress::kReached, std::nullopt};

    case State::kReadyToSwitchOn:
      return {Progress::kInProgress, Command::kDisableVoltage};  // transition 7

    case State::kSwitchedOn:
      return {Progress::kInProgress, Command::kDisableVoltage};  // transition 10

    case State::kOperationEnabled:
      // Transition 9. How the motor comes to a stop is not decided here: that
      // is the «Shutdown option code» (0x605B) inside the drive.
      return {Progress::kInProgress, Command::kDisableVoltage};

    case State::kQuickStopActive:
      return {Progress::kInProgress, Command::kDisableVoltage};  // transition 12

    case State::kFaultReactionActive:
      return {Progress::kInProgress, std::nullopt};

    case State::kFault:
      // Deliberate choice: kReached, not kBlocked. The point of kDisabled is
      // that no power reaches the motor, and in «Fault» none does. Were this
      // kBlocked, on_deactivate() could never complete with a faulted axis,
      // which is exactly when being able to shut down matters most.
      return {Progress::kReached, std::nullopt};
  }

  return {Progress::kBlocked, std::nullopt};
}

}  // namespace


Step PlanStep(State current, Goal goal)
{
  switch (goal) {
    case Goal::kOperational:
      return PlanTowardOperational(current);

    case Goal::kDisabled:
      return PlanTowardDisabled(current);
  }

  return {Progress::kBlocked, std::nullopt};
}

}  // namespace epos4::core
