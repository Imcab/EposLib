// Tests for the CiA 402 state machine.
//
// They run with no bus, no Lely and no hardware: everything in this file is
// bit arithmetic over the Statusword and the Controlword. That is precisely
// why the state machine does not depend on CAN.
//
// Manual references (EPOS4 Firmware Specification, ed. 2026-07):
//   Table 2-5   p.2-15   states and their Statusword pattern
//   Table 2-6   p.2-15   state transitions
//   Table 2-7   p.2-16   commands and their Controlword pattern
//   Table 6-147 p.6-215  Controlword bits
//   Table 6-148 p.6-216  Statusword bits

#include <gtest/gtest.h>

#include <cstddef>
#include <optional>

#include "epos4/core/Cia402StateMachine.hpp"

using namespace epos4::core;
using epos4::signals::State;
namespace status_bits = epos4::signals::status_bits;
namespace control_bits = epos4::signals::control_bits;

// ---------------------------------------------------------------------------
// Decode(): Statusword -> State
// ---------------------------------------------------------------------------

TEST(Decode, RecognisesTheEightStatesOfTable25)
{
  EXPECT_EQ(Decode(0x0000), State::kNotReadyToSwitchOn);
  EXPECT_EQ(Decode(0x0040), State::kSwitchOnDisabled);
  EXPECT_EQ(Decode(0x0021), State::kReadyToSwitchOn);
  EXPECT_EQ(Decode(0x0023), State::kSwitchedOn);
  EXPECT_EQ(Decode(0x0027), State::kOperationEnabled);
  EXPECT_EQ(Decode(0x0007), State::kQuickStopActive);
  EXPECT_EQ(Decode(0x000F), State::kFaultReactionActive);
  EXPECT_EQ(Decode(0x0008), State::kFault);
}

// The whole reason kStateMask exists. Table 2-5 marks bit 7 (Warning) and
// bit 4 (Voltage enabled) as 'x': an energised drive with an active warning
// is still in «Operation enabled».
TEST(Decode, IgnoresWarningAndVoltageEnabled)
{
  EXPECT_EQ(Decode(0x0027), State::kOperationEnabled);  // no warning, no voltage
  EXPECT_EQ(Decode(0x0037), State::kOperationEnabled);  // + Voltage enabled (bit 4)
  EXPECT_EQ(Decode(0x00A7), State::kOperationEnabled);  // + Warning (bit 7)
  EXPECT_EQ(Decode(0x00B7), State::kOperationEnabled);  // + both
}

// This is the test that catches the classic bug of comparing with == instead
// of masking: on the bench 0x0027 arrives, with a warm motor 0x00B7 does.
TEST(Decode, SameStateEvenWhenTheIgnoredBitsChange)
{
  EXPECT_EQ(Decode(0x0027), Decode(0x00B7));
  EXPECT_EQ(Decode(0x0023), Decode(0x00B3));
}

// Bits 8..15 (Remote, Internal limit, the mode-specific ones, Home ref) do
// not take part in the state either.
TEST(Decode, IgnoresTheHighByte)
{
  EXPECT_EQ(Decode(0xFF27), State::kOperationEnabled);
  EXPECT_EQ(Decode(0x9427), State::kOperationEnabled);
}

// The contract of the optional: if the pattern is none of the eight legal
// ones, no state gets invented.
TEST(Decode, ReturnsNulloptWhenThePatternIsNotLegal)
{
  EXPECT_FALSE(Decode(0x002F).has_value());
  EXPECT_FALSE(Decode(0x0060).has_value());
  EXPECT_FALSE(Decode(0x004F).has_value());
}

// ---------------------------------------------------------------------------
// Controlword: Command -> bits
// ---------------------------------------------------------------------------

TEST(ControlwordTest, StartsAtZero)
{
  Controlword cw;
  EXPECT_EQ(cw.Raw(), 0x0000);
}

// Each command is compared only against the bits Table 2-7 pins. The ones
// marked 'x' are left out of the check on purpose: the manual says the drive
// does not care about them, and the driver must not touch them.
TEST(ControlwordTest, PatternsOfTable27)
{
  struct Case
  {
    Command cmd;
    std::uint16_t pinned_bits;
    std::uint16_t expected;
  };

  const Case cases[] = {
    {Command::kShutdown, 0x87, 0x06},                    // 0xxx x110
    {Command::kSwitchOn, 0x87, 0x07},                    // 0xxx x111
    {Command::kSwitchOnAndEnableOperation, 0x8F, 0x0F},  // 0xxx 1111
    {Command::kDisableVoltage, 0x82, 0x00},              // 0xxx xx0x
    {Command::kQuickStop, 0x86, 0x02},                   // 0xxx x01x
    {Command::kDisableOperation, 0x8F, 0x07},            // 0xxx 0111
    {Command::kEnableOperation, 0x8F, 0x0F},             // 0xxx 1111
    {Command::kFaultReset, 0x80, 0x80},                  // 1xxx xxxx
  };

  for (const auto & c : cases) {
    Controlword cw;
    cw.Apply(c.cmd);
    EXPECT_EQ(cw.Raw() & c.pinned_bits, c.expected)
      << "command index " << static_cast<int>(c.cmd)
      << " produced 0x" << std::hex << cw.Raw();
  }
}

// «Quick stop» is active low: bit 2 reads 1 when idle and is driven to 0 to
// trigger the stop (Table 2-7: 0xxx x01x).
TEST(ControlwordTest, QuickStopLowersBit2AndKeepsBit1)
{
  Controlword cw;
  cw.Apply(Command::kEnableOperation);
  ASSERT_NE(cw.Raw() & control_bits::kQuickStop, 0);  // idle value is 1

  cw.Apply(Command::kQuickStop);
  EXPECT_EQ(cw.Raw() & control_bits::kQuickStop, 0);
  EXPECT_NE(cw.Raw() & control_bits::kEnableVoltage, 0);
}

// The reason Controlword is stateful: a state machine command must not clear
// the operating-mode bits. If «Shutdown» wiped bit 4 it would break a PPM
// handshake halfway through.
TEST(ControlwordTest, CommandsPreserveTheModeBits)
{
  Controlword cw;
  cw.SetModeBits(1u << 4);  // New setpoint (PPM), handshake in flight

  cw.Apply(Command::kShutdown);
  EXPECT_NE(cw.Raw() & (1u << 4), 0) << "Shutdown cleared bit 4";

  cw.Apply(Command::kEnableOperation);
  EXPECT_NE(cw.Raw() & (1u << 4), 0) << "EnableOperation cleared bit 4";

  cw.Apply(Command::kDisableVoltage);
  EXPECT_NE(cw.Raw() & (1u << 4), 0) << "DisableVoltage cleared bit 4";
}

// And the symmetric property: the mode layer cannot clobber the state
// machine, not even by mistake. setModeBits filters its input with kModeBits.
TEST(ControlwordTest, SetModeBitsCannotClobberTheStateMachine)
{
  Controlword cw;
  cw.Apply(Command::kEnableOperation);
  const std::uint16_t before = cw.Raw() & control_bits::kStateMachineBits;

  cw.SetModeBits(0xFFFF);  // attempt to write everything

  EXPECT_EQ(cw.Raw() & control_bits::kStateMachineBits, before);
}

TEST(ControlwordTest, ClearModeBitsCannotClobberTheStateMachineEither)
{
  Controlword cw;
  cw.Apply(Command::kEnableOperation);
  const std::uint16_t before = cw.Raw() & control_bits::kStateMachineBits;

  cw.ClearModeBits(0xFFFF);

  EXPECT_EQ(cw.Raw() & control_bits::kStateMachineBits, before);
}

// «Fault reset» is edge triggered (0->1) on bit 7. Leaving it permanently
// high would make the next fault impossible to clear, so apply() lowers it at
// the start of every call.
TEST(ControlwordTest, FaultResetProducesACompleteEdge)
{
  Controlword cw;

  cw.Apply(Command::kShutdown);
  EXPECT_EQ(cw.Raw() & control_bits::kFaultReset, 0) << "did not start at 0";

  cw.Apply(Command::kFaultReset);
  EXPECT_NE(cw.Raw() & control_bits::kFaultReset, 0) << "did not rise to 1";

  cw.Apply(Command::kShutdown);
  EXPECT_EQ(cw.Raw() & control_bits::kFaultReset, 0) << "did not fall back";
}

// The edge must not disturb the rest of the Controlword.
TEST(ControlwordTest, FaultResetLeavesTheOtherBitsAlone)
{
  Controlword cw;
  cw.Apply(Command::kEnableOperation);
  cw.SetModeBits(1u << 6);
  const std::uint16_t before = cw.Raw();

  cw.Apply(Command::kFaultReset);

  EXPECT_EQ(cw.Raw(), before | control_bits::kFaultReset);
}

// ---------------------------------------------------------------------------
// PlanStep(): (state, goal) -> command
// ---------------------------------------------------------------------------

namespace
{
struct PlanCase
{
  State current;
  Goal goal;
  Progress progress;
  std::optional<Command> command;
};
}  // namespace

// All sixteen combinations, exhaustively. These are the two tables derived
// from Table 2-6 of the manual.
TEST(PlanStep, AllSixteenCombinations)
{
  const PlanCase cases[] = {
    // --- Goal::kOperational ---
    {State::kNotReadyToSwitchOn, Goal::kOperational, Progress::kInProgress, std::nullopt},
    {State::kSwitchOnDisabled, Goal::kOperational, Progress::kInProgress, Command::kShutdown},
    {State::kReadyToSwitchOn, Goal::kOperational, Progress::kInProgress, Command::kSwitchOn},
    {State::kSwitchedOn, Goal::kOperational, Progress::kInProgress, Command::kEnableOperation},
    {State::kOperationEnabled, Goal::kOperational, Progress::kReached, std::nullopt},
    {State::kQuickStopActive, Goal::kOperational, Progress::kInProgress, Command::kEnableOperation},
    {State::kFaultReactionActive, Goal::kOperational, Progress::kInProgress, std::nullopt},
    {State::kFault, Goal::kOperational, Progress::kBlocked, std::nullopt},

    // --- Goal::kDisabled ---
    {State::kNotReadyToSwitchOn, Goal::kDisabled, Progress::kInProgress, std::nullopt},
    {State::kSwitchOnDisabled, Goal::kDisabled, Progress::kReached, std::nullopt},
    {State::kReadyToSwitchOn, Goal::kDisabled, Progress::kInProgress, Command::kDisableVoltage},
    {State::kSwitchedOn, Goal::kDisabled, Progress::kInProgress, Command::kDisableVoltage},
    {State::kOperationEnabled, Goal::kDisabled, Progress::kInProgress, Command::kDisableVoltage},
    {State::kQuickStopActive, Goal::kDisabled, Progress::kInProgress, Command::kDisableVoltage},
    {State::kFaultReactionActive, Goal::kDisabled, Progress::kInProgress, std::nullopt},
    {State::kFault, Goal::kDisabled, Progress::kReached, std::nullopt},
  };

  for (const auto & c : cases) {
    const Step s = PlanStep(c.current, c.goal);
    EXPECT_EQ(s.progress, c.progress)
      << "state " << static_cast<int>(c.current)
      << " goal " << static_cast<int>(c.goal);
    EXPECT_EQ(s.command, c.command)
      << "state " << static_cast<int>(c.current)
      << " goal " << static_cast<int>(c.goal);
  }
}

// The property the whole design rests on: the sequence is not stored, it
// emerges from asking every cycle. Here the drive is simulated accepting each
// command and reporting the next state.
TEST(PlanStep, ConvergesToOperationEnabledInThreeCycles)
{
  const State reported[] = {
    State::kSwitchOnDisabled,
    State::kReadyToSwitchOn,
    State::kSwitchedOn,
    State::kOperationEnabled,
  };
  const std::optional<Command> expected[] = {
    Command::kShutdown,
    Command::kSwitchOn,
    Command::kEnableOperation,
    std::nullopt,
  };

  for (std::size_t i = 0; i < 4; ++i) {
    const Step s = PlanStep(reported[i], Goal::kOperational);
    EXPECT_EQ(s.command, expected[i]) << "cycle " << i;
  }

  EXPECT_EQ(PlanStep(reported[3], Goal::kOperational).progress, Progress::kReached);
}

// Idempotence: once arrived, calling a thousand times emits nothing and
// changes nothing.
TEST(PlanStep, EmitsNoFurtherCommandsOnceArrived)
{
  for (int i = 0; i < 1000; ++i) {
    const Step s = PlanStep(State::kOperationEnabled, Goal::kOperational);
    ASSERT_EQ(s.progress, Progress::kReached);
    ASSERT_FALSE(s.command.has_value());
  }
}

// The most important safety property in this file: the sequencer NEVER tries
// to leave a fault on its own, however hard it is asked.
TEST(PlanStep, NeverEmitsFaultResetOnItsOwn)
{
  for (int i = 0; i < 1000; ++i) {
    const Step s = PlanStep(State::kFault, Goal::kOperational);
    ASSERT_EQ(s.progress, Progress::kBlocked);
    ASSERT_FALSE(s.command.has_value()) << "emitted a command while faulted";
  }

  const Step reacting = PlanStep(State::kFaultReactionActive, Goal::kOperational);
  EXPECT_FALSE(reacting.command.has_value());
}

// Shutting down must always be able to complete, even (especially) with a
// faulted axis. Were this kBlocked, on_deactivate() would hang.
TEST(PlanStep, DisablingCompletesEvenWithAFaultedAxis)
{
  EXPECT_EQ(PlanStep(State::kFault, Goal::kDisabled).progress, Progress::kReached);
}
