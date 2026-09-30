#pragma once

#include <cstdint>
#include <optional>

#include "epos4/signals/Enums.hpp"

namespace epos4::core
{

// Section 2.2.1 - State of the drive
//
// The drive state is reported through the Statusword, a 16-bit unsigned
// integer. Example:
//
//   State: Not ready to switch on -> xxxx xxxx x00x 0000
//
// How do we know that? The manual writes the word in groups of four bits, and
// every bit has a defined meaning. See section 6.2.95 Statusword:
//
// Bit  |   Description
// 15   |   Position referenced to home position
// 14   |   reserved(0)
// 13   |   Operating mode specific
// 12   |   Operating mode specific
// 11   |   Internal limit active
// 10   |   Operating mode specific
// 9    |   Remote
// 8    |   reserved(0)
// 7    |   Warning
// 6    |   Switch on disabled
// 5    |   Quick stop
// 4    |   Voltage enabled
// 3    |   Fault
// 2    |   Operation enabled
// 1    |   Switched on
// 0    |   Ready to switch on
//
// The groups run from the most significant bit to the least significant one:
//
/**

  xxxx   xxxx   x01x   0111        <- "Operation enabled"
  ||||   ||||   ||||   ||||
  15..12 11..8  7..4   3..0        <- bit number

Each of the eight low bits has its own name:

 bit 7  6  5  4  3  2  1  0
     |  |  |  |  |  |  |  +-- Ready to switch on
     |  |  |  |  |  |  +----- Switched on
     |  |  |  |  |  +-------- Operation enabled
     |  |  |  |  +----------- Fault
     |  |  |  +-------------- Voltage enabled
     |  |  +----------------- Quick stop
     |  +-------------------- Switch on disabled
     +----------------------- Warning

*/

/**

The mask is 0x6F = 0110 1111, derived below. Every group of four bits is one
hexadecimal digit.

An 'x' in the manual marks a bit that does not take part in the state, so it
is cleared before comparing. Columns 7 (Warning) and 4 (Voltage enabled) are
'x' in all eight rows, which is exactly what the mask drops.

                                   x->0        & 0x6F      result
---------------------------------------------------------------------------
1)  Not ready to switch on      0000 0000   &  0x6F   =   0000 0000   0x00
2)  Switch on disabled          0100 0000   &  0x6F   =   0100 0000   0x40
3)  Ready to switch on          0010 0001   &  0x6F   =   0010 0001   0x21
4)  Switched on                 0010 0011   &  0x6F   =   0010 0011   0x23
5)  Operation enabled           0010 0111   &  0x6F   =   0010 0111   0x27
6)  Quick stop active           0000 0111   &  0x6F   =   0000 0111   0x07
7)  Fault reaction active       0000 1111   &  0x6F   =   0000 1111   0x0F
8)  Fault                       0000 1000   &  0x6F   =   0000 1000   0x08
---------------------------------------------------------------------------

 */


using signals::State;

// ---------------------------------------------------------------------------
// Statusword decoding.
//
// Applies kStateMask and reports which of the eight states of Table 2-5 the
// drive is in. Returns std::nullopt when the masked pattern matches none of
// them (noisy bus, drive still booting, unexpected firmware): the caller is
// forced to decide, and should log the raw Statusword there because by this
// point it has been lost.
// ---------------------------------------------------------------------------
std::optional<State> Decode(std::uint16_t statusword);


// ---------------------------------------------------------------------------
// State machine commands, Table 2-7 (p.2-16).
//
// kSwitchOnAndEnableOperation and kEnableOperation produce the SAME bit
// pattern (0xxx 1111). They are kept apart because the transition they cause
// depends on the starting state: 3+4 from «Ready to switch on», only 4 from
// «Switched on». The name documents the caller's intent.
// ---------------------------------------------------------------------------
enum class Command
{
  kShutdown,
  kSwitchOn,
  kSwitchOnAndEnableOperation,
  kDisableVoltage,
  kQuickStop,
  kDisableOperation,
  kEnableOperation,
  kFaultReset
};


// ---------------------------------------------------------------------------
// Controlword 0x6040: holds the value that gets written to the drive.
//
// It is stateful, and both reasons come from Table 2-7:
//
//  1. A command pins only SOME bits; the rest show up as 'x', which here
//     means "not mine, do not touch". Bits 4, 5, 6, 8 and 15 in particular
//     belong to the operating mode: clearing them while sending a «Shutdown»
//     would break a PPM handshake halfway through. That is why Apply() is a
//     read-modify-write over the stored value rather than a fresh build.
//
//  2. «Fault reset» is EDGE triggered (0->1) on bit 7, and an edge needs to
//     know the previous value.
// ---------------------------------------------------------------------------
class Controlword
{
public:
  // Applies a state machine command, touching only the bits Table 2-7 pins
  // for it. Bit 7 is lowered at the start of every call, so two consecutive
  // Apply() calls produce the 0->1->0 edge the drive expects.
  void Apply(Command command);

  // Operating-mode specific bits (4, 5, 6, 8, 15). Filtered through
  // kModeBits, so it is impossible by construction to clobber a state machine
  // bit from here.
  void SetModeBits(std::uint16_t bits);
  void ClearModeBits(std::uint16_t bits);

  std::uint16_t Raw() const {return value_;}
  void SetRaw(std::uint16_t value) {value_ = value;}

private:
  std::uint16_t value_{0};
};


// ---------------------------------------------------------------------------
// Sequencer: (current state, goal) -> what to send THIS cycle.
//
// Never blocks, never allocates, never spawns a thread. One call emits at
// most one command; reaching «Operation enabled» from scratch costs three.
//
// It does not remember how far along the sequence it is, and does not need
// to: the drive reports its state every time, so the position in the sequence
// arrives for free. If a command is lost the state simply does not change and
// the next call retries.
// ---------------------------------------------------------------------------
enum class Goal
{
  kOperational,  // «Operation enabled»: power applied to the motor
  kDisabled      // «Switch on disabled»: no power
};

enum class Progress
{
  kReached,     // the axis is where it was asked to be
  kInProgress,  // advancing, or waiting for the drive
  kBlocked      // fault: no way out without an external decision
};

struct Step
{
  Progress progress;
  std::optional<Command> command;
};

// Never emits kFaultReset. Leaving a fault requires an explicit request, for
// two reasons:
//
//  1. Safety: a fault is a physical failure (overcurrent, following error,
//     overtemperature). Re-enabling on its own means pushing against the
//     cause again.
//
//  2. Table 2-6, transition 15: «Reset fault condition if no fault is
//     present». If the cause is still there the drive faults again at once,
//     and an automatic reset in a control loop would be a reset->fault->reset
//     cycle that floods the bus and starves the other axes.
Step PlanStep(State current, Goal goal);

}  // namespace epos4::core
