// The cyclic path, without a bus.
//
// core::CyclicState is what a control loop trusts on every cycle:
// read() takes the feedback from it and write() stages the setpoint into it.
// On the bus it is fed from Lely's callbacks; here it is fed by hand, with
// the clock passed in, so staleness is tested between two time points
// instead of by sleeping.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "epos4/core/CyclicState.hpp"

using epos4::core::CyclicState;
using namespace std::chrono_literals;

namespace
{

// Statusword patterns from Table 2-5, the same ones test_cia402_state_machine
// decodes.
constexpr std::uint16_t kOperationEnabled = 0x0037;  // with Voltage enabled
constexpr std::uint16_t kFault = 0x0008;

const CyclicState::Clock::time_point kT0{std::chrono::seconds{100}};

// An active path that has just received a PDO from an enabled drive.
void
MakeHealthy(CyclicState & cyclic, CyclicState::Clock::time_point at)
{
  cyclic.Activate(0, 0x000F);
  cyclic.RecordStatusword(kOperationEnabled);
  cyclic.RecordPdo(at);
}

}  // namespace


// ---- the caches ----------------------------------------------------------

TEST(CyclicState, CachesReturnTheLastValueReceived)
{
  CyclicState cyclic;
  cyclic.RecordPosition(12345);
  cyclic.RecordVelocity(-300);
  cyclic.RecordTorque(-250);
  cyclic.RecordStatusword(kOperationEnabled);

  cyclic.RecordPosition(12400);  // a later PDO replaces, never accumulates

  EXPECT_EQ(cyclic.Position(), 12400);
  EXPECT_EQ(cyclic.Velocity(), -300);
  EXPECT_EQ(cyclic.Torque(), -250);
  EXPECT_EQ(cyclic.Statusword(), kOperationEnabled);
}

TEST(CyclicState, NoPdoYetMeansNoAge)
{
  CyclicState cyclic;
  EXPECT_FALSE(cyclic.HasReceivedPdo());
  EXPECT_EQ(cyclic.TimeSinceLastPdo(kT0), CyclicState::Clock::duration::max());
}

TEST(CyclicState, AgeIsMeasuredFromTheLastPdo)
{
  CyclicState cyclic;
  cyclic.RecordPdo(kT0);
  cyclic.RecordPdo(kT0 + 10ms);

  EXPECT_TRUE(cyclic.HasReceivedPdo());
  EXPECT_EQ(cyclic.TimeSinceLastPdo(kT0 + 25ms), 15ms);
}


// ---- health: the plugin's safety net -------------------------------------

TEST(CyclicState, HealthyWhileActiveFreshAndEnabled)
{
  CyclicState cyclic;
  MakeHealthy(cyclic, kT0);
  EXPECT_TRUE(cyclic.IsHealthy(50ms, kT0 + 10ms));
}

// The important one. A bus that goes quiet leaves every cached value exactly
// where it was, reading back as if nothing happened; only the age tells.
TEST(CyclicState, BecomesUnhealthyWhenPdosStopArriving)
{
  CyclicState cyclic;
  MakeHealthy(cyclic, kT0);

  EXPECT_TRUE(cyclic.IsHealthy(50ms, kT0 + 50ms));    // at the limit
  EXPECT_FALSE(cyclic.IsHealthy(50ms, kT0 + 51ms));   // past it

  // And the feedback still reads back happily - which is why the age check
  // has to exist at all.
  EXPECT_EQ(cyclic.Statusword(), kOperationEnabled);
}

TEST(CyclicState, RecoversWhenPdosResume)
{
  CyclicState cyclic;
  MakeHealthy(cyclic, kT0);
  ASSERT_FALSE(cyclic.IsHealthy(50ms, kT0 + 200ms));

  cyclic.RecordPdo(kT0 + 200ms);
  EXPECT_TRUE(cyclic.IsHealthy(50ms, kT0 + 210ms));
}

TEST(CyclicState, UnhealthyWhenTheDriveIsNotEnabled)
{
  CyclicState cyclic;
  MakeHealthy(cyclic, kT0);
  cyclic.RecordStatusword(kFault);  // fresh PDOs, but reporting a fault

  EXPECT_FALSE(cyclic.IsHealthy(50ms, kT0 + 10ms));
}

TEST(CyclicState, UnhealthyWhenNeverActivated)
{
  CyclicState cyclic;
  cyclic.RecordStatusword(kOperationEnabled);
  cyclic.RecordPdo(kT0);

  EXPECT_FALSE(cyclic.IsHealthy(50ms, kT0 + 10ms));
}

TEST(CyclicState, UnhealthyWithoutAnyPdo)
{
  CyclicState cyclic;
  cyclic.Activate(0, 0x000F);
  cyclic.RecordStatusword(kOperationEnabled);

  EXPECT_FALSE(cyclic.IsHealthy(50ms, kT0));
}


// ---- the setpoint --------------------------------------------------------

// The first SYNC after activating publishes whatever is staged. Seeding it
// with the current position is what keeps that SYNC from commanding zero.
TEST(CyclicState, ActivatingSeedsTheTargetWithTheCurrentPosition)
{
  CyclicState cyclic;
  cyclic.Activate(-48210, 0x000F);

  EXPECT_TRUE(cyclic.IsActive());
  EXPECT_EQ(cyclic.StagedTargetPosition(), -48210);
  EXPECT_EQ(cyclic.Position(), -48210);
  EXPECT_EQ(cyclic.Controlword(), 0x000F);
}

TEST(CyclicState, StagingReplacesTheTarget)
{
  CyclicState cyclic;
  cyclic.Activate(0, 0x000F);
  cyclic.StageTargetPosition(100);
  cyclic.StageTargetPosition(250);

  EXPECT_EQ(cyclic.StagedTargetPosition(), 250);
}

// QuickStop() during cyclic operation writes the Controlword; if the copy
// OnSync republishes were not updated too, the next SYNC would undo it.
TEST(CyclicState, AControlwordWrittenWhileActiveIsWhatGetsPublished)
{
  CyclicState cyclic;
  cyclic.Activate(0, 0x000F);           // Enable operation
  cyclic.SetControlword(0x000B);        // Quick stop

  EXPECT_EQ(cyclic.Controlword(), 0x000B);
}

TEST(CyclicState, DeactivatingStopsPublishingButKeepsTheTarget)
{
  CyclicState cyclic;
  cyclic.Activate(500, 0x000F);
  cyclic.Deactivate();

  EXPECT_FALSE(cyclic.IsActive());
  EXPECT_EQ(cyclic.StagedTargetPosition(), 500);
}


// ---- threads -------------------------------------------------------------

// The control thread stages, the bus thread reads, neither waits. Staged
// values are strictly increasing, so the reader must only ever see them go
// up: a value going backwards or out of range would be a torn or reordered
// read. Atomics guarantee this; the test keeps anyone from "optimising"
// them into plain integers.
TEST(CyclicState, StagingFromAnotherThreadIsSeenInOrder)
{
  CyclicState cyclic;
  cyclic.Activate(0, 0x000F);

  constexpr std::int32_t kLast = 200000;
  std::atomic<bool> done{false};

  std::thread control([&]() {
      for (std::int32_t v = 1; v <= kLast; ++v) {
        cyclic.StageTargetPosition(v);
      }
      done.store(true);
    });

  std::int32_t previous = 0;
  bool inOrder = true;
  while (!done.load()) {
    const std::int32_t seen = cyclic.StagedTargetPosition();
    if (seen < previous || seen > kLast) {inOrder = false;}
    previous = seen;
  }
  control.join();

  EXPECT_TRUE(inOrder);
  EXPECT_EQ(cyclic.StagedTargetPosition(), kLast);
}

// A reader that sees the path active must also see the seeded target, or
// the first SYNC publishes a stale zero. Activate() releases, IsActive()
// acquires; on x86 this passes either way, on ARM it is what the ordering
// is for.
TEST(CyclicState, AReaderThatSeesActiveSeesTheSeededTarget)
{
  for (int round = 0; round < 200; ++round) {
    CyclicState cyclic;
    constexpr std::int32_t kSeed = 777777;

    std::thread control([&]() {cyclic.Activate(kSeed, 0x000F);});
    while (!cyclic.IsActive()) {
    }
    EXPECT_EQ(cyclic.StagedTargetPosition(), kSeed);
    control.join();
  }
}
