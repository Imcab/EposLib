// Touch probe 1, encoded and decoded against Tables 6-164 and 6-165.

#include <gtest/gtest.h>

#include "epos4/controls/ControlRequests.hpp"
#include "epos4/signals/TouchProbe.hpp"

using epos4::controls::TouchProbe;

TEST(TouchProbe, DefaultIsOneShotPositiveEdgeOnTheInput)
{
  // enable (bit 0) + positive edge (bit 4); trigger 00, single event.
  EXPECT_EQ(TouchProbe{}.ToFunctionWord(), 0x0011);
}

TEST(TouchProbe, EveryFieldLandsOnItsBit)
{
  const auto word = TouchProbe{}
  .WithTrigger(TouchProbe::Trigger::kSourceObject)    // bits 3..2 = 10
  .WithContinuous(true)                               // bit 1
  .WithPositiveEdge(false)
  .WithNegativeEdge(true)                             // bit 5
  .ToFunctionWord();
  EXPECT_EQ(word, 0x0001 | 0x0002 | 0x0008 | 0x0020);

  EXPECT_EQ(
    TouchProbe{}.WithTrigger(TouchProbe::Trigger::kIndexPulse).ToFunctionWord(),
    0x0001 | 0x0004 | 0x0010);
}

// Table 6-164 note [a].
TEST(TouchProbe, TheIndexPulseCannotTakeBothEdges)
{
  EXPECT_FALSE(
    TouchProbe{}.WithTrigger(TouchProbe::Trigger::kIndexPulse)
    .WithNegativeEdge(true).IsValid());
  EXPECT_TRUE(TouchProbe{}.WithNegativeEdge(true).IsValid());  // the input can
}

TEST(TouchProbe, ArmingWithNoEdgeIsRefused)
{
  EXPECT_FALSE(TouchProbe{}.WithPositiveEdge(false).IsValid());
}

TEST(TouchProbe, StatusDecodesIntoItsThreeFlags)
{
  epos4::signals::TouchProbeState state;
  epos4::signals::DecodeTouchProbeStatus(0x0005, state);  // enabled, negative stored
  EXPECT_TRUE(state.enabled);
  EXPECT_FALSE(state.positiveEdgeStored);
  EXPECT_TRUE(state.negativeEdgeStored);
}
