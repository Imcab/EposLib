// StatusSignal without a bus: its refresher is whatever the test hands it.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <system_error>
#include <thread>

#include <ros2units/units.h>

#include "epos4/signals/StatusSignal.hpp"

using epos4::signals::IsAllGood;
using epos4::signals::RefreshAll;
using epos4::signals::StatusSignal;
using namespace std::chrono_literals;

namespace
{

// A refresher that answers `value` after `delay`, like an SDO round trip.
template<typename T>
StatusSignal<T>
Slow(T value, std::chrono::milliseconds delay = 0ms)
{
  return StatusSignal<T>(
    [value, delay](T & out) {
      std::this_thread::sleep_for(delay);
      out = value;
      return std::error_code{};
    });
}

template<typename T>
StatusSignal<T>
Failing()
{
  return StatusSignal<T>(
    [](T &) {return std::make_error_code(std::errc::timed_out);});
}

}  // namespace


// ---- the basics ------------------------------------------------------------

TEST(StatusSignal, HoldsNothingUntilRefreshed)
{
  auto signal = Slow<std::int32_t>(42);
  EXPECT_FALSE(signal.HasValue());

  EXPECT_EQ(signal.Refresh().GetValue(), 42);
  EXPECT_TRUE(signal.HasValue());
  EXPECT_FALSE(signal.GetStatus());
}

// A failed read keeps the last good value rather than inventing one, and
// says so through the status.
TEST(StatusSignal, AFailedRefreshKeepsTheLastGoodValue)
{
  std::int32_t answer = 10;
  bool fail = false;
  StatusSignal<std::int32_t> signal(
    [&](std::int32_t & out) {
      if (fail) {return std::make_error_code(std::errc::timed_out);}
      out = answer;
      return std::error_code{};
    });

  signal.Refresh();
  fail = true;
  answer = 99;
  signal.Refresh();

  EXPECT_EQ(signal.GetValue(), 10);
  EXPECT_EQ(signal.GetStatus(), std::make_error_code(std::errc::timed_out));
}


// ---- IsNear ----------------------------------------------------------------

TEST(StatusSignal, IsNearIsInclusiveOnBothSides)
{
  auto signal = Slow<std::int32_t>(1000);
  signal.Refresh();

  EXPECT_TRUE(signal.IsNear(1050, 50));
  EXPECT_TRUE(signal.IsNear(950, 50));
  EXPECT_FALSE(signal.IsNear(1051, 50));
  EXPECT_FALSE(signal.IsNear(949, 50));
}

// value - target would wrap for an unsigned type and report "far" for a
// value just below the target.
TEST(StatusSignal, IsNearDoesNotWrapForUnsignedValues)
{
  auto signal = Slow<std::uint16_t>(100);
  signal.Refresh();

  EXPECT_TRUE(signal.IsNear(105, 10));
  EXPECT_TRUE(signal.IsNear(95, 10));
  EXPECT_FALSE(signal.IsNear(200, 10));
}

TEST(StatusSignal, IsNearWorksInUnits)
{
  auto signal = Slow(units::temperature::celsius_t{61.5});
  signal.Refresh();

  EXPECT_TRUE(
    signal.IsNear(
      units::temperature::celsius_t{60.0},
      units::temperature::celsius_t{2.0}));
  EXPECT_FALSE(
    signal.IsNear(
      units::temperature::celsius_t{58.0},
      units::temperature::celsius_t{2.0}));
}


// ---- RefreshAll ------------------------------------------------------------

TEST(RefreshAll, RefreshesEverySignal)
{
  auto a = Slow<std::int32_t>(1);
  auto b = Slow<std::uint16_t>(2);
  auto c = Slow(units::voltage::volt_t{48.0});

  EXPECT_FALSE(RefreshAll(a, b, c));
  EXPECT_EQ(a.GetValue(), 1);
  EXPECT_EQ(b.GetValue(), 2);
  EXPECT_DOUBLE_EQ(c.GetValue().value(), 48.0);
}

// The reason it exists: four reads of 50 ms each finish in about 50 ms, not
// 200. On the bus, those are SDO reads to four different drives, each on
// its own driver thread.
TEST(RefreshAll, RunsTheReadsConcurrently)
{
  auto a = Slow<std::int32_t>(1, 50ms);
  auto b = Slow<std::int32_t>(2, 50ms);
  auto c = Slow<std::int32_t>(3, 50ms);
  auto d = Slow<std::int32_t>(4, 50ms);

  const auto start = std::chrono::steady_clock::now();
  RefreshAll(a, b, c, d);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, 150ms);  // sequential would be 200 ms
  EXPECT_EQ(a.GetValue() + b.GetValue() + c.GetValue() + d.GetValue(), 10);
}

// One dead axis must not stop the others from being read, and must not be
// hidden either.
TEST(RefreshAll, ReportsAFailureButStillRefreshesTheRest)
{
  auto good = Slow<std::int32_t>(7);
  auto bad = Failing<std::int32_t>();
  auto alsoGood = Slow<std::int32_t>(8);

  EXPECT_EQ(RefreshAll(good, bad, alsoGood), std::make_error_code(std::errc::timed_out));
  EXPECT_EQ(good.GetValue(), 7);
  EXPECT_EQ(alsoGood.GetValue(), 8);
}


// ---- IsAllGood -------------------------------------------------------------

TEST(IsAllGood, NeedsAValueAndASuccessfulReadEverywhere)
{
  auto a = Slow<std::int32_t>(1);
  auto b = Slow<std::int32_t>(2);
  EXPECT_FALSE(IsAllGood(a, b));   // never refreshed

  RefreshAll(a, b);
  EXPECT_TRUE(IsAllGood(a, b));

  auto bad = Failing<std::int32_t>();
  bad.Refresh();
  EXPECT_FALSE(IsAllGood(a, b, bad));
}
