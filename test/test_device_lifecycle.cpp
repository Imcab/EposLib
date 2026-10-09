// What a device must allow between being declared and the bus starting.
//
// Devices have to be constructed BEFORE CanBus::Start(): the master only
// routes a node's PDOs to a driver registered when it boots the node. So
// everything a caller legitimately does to set a device up - telling it the
// mechanism, reaching its configurator or its encoder - happens while there
// is no master yet. None of it may need one.
//
// None of these tests opens a socket: constructing a CanBus only stores its
// options, and nothing here calls Start().

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <system_error>

#include <ros2units/units.h>

#include "epos4/CanBus.hpp"
#include "epos4/hardware/Encoder.hpp"
#include "epos4/hardware/Epos4.hpp"

namespace
{

epos4::CanBus::Options
UnstartedBus()
{
  epos4::CanBus::Options options;
  options.interface = "unused";
  options.masterDcf = "unused.dcf";
  options.masterNodeId = 1;
  return options;
}

}  // namespace


// A control application does exactly this at start-up, and it used to
// crash there: the encoder that holds the mechanism was only created when the
// bus attached the device, so SetMechanism dereferenced a null pointer.
TEST(DeviceLifecycle, TheMechanismCanBeSetBeforeTheBusStarts)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};

  motor.SetMechanism(2000, 0.01);

  EXPECT_EQ(motor.GetMechanism().GetQuadCountsPerRevolution(), 2000u);
  EXPECT_DOUBLE_EQ(motor.GetMechanism().GetGearRatio(), 0.01);
}

TEST(DeviceLifecycle, TheMechanismIsSharedWithTheEncoder)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};

  motor.SetMechanism(4096, 0.5);

  EXPECT_EQ(motor.GetEncoder().GetMechanism().GetQuadCountsPerRevolution(), 4096u);
}

TEST(DeviceLifecycle, SubsystemsExistBeforeTheBusStarts)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};

  // Taking the references is what matters: it must not touch the bus.
  epos4::Configurator & configurator = motor.GetConfigurator();
  epos4::Encoder & encoder = motor.GetEncoder();
  (void)configurator;
  (void)encoder;
  SUCCEED();
}

// Quantities are resolved against the mechanism, so a mechanism set before
// Start() has to be the one used afterwards.
TEST(DeviceLifecycle, AMechanismSetEarlyConvertsAnglesToCounts)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};
  motor.SetMechanism(2000, 0.01);

  // A quarter turn at the output is 25 motor turns through a 1:100 gearbox.
  EXPECT_EQ(
    motor.GetMechanism().ToQuadCounts(units::angle::degree_t{90.0}), 50000);
}

// Everything else on a device needs a driver, which only exists once the bus
// has started. Called before that, each of these dereferenced a null pointer
// and took the process down; they now report that there is nothing to talk to.
TEST(DeviceLifecycle, CommandsBeforeTheBusStartsFailInsteadOfCrashing)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};
  const auto notConnected = std::make_error_code(std::errc::not_connected);

  EXPECT_FALSE(motor.Enable(std::chrono::milliseconds{1}));
  EXPECT_FALSE(motor.Disable(std::chrono::milliseconds{1}));
  EXPECT_FALSE(motor.ClearFault(std::chrono::milliseconds{1}));
  EXPECT_FALSE(motor.Home(epos4::controls::Homing{}, std::chrono::milliseconds{1}));
  EXPECT_EQ(motor.Halt(), notConnected);
  EXPECT_EQ(motor.QuickStop(), notConnected);
  EXPECT_EQ(motor.SetControl(epos4::controls::ProfilePosition{}.WithPosition(1000)), notConnected);
  EXPECT_EQ(motor.SetControl(epos4::controls::CyclicPosition{}.WithPosition(1000)), notConnected);

  std::uint16_t raw{};
  EXPECT_EQ(motor.ReadObject(epos4::od::At(epos4::od::cia402::kStatusword), raw), notConnected);
  EXPECT_EQ(
    motor.WriteObject<std::uint16_t>(epos4::od::At(epos4::od::cia402::kControlword), 0),
    notConnected);
}

TEST(DeviceLifecycle, SignalsBeforeTheBusStartsReportNotConnected)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};
  const auto notConnected = std::make_error_code(std::errc::not_connected);

  auto & position = motor.GetPosition().Refresh();
  EXPECT_EQ(position.GetStatus(), notConnected);
  EXPECT_FALSE(position.HasValue());

  EXPECT_EQ(motor.GetState().Refresh().GetStatus(), notConnected);
  EXPECT_EQ(motor.GetSupplyVoltage().Refresh().GetStatus(), notConnected);
  EXPECT_EQ(motor.IsTargetReached().Refresh().GetStatus(), notConnected);
  EXPECT_FALSE(motor.IsEnabled());
  EXPECT_FALSE(motor.IsFaulted());

  // The reference is the same object before and after: callers may keep it.
  EXPECT_EQ(&motor.GetPosition(), &position);
}

TEST(DeviceLifecycle, ConfigurationBeforeTheBusStartsFailsInsteadOfCrashing)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};
  const auto notConnected = std::make_error_code(std::errc::not_connected);

  epos4::configs::MotionProfileConfigs profile;
  profile.profileAcceleration = 1000;
  EXPECT_EQ(motor.GetConfigurator().Apply(profile), notConnected);

  epos4::configs::MotionProfileConfigs readBack;
  EXPECT_EQ(motor.GetConfigurator().Refresh(readBack), notConnected);
  EXPECT_EQ(motor.GetConfigurator().Save(), notConnected);
}

TEST(DeviceLifecycle, TheCyclicPathBeforeTheBusStartsIsIdle)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};

  EXPECT_FALSE(motor.IsPdoActive());
  EXPECT_FALSE(motor.IsCyclicModeActive());
  EXPECT_FALSE(motor.IsCyclicHealthy());
  EXPECT_EQ(motor.GetTimeSinceLastPdo(), std::chrono::steady_clock::duration::max());
  EXPECT_EQ(motor.GetCachedPosition(), 0);
  EXPECT_EQ(motor.GetCachedErrorCode(), 0u);
  EXPECT_EQ(
    motor.EnterCyclicPositionMode(), std::make_error_code(std::errc::not_connected));
}

TEST(DeviceLifecycle, NoBootIsReportedBeforeTheBusStarts)
{
  epos4::CanBus bus{UnstartedBus()};
  epos4::Epos4 motor{bus, 2};

  const auto boot = motor.GetBootStatus();
  EXPECT_EQ(boot.count, 0u);
  EXPECT_FALSE(boot.Succeeded());
}

TEST(DeviceLifecycle, BootStatusSucceedsOnlyForACleanOrAdvisoryBoot)
{
  epos4::signals::BootStatus boot;
  boot.count = 1;
  EXPECT_TRUE(boot.Succeeded());
  boot.errorStatus = 'L';  // already operational: advisory
  EXPECT_TRUE(boot.Succeeded());
  boot.errorStatus = 'D';  // vendor-ID mismatch
  EXPECT_FALSE(boot.Succeeded());
  boot.errorStatus = 'J';  // configuration download failed
  EXPECT_FALSE(boot.Succeeded());
}

// Devices may come and go before the bus starts; a destroyed one must not be
// left behind for Start() to attach.
TEST(DeviceLifecycle, DevicesCanBeDestroyedBeforeTheBusStarts)
{
  epos4::CanBus bus{UnstartedBus()};
  {
    epos4::Epos4 first{bus, 2};
    epos4::Epos4 second{bus, 3};
  }
  epos4::Epos4 third{bus, 4};
  EXPECT_EQ(third.GetNodeId(), 4u);
}
