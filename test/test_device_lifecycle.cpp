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
