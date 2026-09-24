// Tests for the ros2_control plugin's configuration handling.
//
// on_init is the only part that can be exercised without a bus, and it is
// also where a misconfigured URDF has to be caught: every check here is one
// that would otherwise surface as an arm behaving strangely rather than as an
// error at startup.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "epos4_ros2_control/Epos4System.hpp"

using epos4_ros2_control::Epos4System;
using hardware_interface::CallbackReturn;

namespace
{

hardware_interface::ComponentInfo
MakeJoint(
  const std::string & name, const std::string & nodeId,
  const std::string & counts = "2000", const std::string & gear = "0.01")
{
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.command_interfaces.push_back({hardware_interface::HW_IF_POSITION, "", "", "", "double", 1});
  joint.state_interfaces.push_back({hardware_interface::HW_IF_POSITION, "", "", "", "double", 1});
  joint.state_interfaces.push_back({hardware_interface::HW_IF_VELOCITY, "", "", "", "double", 1});
  joint.state_interfaces.push_back({hardware_interface::HW_IF_EFFORT, "", "", "", "double", 1});
  joint.parameters["node_id"] = nodeId;
  joint.parameters["quadcounts_per_revolution"] = counts;
  joint.parameters["gear_ratio"] = gear;
  return joint;
}

hardware_interface::HardwareInfo
MakeInfo(std::vector<hardware_interface::ComponentInfo> joints)
{
  hardware_interface::HardwareInfo info;
  info.name = "epos4_arm";
  info.type = "system";
  info.hardware_parameters["can_interface"] = "vcan0";
  info.hardware_parameters["master_dcf"] = "/tmp/master.dcf";
  info.hardware_parameters["master_node_id"] = "1";
  info.joints = std::move(joints);
  return info;
}

}  // namespace


TEST(Epos4System, AcceptsAWellFormedConfiguration)
{
  Epos4System system;
  const auto info = MakeInfo({MakeJoint("shoulder", "2"), MakeJoint("elbow", "3")});
  EXPECT_EQ(system.on_init(info), CallbackReturn::SUCCESS);
}

TEST(Epos4System, ExportsOneCommandAndThreeStateInterfacesPerJoint)
{
  Epos4System system;
  ASSERT_EQ(
    system.on_init(MakeInfo({MakeJoint("shoulder", "2"), MakeJoint("elbow", "3")})),
    CallbackReturn::SUCCESS);

  EXPECT_EQ(system.export_command_interfaces().size(), 2u);
  EXPECT_EQ(system.export_state_interfaces().size(), 6u);
}

// The master DCF is what tells the CANopen master which nodes belong on the
// bus. Without it nothing can boot, so it is required rather than defaulted.
TEST(Epos4System, RequiresTheMasterDcf)
{
  Epos4System system;
  auto info = MakeInfo({MakeJoint("shoulder", "2")});
  info.hardware_parameters.erase("master_dcf");
  EXPECT_EQ(system.on_init(info), CallbackReturn::ERROR);
}

// This plugin drives Cyclic Synchronous Position. A joint asking for velocity
// would be silently ignored, which is worse than refusing to start.
TEST(Epos4System, RejectsACommandInterfaceItCannotDrive)
{
  Epos4System system;
  auto joint = MakeJoint("shoulder", "2");
  joint.command_interfaces[0].name = hardware_interface::HW_IF_VELOCITY;
  EXPECT_EQ(system.on_init(MakeInfo({joint})), CallbackReturn::ERROR);
}

TEST(Epos4System, RejectsMoreThanOneCommandInterface)
{
  Epos4System system;
  auto joint = MakeJoint("shoulder", "2");
  joint.command_interfaces.push_back(
    {hardware_interface::HW_IF_VELOCITY, "", "", "", "double", 1});
  EXPECT_EQ(system.on_init(MakeInfo({joint})), CallbackReturn::ERROR);
}

TEST(Epos4System, RejectsAStateInterfaceItDoesNotProvide)
{
  Epos4System system;
  auto joint = MakeJoint("shoulder", "2");
  joint.state_interfaces.push_back(
    {hardware_interface::HW_IF_ACCELERATION, "", "", "", "double", 1});
  EXPECT_EQ(system.on_init(MakeInfo({joint})), CallbackReturn::ERROR);
}

// Two joints on one node-ID would fight over the same drive, and the symptom
// - both reading the same position - is confusing enough to catch here.
TEST(Epos4System, RejectsDuplicateNodeIds)
{
  Epos4System system;
  EXPECT_EQ(
    system.on_init(MakeInfo({MakeJoint("shoulder", "2"), MakeJoint("elbow", "2")})),
    CallbackReturn::ERROR);
}

TEST(Epos4System, RequiresTheEncoderResolution)
{
  Epos4System system;
  auto joint = MakeJoint("shoulder", "2");
  joint.parameters.erase("quadcounts_per_revolution");
  EXPECT_EQ(system.on_init(MakeInfo({joint})), CallbackReturn::ERROR);
}

// A gear ratio of zero would make every conversion divide by zero.
TEST(Epos4System, RejectsAZeroGearRatio)
{
  Epos4System system;
  EXPECT_EQ(
    system.on_init(MakeInfo({MakeJoint("shoulder", "2", "2000", "0.0")})),
    CallbackReturn::ERROR);
}

TEST(Epos4System, RejectsUnparseableParameters)
{
  Epos4System system;
  EXPECT_EQ(
    system.on_init(MakeInfo({MakeJoint("shoulder", "not-a-number")})),
    CallbackReturn::ERROR);
}

TEST(Epos4System, RequiresANodeIdPerJoint)
{
  Epos4System system;
  auto joint = MakeJoint("shoulder", "2");
  joint.parameters.erase("node_id");
  EXPECT_EQ(system.on_init(MakeInfo({joint})), CallbackReturn::ERROR);
}

// ros2_control works in radians; the drive works in encoder counts. The
// conversion is MechanismScale's, and this pins the arithmetic the plugin
// relies on: 2000 counts per motor turn through a 1:100 reduction.
TEST(Epos4System, RadiansConvertToCountsThroughTheGearbox)
{
  const epos4::MechanismScale scale{2000, 1.0 / 100.0};

  const units::angle::turn_t quarterTurn{(M_PI / 2.0) / (2.0 * M_PI)};
  EXPECT_EQ(scale.ToQuadCounts(quarterTurn), 50000);

  const double radians = scale.ToAngle(50000).value() * 2.0 * M_PI;
  EXPECT_NEAR(radians, M_PI / 2.0, 1e-9);
}
