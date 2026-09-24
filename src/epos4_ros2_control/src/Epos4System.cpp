#include "epos4_ros2_control/Epos4System.hpp"

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <units.h>

#include "epos4/signals/Errors.hpp"

namespace epos4_ros2_control
{

namespace
{

rclcpp::Logger
Log()
{
  return rclcpp::get_logger("Epos4System");
}

// Reads a parameter, falling back when it is absent. Throws on a value that
// is present but unparseable, because a typo in a gear ratio is not something
// to paper over with a default.
std::string
GetParam(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & fallback = "")
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

}  // namespace


hardware_interface::CallbackReturn
Epos4System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  canInterface_ = GetParam(info.hardware_parameters, "can_interface", "can0");
  masterDcf_ = GetParam(info.hardware_parameters, "master_dcf");
  if (masterDcf_.empty()) {
    RCLCPP_ERROR(
      Log(), "the 'master_dcf' hardware parameter is required: it is what tells "
      "the CANopen master which nodes are supposed to be on the bus");
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    masterNodeId_ = static_cast<std::uint8_t>(
      std::stoi(GetParam(info.hardware_parameters, "master_node_id", "1")));
  } catch (const std::exception &) {
    RCLCPP_ERROR(Log(), "'master_node_id' is not a number");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Sized once and never resized: ros2_control takes the addresses of the
  // state and command members, so the vector must not reallocate afterwards.
  axes_.resize(info.joints.size());

  std::set<std::uint8_t> seenNodeIds;

  for (std::size_t i = 0; i < info.joints.size(); ++i) {
    const auto & joint = info.joints[i];
    Axis & axis = axes_[i];
    axis.name = joint.name;

    // --- interfaces ---
    //
    // Exactly one command interface, position. This plugin drives Cyclic
    // Synchronous Position; a joint asking for velocity or effort would be
    // silently ignored, which is worse than refusing to start.
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        Log(), "joint '%s' must declare exactly one command interface, '%s'",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto & state : joint.state_interfaces) {
      if (state.name != hardware_interface::HW_IF_POSITION &&
        state.name != hardware_interface::HW_IF_VELOCITY &&
        state.name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_ERROR(
          Log(), "joint '%s' asks for state interface '%s', which this hardware "
          "does not provide", joint.name.c_str(), state.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // --- parameters ---
    try {
      const std::string nodeIdParam = GetParam(joint.parameters, "node_id");
      if (nodeIdParam.empty()) {
        RCLCPP_ERROR(Log(), "joint '%s' has no 'node_id'", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      axis.nodeId = static_cast<std::uint8_t>(std::stoi(nodeIdParam));

      const auto counts = static_cast<std::uint32_t>(
        std::stoul(GetParam(joint.parameters, "quadcounts_per_revolution", "0")));
      const double gearRatio =
        std::stod(GetParam(joint.parameters, "gear_ratio", "1.0"));

      if (counts == 0) {
        RCLCPP_ERROR(
          Log(), "joint '%s' has no 'quadcounts_per_revolution'. Note this is "
          "FOUR TIMES the encoder's pulses per revolution: a 500 CPR encoder "
          "is 2000 here", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (gearRatio == 0.0) {
        RCLCPP_ERROR(Log(), "joint '%s' has a 'gear_ratio' of zero", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      axis.scale = epos4::MechanismScale{counts, gearRatio};
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        Log(), "joint '%s' has an unparseable parameter: %s",
        joint.name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Two joints on the same node-ID would have them fighting over one drive,
    // and the symptom - both reading the same position - is confusing enough
    // to be worth catching here.
    if (!seenNodeIds.insert(axis.nodeId).second) {
      RCLCPP_ERROR(
        Log(), "node_id %u is used by more than one joint", axis.nodeId);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      Log(), "joint '%s': node %u, %u counts/rev, gear ratio %.6f",
      axis.name.c_str(), axis.nodeId, axis.scale.GetQuadCountsPerRevolution(),
      axis.scale.GetGearRatio());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
Epos4System::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    bus_ = std::make_unique<epos4::CanBus>(
      epos4::CanBus::Options{canInterface_, masterDcf_, masterNodeId_});

    // Devices BEFORE Start(). A CANopen master boots each slave once, at
    // reset, and only routes a node's PDOs to a driver registered at that
    // moment. Constructing them afterwards gives nodes that answer SDO
    // perfectly and never deliver a single PDO - which here would look like
    // every joint frozen at its startup position.
    for (auto & axis : axes_) {
      axis.device = std::make_unique<epos4::Epos4>(*bus_, axis.nodeId);
      axis.device->SetMechanism(
        axis.scale.GetQuadCountsPerRevolution(), axis.scale.GetGearRatio());
    }

    bus_->Start();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(Log(), "could not bring up the CAN bus: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & axis : axes_) {
    if (!axis.device->WaitUntilReady()) {
      RCLCPP_ERROR(
        Log(), "joint '%s' (node %u) did not answer. Check the wiring, the "
        "node-ID, and that the identity in the DCF matches the drive",
        axis.name.c_str(), axis.nodeId);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read once, for the effort interface. Left at zero if the motor data was
    // never configured, in which case effort reports zero rather than a
    // number scaled by an unknown.
    auto & rated = axis.device->GetMotorRatedTorque();
    rated.Refresh();
    axis.ratedTorqueMicroNm = rated.GetStatus() ? 0 : rated.GetValue();
    if (axis.ratedTorqueMicroNm == 0) {
      RCLCPP_WARN(
        Log(), "joint '%s' reports no rated torque, so the effort interface "
        "will stay at zero. Configure the motor data (0x3001) to get it",
        axis.name.c_str());
    }

    RCLCPP_INFO(Log(), "joint '%s' (node %u) ready", axis.name.c_str(), axis.nodeId);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
Epos4System::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & axis : axes_) {
    if (!axis.device->Enable()) {
      RCLCPP_ERROR(
        Log(), "joint '%s' would not enable. %s", axis.name.c_str(),
        axis.device->DescribeLastError().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (auto ec = axis.device->EnterCyclicPositionMode()) {
      RCLCPP_ERROR(
        Log(), "joint '%s' would not enter cyclic position mode: %s",
        axis.name.c_str(), ec.message().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Seed the command with where the axis already is. Without this the first
    // write() sends the default command of zero and the arm swings to its
    // origin the instant the controller activates.
    const double position =
      axis.scale.ToAngle(axis.device->GetCachedPosition()).value() * 2.0 * M_PI;
    axis.statePosition = position;
    axis.commandPosition = position;

    RCLCPP_INFO(
      Log(), "joint '%s' active at %.4f rad", axis.name.c_str(), position);
  }

  unhealthyReported_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
Epos4System::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivation must complete even with a faulted axis: being unable to shut
  // down is worst exactly when something has already gone wrong. Failures are
  // logged, not propagated.
  for (auto & axis : axes_) {
    axis.device->ExitCyclicMode();
    if (!axis.device->Disable()) {
      RCLCPP_WARN(Log(), "joint '%s' did not confirm disable", axis.name.c_str());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
Epos4System::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Devices before the bus: an Epos4 holds a reference to the master for as
  // long as it exists.
  for (auto & axis : axes_) {
    axis.device.reset();
  }
  bus_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
Epos4System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & axis : axes_) {
    interfaces.emplace_back(
      axis.name, hardware_interface::HW_IF_POSITION, &axis.statePosition);
    interfaces.emplace_back(
      axis.name, hardware_interface::HW_IF_VELOCITY, &axis.stateVelocity);
    interfaces.emplace_back(
      axis.name, hardware_interface::HW_IF_EFFORT, &axis.stateEffort);
  }
  return interfaces;
}


std::vector<hardware_interface::CommandInterface>
Epos4System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & axis : axes_) {
    interfaces.emplace_back(
      axis.name, hardware_interface::HW_IF_POSITION, &axis.commandPosition);
  }
  return interfaces;
}


hardware_interface::return_type
Epos4System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  bool healthy = true;

  for (auto & axis : axes_) {
    // Relaxed atomic loads. No bus traffic, no locking, no thread hop.
    const std::int32_t counts = axis.device->GetCachedPosition();
    const std::int32_t rpm = axis.device->GetCachedVelocity();
    const std::int16_t perThousand = axis.device->GetCachedTorque();

    axis.statePosition = axis.scale.ToAngle(counts).value() * 2.0 * M_PI;
    axis.stateVelocity =
      units::angular_velocity::radians_per_second_t(
      axis.scale.ToAngularVelocity(rpm)).value();
    axis.stateEffort =
      epos4::ToTorque(perThousand, axis.ratedTorqueMicroNm).value();

    if (!axis.device->IsCyclicHealthy()) {
      healthy = false;
      if (!unhealthyReported_) {
        RCLCPP_ERROR(
          Log(), "joint '%s' stopped delivering PDOs or left «Operation "
          "enabled». %s", axis.name.c_str(),
          axis.device->DescribeLastError().c_str());
      }
    }
  }

  if (!healthy) {
    // Reported once. A quiet bus would otherwise produce a message per axis
    // per cycle and bury everything else in the log.
    unhealthyReported_ = true;
    return hardware_interface::return_type::ERROR;
  }

  unhealthyReported_ = false;
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type
Epos4System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & axis : axes_) {
    if (!std::isfinite(axis.commandPosition)) {
      // A NaN would convert to an arbitrary count and command the axis
      // somewhere unrelated. Hold the last good setpoint instead.
      continue;
    }
    const units::angle::turn_t target{axis.commandPosition / (2.0 * M_PI)};
    axis.device->StageTargetPosition(axis.scale.ToQuadCounts(target));
  }
  return hardware_interface::return_type::OK;
}

}  // namespace epos4_ros2_control

PLUGINLIB_EXPORT_CLASS(
  epos4_ros2_control::Epos4System, hardware_interface::SystemInterface)
