#include "epos4/configs/Configs.hpp"

namespace epos4::configs
{


std::error_code
DigitalInputConfigs::Validate() const
{
  const std::optional<signals::DigitalInputFunction> pins[] = {
    input1, input2, input3, input4,
    highSpeedInput1, highSpeedInput2, highSpeedInput3, highSpeedInput4};

  // "Each function can only be mapped once". kNone is exempt: several inputs
  // may legitimately carry no function.
  for (std::size_t i = 0; i < 8; ++i) {
    if (!pins[i] || *pins[i] == signals::DigitalInputFunction::kNone) {
      continue;
    }
    for (std::size_t j = i + 1; j < 8; ++j) {
      if (pins[j] && *pins[j] == *pins[i]) {
        return std::make_error_code(std::errc::invalid_argument);
      }
    }
  }

  // Table 6-131, note [a]: touch probe is "not available with HsDigIn1 and
  // HsDigIn3" - the drive refuses the write.
  for (const auto & pin : {highSpeedInput1, highSpeedInput3}) {
    if (pin == signals::DigitalInputFunction::kTouchProbe) {
      return std::make_error_code(std::errc::invalid_argument);
    }
  }
  return {};
}


bool
RequiresPowerDisabled(od::Entry entry)
{
  switch (entry.index) {
    case od::maxon::kAxisConfiguration:
    case od::maxon::kDigitalIncrementalEncoder1:
    case od::maxon::kAnalogIncrementalEncoder:
    case od::maxon::kSSIAbsoluteEncoder:
    case od::maxon::kDigitalIncrementalEncoder2:
    case od::cia402::kSIUnitPosition:
    case od::cia402::kSIUnitVelocity:
    case od::cia402::kSIUnitAcceleration:
      return true;
    case od::maxon::kMotorData:
      return entry.subindex == od::maxon::kMotorData_NumberOfPolePairs.subindex;
    case od::maxon::kGearConfiguration:
      // "Write access is permitted in device state «Power Enabled»" for the
      // max gear input speed alone (6.2.55.3).
      return entry.subindex != od::maxon::kGearConfiguration_MaxGearInputSpeed.subindex;
    case od::maxon::kDigitalHallSensor:
      return entry.subindex == od::maxon::kDigitalHallSensor_DigitalHallSensorType.subindex;
    default:
      return false;
  }
}

std::error_code
DualLoopConfigs::Validate() const
{
  // 0x30AE:40 carries both the coefficient update and the filter enable;
  // see the struct.
  if (AnyFilterCoefficient() && !filterActive) {
    return std::make_error_code(std::errc::invalid_argument);
  }
  return {};
}

std::error_code
AnalogInputConfigs::Validate() const
{
  // Two inputs: the only clash is both carrying the same real function.
  if (input1 && input2 && *input1 == *input2 &&
    *input1 != signals::AnalogInputFunction::kNone)
  {
    return std::make_error_code(std::errc::invalid_argument);
  }
  return {};
}


ConfigWrites
Epos4Configuration::ToWrites() const
{
  ConfigWrites out;

  // Order matters. Motor and axis data define what the controllers are
  // controlling, so they go first; limits go last so they are in force before
  // anything can command a move.
  axis.AppendTo(out);
  motor.AppendTo(out);
  gear.AppendTo(out);
  currentControl.AppendTo(out);
  velocityControl.AppendTo(out);
  positionControl.AppendTo(out);
  velocityObserver.AppendTo(out);
  dualLoop.AppendTo(out);
  motionProfile.AppendTo(out);
  siUnits.AppendTo(out);
  cyclic.AppendTo(out);
  homing.AppendTo(out);
  stopOptions.AppendTo(out);

  // Standstill BEFORE the brake, and the output assignment last. The manual
  // is explicit that clamping a brake on an axis that has not stopped damages
  // the brake or the motor, so the condition that defines "stopped" is in
  // place before anything can drive a brake pin.
  standstill.AppendTo(out);
  holdingBrake.AppendTo(out);
  digitalInputs.AppendTo(out);
  digitalOutputs.AppendTo(out);

  limits.AppendTo(out);
  // The device around the axis. Protection limits do not depend on the
  // motor; the I/O functions do, which is why they follow it.
  protection.AppendTo(out);
  analogInputs.AppendTo(out);
  analogOutputs.AppendTo(out);
  customMemory.AppendTo(out);
  // Last: a Node-ID or bit rate change waits for a restart anyway, and
  // everything before it is written to the drive as the master knows it.
  communication.AppendTo(out);

  return out;
}

std::error_code
Epos4Configuration::ReadFrom(const ConfigReader & read)
{
  std::error_code first;
  auto keep = [&first](std::error_code ec) {
      if (ec && !first) {first = ec;}
    };
  keep(axis.ReadFrom(read));
  keep(motor.ReadFrom(read));
  keep(gear.ReadFrom(read));
  keep(currentControl.ReadFrom(read));
  keep(velocityControl.ReadFrom(read));
  keep(positionControl.ReadFrom(read));
  keep(velocityObserver.ReadFrom(read));
  keep(dualLoop.ReadFrom(read));
  keep(motionProfile.ReadFrom(read));
  keep(siUnits.ReadFrom(read));
  keep(cyclic.ReadFrom(read));
  keep(homing.ReadFrom(read));
  keep(stopOptions.ReadFrom(read));
  keep(standstill.ReadFrom(read));
  keep(holdingBrake.ReadFrom(read));
  keep(digitalInputs.ReadFrom(read));
  keep(digitalOutputs.ReadFrom(read));
  keep(limits.ReadFrom(read));
  keep(protection.ReadFrom(read));
  keep(analogInputs.ReadFrom(read));
  keep(analogOutputs.ReadFrom(read));
  keep(customMemory.ReadFrom(read));
  keep(communication.ReadFrom(read));
  return first;
}

std::error_code
Epos4Configuration::Validate() const
{
  if (auto ec = digitalInputs.Validate()) {
    return ec;
  }
  if (auto ec = dualLoop.Validate()) {
    return ec;
  }
  return analogInputs.Validate();
}

}  // namespace epos4::configs
