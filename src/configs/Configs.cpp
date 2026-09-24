#include "epos4/configs/Configs.hpp"

namespace epos4::configs
{

namespace
{

// Appends only if the option holds a value. Unset means "do not touch".
template<typename T, typename Stored>
void
Put(ConfigWrites & out, od::Entry entry, const std::optional<T> & field)
{
  if (field) {
    out.push_back(ConfigWrite{entry, static_cast<Stored>(*field)});
  }
}

template<typename Enum, typename Stored>
void
PutEnum(ConfigWrites & out, od::Entry entry, const std::optional<Enum> & field)
{
  if (field) {
    out.push_back(
      ConfigWrite{entry,
        static_cast<Stored>(static_cast<std::underlying_type_t<Enum>>(*field))});
  }
}

}  // namespace


void
MotorConfigs::AppendTo(ConfigWrites & out) const
{
  PutEnum<signals::MotorType, std::uint16_t>(out, od::At(od::cia402::kMotorType), motorType);
  Put<std::uint32_t, std::uint32_t>(out, od::maxon::kMotorData_NominalCurrent, nominalCurrent);
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kMotorData_OutputCurrentLimit,
    outputCurrentLimit);
  Put<std::uint8_t, std::uint8_t>(out, od::maxon::kMotorData_NumberOfPolePairs, numberOfPolePairs);
  Put<std::uint16_t, std::uint16_t>(
    out, od::maxon::kMotorData_ThermalTimeConstantWinding,
    thermalTimeConstant);
  Put<std::uint32_t, std::uint32_t>(out, od::maxon::kMotorData_TorqueConstant, torqueConstant);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kElectricalSystemParameters_ElectricalResistance,
    electricalResistance);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kElectricalSystemParameters_ElectricalInductance,
    electricalInductance);
}

void
GearConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kGearConfiguration_GearReductionNumerator,
    reductionNumerator);
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kGearConfiguration_GearReductionDenominator,
    reductionDenominator);
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kGearConfiguration_MaxGearInputSpeed,
    maxGearInputSpeed);
}

void
AxisConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kAxisConfiguration_SensorsConfiguration,
    sensorsConfiguration);
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kAxisConfiguration_ControlStructure,
    controlStructure);
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kAxisConfiguration_CommutationSensors,
    commutationSensors);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kAxisConfiguration_AxisConfigurationMiscellaneous,
    miscellaneous);
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kAxisConfiguration_MainSensorResolution,
    mainSensorResolution);
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kAxisConfiguration_MaxSystemSpeed,
    maxSystemSpeed);
}

void
CurrentControlConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kCurrentControlParameterSet_CurrentControllerPGain,
    p);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kCurrentControlParameterSet_CurrentControllerIGain,
    i);
}

void
PositionControlConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kPositionControlParameterSet_PositionControllerPGain,
    p);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kPositionControlParameterSet_PositionControllerIGain,
    i);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kPositionControlParameterSet_PositionControllerDGain,
    d);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kPositionControlParameterSet_PositionControllerFFVelocityGain,
    feedForwardVelocity);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kPositionControlParameterSet_PositionControllerFFAccelerationGain,
    feedForwardAcceleration);
}

void
VelocityControlConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kVelocityControlParameterSet_VelocityControllerPGain,
    p);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kVelocityControlParameterSet_VelocityControllerIGain,
    i);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kVelocityControlParameterSet_VelocityControllerFFVelocityGain,
    feedForwardVelocity);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kVelocityControlParameterSet_VelocityControllerFFAccelerationGain,
    feedForwardAcceleration);
  Put<std::uint32_t, std::uint32_t>(
    out,
    od::maxon::kVelocityControlParameterSet_VelocityControllerFilterCutOffFrequency,
    filterCutOffFrequency);
}

void
MotionProfileConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::uint32_t, std::uint32_t>(out, od::At(od::cia402::kProfileVelocity), profileVelocity);
  Put<std::uint32_t, std::uint32_t>(
    out, od::At(
      od::cia402::kProfileAcceleration), profileAcceleration);
  Put<std::uint32_t, std::uint32_t>(
    out, od::At(
      od::cia402::kProfileDeceleration), profileDeceleration);
  Put<std::uint32_t, std::uint32_t>(
    out, od::At(
      od::cia402::kQuickStopDeceleration), quickStopDeceleration);
  PutEnum<signals::MotionProfileType, std::int16_t>(
    out, od::At(
      od::cia402::kMotionProfileType), motionProfileType);
  Put<std::uint32_t, std::uint32_t>(
    out, od::At(od::cia402::kMaxProfileVelocity),
    maxProfileVelocity);
  Put<std::uint32_t, std::uint32_t>(out, od::At(od::cia402::kMaxAcceleration), maxAcceleration);
}

void
SiUnitConfigs::AppendTo(ConfigWrites & out) const
{
  if (!velocityPrefix) {
    return;
  }
  // Table 6-160: prefix in bits 31..24, numerator 23..16, denominator 15..8.
  // The unit is rev/min, so numerator 0xB4 (revolutions) over denominator
  // 0x47 (minute) - and those two never change on this device.
  constexpr std::uint32_t kRevPerMin = 0x00B44700u;
  const std::uint32_t value =
    (static_cast<std::uint32_t>(*velocityPrefix) << 24) | kRevPerMin;
  out.push_back(ConfigWrite{od::At(od::cia402::kSIUnitVelocity), value});
}

void
CyclicConfigs::AppendTo(ConfigWrites & out) const
{
  // Sub-index 1 only. Sub-index 2 (the time index) is fixed at -3 by the
  // device, so the unit is always milliseconds and there is nothing to write.
  Put<std::uint8_t, std::uint8_t>(
    out, od::cia402::kInterpolationTimePeriod_InterpolationTimePeriodValue,
    interpolationTimePeriodMs);
}

void
LimitConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::int32_t, std::int32_t>(
    out, od::cia402::kSoftwarePositionLimit_MinPositionLimit,
    minPositionLimit);
  Put<std::int32_t, std::int32_t>(
    out, od::cia402::kSoftwarePositionLimit_MaxPositionLimit,
    maxPositionLimit);
  Put<std::uint32_t, std::uint32_t>(out, od::At(od::cia402::kMaxMotorSpeed), maxMotorSpeed);
  Put<std::uint32_t, std::uint32_t>(
    out, od::At(
      od::cia402::kFollowingErrorWindow), followingErrorWindow);
  Put<std::uint16_t, std::uint16_t>(
    out, od::At(
      od::cia402::kFollowingErrorTimeOut), followingErrorTimeout);
  Put<std::uint32_t, std::uint32_t>(out, od::At(od::cia402::kPositionWindow), positionWindow);
  Put<std::uint16_t, std::uint16_t>(
    out, od::At(od::cia402::kPositionWindowTime),
    positionWindowTime);
}

void
HomingConfigs::AppendTo(ConfigWrites & out) const
{
  PutEnum<signals::HomingMethod, std::int8_t>(out, od::At(od::cia402::kHomingMethod), method);
  Put<std::uint32_t, std::uint32_t>(
    out, od::cia402::kHomingSpeeds_SpeedForSwitchSearch,
    speedForSwitchSearch);
  Put<std::uint32_t, std::uint32_t>(
    out, od::cia402::kHomingSpeeds_SpeedForZeroSearch,
    speedForZeroSearch);
  Put<std::uint32_t, std::uint32_t>(out, od::At(od::cia402::kHomingAcceleration), acceleration);
  Put<std::int32_t, std::int32_t>(out, od::At(od::maxon::kHomePosition), homePosition);
  Put<std::int32_t, std::int32_t>(
    out, od::At(
      od::maxon::kHomeOffsetMoveDistance), homeOffsetMoveDistance);
  Put<std::int16_t, std::int16_t>(
    out, od::At(
      od::maxon::kCurrentThresholdForHomingMode), currentThreshold);
}

void
StopOptionConfigs::AppendTo(ConfigWrites & out) const
{
  PutEnum<signals::QuickStopOption, std::int16_t>(
    out, od::At(
      od::cia402::kQuickStopOptionCode), quickStop);
  PutEnum<signals::ShutdownOption, std::int16_t>(
    out, od::At(
      od::cia402::kShutdownOptionCode), shutdown);
  PutEnum<signals::DisableOperationOption, std::int16_t>(
    out,
    od::At(od::cia402::kDisableOperationOptionCode), disableOperation);
  PutEnum<signals::FaultReactionOption, std::int16_t>(
    out,
    od::At(od::cia402::kFaultReactionOptionCode), faultReaction);
  PutEnum<signals::AbortConnectionOption, std::int16_t>(
    out,
    od::At(od::cia402::kAbortConnectionOptionCode), abortConnectionOption);
}

void
HoldingBrakeConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::uint16_t, std::uint16_t>(
    out, od::maxon::kHoldingBrakeParameters_HoldingBrakeRiseTime, couplingTimeMs);
  Put<std::uint16_t, std::uint16_t>(
    out, od::maxon::kHoldingBrakeParameters_HoldingBrakeFallTime, openingTimeMs);
  // Sub-indices 4 and 5 exist only on some hardware variants, so they are
  // addressed literally rather than through a generated constant the EDS of
  // other variants does not contain.
  Put<std::uint16_t, std::uint16_t>(
    out, od::At(od::maxon::kHoldingBrakeParameters, 4), openingVoltageDeciVolt);
  Put<std::uint16_t, std::uint16_t>(
    out, od::At(od::maxon::kHoldingBrakeParameters, 5), retainingVoltageDeciVolt);
}

void
StandstillConfigs::AppendTo(ConfigWrites & out) const
{
  Put<std::uint32_t, std::uint32_t>(
    out, od::maxon::kStandstillWindowConfiguration_StandstillWindow, window);
  Put<std::uint16_t, std::uint16_t>(
    out, od::maxon::kStandstillWindowConfiguration_StandstillWindowTime, windowTimeMs);
  Put<std::uint16_t, std::uint16_t>(
    out, od::maxon::kStandstillWindowConfiguration_StandstillWindowTimeout, windowTimeoutMs);
}

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
  return {};
}

void
DigitalInputConfigs::AppendTo(ConfigWrites & out) const
{
  // Polarity first: it decides how a pin's level is interpreted, so it should
  // be in force before a function starts acting on that pin.
  Put<std::uint16_t, std::uint16_t>(
    out, od::maxon::kDigitalInputProperties_DigitalInputsPolarity, polarity);

  const std::optional<signals::DigitalInputFunction> pins[] = {
    input1, input2, input3, input4,
    highSpeedInput1, highSpeedInput2, highSpeedInput3, highSpeedInput4};

  for (std::uint8_t i = 0; i < 8; ++i) {
    PutEnum<signals::DigitalInputFunction, std::uint8_t>(
      out, od::At(
        od::maxon::kConfigurationOfDigitalInputs,
        static_cast<std::uint8_t>(i + 1)), pins[i]);
  }
}

void
DigitalOutputConfigs::AppendTo(ConfigWrites & out) const
{
  PutEnum<signals::DigitalOutputFunction, std::uint8_t>(
    out, od::maxon::kConfigurationOfDigitalOutputs_DigitalOutput1Configuration, output1);
  PutEnum<signals::DigitalOutputFunction, std::uint8_t>(
    out, od::maxon::kConfigurationOfDigitalOutputs_DigitalOutput2Configuration, output2);
  PutEnum<signals::DigitalOutputFunction, std::uint8_t>(
    out, od::maxon::kConfigurationOfDigitalOutputs_HighSpeedDigitalOutput1Configuration,
    highSpeedOutput1);
  Put<std::uint16_t, std::uint16_t>(
    out, od::maxon::kDigitalOutputProperties_DigitalOutputsPolarity, polarity);
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

  return out;
}

}  // namespace epos4::configs
