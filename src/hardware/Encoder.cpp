#include "epos4/hardware/Encoder.hpp"

#include <utility>

#include "epos4/hardware/Epos4.hpp"

namespace epos4
{

namespace
{

// The manual repeats this for every encoder type word: "Write access is only
// permitted in device state «Power Disable»". These are the CiA 402 states in
// which no power reaches the motor.
bool
IsPowerDisabled(signals::State state)
{
  switch (state) {
    case signals::State::kNotReadyToSwitchOn:
    case signals::State::kSwitchOnDisabled:
    case signals::State::kReadyToSwitchOn:
    case signals::State::kFault:
      return true;
    case signals::State::kSwitchedOn:
    case signals::State::kOperationEnabled:
    case signals::State::kQuickStopActive:
    case signals::State::kFaultReactionActive:
      return false;
  }
  return false;
}

}  // namespace


Encoder::Encoder(Epos4 & device)
: device_(device)
{
  auto i32 = [this](std::uint16_t index, std::uint8_t sub) {
      return [this, index, sub](std::int32_t & out) {
               return device_.ReadObject(od::At(index, sub), out);
             };
    };
  auto u32 = [this](od::Entry entry) {
      return [this, entry](std::uint32_t & out) {
               return device_.ReadObject(entry, out);
             };
    };

  indexPosition1_ = signals::StatusSignal<std::int32_t>(
    i32(od::maxon::kDigitalIncrementalEncoder1, 4));
  indexPosition2_ = signals::StatusSignal<std::int32_t>(
    i32(od::maxon::kDigitalIncrementalEncoder2, 4));

  hallPattern_ = signals::StatusSignal<std::uint16_t>(
    [this](std::uint16_t & out) {
      return device_.ReadObject(
        od::maxon::kDigitalHallSensor_DigitalHallSensorPattern, out);
    });

  ssiRawPosition_ = signals::StatusSignal<std::uint32_t>(
    u32(od::maxon::kSSIAbsoluteEncoder_SSIPositionRawValue));
  mainSensorResolution_ = signals::StatusSignal<std::uint32_t>(
    u32(od::maxon::kAxisConfiguration_MainSensorResolution));
}

std::error_code
Encoder::ApplyWhilePowerDisabled(const configs::ConfigWrites & writes)
{
  if (writes.empty()) {
    return {};
  }

  auto & state = device_.GetState();
  state.Refresh();
  if (state.GetStatus()) {
    return state.GetStatus();
  }
  if (!IsPowerDisabled(state.GetValue())) {
    // Refuse up front rather than letting the drive abort each write with
    // "Wrong device state error" (0x08000022) one at a time, which would
    // leave the configuration half applied.
    return std::make_error_code(std::errc::operation_not_permitted);
  }

  for (const auto & w : writes) {
    const std::error_code ec = std::visit(
      [&](auto value) {return device_.WriteObject(w.entry, value);}, w.value);
    if (ec) {
      return ec;
    }
  }
  return {};
}

#define EPOS4_ENCODER_APPLY(Type) \
  std::error_code Encoder::Apply(const configs::Type & config) \
  { \
    configs::ConfigWrites writes; \
    config.AppendTo(writes); \
    return ApplyWhilePowerDisabled(writes); \
  }

EPOS4_ENCODER_APPLY(SensorsConfigs)
EPOS4_ENCODER_APPLY(DigitalIncrementalEncoderConfigs)
EPOS4_ENCODER_APPLY(AnalogIncrementalEncoderConfigs)
EPOS4_ENCODER_APPLY(SsiAbsoluteEncoderConfigs)
EPOS4_ENCODER_APPLY(HallSensorConfigs)

#undef EPOS4_ENCODER_APPLY

std::error_code
Encoder::ReadSensorsConfiguration(configs::SensorsConfigs & out)
{
  std::uint32_t raw{};
  if (auto ec = device_.ReadObject(
      od::maxon::kAxisConfiguration_SensorsConfiguration, raw))
  {
    return ec;
  }
  out = configs::SensorsConfigs::Decode(raw);

  std::uint32_t resolution{};
  if (!device_.ReadObject(od::maxon::kAxisConfiguration_MainSensorResolution, resolution)) {
    out.mainSensorResolution = resolution;
  }
  return {};
}

signals::StatusSignal<std::int32_t> &
Encoder::GetPosition()
{
  return device_.GetPosition();
}

signals::StatusSignal<std::int32_t> &
Encoder::GetVelocity()
{
  return device_.GetVelocity();
}

signals::StatusSignal<std::int32_t> &
Encoder::GetIndexPosition(std::uint8_t encoderNumber)
{
  return (encoderNumber == 2) ? indexPosition2_ : indexPosition1_;
}

signals::StatusSignal<std::uint16_t> &
Encoder::GetHallPattern()
{
  return hallPattern_;
}

signals::StatusSignal<std::uint32_t> &
Encoder::GetSsiRawPosition()
{
  return ssiRawPosition_;
}

signals::StatusSignal<std::uint32_t> &
Encoder::GetMainSensorResolution()
{
  return mainSensorResolution_;
}

}  // namespace epos4

namespace epos4
{

void
Encoder::SetMechanism(std::uint32_t quadCountsPerRevolution, double gearRatio)
{
  converter_ = MechanismScale{quadCountsPerRevolution, gearRatio};
}

std::error_code
Encoder::SetMechanismFromDevice(double gearRatio)
{
  auto & resolution = GetMainSensorResolution();
  resolution.Refresh();
  if (resolution.GetStatus()) {
    return resolution.GetStatus();
  }
  if (resolution.GetValue() == 0) {
    // The drive reports zero until the encoder has been configured. Taking it
    // would make every later conversion divide by zero.
    return std::make_error_code(std::errc::invalid_argument);
  }
  converter_ = MechanismScale{resolution.GetValue(), gearRatio};
  return {};
}

std::optional<units::angle::turn_t>
Encoder::GetAngle()
{
  if (!converter_.IsValid()) {
    return std::nullopt;
  }
  auto & position = GetPosition();
  position.Refresh();
  if (position.GetStatus()) {
    return std::nullopt;
  }
  return converter_.ToAngle(position.GetValue());
}

std::optional<units::angular_velocity::revolutions_per_minute_t>
Encoder::GetAngularVelocity()
{
  if (!converter_.IsValid()) {
    return std::nullopt;
  }
  auto & velocity = GetVelocity();
  velocity.Refresh();
  if (velocity.GetStatus()) {
    return std::nullopt;
  }
  return converter_.ToAngularVelocity(velocity.GetValue());
}

}  // namespace epos4
