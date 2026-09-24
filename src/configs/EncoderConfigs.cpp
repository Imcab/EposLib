#include "epos4/configs/EncoderConfigs.hpp"

namespace epos4::configs
{

namespace
{
template<typename T>
std::uint32_t Byte(const std::optional<T> & field)
{
  return field ? static_cast<std::uint32_t>(static_cast<std::uint8_t>(*field)) : 0u;
}
}  // namespace


// --- Sensors configuration, 0x3000:01, Table 6-102 -------------------------

std::uint32_t
SensorsConfigs::Encode() const
{
  return (Byte(sensor3) << 16) | (Byte(sensor2) << 8) | Byte(sensor1);
}

SensorsConfigs
SensorsConfigs::Decode(std::uint32_t value)
{
  SensorsConfigs out;
  out.sensor1 = static_cast<Sensor1Type>(value & 0xFFu);
  out.sensor2 = static_cast<Sensor2Type>((value >> 8) & 0xFFu);
  out.sensor3 = static_cast<Sensor3Type>((value >> 16) & 0xFFu);
  return out;
}

void
SensorsConfigs::AppendTo(ConfigWrites & out) const
{
  if (sensor1 || sensor2 || sensor3) {
    out.push_back(
      ConfigWrite{od::maxon::kAxisConfiguration_SensorsConfiguration, Encode()});
  }
  if (mainSensorResolution) {
    out.push_back(
      ConfigWrite{od::maxon::kAxisConfiguration_MainSensorResolution, *mainSensorResolution});
  }
}


// --- Digital incremental encoder type, 0x3010:02, Table 6-117 --------------

std::uint16_t
IncrementalEncoderType::Encode() const
{
  std::uint16_t value = static_cast<std::uint16_t>(index) & 0x0003u;
  value = static_cast<std::uint16_t>(
    value | ((static_cast<std::uint16_t>(direction) & 0x1u) << 4));
  value = static_cast<std::uint16_t>(
    value | ((static_cast<std::uint16_t>(method) & 0x1u) << 9));
  return value;
}

IncrementalEncoderType
IncrementalEncoderType::Decode(std::uint16_t value)
{
  IncrementalEncoderType out;
  out.index = static_cast<IndexType>(value & 0x0003u);
  out.direction = static_cast<EncoderDirection>((value >> 4) & 0x1u);
  out.method = static_cast<SpeedMeasurementMethod>((value >> 9) & 0x1u);
  return out;
}

void
DigitalIncrementalEncoderConfigs::AppendTo(ConfigWrites & out) const
{
  // Encoder 1 lives at 0x3010, encoder 2 at 0x3020; the sub-index layout is
  // identical, which is why one struct covers both.
  const std::uint16_t base =
    (encoderNumber == 2) ? od::maxon::kDigitalIncrementalEncoder2 :
    od::maxon::kDigitalIncrementalEncoder1;

  if (pulsesPerRevolution) {
    out.push_back(ConfigWrite{od::At(base, 1), *pulsesPerRevolution});
  }
  if (type) {
    out.push_back(ConfigWrite{od::At(base, 2), type->Encode()});
  }
}


// --- Analog incremental encoder, 0x3011, Table 6-118 -----------------------

std::uint16_t
AnalogIncrementalEncoderType::Encode() const
{
  std::uint16_t value = withIndex ? 1u : 0u;
  value = static_cast<std::uint16_t>(
    value | ((static_cast<std::uint16_t>(direction) & 0x1u) << 4));
  return value;
}

AnalogIncrementalEncoderType
AnalogIncrementalEncoderType::Decode(std::uint16_t value)
{
  AnalogIncrementalEncoderType out;
  out.withIndex = (value & 0x1u) != 0;
  out.direction = static_cast<EncoderDirection>((value >> 4) & 0x1u);
  return out;
}

void
AnalogIncrementalEncoderConfigs::AppendTo(ConfigWrites & out) const
{
  if (type) {
    out.push_back(
      ConfigWrite{od::maxon::kAnalogIncrementalEncoder_AnalogIncrementalEncoderType,
        type->Encode()});
  }
  if (periodsPerTurn || interpolationBits) {
    // Bits 31..8 periods per turn, bits 7..0 interpolation bits. Defaults
    // taken from the object's own default value 0x00080004: 8 periods,
    // 4 interpolation bits.
    const std::uint32_t periods = periodsPerTurn.value_or(8u);
    const std::uint32_t bits = interpolationBits.value_or(4u);
    out.push_back(
      ConfigWrite{od::maxon::kAnalogIncrementalEncoder_AnalogIncrementalEncoderResolution,
        static_cast<std::uint32_t>((periods << 8) | (bits & 0xFFu))});
  }
}


// --- SSI absolute encoder, 0x3012, Tables 6-120 and 6-121 ------------------

std::uint16_t
SsiEncodingType::Encode() const
{
  std::uint16_t value = static_cast<std::uint16_t>(encoding) & 0x000Fu;
  value = static_cast<std::uint16_t>(
    value | ((static_cast<std::uint16_t>(direction) & 0x1u) << 4));
  value = static_cast<std::uint16_t>(value | ((checkFrame ? 1u : 0u) << 8));
  value = static_cast<std::uint16_t>(
    value | ((resetReferenceOnFrameError ? 1u : 0u) << 9));
  return value;
}

SsiEncodingType
SsiEncodingType::Decode(std::uint16_t value)
{
  SsiEncodingType out;
  out.encoding = static_cast<SsiEncoding>(value & 0x000Fu);
  out.direction = static_cast<EncoderDirection>((value >> 4) & 0x1u);
  out.checkFrame = ((value >> 8) & 0x1u) != 0;
  out.resetReferenceOnFrameError = ((value >> 9) & 0x1u) != 0;
  return out;
}

std::optional<std::uint32_t>
SsiAbsoluteEncoderConfigs::EncodeDataBits() const
{
  if (!specialBitsLeading && !multiTurnBits && !singleTurnBits && !specialBitsTrailing) {
    return std::nullopt;
  }
  // Defaults from the object's own default 0x00000C00: 12 single-turn bits,
  // everything else zero.
  const std::uint32_t leading = specialBitsLeading.value_or(0u);
  const std::uint32_t multi = multiTurnBits.value_or(0u);
  const std::uint32_t single = singleTurnBits.value_or(12u);
  const std::uint32_t trailing = specialBitsTrailing.value_or(0u);
  return (leading << 24) | (multi << 16) | (single << 8) | trailing;
}

void
SsiAbsoluteEncoderConfigs::AppendTo(ConfigWrites & out) const
{
  if (dataRateKbitPerSecond) {
    out.push_back(
      ConfigWrite{od::maxon::kSSIAbsoluteEncoder_SSIDataRate, *dataRateKbitPerSecond});
  }
  if (const auto bits = EncodeDataBits()) {
    out.push_back(
      ConfigWrite{od::maxon::kSSIAbsoluteEncoder_SSINumberOfDataBits, *bits});
  }
  if (encodingType) {
    out.push_back(
      ConfigWrite{od::maxon::kSSIAbsoluteEncoder_SSIEncodingType, encodingType->Encode()});
  }
  if (timeoutTimeUs) {
    out.push_back(
      ConfigWrite{od::maxon::kSSIAbsoluteEncoder_SSITimeoutTime, *timeoutTimeUs});
  }
  if (refreshFrequency) {
    out.push_back(
      ConfigWrite{od::maxon::kSSIAbsoluteEncoder_SSIRefreshFrequency, *refreshFrequency});
  }
  if (powerUpTimeMs) {
    out.push_back(
      ConfigWrite{od::maxon::kSSIAbsoluteEncoder_SSIPowerUpTime, *powerUpTimeMs});
  }
}


// --- Digital Hall sensor, 0x301A, Table 6-123 ------------------------------

std::uint16_t
HallSensorType::Encode() const
{
  // Note the layout differs from the incremental encoder's word: polarity is
  // bit 0 and the method is bit 4, not bits 4 and 9.
  std::uint16_t value = static_cast<std::uint16_t>(polarity) & 0x1u;
  value = static_cast<std::uint16_t>(
    value | ((static_cast<std::uint16_t>(method) & 0x1u) << 4));
  return value;
}

HallSensorType
HallSensorType::Decode(std::uint16_t value)
{
  HallSensorType out;
  out.polarity = static_cast<EncoderDirection>(value & 0x1u);
  out.method = static_cast<SpeedMeasurementMethod>((value >> 4) & 0x1u);
  return out;
}

void
HallSensorConfigs::AppendTo(ConfigWrites & out) const
{
  if (type) {
    out.push_back(
      ConfigWrite{od::maxon::kDigitalHallSensor_DigitalHallSensorType, type->Encode()});
  }
}

}  // namespace epos4::configs
