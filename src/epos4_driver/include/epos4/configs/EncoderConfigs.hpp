#pragma once

#include <cstdint>
#include <optional>

#include "epos4/configs/Configs.hpp"

namespace epos4::configs
{

// ---------------------------------------------------------------------------
// Feedback configuration, sections 6.2.52 and 6.2.56 to 6.2.60.
//
// TWO THINGS THAT WILL BITE, both straight from the manual:
//
//  1. "Write access is only permitted in device state «Power Disable»."
//     Every encoder type word below refuses to be written while the drive is
//     enabled. Encoder::Apply checks the state first and returns
//     std::errc::operation_not_permitted rather than letting the drive abort
//     each write one by one.
//
//  2. "Upon changing this parameter, the absolute position may be corrupted.
//     Therefore, «Position referenced to home position», Position actual
//     value, and Additional position actual values will be cleared."
//     Changing the sensor layout loses homing. An arm that reconfigures its
//     encoders at startup and then moves to a stored pose goes somewhere
//     else entirely.
// ---------------------------------------------------------------------------


// Which physical sensor slot. The slot constrains the type: the manual only
// allows certain sensors in certain slots (Table 6-103).
enum class SensorSlot : std::uint8_t
{
  kSensor1 = 1,  // none, or digital incremental encoder 1
  kSensor2 = 2,  // none, digital incremental encoder 2, analog SinCos, or SSI
  kSensor3 = 3   // none, or digital Hall (EC motors only)
};

// Values for each slot's byte in 0x3000:01, Table 6-103.
enum class Sensor1Type : std::uint8_t
{
  kNone = 0x00,
  kDigitalIncrementalEncoder1 = 0x01
};

enum class Sensor2Type : std::uint8_t
{
  kNone = 0x00,
  // Not available on EPOS4 Disk 60/8, Disk 60/12, Micro 24/1.5, Micro 24/5.
  kDigitalIncrementalEncoder2 = 0x01,
  kAnalogIncrementalSinCos = 0x02,
  kSsiAbsoluteEncoder = 0x03
};

enum class Sensor3Type : std::uint8_t
{
  kNone = 0x00,
  kDigitalHallSensor = 0x10  // EC motors only
};


// Object 0x3000:01, packed as Table 6-102:
//   bits 31..24 reserved, 23..16 sensor 3, 15..8 sensor 2, 7..0 sensor 1
struct SensorsConfigs
{
  std::optional<Sensor1Type> sensor1;
  std::optional<Sensor2Type> sensor2;
  std::optional<Sensor3Type> sensor3;

  // Main sensor resolution, 0x3000:05 [quadcounts/revolution]. Read back
  // after configuring an encoder to confirm the drive agrees with the count
  // the maths below produces.
  std::optional<std::uint32_t> mainSensorResolution;

  // The three slots share one object, so they are written as one word.
  // Unset slots are encoded as kNone, which is why this whole struct is
  // all-or-nothing rather than per-field.
  std::uint32_t Encode() const;
  static SensorsConfigs Decode(std::uint32_t value);

  void AppendTo(ConfigWrites & out) const;
};


// ---------------------------------------------------------------------------
// Shared type-word fields
// ---------------------------------------------------------------------------

enum class EncoderDirection : std::uint8_t
{
  kMaxon = 0,
  kInverted = 1  // also: encoder mounted on the motor shaft
};

enum class SpeedMeasurementMethod : std::uint8_t
{
  kTimeBetweenEdges = 0,   // better at low speed
  kEdgesPerControlCycle = 1  // better at high speed
};

enum class IndexType : std::uint8_t
{
  kNoIndex = 0,                 // 2-channel
  kWithIndex = 1,               // 3-channel
  kWithIndexNoSupervision = 3   // 3-channel, index not supervised
};


// Object 0x3010:02 / 0x3020:02, Table 6-117.
//
// A bitfield, so it is set as a whole rather than field by field: the drive
// has one word and a partial write would have to invent the rest.
struct IncrementalEncoderType
{
  IndexType index{IndexType::kWithIndex};
  EncoderDirection direction{EncoderDirection::kMaxon};
  SpeedMeasurementMethod method{SpeedMeasurementMethod::kTimeBetweenEdges};

  std::uint16_t Encode() const;
  static IncrementalEncoderType Decode(std::uint16_t value);
};


// Digital incremental encoder, 0x3010 (encoder 1) or 0x3020 (encoder 2).
struct DigitalIncrementalEncoderConfigs
{
  // Which of the two objects this configures. Encoder 1 belongs in sensor
  // slot 1, encoder 2 in slot 2.
  std::uint8_t encoderNumber{1};

  // 0x3010:01 [pulses/revolution], 16 to 2'500'000.
  //
  // Careful with units. The manual's conversion is
  //     4 x pulses/rev = increments/rev = quadcounts/rev
  // so an encoder sold as "500 CPR" is 500 here and 2000 quadcounts per
  // revolution everywhere position is reported. Getting this wrong scales
  // every move by four.
  std::optional<std::uint32_t> pulsesPerRevolution;

  std::optional<IncrementalEncoderType> type;  // 0x3010:02

  void AppendTo(ConfigWrites & out) const;
};


// Object 0x3011:01, Table 6-118. Fewer fields than the digital one: no speed
// measurement method, and the index is a single bit.
struct AnalogIncrementalEncoderType
{
  bool withIndex{false};
  EncoderDirection direction{EncoderDirection::kMaxon};

  std::uint16_t Encode() const;
  static AnalogIncrementalEncoderType Decode(std::uint16_t value);
};


// Analog incremental encoder SinCos, 0x3011.
struct AnalogIncrementalEncoderConfigs
{
  std::optional<AnalogIncrementalEncoderType> type;  // 0x3011:01

  // 0x3011:02, packed: bits 31..8 periods per turn, bits 7..0 interpolation
  // bits. Resolution = 2^interpolationBits x periodsPerTurn [inc/rev], and
  // the manual bounds that product to 64 .. 10'000'000.
  std::optional<std::uint32_t> periodsPerTurn;
  std::optional<std::uint8_t> interpolationBits;

  void AppendTo(ConfigWrites & out) const;
};


enum class SsiEncoding : std::uint8_t
{
  kBinary = 0,
  kGray = 1
};


// Object 0x3012:03, Table 6-121.
struct SsiEncodingType
{
  SsiEncoding encoding{SsiEncoding::kBinary};
  EncoderDirection direction{EncoderDirection::kMaxon};
  bool checkFrame{false};      // bit 8: frame start and end bit checking
  bool resetReferenceOnFrameError{false};  // bit 9

  std::uint16_t Encode() const;
  static SsiEncodingType Decode(std::uint16_t value);
};


// SSI absolute encoder, 0x3012.
struct SsiAbsoluteEncoderConfigs
{
  // 0x3012:01 [kbit/s], 400 to 2000.
  std::optional<std::uint16_t> dataRateKbitPerSecond;

  // 0x3012:02, packed as Table 6-120:
  //   31..24 special bits leading (0..16)
  //   23..16 multi-turn bits      (0..32)
  //   15..8  single-turn bits     (6..32)
  //   7..0   special bits trailing(0..16)
  // The four together must not exceed 62 bits.
  std::optional<std::uint8_t> specialBitsLeading;
  std::optional<std::uint8_t> multiTurnBits;
  std::optional<std::uint8_t> singleTurnBits;
  std::optional<std::uint8_t> specialBitsTrailing;

  std::optional<SsiEncodingType> encodingType;  // 0x3012:03
  std::optional<std::uint16_t> timeoutTimeUs;   // 0x3012:05
  std::optional<std::uint16_t> refreshFrequency;  // 0x3012:07
  std::optional<std::uint16_t> powerUpTimeMs;   // 0x3012:08

  // The data-bit fields share one object, so they are encoded together.
  // Returns nullopt when none of them were set.
  std::optional<std::uint32_t> EncodeDataBits() const;

  void AppendTo(ConfigWrites & out) const;
};


// Object 0x301A:01, Table 6-123. Note the bit positions differ from the
// incremental encoder's word: polarity is bit 0, not bit 4, and the method is
// bit 4, not bit 9. Reusing the incremental encoding here would silently
// invert the sensor.
struct HallSensorType
{
  EncoderDirection polarity{EncoderDirection::kMaxon};
  SpeedMeasurementMethod method{SpeedMeasurementMethod::kTimeBetweenEdges};

  std::uint16_t Encode() const;
  static HallSensorType Decode(std::uint16_t value);
};


// Digital Hall sensor, 0x301A. EC (brushless) motors only: with a brushed DC
// motor the manual says the Hall field is forced to "none".
struct HallSensorConfigs
{
  std::optional<HallSensorType> type;  // 0x301A:01

  void AppendTo(ConfigWrites & out) const;
};

}  // namespace epos4::configs
