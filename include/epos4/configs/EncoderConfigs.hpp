#pragma once

#include <cstdint>
#include <optional>
#include <system_error>

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

  // Main sensor resolution, 0x3000:05 [quadcounts/revolution]. Read-only:
  // the drive derives it from the sensor settings, so Apply() never writes
  // it (a write aborts). Read back after configuring an encoder to confirm
  // the drive agrees with the count the maths below produces.
  std::optional<std::uint32_t> mainSensorResolution;

  // The three slots share one object, so they are written as one word.
  // Unset slots are encoded as kNone, which is why this whole struct is
  // all-or-nothing rather than per-field.
  std::uint32_t Encode() const;
  static SensorsConfigs Decode(std::uint32_t value);

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v.template Composite<std::uint32_t>(
      od::maxon::kAxisConfiguration_SensorsConfiguration,
      [&s]() -> std::optional<std::uint32_t> {
        if (!s.sensor1 && !s.sensor2 && !s.sensor3) {return std::nullopt;}
        return s.Encode();
      },
      [&s](auto word) {
        const SensorsConfigs d = Decode(word);
        s.sensor1 = d.sensor1;
        s.sensor2 = d.sensor2;
        s.sensor3 = d.sensor3;
      });
    v.ReadOnly(od::maxon::kAxisConfiguration_MainSensorResolution, s.mainSensorResolution);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
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

  // 0x3010:04 / 0x3020:04 [inc], read-only: where the drive last saw the
  // index pulse. Refresh() reports it; nothing writes it.
  std::optional<std::int32_t> indexPosition;

  // Encoder 1 lives at 0x3010, encoder 2 at 0x3020; the sub-index layout is
  // identical, which is why one struct covers both.
  std::uint16_t Object() const
  {
    return encoderNumber == 2 ? od::maxon::kDigitalIncrementalEncoder2 :
           od::maxon::kDigitalIncrementalEncoder1;
  }

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::At(s.Object(), 1), s.pulsesPerRevolution);
    v.template Packed<SelfCodec<IncrementalEncoderType, std::uint16_t>>(
      od::At(
        s.Object(), 2), s.type);
    v.ReadOnly(od::At(s.Object(), 4), s.indexPosition);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


// Object 0x3011:01, Table 6-118. Fewer fields than the digital one: no speed
// measurement method, and the index is a single bit.
struct AnalogIncrementalEncoderType
{
  bool withIndex{true};  // the drive's default, 0x0001
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

  // 0x3011:03 [inc], read-only: where the drive last saw the index.
  std::optional<std::int32_t> indexPosition;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v.template Packed<SelfCodec<AnalogIncrementalEncoderType, std::uint16_t>>(
      od::maxon::kAnalogIncrementalEncoder_AnalogIncrementalEncoderType, s.type);
    // A half-set pair takes the other half from the object's default
    // 0x00080004 - bits 31..8 are 0x000800 = 2048 periods (Table 6-119),
    // bits 7..0 are 4 interpolation bits.
    v.template Composite<std::uint32_t>(
      od::maxon::kAnalogIncrementalEncoder_AnalogIncrementalEncoderResolution,
      [&s]() -> std::optional<std::uint32_t> {
        if (!s.periodsPerTurn && !s.interpolationBits) {return std::nullopt;}
        return (s.periodsPerTurn.value_or(2048u) << 8) | s.interpolationBits.value_or(4u);
      },
      [&s](auto word) {
        s.periodsPerTurn = word >> 8;
        s.interpolationBits = static_cast<std::uint8_t>(word & 0xFFu);
      });
    v.ReadOnly(
      od::maxon::kAnalogIncrementalEncoder_AnalogIncrementalEncoderIndexPosition,
      s.indexPosition);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};


enum class SsiEncoding : std::uint8_t
{
  kBinary = 0,
  kGray = 1
};


// Object 0x3012:03, Table 6-121.
struct SsiEncodingType
{
  SsiEncoding encoding{SsiEncoding::kGray};  // the drive's default, 0x001
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
  std::optional<std::uint16_t> powerUpTimeMs;   // 0x3012:08

  // 0x3012:0A [inc]. Aligns the encoder's zero with the motor's 0 degree
  // commutation angle, 0..resolution for 0..360 degrees. maxon encoders come
  // aligned; a third-party one on an EC motor needs it (6.2.58.9).
  std::optional<std::uint32_t> commutationOffset;

  // 0x3012:0B, Table 6-122: how many of the frame's bits the position uses,
  // multi-turn in bits 15..8, single-turn in 7..0. The position is 32 bits
  // at most, so a long multi-turn count has to be cut here. Resolution is
  // 2^singleTurn inc/rev, and the velocity is computed from it.
  std::optional<std::uint8_t> positionMultiTurnBits;
  std::optional<std::uint8_t> positionSingleTurnBits;

  // 0x3012:0E [0.001 ms], -1..1000. Extrapolates the single-turn position
  // for commutation over the encoder's delay; -1 (default) turns it off.
  // Documented, but beyond the object's own highest sub-index (13) and not
  // in the EDS: present only on firmware that has it.
  std::optional<std::int32_t> additionalDelay;

  // 0x3012:07 [Hz], read-only: how often the drive actually reads the
  // encoder - the result of the data rate and frame length above.
  std::optional<std::uint32_t> refreshFrequency;

  // The data-bit fields share one object, so they are encoded together.
  // Returns nullopt when none of them were set.
  std::optional<std::uint32_t> EncodeDataBits() const;
  std::optional<std::uint32_t> EncodePositionBits() const;

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v(od::maxon::kSSIAbsoluteEncoder_SSIDataRate, s.dataRateKbitPerSecond);
    v.template Composite<std::uint32_t>(
      od::maxon::kSSIAbsoluteEncoder_SSINumberOfDataBits,
      [&s] {return s.EncodeDataBits();},
      [&s](auto word) {
        s.specialBitsLeading = static_cast<std::uint8_t>(word >> 24);
        s.multiTurnBits = static_cast<std::uint8_t>(word >> 16);
        s.singleTurnBits = static_cast<std::uint8_t>(word >> 8);
        s.specialBitsTrailing = static_cast<std::uint8_t>(word);
      });
    v.template Packed<SelfCodec<SsiEncodingType, std::uint16_t>>(
      od::maxon::kSSIAbsoluteEncoder_SSIEncodingType, s.encodingType);
    v(od::maxon::kSSIAbsoluteEncoder_SSITimeoutTime, s.timeoutTimeUs);
    v(od::maxon::kSSIAbsoluteEncoder_SSIPowerUpTime, s.powerUpTimeMs);
    v(od::maxon::kSSIAbsoluteEncoder_SSICommutationOffsetValue, s.commutationOffset);
    v.template Composite<std::uint32_t>(
      od::maxon::kSSIAbsoluteEncoder_SSIPositionBits,
      [&s] {return s.EncodePositionBits();},
      [&s](auto word) {
        s.positionMultiTurnBits = static_cast<std::uint8_t>(word >> 8);
        s.positionSingleTurnBits = static_cast<std::uint8_t>(word);
      });
    v(
      od::At(od::maxon::kSSIAbsoluteEncoder, 0x0E), s.additionalDelay,
      Presence::kFirmwareDependent);
    v.ReadOnly(od::maxon::kSSIAbsoluteEncoder_SSIRefreshFrequency, s.refreshFrequency);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
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

  // The fields and the objects they live in; see ConfigFields.hpp.
  template<typename Self, typename V>
  static void Visit(Self & s, V & v)
  {
    v.template Packed<SelfCodec<HallSensorType, std::uint16_t>>(
      od::maxon::kDigitalHallSensor_DigitalHallSensorType, s.type);
  }

  void AppendTo(ConfigWrites & out) const {WriteFields(*this, out);}
  std::error_code ReadFrom(const ConfigReader & read) {return ReadFields(*this, read);}
};

}  // namespace epos4::configs
