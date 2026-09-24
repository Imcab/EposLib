// Tests for the feedback bitfields.
//
// Every encoder "type" object is a packed word, and the packing differs
// between sensor types. A bit in the wrong place does not fail loudly: it
// inverts a direction, disables index supervision, or changes how speed is
// measured, and the axis simply misbehaves. These tests pin each layout to
// the table it came from.

#include <gtest/gtest.h>

#include "epos4/configs/EncoderConfigs.hpp"
#include "epos4/hardware/Encoder.hpp"

using namespace epos4::configs;

// Table 6-102: bits 23..16 sensor 3, 15..8 sensor 2, 7..0 sensor 1.
TEST(SensorsConfig, PacksSlotsIntoTheDocumentedBytes)
{
  SensorsConfigs config;
  config.sensor1 = Sensor1Type::kDigitalIncrementalEncoder1;  // 0x01
  config.sensor2 = Sensor2Type::kSsiAbsoluteEncoder;          // 0x03
  config.sensor3 = Sensor3Type::kDigitalHallSensor;           // 0x10

  EXPECT_EQ(config.Encode(), 0x00100301u);
}

// The object's own default is 0x00100001: incremental encoder 1 on slot 1,
// nothing on slot 2, Hall on slot 3.
TEST(SensorsConfig, MatchesTheDocumentedDefault)
{
  SensorsConfigs config;
  config.sensor1 = Sensor1Type::kDigitalIncrementalEncoder1;
  config.sensor2 = Sensor2Type::kNone;
  config.sensor3 = Sensor3Type::kDigitalHallSensor;

  EXPECT_EQ(config.Encode(), 0x00100001u);
}

TEST(SensorsConfig, RoundTripsThroughDecode)
{
  const auto decoded = SensorsConfigs::Decode(0x00100301u);
  EXPECT_EQ(*decoded.sensor1, Sensor1Type::kDigitalIncrementalEncoder1);
  EXPECT_EQ(*decoded.sensor2, Sensor2Type::kSsiAbsoluteEncoder);
  EXPECT_EQ(*decoded.sensor3, Sensor3Type::kDigitalHallSensor);
  EXPECT_EQ(decoded.Encode(), 0x00100301u);
}

// Table 6-117: bits 1..0 index, bit 4 direction, bit 9 method.
TEST(IncrementalEncoderTypeWord, MatchesTable6_117)
{
  IncrementalEncoderType type;
  type.index = IndexType::kWithIndex;                       // 0b01
  type.direction = EncoderDirection::kMaxon;                // bit 4 = 0
  type.method = SpeedMeasurementMethod::kTimeBetweenEdges;  // bit 9 = 0
  EXPECT_EQ(type.Encode(), 0x0001u);  // the object's documented default

  type.direction = EncoderDirection::kInverted;
  EXPECT_EQ(type.Encode(), 0x0011u);

  type.method = SpeedMeasurementMethod::kEdgesPerControlCycle;
  EXPECT_EQ(type.Encode(), 0x0211u);

  type.index = IndexType::kWithIndexNoSupervision;  // 0b11
  EXPECT_EQ(type.Encode(), 0x0213u);

  type.index = IndexType::kNoIndex;
  EXPECT_EQ(type.Encode(), 0x0210u);
}

TEST(IncrementalEncoderTypeWord, RoundTrips)
{
  for (std::uint16_t raw : {0x0000u, 0x0001u, 0x0011u, 0x0211u, 0x0213u}) {
    EXPECT_EQ(IncrementalEncoderType::Decode(raw).Encode(), raw) << std::hex << raw;
  }
}

// Table 6-118: bit 0 index, bit 4 direction. No method bit - the analog
// encoder's word is NOT the digital one's, and reusing it would set bit 9 on
// a device that reserves it.
TEST(AnalogEncoderTypeWord, MatchesTable6_118)
{
  AnalogIncrementalEncoderType type;
  type.withIndex = true;
  type.direction = EncoderDirection::kMaxon;
  EXPECT_EQ(type.Encode(), 0x0001u);  // documented default

  type.direction = EncoderDirection::kInverted;
  EXPECT_EQ(type.Encode(), 0x0011u);

  type.withIndex = false;
  EXPECT_EQ(type.Encode(), 0x0010u);
}

// Table 6-123. The trap: polarity is bit 0 and the method is bit 4, the
// mirror image of the incremental encoder's layout. Using the incremental
// encoding here would invert the sensor while looking correct.
TEST(HallSensorTypeWord, UsesItsOwnLayoutNotTheIncrementalOne)
{
  HallSensorType hall;
  hall.polarity = EncoderDirection::kMaxon;
  hall.method = SpeedMeasurementMethod::kTimeBetweenEdges;
  EXPECT_EQ(hall.Encode(), 0x0000u);  // documented default

  hall.polarity = EncoderDirection::kInverted;
  EXPECT_EQ(hall.Encode(), 0x0001u) << "polarity must be bit 0, not bit 4";

  hall.polarity = EncoderDirection::kMaxon;
  hall.method = SpeedMeasurementMethod::kEdgesPerControlCycle;
  EXPECT_EQ(hall.Encode(), 0x0010u) << "method must be bit 4, not bit 9";

  // And the two layouts must not agree, or one of them is wrong.
  IncrementalEncoderType inc;
  inc.direction = EncoderDirection::kInverted;
  inc.index = IndexType::kNoIndex;
  EXPECT_NE(inc.Encode(), 0x0001u);
}

// Table 6-121: bits 3..0 encoding, bit 4 direction, bit 8 check frame,
// bit 9 reference reset.
TEST(SsiEncodingTypeWord, MatchesTable6_121)
{
  SsiEncodingType ssi;
  ssi.encoding = SsiEncoding::kGray;
  EXPECT_EQ(ssi.Encode(), 0x0001u);  // documented default

  ssi.direction = EncoderDirection::kInverted;
  EXPECT_EQ(ssi.Encode(), 0x0011u);

  ssi.checkFrame = true;
  EXPECT_EQ(ssi.Encode(), 0x0111u);

  ssi.resetReferenceOnFrameError = true;
  EXPECT_EQ(ssi.Encode(), 0x0311u);

  EXPECT_EQ(SsiEncodingType::Decode(0x0311u).Encode(), 0x0311u);
}

// Table 6-120: 31..24 leading, 23..16 multi-turn, 15..8 single-turn,
// 7..0 trailing.
TEST(SsiDataBits, PackInTheDocumentedOrder)
{
  SsiAbsoluteEncoderConfigs ssi;
  ssi.singleTurnBits = 12;
  EXPECT_EQ(*ssi.EncodeDataBits(), 0x00000C00u);  // documented default

  ssi.multiTurnBits = 12;
  EXPECT_EQ(*ssi.EncodeDataBits(), 0x000C0C00u);

  ssi.specialBitsLeading = 1;
  ssi.specialBitsTrailing = 2;
  EXPECT_EQ(*ssi.EncodeDataBits(), 0x010C0C02u);
}

TEST(SsiDataBits, NothingSetWritesNothing)
{
  SsiAbsoluteEncoderConfigs ssi;
  EXPECT_FALSE(ssi.EncodeDataBits().has_value());

  ConfigWrites writes;
  ssi.AppendTo(writes);
  EXPECT_TRUE(writes.empty());
}

// The factor of four between the number a vendor prints on an encoder and
// the counts the drive reports. Getting it wrong scales every move.
TEST(UnitHelpers, QuadCountsAreFourTimesThePulses)
{
  EXPECT_EQ(epos4::Encoder::QuadCountsPerRevolution(500), 2000u);
  EXPECT_EQ(epos4::Encoder::QuadCountsPerRevolution(1024), 4096u);
}

TEST(UnitHelpers, SinCosResolutionIsPeriodsTimesTwoToTheBits)
{
  // The object's default, 0x00080004: 8 periods, 4 interpolation bits.
  EXPECT_EQ(epos4::Encoder::SinCosResolution(8, 4), 128u);
  EXPECT_EQ(epos4::Encoder::SinCosResolution(1024, 10), 1048576u);
}

TEST(DigitalIncrementalEncoder, Encoder2TargetsTheOtherObject)
{
  DigitalIncrementalEncoderConfigs enc;
  enc.encoderNumber = 2;
  enc.pulsesPerRevolution = 1024;

  ConfigWrites writes;
  enc.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(writes[0].entry.index, 0x3020) << "encoder 2 must not write 0x3010";

  enc.encoderNumber = 1;
  ConfigWrites first;
  enc.AppendTo(first);
  EXPECT_EQ(first[0].entry.index, 0x3010);
}
