// Refresh() and Apply() come from one list of fields per group. These tests
// hold that promise: whatever a group reads back from a drive, it writes back
// to the same objects with the same values - so no field can be written to
// one object and read from another, or written and never read.
//
// The "drive" is a map of objects, each answering with a distinctive value
// of the type the field asks for.

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <set>
#include <system_error>
#include <utility>

#include "epos4/configs/Configs.hpp"
#include "epos4/configs/EncoderConfigs.hpp"

using namespace epos4;           // NOLINT(build/namespaces)
using namespace epos4::configs;  // NOLINT(build/namespaces)

namespace
{

using Key = std::pair<std::uint16_t, std::uint8_t>;

Key KeyOf(od::Entry entry) {return {entry.index, entry.subindex};}

// Answers every read with a value derived from the object's address, so two
// fields mixed up would carry each other's numbers. Objects listed in
// `missing` answer with an SDO-style error instead.
struct FakeDrive
{
  std::set<Key> missing;
  std::map<Key, ConfigValue> served;

  ConfigReader Reader()
  {
    return [this](od::Entry entry, ConfigValue & value) -> std::error_code {
             if (missing.count(KeyOf(entry))) {
               return std::make_error_code(std::errc::no_such_device_or_address);
             }
             const auto seed = static_cast<std::uint32_t>(entry.index) * 7u + entry.subindex;
             std::visit(
               [seed](auto & typed) {
                 using T = std::decay_t<decltype(typed)>;
                 typed = static_cast<T>(seed % 100u + 1u);
               }, value);
             served[KeyOf(entry)] = value;
             return {};
           };
  }
};

// Walks a group's field list and keeps only the read-only objects - the ones
// Refresh() reads and Apply() must not write.
struct ReadOnlyCollector
{
  std::set<Key> entries;

  template<typename T>
  void operator()(od::Entry, const std::optional<T> &, Presence = Presence::kAlways) {}
  template<typename Codec, typename T>
  void Packed(od::Entry, const std::optional<T> &, Presence = Presence::kAlways) {}
  template<typename Wire, typename E, typename D>
  void Composite(od::Entry, E, D, Presence = Presence::kAlways) {}
  template<typename T>
  void ReadOnly(od::Entry entry, const std::optional<T> &, Presence = Presence::kAlways)
  {
    entries.insert(KeyOf(entry));
  }
};

template<typename Group>
std::set<Key>
ReadOnlyEntriesOf(const Group & group)
{
  ReadOnlyCollector collector;
  Group::Visit(group, collector);
  return collector.entries;
}

// Reads the group from the fake drive, writes it back, and checks the writes
// go to exactly the objects that were read - less the read-only ones, which
// must never be written. Then serves those writes back as a drive and reads
// again: the second writes must equal the first, value for value. (The first
// values need not: a drive's reserved bits are not the library's to keep.)
template<typename Group>
void
ExpectReadBackWritesTheSame(Group group = {})
{
  // What is not a field (which encoder object, say) carries over to pass 2.
  const Group original = group;
  FakeDrive drive;
  ASSERT_FALSE(group.ReadFrom(drive.Reader()));

  ConfigWrites writes;
  group.AppendTo(writes);

  const auto readOnly = ReadOnlyEntriesOf(group);
  ASSERT_EQ(writes.size() + readOnly.size(), drive.served.size())
    << "a field is read but not written, or vice versa";
  std::map<Key, ConfigValue> written;
  for (const auto & w : writes) {
    ASSERT_EQ(readOnly.count(KeyOf(w.entry)), 0u)
      << "read-only object written: 0x" << std::hex << w.entry.index << ":" <<
      int(w.entry.subindex);
    ASSERT_EQ(drive.served.count(KeyOf(w.entry)), 1u)
      << "written but never read: 0x" << std::hex << w.entry.index << ":" << int(w.entry.subindex);
    written[KeyOf(w.entry)] = w.value;
  }

  Group again = original;
  ASSERT_FALSE(
    again.ReadFrom(
      [&](od::Entry entry, ConfigValue & value) {
        const auto it = written.find(KeyOf(entry));
        if (it != written.end()) {value = it->second;}
        return std::error_code{};
      }));
  ConfigWrites rewrites;
  again.AppendTo(rewrites);
  ASSERT_EQ(rewrites.size(), writes.size());
  for (std::size_t n = 0; n < writes.size(); ++n) {
    EXPECT_EQ(rewrites[n].value, writes[n].value)
      << "value changed on the way back: 0x" << std::hex << writes[n].entry.index << ":" <<
      int(writes[n].entry.subindex);
  }
}

}  // namespace


TEST(ConfigReadback, EveryGroupWritesBackWhatItRead)
{
  ExpectReadBackWritesTheSame<MotorConfigs>();
  ExpectReadBackWritesTheSame<GearConfigs>();
  ExpectReadBackWritesTheSame<AxisConfigs>();
  ExpectReadBackWritesTheSame<CurrentControlConfigs>();
  ExpectReadBackWritesTheSame<PositionControlConfigs>();
  ExpectReadBackWritesTheSame<VelocityControlConfigs>();
  ExpectReadBackWritesTheSame<MotionProfileConfigs>();
  ExpectReadBackWritesTheSame<CyclicConfigs>();
  ExpectReadBackWritesTheSame<LimitConfigs>();
  ExpectReadBackWritesTheSame<HomingConfigs>();
  ExpectReadBackWritesTheSame<StopOptionConfigs>();
  ExpectReadBackWritesTheSame<StandstillConfigs>();
  ExpectReadBackWritesTheSame<HoldingBrakeConfigs>();
  ExpectReadBackWritesTheSame<DigitalInputConfigs>();
  ExpectReadBackWritesTheSame<DigitalOutputConfigs>();
  ExpectReadBackWritesTheSame<CommunicationConfigs>();
  ExpectReadBackWritesTheSame<ProtectionConfigs>();
  ExpectReadBackWritesTheSame<CustomPersistentMemoryConfigs>();
  ExpectReadBackWritesTheSame<AnalogInputConfigs>();
  ExpectReadBackWritesTheSame<AnalogOutputConfigs>();
  ExpectReadBackWritesTheSame<VelocityObserverConfigs>();
  ExpectReadBackWritesTheSame<DualLoopConfigs>();
  ExpectReadBackWritesTheSame<SensorsConfigs>();
  ExpectReadBackWritesTheSame<AnalogIncrementalEncoderConfigs>();
  ExpectReadBackWritesTheSame<SsiAbsoluteEncoderConfigs>();
  ExpectReadBackWritesTheSame<HallSensorConfigs>();
  ExpectReadBackWritesTheSame<DigitalIncrementalEncoderConfigs>();
  DigitalIncrementalEncoderConfigs second;
  second.encoderNumber = 2;
  ExpectReadBackWritesTheSame(second);
}

// The encoder number picks the object - for reading as much as writing.
TEST(ConfigReadback, TheSecondEncoderIsReadFromItsOwnObject)
{
  FakeDrive drive;
  DigitalIncrementalEncoderConfigs encoder;
  encoder.encoderNumber = 2;
  ASSERT_FALSE(encoder.ReadFrom(drive.Reader()));
  for (const auto & [key, value] : drive.served) {
    EXPECT_EQ(key.first, 0x3020);
  }
}

// 0x3012:07 is read-only and UNSIGNED32. It used to be written as a 16-bit
// value, which a drive aborts twice over.
TEST(ConfigReadback, TheSsiRefreshFrequencyIsOnlyRead)
{
  SsiAbsoluteEncoderConfigs ssi;
  ssi.refreshFrequency = 50000;
  ssi.positionSingleTurnBits = 13;
  ConfigWrites writes;
  ssi.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(writes[0].entry.subindex, 0x0B);
  EXPECT_EQ(std::get<std::uint32_t>(writes[0].value), 13u) << "multi-turn bits default to 0";
}

// Filter coefficients take effect only when 0x30AE:40 bit 0 is written -
// after them - and that word also holds the enable bit, so it must be known.
TEST(ConfigReadback, DualLoopCoefficientsTriggerTheUpdateAfterThem)
{
  DualLoopConfigs dual;
  dual.filterA = 900;
  EXPECT_TRUE(dual.Validate()) << "coefficients without filterActive";

  dual.filterActive = true;
  EXPECT_FALSE(dual.Validate());
  ConfigWrites writes;
  dual.AppendTo(writes);
  ASSERT_EQ(writes.size(), 2u);
  EXPECT_EQ(writes[0].entry.subindex, 0x10);
  EXPECT_EQ(writes[1].entry.subindex, 0x40);
  EXPECT_EQ(std::get<std::uint16_t>(writes[1].value), 0x3u);

  DualLoopConfigs justDisable;
  justDisable.filterActive = false;
  writes.clear();
  justDisable.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(std::get<std::uint16_t>(writes[0].value), 0x0u) << "no update without coefficients";
}

// The aggregate reads every group - the old hand-written Refresh() covered a
// dozen fields.
TEST(ConfigReadback, TheWholeConfigurationRoundTrips)
{
  FakeDrive drive;
  Epos4Configuration config;
  ASSERT_FALSE(config.ReadFrom(drive.Reader()));
  const auto readOnly = ReadOnlyEntriesOf(config.axis).size() +
    ReadOnlyEntriesOf(config.motor).size() + ReadOnlyEntriesOf(config.limits).size() +
    ReadOnlyEntriesOf(config.communication).size();
  // The aggregate validates the dual loop: a drive with its filter off reads
  // back filterActive = false, which is a valid configuration.
  EXPECT_EQ(config.ToWrites().size() + readOnly, drive.served.size());
}

// 0x1016 packs the producer's Node-ID above its time; a mix-up of the two
// halves is a consumer watching the wrong node.
TEST(ConfigReadback, TheHeartbeatConsumerPacksNodeAboveTime)
{
  CommunicationConfigs comm;
  comm.heartbeatConsumer1 = HeartbeatConsumer{0x7F, 150};
  ConfigWrites writes;
  comm.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(writes[0].entry.index, 0x1016);
  EXPECT_EQ(std::get<std::uint32_t>(writes[0].value), 0x007F0096u);

  const auto back = HeartbeatConsumerCodec::Decode(0x007F0096u);
  EXPECT_EQ(back.nodeId, 0x7F);
  EXPECT_EQ(back.timeMs, 150);
}

// Node-ID and bit rate go last of everything: once they are written, the
// drive is one Save() and restart away from leaving the master's network.
TEST(ConfigReadback, NodeIdAndBitRateAreWrittenLast)
{
  Epos4Configuration config;
  config.communication.nodeId = 5;
  config.communication.canBitRate = signals::CanBitRate::k500kbit;
  config.motor.nominalCurrent = 2000;
  config.limits.maxMotorSpeed = 8000;
  config.analogInputs.input1 = signals::AnalogInputFunction::kGeneralPurposeA;
  const auto writes = config.ToWrites();
  ASSERT_EQ(writes.size(), 5u);
  EXPECT_EQ(writes[3].entry.index, 0x2001);
  EXPECT_EQ(writes[4].entry.index, 0x2000);
}

// An analog input commanding current must have its slope before its
// function, or it commands through whatever slope the drive held.
TEST(ConfigReadback, AnalogScalingIsWrittenBeforeTheFunction)
{
  AnalogInputConfigs analog;
  analog.input1 = signals::AnalogInputFunction::kCurrentSetValue;
  analog.currentFirstVoltageMv = 0;
  analog.currentFirstMa = 0;
  analog.currentSecondVoltageMv = 10000;
  analog.currentSecondMa = 2000;
  ConfigWrites writes;
  analog.AppendTo(writes);
  ASSERT_EQ(writes.size(), 5u);
  EXPECT_EQ(writes.back().entry.index, 0x3161);
}

TEST(ConfigReadback, TwoAnalogInputsCannotShareAFunction)
{
  Epos4Configuration config;
  config.analogInputs.input1 = signals::AnalogInputFunction::kVelocitySetValue;
  config.analogInputs.input2 = signals::AnalogInputFunction::kVelocitySetValue;
  EXPECT_TRUE(config.Validate());
  config.analogInputs.input2 = signals::AnalogInputFunction::kNone;
  EXPECT_FALSE(config.Validate());
  config.analogInputs.input1 = signals::AnalogInputFunction::kNone;
  EXPECT_FALSE(config.Validate()) << "several inputs may carry no function";

  config.digitalInputs.input1 = signals::DigitalInputFunction::kHomeSwitch;
  config.digitalInputs.input2 = signals::DigitalInputFunction::kHomeSwitch;
  EXPECT_TRUE(config.Validate()) << "the aggregate checks the digital inputs too";
}

// 0x3000:05 and :06 are computed by the drive. Writing them aborts with
// "attempt to write a read only object" - and since Apply() stops at the
// first failure, everything after them in the configuration never lands.
TEST(ConfigReadback, ReadOnlyObjectsAreNeverWritten)
{
  AxisConfigs axis;
  axis.mainSensorResolution = 4096;
  axis.maxSystemSpeed = 10000;
  ConfigWrites writes;
  axis.AppendTo(writes);
  EXPECT_TRUE(writes.empty());
}

// Objects a newer firmware added are optional on read, like the variant-only
// ones: a 0x0170 drive has no 0x605D and no 0x30A1:09.
TEST(ConfigReadback, FirmwareDependentObjectsMaySimplyBeAbsent)
{
  FakeDrive drive;
  drive.missing = {{0x605D, 0}, {0x30A1, 9}};
  StopOptionConfigs stops;
  PositionControlConfigs gains;
  EXPECT_FALSE(stops.ReadFrom(drive.Reader()));
  EXPECT_FALSE(gains.ReadFrom(drive.Reader()));
  EXPECT_FALSE(stops.halt.has_value());
  EXPECT_TRUE(stops.quickStop.has_value());
  EXPECT_FALSE(gains.iGainUnit.has_value());
  EXPECT_TRUE(gains.p.has_value());
}

// A flag in a bit field round-trips through its bit and nothing else.
TEST(ConfigReadback, TheGearDirectionIsBitZero)
{
  GearConfigs gear;
  gear.invertedDirection = true;
  ConfigWrites writes;
  gear.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(std::get<std::uint32_t>(writes[0].value), 1u);

  GearConfigs back;
  ASSERT_FALSE(
    back.ReadFrom(
      [](od::Entry entry, ConfigValue & value) {
        // Reserved bits set on the drive's side must not read as "inverted".
        value = entry.subindex == 4 ? std::uint32_t{0xFFFFFFFEu} : std::uint32_t{1};
        return std::error_code{};
      }));
  EXPECT_EQ(back.invertedDirection, false);
}

// A packed field decodes back to what it encodes: the velocity prefix lives
// in the top byte of 0x60A9, with rev/min fixed below it.
TEST(ConfigReadback, ThePackedVelocityUnitRoundTrips)
{
  SiUnitConfigs units;
  units.velocityPrefix = signals::VelocityPrefix::kMilli;
  ConfigWrites writes;
  units.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(std::get<std::uint32_t>(writes[0].value), 0xFDB44700u);

  SiUnitConfigs back;
  ASSERT_FALSE(
    back.ReadFrom(
      [&](od::Entry, ConfigValue & value) {
        value = writes[0].value;
        return std::error_code{};
      }));
  EXPECT_EQ(back.velocityPrefix, signals::VelocityPrefix::kMilli);
}

// Holding brake sub-indices 4 and 5 exist only on some variants. A drive
// without them leaves those fields unset and the read still succeeds; a
// drive missing an object every variant has fails it.
TEST(ConfigReadback, HardwareDependentObjectsMaySimplyBeAbsent)
{
  FakeDrive drive;
  drive.missing = {{0x3158, 4}, {0x3158, 5}};
  HoldingBrakeConfigs brake;
  EXPECT_FALSE(brake.ReadFrom(drive.Reader()));
  EXPECT_TRUE(brake.couplingTimeMs.has_value());
  EXPECT_FALSE(brake.openingVoltageDeciVolt.has_value());

  drive.missing = {{0x3158, 1}};
  HoldingBrakeConfigs broken;
  EXPECT_TRUE(broken.ReadFrom(drive.Reader()));
  EXPECT_TRUE(broken.openingTimeMs.has_value()) << "the rest is still read";
}


// The objects the manual lets be written only with the motor unpowered.
TEST(ConfigReadback, PowerDisableObjectsAreTheManualsList)
{
  EXPECT_TRUE(RequiresPowerDisabled(od::maxon::kAxisConfiguration_SensorsConfiguration));
  EXPECT_TRUE(RequiresPowerDisabled(od::maxon::kMotorData_NumberOfPolePairs));
  EXPECT_FALSE(RequiresPowerDisabled(od::maxon::kMotorData_NominalCurrent));
  EXPECT_TRUE(RequiresPowerDisabled(od::maxon::kGearConfiguration_GearReductionNumerator));
  EXPECT_TRUE(RequiresPowerDisabled(od::maxon::kGearConfiguration_GearMiscellaneousConfiguration));
  EXPECT_FALSE(RequiresPowerDisabled(od::maxon::kGearConfiguration_MaxGearInputSpeed))
    << "6.2.55.3: writable with power enabled";
  EXPECT_TRUE(RequiresPowerDisabled(od::At(od::cia402::kSIUnitVelocity)));
  EXPECT_TRUE(RequiresPowerDisabled(od::maxon::kSSIAbsoluteEncoder_SSIDataRate));
  EXPECT_TRUE(RequiresPowerDisabled(od::maxon::kDigitalHallSensor_DigitalHallSensorType));
  EXPECT_FALSE(RequiresPowerDisabled(od::At(od::cia402::kProfileVelocity)));
  EXPECT_FALSE(
    RequiresPowerDisabled(
      od::maxon::kPositionControlParameterSet_PositionControllerPGain));
}

// 0x3011:02 default 0x00080004 is 2048 periods (bits 31..8) and 4 bits.
// A half-set pair used to take 8 periods, dividing the resolution by 256.
TEST(ConfigReadback, AHalfSetSinCosResolutionKeepsTheDrivesDefaultPeriods)
{
  AnalogIncrementalEncoderConfigs sincos;
  sincos.interpolationBits = 10;
  ConfigWrites writes;
  sincos.AppendTo(writes);
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(std::get<std::uint32_t>(writes[0].value), (2048u << 8) | 10u);
}

TEST(ConfigReadback, TouchProbeIsRefusedOnHighSpeedInputsOneAndThree)
{
  DigitalInputConfigs inputs;
  inputs.highSpeedInput1 = signals::DigitalInputFunction::kTouchProbe;
  EXPECT_TRUE(inputs.Validate());
  inputs.highSpeedInput1.reset();
  inputs.highSpeedInput2 = signals::DigitalInputFunction::kTouchProbe;
  EXPECT_FALSE(inputs.Validate());
}

TEST(ConfigReadback, PowerDisableIsTheStatesWithoutPower)
{
  using signals::State;
  EXPECT_TRUE(signals::IsPowerDisabled(State::kSwitchOnDisabled));
  EXPECT_TRUE(signals::IsPowerDisabled(State::kReadyToSwitchOn));
  EXPECT_TRUE(signals::IsPowerDisabled(State::kFault));
  EXPECT_FALSE(signals::IsPowerDisabled(State::kSwitchedOn));
  EXPECT_FALSE(signals::IsPowerDisabled(State::kOperationEnabled));
  EXPECT_FALSE(signals::IsPowerDisabled(State::kQuickStopActive));
  EXPECT_FALSE(signals::IsPowerDisabled(State::kFaultReactionActive));
}
