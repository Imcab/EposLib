// Every configuration field against the drive's own description of the
// object it lives in: the maxon EDS in config/epos4_network.
//
// A field one size too wide reads back fine from nothing and only fails on a
// drive, with an SDO abort that stops Apply() halfway: 0x3002:02 and
// 0x30A2:05 were UNSIGNED32 here and UNSIGNED16 on the drive until a round
// trip against the simulator caught them. This test catches the next one
// without a bus.

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "epos4/configs/Configs.hpp"
#include "epos4/configs/EncoderConfigs.hpp"

using namespace epos4;           // NOLINT(build/namespaces)
using namespace epos4::configs;  // NOLINT(build/namespaces)

namespace
{

struct EdsObject
{
  int dataType{-1};
  std::string access;
};

using Key = std::pair<std::uint16_t, std::uint8_t>;

// The few EDS keys this needs, per "[1234]" or "[1234sub5]" section.
std::map<Key, EdsObject>
LoadEds(const std::string & path)
{
  std::ifstream file(path);
  std::map<Key, EdsObject> out;
  const std::regex header(R"(^\[([0-9A-Fa-f]{4})(?:sub([0-9A-Fa-f]+))?\]\s*$)");
  EdsObject * current = nullptr;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    std::smatch m;
    if (std::regex_match(line, m, header)) {
      const Key key{
        static_cast<std::uint16_t>(std::stoul(m[1], nullptr, 16)),
        static_cast<std::uint8_t>(m[2].matched ? std::stoul(m[2], nullptr, 16) : 0u)};
      current = &out[key];
    } else if (!line.empty() && line[0] == '[') {
      current = nullptr;
    } else if (current && line.rfind("DataType=", 0) == 0) {
      current->dataType = static_cast<int>(std::stoul(line.substr(9), nullptr, 16));
    } else if (current && line.rfind("AccessType=", 0) == 0) {
      current->access = line.substr(11);
      for (auto & c : current->access) {
        c = static_cast<char>(std::tolower(c));
      }
    }
  }
  return out;
}

// CANopen DataType codes (CiA 301, Table 44).
template<typename W>
constexpr int
DataTypeOf()
{
  if constexpr (std::is_same_v<W, std::int8_t>) {return 0x2;}
  if constexpr (std::is_same_v<W, std::int16_t>) {return 0x3;}
  if constexpr (std::is_same_v<W, std::int32_t>) {return 0x4;}
  if constexpr (std::is_same_v<W, std::uint8_t>) {return 0x5;}
  if constexpr (std::is_same_v<W, std::uint16_t>) {return 0x6;}
  if constexpr (std::is_same_v<W, std::uint32_t>) {return 0x7;}
  return -1;
}

struct Field
{
  od::Entry entry;
  int dataType;
  bool written;
  Presence presence;
};

// Walks a group's field list and records what each field puts on the wire.
struct FieldCollector
{
  std::vector<Field> fields;

  template<typename T>
  void operator()(od::Entry e, const std::optional<T> &, Presence p = Presence::kAlways)
  {
    fields.push_back({e, DataTypeOf<typename detail::WireOf<T>::type>(), true, p});
  }
  template<typename Codec, typename T>
  void Packed(od::Entry e, const std::optional<T> &, Presence p = Presence::kAlways)
  {
    fields.push_back({e, DataTypeOf<typename Codec::Wire>(), true, p});
  }
  template<typename Wire, typename E, typename D>
  void Composite(od::Entry e, E, D, Presence p = Presence::kAlways)
  {
    fields.push_back({e, DataTypeOf<Wire>(), true, p});
  }
  template<typename T>
  void ReadOnly(od::Entry e, const std::optional<T> &, Presence p = Presence::kAlways)
  {
    fields.push_back({e, DataTypeOf<typename detail::WireOf<T>::type>(), false, p});
  }
};

const std::map<Key, EdsObject> & Eds()
{
  static const auto eds = LoadEds(EPOSLIB_TEST_EDS);
  return eds;
}

template<typename Group>
void
ExpectFieldsMatchTheEds(const char * name, const Group & group = {})
{
  FieldCollector collector;
  Group::Visit(group, collector);
  for (const auto & f : collector.fields) {
    const auto where = ::testing::Message() << name << " 0x" << std::hex << f.entry.index <<
      ":" << int(f.entry.subindex);
    const auto it = Eds().find({f.entry.index, f.entry.subindex});
    if (it == Eds().end()) {
      // The EDS is one variant at one firmware (Module 50/15, 0x0170); what
      // it lacks must be what the field declares optional.
      EXPECT_NE(f.presence, Presence::kAlways) << where << " is not in the EDS";
      continue;
    }
    EXPECT_EQ(f.dataType, it->second.dataType) << where << ": data type";
    if (f.written) {
      EXPECT_TRUE(it->second.access != "ro" && it->second.access != "const") <<
        where << ": written, but the drive has it " << it->second.access;
    }
  }
}

}  // namespace


TEST(ConfigTypes, TheEdsIsThere)
{
  ASSERT_GT(Eds().size(), 500u) << EPOSLIB_TEST_EDS;
}

TEST(ConfigTypes, EveryFieldHasTheDrivesTypeAndAccess)
{
#define EXPECT_GROUP(T) ExpectFieldsMatchTheEds<T>(#T)
  EXPECT_GROUP(MotorConfigs);
  EXPECT_GROUP(GearConfigs);
  EXPECT_GROUP(AxisConfigs);
  EXPECT_GROUP(CurrentControlConfigs);
  EXPECT_GROUP(PositionControlConfigs);
  EXPECT_GROUP(VelocityControlConfigs);
  EXPECT_GROUP(VelocityObserverConfigs);
  EXPECT_GROUP(DualLoopConfigs);
  EXPECT_GROUP(MotionProfileConfigs);
  EXPECT_GROUP(CyclicConfigs);
  EXPECT_GROUP(SiUnitConfigs);
  EXPECT_GROUP(LimitConfigs);
  EXPECT_GROUP(HomingConfigs);
  EXPECT_GROUP(StopOptionConfigs);
  EXPECT_GROUP(StandstillConfigs);
  EXPECT_GROUP(HoldingBrakeConfigs);
  EXPECT_GROUP(DigitalInputConfigs);
  EXPECT_GROUP(DigitalOutputConfigs);
  EXPECT_GROUP(AnalogInputConfigs);
  EXPECT_GROUP(AnalogOutputConfigs);
  EXPECT_GROUP(ProtectionConfigs);
  EXPECT_GROUP(CustomPersistentMemoryConfigs);
  EXPECT_GROUP(CommunicationConfigs);
  EXPECT_GROUP(SensorsConfigs);
  EXPECT_GROUP(AnalogIncrementalEncoderConfigs);
  EXPECT_GROUP(SsiAbsoluteEncoderConfigs);
  EXPECT_GROUP(HallSensorConfigs);
  EXPECT_GROUP(DigitalIncrementalEncoderConfigs);
#undef EXPECT_GROUP
  DigitalIncrementalEncoderConfigs second;
  second.encoderNumber = 2;
  ExpectFieldsMatchTheEds("DigitalIncrementalEncoderConfigs(2)", second);
}
