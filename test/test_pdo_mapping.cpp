// The PDO mapping model and its comparison with the master's concise DCF -
// what turns "the master decodes plausible garbage" into a list of
// differences. No bus: the drive's side is a PdoMapping filled by hand.

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "epos4/signals/PdoMapping.hpp"

using namespace epos4;           // NOLINT(build/namespaces)
using namespace epos4::signals;  // NOLINT(build/namespaces)

namespace
{

// Builds a concise DCF the way dcfgen lays one out.
class ConciseBuilder
{
public:
  ConciseBuilder & Write(std::uint16_t index, std::uint8_t sub, std::uint32_t value, unsigned size)
  {
    entries_.push_back({index, sub, value, size});
    return *this;
  }

  std::vector<std::uint8_t> Bytes() const
  {
    std::vector<std::uint8_t> out;
    auto put = [&out](std::uint32_t v, unsigned n) {
        for (unsigned i = 0; i < n; ++i) {out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));}
      };
    put(static_cast<std::uint32_t>(entries_.size()), 4);
    for (const auto & e : entries_) {
      put(e.index, 2);
      put(e.sub, 1);
      put(e.size, 4);
      put(e.value, e.size);
    }
    return out;
  }

private:
  struct Entry
  {
    std::uint16_t index;
    std::uint8_t sub;
    std::uint32_t value;
    unsigned size;
  };
  std::vector<Entry> entries_;
};

// TPDO1 = Statusword + Position actual, synchronous, the way dcfgen writes
// it: disable, remap, enable.
ConciseBuilder
Tpdo1StatusAndPosition()
{
  ConciseBuilder b;
  b.Write(0x1800, 1, 0x80000182, 4).Write(0x1800, 2, 1, 1)
  .Write(0x1A00, 0, 0, 1).Write(0x1A00, 1, 0x60410010, 4).Write(0x1A00, 2, 0x60640020, 4)
  .Write(0x1A00, 0, 2, 1).Write(0x1800, 1, 0x00000182, 4);
  return b;
}

// A drive configured exactly as Tpdo1StatusAndPosition() asks.
PdoMapping
DriveWithTpdo1StatusAndPosition()
{
  PdoMapping m;
  m.tpdo[0].cobId = 0x40000182;  // RTR bit set, as the drive's default has it
  m.tpdo[0].transmissionType = 1;
  m.tpdo[0].inhibitTime100us = 10;
  m.tpdo[0].objects = {{0x6041, 0, 16}, {0x6064, 0, 32}};
  return m;
}

std::vector<ConciseWrite>
Parse(const ConciseBuilder & b)
{
  auto parsed = ParseConciseDcf(b.Bytes());
  EXPECT_TRUE(parsed.has_value());
  return parsed.value_or(std::vector<ConciseWrite>{});
}

}  // namespace


TEST(PdoMapping, AMappedObjectIsIndexSubindexAndLength)
{
  const auto o = PdoObject::Decode(0x607A0020);
  EXPECT_EQ(o.index, 0x607A);
  EXPECT_EQ(o.subindex, 0);
  EXPECT_EQ(o.bits, 32);
  EXPECT_EQ(o.Encode(), 0x607A0020u);
}

TEST(PdoMapping, TheCobIdSaysWhetherTheChannelExists)
{
  PdoChannel c;
  c.cobId = 0x80000202;
  EXPECT_FALSE(c.IsValid());
  c.cobId = 0x40000182;
  EXPECT_TRUE(c.IsValid()) << "bit 30 is RTR, not validity";
  EXPECT_EQ(c.CanId(), 0x182);
}

TEST(PdoMapping, FindOnlyLooksAtValidChannelsOfThatDirection)
{
  auto m = DriveWithTpdo1StatusAndPosition();
  EXPECT_NE(m.Find(od::At(0x6064), PdoDirection::kTransmit), nullptr);
  EXPECT_EQ(m.Find(od::At(0x6064), PdoDirection::kReceive), nullptr);
  m.tpdo[0].cobId |= 0x80000000u;
  EXPECT_EQ(m.Find(od::At(0x6064), PdoDirection::kTransmit), nullptr);
}

TEST(PdoMapping, ParsesAConciseDcfAndRejectsABrokenOne)
{
  auto bytes = Tpdo1StatusAndPosition().Bytes();
  const auto parsed = ParseConciseDcf(bytes);
  ASSERT_TRUE(parsed);
  ASSERT_EQ(parsed->size(), 7u);
  EXPECT_EQ((*parsed)[3].entry.index, 0x1A00);
  EXPECT_EQ((*parsed)[3].AsUnsigned(), 0x60410010u);

  bytes.pop_back();
  EXPECT_FALSE(ParseConciseDcf(bytes)) << "truncated";
  bytes = Tpdo1StatusAndPosition().Bytes();
  bytes.push_back(0);
  EXPECT_FALSE(ParseConciseDcf(bytes)) << "trailing bytes";
  EXPECT_FALSE(ParseConciseDcf({}));
}

TEST(PdoMapping, AMatchingDriveHasNoMismatches)
{
  EXPECT_TRUE(
    CompareWithConciseDcf(Parse(Tpdo1StatusAndPosition()), DriveWithTpdo1StatusAndPosition())
    .empty()) << "only the final value of each object counts, and not the RTR bit";
}

// The failure this exists for: the drive kept its default TPDO1 (Statusword
// only, asynchronous) - say the boot download was aborted - while the master
// decodes 48 bits of Statusword and position from every frame.
TEST(PdoMapping, ADriveThatKeptItsDefaultsIsReportedObjectByObject)
{
  PdoMapping m;
  m.tpdo[0].cobId = 0x40000182;
  m.tpdo[0].transmissionType = 255;
  m.tpdo[0].objects = {{0x6041, 0, 16}};

  const auto mismatches = CompareWithConciseDcf(Parse(Tpdo1StatusAndPosition()), m);
  ASSERT_EQ(mismatches.size(), 3u);
  // In the order the master first writes each object.
  EXPECT_EQ(mismatches[0].entry.subindex, 2);  // transmission type
  EXPECT_EQ(mismatches[1].entry.index, 0x1A00);  // the count
  EXPECT_EQ(mismatches[1].entry.subindex, 0);
  EXPECT_EQ(mismatches[2].entry.subindex, 2);  // the object the drive lacks
  EXPECT_NE(Describe(mismatches[2]).find("0x6064:00/32"), std::string::npos)
    << Describe(mismatches[2]);
}

TEST(PdoMapping, AChannelEnabledOnOneSideOnlyIsAMismatch)
{
  auto m = DriveWithTpdo1StatusAndPosition();
  m.tpdo[0].cobId = 0xC0000182;
  const auto mismatches = CompareWithConciseDcf(Parse(Tpdo1StatusAndPosition()), m);
  ASSERT_EQ(mismatches.size(), 1u);
  EXPECT_NE(Describe(mismatches[0]).find("enabled on one side only"), std::string::npos);
}

TEST(PdoMapping, EntriesBeyondTheMappedCountAreNotCompared)
{
  ConciseBuilder b = Tpdo1StatusAndPosition();
  b.Write(0x1A00, 3, 0x606C0020, 4);  // written, but the count stays 2
  EXPECT_TRUE(CompareWithConciseDcf(Parse(b), DriveWithTpdo1StatusAndPosition()).empty());
}

// The drive's defaults leave RPDO3/4 valid; a network that does not mention
// them leaves the axis open to commands from any device on those IDs.
TEST(PdoMapping, AValidChannelTheNetworkDoesNotConfigureIsReported)
{
  auto m = DriveWithTpdo1StatusAndPosition();
  m.rpdo[2].cobId = 0x00000402;
  m.rpdo[2].objects = {{0x6040, 0, 16}, {0x607A, 0, 32}};
  const auto mismatches = CompareWithConciseDcf(Parse(Tpdo1StatusAndPosition()), m);
  ASSERT_EQ(mismatches.size(), 1u);
  EXPECT_EQ(mismatches[0].kind, PdoMismatch::Kind::kNotConfigured);
  EXPECT_EQ(mismatches[0].entry.index, 0x1402);
  EXPECT_NE(Describe(mismatches[0]).find("accepts commands"), std::string::npos)
    << Describe(mismatches[0]);

  // Disabled by the network - here with a CAN-ID of 0x000 on the drive, which
  // an invalid PDO may carry: nothing to report.
  ConciseBuilder b = Tpdo1StatusAndPosition();
  b.Write(0x1402, 1, 0x80000402, 4);
  m.rpdo[2].cobId = 0x80000000;
  EXPECT_TRUE(CompareWithConciseDcf(Parse(b), m).empty());
}

TEST(PdoMapping, DescribeShowsEveryChannel)
{
  const auto text = Describe(DriveWithTpdo1StatusAndPosition());
  EXPECT_NE(text.find("TPDO1  0x182  sync"), std::string::npos) << text;
  EXPECT_NE(text.find("0x6064:00/32"), std::string::npos) << text;
  EXPECT_NE(text.find("RPDO1  disabled"), std::string::npos) << text;
}

// The concise DCF dcfgen generated for node 2 from config/epos4_network,
// read back: proves the parser on the real thing and pins bus.yml's mapping
// to what the cyclic path relies on.
TEST(PdoMapping, TheExampleNetworksConciseDcfSaysWhatBusYmlSays)
{
  std::ifstream file(EPOSLIB_TEST_NODE_BIN, std::ios::binary);
  if (!file) {
    GTEST_SKIP() << "no " << EPOSLIB_TEST_NODE_BIN << " (dcfgen not installed)";
  }
  const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(file), {}};
  const auto concise = ParseConciseDcf(bytes);
  ASSERT_TRUE(concise);

  // A drive set up exactly as bus.yml declares matches it.
  PdoMapping m;
  m.rpdo[0] = {PdoDirection::kReceive, 1, 0x00000202, 1, std::nullopt,
    {{0x6040, 0, 16}, {0x607A, 0, 32}}};
  m.rpdo[1] = {PdoDirection::kReceive, 2, 0x00000302, 1, std::nullopt,
    {{0x60FF, 0, 32}, {0x6071, 0, 16}}};
  m.tpdo[0] = {PdoDirection::kTransmit, 1, 0x40000182, 1, 10,
    {{0x6041, 0, 16}, {0x6064, 0, 32}}};
  m.tpdo[1] = {PdoDirection::kTransmit, 2, 0x40000282, 1, 10,
    {{0x606C, 0, 32}, {0x6077, 0, 16}}};
  // bus.yml disables RPDO3/4, which the drive's defaults leave valid.
  m.rpdo[2].cobId = 0x80000402;
  m.rpdo[3].cobId = 0x80000502;
  const auto mismatches = CompareWithConciseDcf(*concise, m);
  for (const auto & mm : mismatches) {
    ADD_FAILURE() << Describe(mm);
  }
}
