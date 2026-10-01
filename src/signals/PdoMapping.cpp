#include "epos4/signals/PdoMapping.hpp"

#include <cstdio>
#include <map>
#include <utility>

namespace epos4::signals
{

namespace
{

constexpr std::uint8_t kMaxMappedObjects = 12;  // sub-indices 1..12, 6.2.19

// Which channel, and which half of it, a PDO object belongs to.
struct Located
{
  PdoDirection direction;
  std::uint8_t number;  // 1..4
  bool mapping;         // 0x16xx / 0x1Axx rather than 0x14xx / 0x18xx
};

std::optional<Located>
Locate(std::uint16_t index)
{
  struct Range
  {
    std::uint16_t first;
    PdoDirection direction;
    bool mapping;
  };
  constexpr Range kRanges[] = {
    {od::comm::kReceivePDO1Parameter, PdoDirection::kReceive, false},
    {od::comm::kReceivePDO1Mapping, PdoDirection::kReceive, true},
    {od::comm::kTransmitPDO1Parameter, PdoDirection::kTransmit, false},
    {od::comm::kTransmitPDO1Mapping, PdoDirection::kTransmit, true},
  };
  for (const auto & r : kRanges) {
    if (index >= r.first && index < r.first + 4) {
      return Located{r.direction, static_cast<std::uint8_t>(index - r.first + 1), r.mapping};
    }
  }
  return std::nullopt;
}

const char *
Prefix(PdoDirection direction)
{
  return direction == PdoDirection::kReceive ? "RPDO" : "TPDO";
}

}  // namespace


PdoObject
PdoObject::Decode(std::uint32_t value)
{
  return PdoObject{
    static_cast<std::uint16_t>(value >> 16),
    static_cast<std::uint8_t>((value >> 8) & 0xFFu),
    static_cast<std::uint8_t>(value & 0xFFu)};
}

std::uint32_t
PdoObject::Encode() const
{
  return (static_cast<std::uint32_t>(index) << 16) |
         (static_cast<std::uint32_t>(subindex) << 8) | bits;
}

unsigned
PdoChannel::TotalBits() const
{
  unsigned total = 0;
  for (const auto & o : objects) {
    total += o.bits;
  }
  return total;
}

bool
PdoChannel::Maps(od::Entry entry) const
{
  for (const auto & o : objects) {
    if (o.index == entry.index && o.subindex == entry.subindex) {return true;}
  }
  return false;
}

std::uint16_t
PdoChannel::ParameterIndex() const
{
  const std::uint16_t first = direction == PdoDirection::kReceive ?
    od::comm::kReceivePDO1Parameter : od::comm::kTransmitPDO1Parameter;
  return static_cast<std::uint16_t>(first + number - 1);
}

std::uint16_t
PdoChannel::MappingIndex() const
{
  const std::uint16_t first = direction == PdoDirection::kReceive ?
    od::comm::kReceivePDO1Mapping : od::comm::kTransmitPDO1Mapping;
  return static_cast<std::uint16_t>(first + number - 1);
}


PdoMapping::PdoMapping()
{
  for (std::uint8_t n = 0; n < 4; ++n) {
    rpdo[n].direction = PdoDirection::kReceive;
    rpdo[n].number = static_cast<std::uint8_t>(n + 1);
    tpdo[n].direction = PdoDirection::kTransmit;
    tpdo[n].number = static_cast<std::uint8_t>(n + 1);
  }
}

const PdoChannel *
PdoMapping::Find(od::Entry entry, PdoDirection direction) const
{
  const auto & channels = direction == PdoDirection::kReceive ? rpdo : tpdo;
  for (const auto & c : channels) {
    if (c.IsValid() && c.Maps(entry)) {return &c;}
  }
  return nullptr;
}

std::optional<std::uint32_t>
PdoMapping::ValueOf(od::Entry entry) const
{
  const auto where = Locate(entry.index);
  if (!where) {return std::nullopt;}
  const auto & channels = where->direction == PdoDirection::kReceive ? rpdo : tpdo;
  const PdoChannel & c = channels[where->number - 1];

  if (where->mapping) {
    if (entry.subindex == 0) {return static_cast<std::uint32_t>(c.objects.size());}
    if (entry.subindex > kMaxMappedObjects) {return std::nullopt;}
    return entry.subindex <= c.objects.size() ? c.objects[entry.subindex - 1].Encode() : 0u;
  }
  switch (entry.subindex) {
    case 1: return c.cobId;
    case 2: return c.transmissionType;
    case 3:
      if (c.inhibitTime100us) {return *c.inhibitTime100us;}
      return std::nullopt;
    default: return std::nullopt;
  }
}


std::string
Describe(const PdoMapping & mapping)
{
  std::string out;
  char line[64];
  auto channel = [&](const PdoChannel & c) {
      std::snprintf(line, sizeof line, "%s%u  ", Prefix(c.direction), c.number);
      out += line;
      if (!c.IsValid()) {
        out += "disabled\n";
        return;
      }
      const char * type = c.IsSynchronous() ? "sync" :
        c.transmissionType == 253 ? "rtr" : c.transmissionType == 255 ? "async" : "other";
      std::snprintf(line, sizeof line, "0x%03X  %-5s ", c.CanId(), type);
      out += line;
      for (const auto & o : c.objects) {
        std::snprintf(line, sizeof line, " 0x%04X:%02X/%u", o.index, o.subindex, o.bits);
        out += line;
      }
      std::snprintf(
        line, sizeof line, "  (%u bits%s)\n", c.TotalBits(),
        c.TotalBits() > 64 ? ", MORE THAN A CAN FRAME" : "");
      out += line;
    };
  for (const auto & c : mapping.rpdo) {
    channel(c);
  }
  for (const auto & c : mapping.tpdo) {
    channel(c);
  }
  return out;
}


std::uint32_t
ConciseWrite::AsUnsigned() const
{
  std::uint32_t value = 0;
  for (std::size_t i = 0; i < data.size() && i < 4; ++i) {
    value |= static_cast<std::uint32_t>(data[i]) << (8 * i);
  }
  return value;
}

std::optional<std::vector<ConciseWrite>>
ParseConciseDcf(const std::vector<std::uint8_t> & bytes)
{
  std::size_t pos = 0;
  auto take = [&](std::size_t n, std::uint32_t & out) {
      if (bytes.size() - pos < n) {return false;}
      out = 0;
      for (std::size_t i = 0; i < n; ++i) {
        out |= static_cast<std::uint32_t>(bytes[pos + i]) << (8 * i);
      }
      pos += n;
      return true;
    };

  std::uint32_t count = 0;
  if (!take(4, count)) {return std::nullopt;}
  std::vector<ConciseWrite> out;
  for (std::uint32_t n = 0; n < count; ++n) {
    std::uint32_t index = 0, sub = 0, size = 0;
    if (!take(2, index) || !take(1, sub) || !take(4, size) || bytes.size() - pos < size) {
      return std::nullopt;
    }
    ConciseWrite w;
    w.entry = od::At(static_cast<std::uint16_t>(index), static_cast<std::uint8_t>(sub));
    w.data.assign(
      bytes.begin() + static_cast<std::ptrdiff_t>(pos),
      bytes.begin() + static_cast<std::ptrdiff_t>(pos + size));
    pos += size;
    out.push_back(std::move(w));
  }
  if (pos != bytes.size()) {return std::nullopt;}
  return out;
}


std::string
Describe(const PdoMismatch & m)
{
  const auto where = Locate(m.entry.index);
  char head[48];
  std::snprintf(
    head, sizeof head, "%s%u ", where ? Prefix(where->direction) : "?",
    where ? where->number : 0u);
  char body[200];
  if (m.kind == PdoMismatch::Kind::kNotConfigured) {
    const bool receive = where && where->direction == PdoDirection::kReceive;
    std::snprintf(
      body, sizeof body,
      "is valid on the drive (CAN-ID 0x%03X) but the network description does not "
      "configure it: the drive %s - disable it in bus.yml",
      m.actual & 0x7FFu,
      receive ? "accepts commands sent on that ID" : "transmits it for no one");
  } else if (where && where->mapping && m.entry.subindex == 0) {
    std::snprintf(
      body, sizeof body, "maps %u objects, the master expects %u",
      m.actual, m.expected);
  } else if (where && where->mapping) {
    const auto a = PdoObject::Decode(m.actual), e = PdoObject::Decode(m.expected);
    std::snprintf(
      body, sizeof body,
      "object %u is 0x%04X:%02X/%u, the master expects 0x%04X:%02X/%u",
      m.entry.subindex, a.index, a.subindex, a.bits, e.index, e.subindex, e.bits);
  } else if (m.entry.subindex == 1) {
    std::snprintf(
      body, sizeof body, "COB-ID is 0x%08X, the master expects 0x%08X%s",
      m.actual, m.expected,
      ((m.actual ^ m.expected) & 0x80000000u) ? " (enabled on one side only)" : "");
  } else if (m.entry.subindex == 2) {
    std::snprintf(
      body, sizeof body, "transmission type is %u, the master expects %u",
      m.actual, m.expected);
  } else {
    std::snprintf(
      body, sizeof body, "0x%04X:%02X is 0x%X, the master expects 0x%X",
      m.entry.index, m.entry.subindex, m.actual, m.expected);
  }
  return std::string{head} + body;
}

std::vector<PdoMismatch>
CompareWithConciseDcf(const std::vector<ConciseWrite> & concise, const PdoMapping & actual)
{
  // Final value per PDO object, in first-written order for a stable report.
  std::vector<std::pair<std::uint16_t, std::uint8_t>> order;
  std::map<std::pair<std::uint16_t, std::uint8_t>, std::uint32_t> last;
  for (const auto & w : concise) {
    if (!Locate(w.entry.index)) {continue;}
    const auto key = std::make_pair(w.entry.index, w.entry.subindex);
    if (!last.count(key)) {order.push_back(key);}
    last[key] = w.AsUnsigned();
  }

  std::vector<PdoMismatch> out;
  for (const auto & key : order) {
    const od::Entry entry = od::At(key.first, key.second);
    const auto where = Locate(key.first);
    std::uint32_t expected = last[key];

    // Mapping entries past the count the master sets are not used.
    if (where->mapping && key.second != 0) {
      const auto count = last.find({key.first, 0});
      if (count != last.end() && key.second > count->second) {continue;}
    }

    const auto value = actual.ValueOf(entry);
    if (!value) {continue;}  // not something the drive was read for
    std::uint32_t got = *value;
    if (!where->mapping && key.second == 1) {
      // Valid bit and CAN-ID only: see the header. And a PDO disabled on
      // both sides agrees whatever its CAN-ID - the manual lets an invalid
      // PDO carry 0x000 (Table 6-72), and the drive does.
      constexpr std::uint32_t kInvalid = 0x80000000u;
      constexpr std::uint32_t kCompared = kInvalid | 0x1FFFFFFFu;
      if ((got & kCompared) == (expected & kCompared)) {continue;}
      if ((got & kInvalid) && (expected & kInvalid)) {continue;}
    } else if (got == expected) {
      continue;
    }
    out.push_back(PdoMismatch{entry, expected, got, PdoMismatch::Kind::kValue});
  }

  for (const auto * channels : {&actual.rpdo, &actual.tpdo}) {
    for (const auto & c : *channels) {
      if (c.IsValid() && !last.count({c.ParameterIndex(), 1})) {
        out.push_back(
          PdoMismatch{od::At(c.ParameterIndex(), 1), 0u, c.cobId,
            PdoMismatch::Kind::kNotConfigured});
      }
    }
  }
  return out;
}

}  // namespace epos4::signals
