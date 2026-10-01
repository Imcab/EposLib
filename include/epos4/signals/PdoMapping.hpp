#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "epos4/core/ObjectDictionary.hpp"

namespace epos4::signals
{

// ---------------------------------------------------------------------------
// PDO mapping, as the drive holds it - read, never written.
//
// The mapping is declared in the network description (bus.yml) and pushed
// by the master at boot, from the concise DCF dcfgen generates for each
// node. Both ends have to agree on which bytes of a frame mean what, and when
// they do not, nothing fails: the master decodes the drive's frames with its
// own layout and reads plausible garbage - a position that is really a
// velocity, a Statusword that is half of something else. bus.yml records one
// afternoon lost that way.
//
// So this does two things: report what the drive is actually configured
// with (sections 6.2.15-6.2.30), and compare it with what the master
// believes it configured (CompareWithConciseDcf), which turns that silent
// failure into a list of differences.
// ---------------------------------------------------------------------------

// One mapped object: 0x16xx / 0x1Axx sub 1..12, packed as Table 6-73 -
// index in bits 31..16, sub-index in 15..8, length in bits in 7..0.
struct PdoObject
{
  std::uint16_t index{0};
  std::uint8_t subindex{0};
  std::uint8_t bits{0};

  static PdoObject Decode(std::uint32_t value);
  std::uint32_t Encode() const;
  bool operator==(const PdoObject & other) const
  {
    return index == other.index && subindex == other.subindex && bits == other.bits;
  }
};

enum class PdoDirection : std::uint8_t
{
  kReceive,   // RPDO: master -> drive (setpoints, Controlword)
  kTransmit,  // TPDO: drive -> master (feedback, Statusword)
};

// One PDO channel: its communication parameter (0x14xx / 0x18xx) and its
// mapping (0x16xx / 0x1Axx).
struct PdoChannel
{
  PdoDirection direction{PdoDirection::kReceive};
  std::uint8_t number{1};  // 1..4

  // Table 6-71 / 6-78: bit 31 set = "PDO does not exist / is not valid",
  // bit 30 set = no RTR, bits 10..0 the CAN-ID.
  std::uint32_t cobId{0x80000000u};

  // 1 = synchronous: an RPDO is applied, a TPDO sent, on every SYNC.
  // 255 = asynchronous (on change for a TPDO); 253 = TPDO on RTR only.
  std::uint8_t transmissionType{255};

  // TPDO only (0x18xx:03), in multiples of 100 us: the minimum interval
  // between event-driven transmissions.
  std::optional<std::uint16_t> inhibitTime100us;

  std::vector<PdoObject> objects;

  bool IsValid() const {return (cobId & 0x80000000u) == 0u;}
  std::uint16_t CanId() const {return static_cast<std::uint16_t>(cobId & 0x7FFu);}
  bool IsSynchronous() const {return transmissionType >= 1u && transmissionType <= 240u;}
  // Total of the mapped lengths; a CAN frame carries 64 at most (6.2.19).
  unsigned TotalBits() const;
  bool Maps(od::Entry entry) const;

  // The communication and mapping objects of this channel.
  std::uint16_t ParameterIndex() const;
  std::uint16_t MappingIndex() const;
};

struct PdoMapping
{
  std::array<PdoChannel, 4> rpdo;
  std::array<PdoChannel, 4> tpdo;

  PdoMapping();

  // The valid channel of that direction that carries the object, if any.
  const PdoChannel * Find(od::Entry entry, PdoDirection direction) const;

  // The value the drive would answer for one of the PDO objects (0x1400-
  // 0x1A03), or nullopt for an object outside them. Mapping entries beyond
  // the mapped count read as 0, like the drive's unused ones.
  std::optional<std::uint32_t> ValueOf(od::Entry entry) const;
};

// One line per channel, e.g.
//   TPDO1  0x182  sync   0x6041:00/16 0x6064:00/32  (48 bits)
std::string Describe(const PdoMapping & mapping);


// ---------------------------------------------------------------------------
// Concise DCF (CiA 302-3): the list of SDO writes the master performs on a
// node at boot - Lely keeps it in the master's 0x1F22 sub <node-ID>. Laid
// out as a UNSIGNED32 count, then per entry a UNSIGNED16 index, UNSIGNED8
// sub-index, UNSIGNED32 size and that many data bytes, all little-endian.
// ---------------------------------------------------------------------------
struct ConciseWrite
{
  od::Entry entry;
  std::vector<std::uint8_t> data;

  // The data as an unsigned little-endian integer (up to 4 bytes).
  std::uint32_t AsUnsigned() const;
};

// nullopt when the bytes are not a well-formed concise DCF (truncated, or a
// count that does not match the entries).
std::optional<std::vector<ConciseWrite>> ParseConciseDcf(const std::vector<std::uint8_t> & bytes);

// A PDO object whose value on the drive is not what the master configured.
struct PdoMismatch
{
  enum class Kind : std::uint8_t
  {
    // The drive holds a different value than the concise DCF writes.
    kValue,
    // A channel valid on the drive that the network description does not
    // configure at all (entry = its COB-ID object, expected unused). The
    // master neither sends nor decodes it - but an RPDO like that still
    // takes Controlword and setpoints from any device sending on its ID,
    // and a TPDO like that loads the bus. Disable it in bus.yml.
    kNotConfigured,
  };

  od::Entry entry;
  std::uint32_t expected{0};  // the final value the concise DCF writes
  std::uint32_t actual{0};    // the value read from the drive
  Kind kind{Kind::kValue};
};

std::string Describe(const PdoMismatch & mismatch);

// Every PDO object the concise DCF sets, compared with the drive. Only the
// final value of each object counts: dcfgen disables a PDO, remaps it and
// enables it again, so the same object is written more than once. COB-IDs
// are compared on the bits that decide what is on the bus - valid and the
// CAN-ID - not the RTR bit, which dcfgen and the drive's defaults set
// differently with no effect on a SYNC-driven network. Mapping entries are
// compared up to the count the master sets. Then every channel valid on the
// drive whose COB-ID the concise DCF never writes is reported as
// kNotConfigured.
std::vector<PdoMismatch> CompareWithConciseDcf(
  const std::vector<ConciseWrite> & concise, const PdoMapping & actual);

}  // namespace epos4::signals
