#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "epos4/core/ObjectDictionary.hpp"

namespace epos4::configs
{

// ---------------------------------------------------------------------------
// One list of fields per configuration group, walked two ways.
//
// Each group declares, once, which object every field lives in:
//
//   template<typename Self, typename V>
//   static void Visit(Self & self, V & v)
//   {
//     v(od::maxon::kMotorData_NominalCurrent, self.nominalCurrent);
//     ...
//   }
//
// FieldWriter walks it to turn the fields that are set into object writes -
// Apply(). FieldReader walks the same list to fill every field from the
// drive - Refresh(). Because both come from one list, a field cannot be
// written without also being read back, or read from a different object
// than it is written to: the hand-written Refresh() this replaces covered a
// dozen fields of the hundreds Apply() could write.
//
// The order of the list is the order of the writes, which matters in places
// (a polarity before the functions it qualifies, standstill before the
// brake) - see the groups.
// ---------------------------------------------------------------------------

// The integer types the drive's configuration objects use.
using ConfigValue = std::variant<
  std::int8_t, std::int16_t, std::int32_t,
  std::uint8_t, std::uint16_t, std::uint32_t>;

// A single pending object write. The Configurator turns a configuration
// object into a list of these and pushes them over SDO.
struct ConfigWrite
{
  od::Entry entry;
  ConfigValue value;
};

using ConfigWrites = std::vector<ConfigWrite>;

// Reads one object into `value`, with the width and signedness `value`
// already holds - the reader does not guess the type, the field says it.
using ConfigReader = std::function<std::error_code(od::Entry, ConfigValue &)>;

// Whether every drive has the object. An optional one is skipped on read
// when the drive refuses it, instead of failing the whole Refresh():
//   kHardwareDependent  sub-indices only some variants have (the manual's
//                       "only with «EPOS4 Disk 60/8»" and the like)
//   kFirmwareDependent  objects newer firmware added (chapter 8 lists when)
enum class Presence
{
  kAlways,
  kHardwareDependent,
  kFirmwareDependent,
};

namespace detail
{

template<typename T, typename = void>
struct WireOf {using type = T;};

template<typename T>
struct WireOf<T, std::enable_if_t<std::is_enum_v<T>>> {using type = std::underlying_type_t<T>;};

// The default codec: an integer as itself, an enum as its underlying type.
// Every enum in this library is declared with the object's own type as its
// underlying type, which is what makes this correct.
template<typename T>
struct Plain
{
  using Wire = typename WireOf<T>::type;
  static constexpr Wire Encode(const T & value) {return static_cast<Wire>(value);}
  static constexpr T Decode(Wire wire) {return static_cast<T>(wire);}
};

}  // namespace detail


// A struct that already knows its own bit layout - `Wire Encode() const` and
// `static T Decode(Wire)` - used as a codec.
template<typename T, typename WireT>
struct SelfCodec
{
  using Wire = WireT;
  static Wire Encode(const T & value) {return value.Encode();}
  static T Decode(Wire value) {return T::Decode(value);}
};


// One bit of an object that is a bit field, as a bool. Only for objects whose
// other bits the manual marks reserved (so writing zeros there is what the
// drive expects); anything with neighbours worth keeping gets a codec of its
// own.
template<typename WireT, unsigned kBit>
struct FlagCodec
{
  using Wire = WireT;
  static constexpr Wire Encode(bool on) {return on ? static_cast<Wire>(Wire{1} << kBit) : Wire{0};}
  static constexpr bool Decode(Wire value) {return ((value >> kBit) & 1u) != 0u;}
};


// Turns the fields that are set into writes, in visiting order.
class FieldWriter
{
public:
  explicit FieldWriter(ConfigWrites & out)
  : out_(out) {}

  // A field stored in an object of its own type (or an enum over it).
  template<typename T>
  void operator()(od::Entry entry, const std::optional<T> & field, Presence = Presence::kAlways)
  {
    Packed<detail::Plain<T>>(entry, field);
  }

  // A field that needs converting: Codec::Wire is the object's type,
  // Codec::Encode / Decode convert (e.g. a struct of bit fields to a word).
  template<typename Codec, typename T>
  void Packed(od::Entry entry, const std::optional<T> & field, Presence = Presence::kAlways)
  {
    if (field) {
      out_.push_back(ConfigWrite{entry, ConfigValue{Codec::Encode(*field)}});
    }
  }

  // Several fields sharing one object. `encode()` returns the object's value
  // or nullopt when none of its fields is set; `decode` is the reader's.
  template<typename Wire, typename Encode, typename Decode>
  void Composite(od::Entry entry, Encode encode, Decode, Presence = Presence::kAlways)
  {
    if (const std::optional<Wire> value = encode()) {
      out_.push_back(ConfigWrite{entry, ConfigValue{*value}});
    }
  }

  // A value the drive computes (a resolution, a speed limit it derives).
  // Filled by Refresh(), never written: the object is read-only, and a write
  // would abort with 0x06010002 and stop Apply() halfway.
  template<typename T>
  void ReadOnly(od::Entry, const std::optional<T> &, Presence = Presence::kAlways) {}

private:
  ConfigWrites & out_;
};


// Fills fields from the drive, in visiting order. Keeps going past a failed
// read, so one missing object does not leave everything after it unread,
// and reports the first failure of an object every drive should have.
class FieldReader
{
public:
  explicit FieldReader(const ConfigReader & read)
  : read_(read) {}

  template<typename T>
  void operator()(od::Entry entry, std::optional<T> & field, Presence presence = Presence::kAlways)
  {
    Packed<detail::Plain<T>>(entry, field, presence);
  }

  template<typename Codec, typename T>
  void Packed(od::Entry entry, std::optional<T> & field, Presence presence = Presence::kAlways)
  {
    ConfigValue value{typename Codec::Wire{}};
    if (Read(entry, value, presence)) {
      field = Codec::Decode(std::get<typename Codec::Wire>(value));
    }
  }

  template<typename Wire, typename Encode, typename Decode>
  void Composite(od::Entry entry, Encode, Decode decode, Presence presence = Presence::kAlways)
  {
    ConfigValue value{Wire{}};
    if (Read(entry, value, presence)) {
      decode(std::get<Wire>(value));
    }
  }

  template<typename T>
  void ReadOnly(od::Entry entry, std::optional<T> & field, Presence presence = Presence::kAlways)
  {
    Packed<detail::Plain<T>>(entry, field, presence);
  }

  std::error_code Error() const {return error_;}

private:
  bool Read(od::Entry entry, ConfigValue & value, Presence presence)
  {
    if (const auto ec = read_(entry, value)) {
      if (presence == Presence::kAlways && !error_) {
        error_ = ec;
      }
      return false;
    }
    return true;
  }

  const ConfigReader & read_;
  std::error_code error_{};
};


// Every group gets these two from its Visit(); spelled out in each group so
// that reading a header shows the whole interface.
template<typename Group>
void
WriteFields(const Group & group, ConfigWrites & out)
{
  FieldWriter writer{out};
  Group::Visit(group, writer);
}

template<typename Group>
std::error_code
ReadFields(Group & group, const ConfigReader & read)
{
  FieldReader reader{read};
  Group::Visit(group, reader);
  return reader.Error();
}

}  // namespace epos4::configs
