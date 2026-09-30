#pragma once

#include <cstdint>
#include <type_traits>

#include <ros2units/units.h>

#include "epos4/core/UnitConversion.hpp"

namespace epos4
{

// ---------------------------------------------------------------------------
// A setpoint given either in the drive's raw units or as a physical quantity.
//
//   motor.SetControl(controls::ProfilePosition{}
//                      .WithPosition(50000)        // quadcounts
//                      .WithVelocity(1500_rpm));   // at the output shaft
//
//   motor.SetControl(controls::ProfilePosition{}
//                      .WithPosition(90_deg)       // needs the mechanism set
//                      .WithVelocity(2000));       // rpm at the motor
//
// Both forms are kept because both are legitimate. Raw counts are what the
// manual, EPOS Studio and every existing tuning note are written in, so
// forcing everything through units would make the library harder to check
// against the drive. Quantities are what application code should use, because
// a joint angle in degrees cannot silently be a joint angle in counts.
//
// Resolving the quantity form needs the encoder resolution and the gear
// ratio, which live in the device's MechanismScale. A request that uses a
// quantity against a device whose mechanism was never configured is rejected
// rather than guessed at.
// ---------------------------------------------------------------------------
template<typename Raw, typename Quantity>
class Setpoint
{
public:
  constexpr Setpoint() = default;

  // Both constructors are implicit on purpose: it is what makes
  // WithPosition(50000) and WithPosition(90_deg) both read naturally.
  // The enable_if is a non-type parameter, and the two constructors use
  // different types for it (int and bool). Defaulted TYPE parameters are
  // ignored when comparing signatures, so two SFINAE'd constructors written
  // that way collide as redeclarations of each other.
  template<typename T,
    typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
  constexpr Setpoint(T raw)  // NOLINT(runtime/explicit)
  : raw_(static_cast<Raw>(raw)), isRaw_(true) {}

  // Templated over the incoming unit rather than taking Quantity directly.
  // Taking Quantity would mean a caller passing degree_t needs two
  // user-defined conversions - degree_t to turn_t, then turn_t to Setpoint -
  // and C++ allows only one, so WithPosition(90_deg) would not compile even
  // though degrees and turns are the same dimension. Accepting any
  // convertible unit and letting nholthaus do the scaling fixes that.
  template<typename U,
    typename std::enable_if<
      units::traits::is_convertible_unit_t<U, Quantity>::value, bool>::type = true>
  constexpr Setpoint(U quantity)  // NOLINT(runtime/explicit)
  : quantity_(quantity), isRaw_(false) {}

  constexpr bool IsRaw() const {return isRaw_;}
  constexpr Raw GetRaw() const {return raw_;}
  constexpr Quantity GetQuantity() const {return quantity_;}

private:
  Raw raw_{};
  Quantity quantity_{};
  bool isRaw_{true};
};


// The three the control requests use.
using PositionSetpoint = Setpoint<std::int32_t, units::angle::turn_t>;
using VelocitySetpoint =
  Setpoint<std::int32_t, units::angular_velocity::revolutions_per_minute_t>;
using SpeedSetpoint =
  Setpoint<std::uint32_t, units::angular_velocity::revolutions_per_minute_t>;

// Torque: raw thousandths of «Motor rated torque» (0x6076), the unit of
// Target torque (0x6071) and Torque offset (0x60B2), or a real torque.
using TorqueSetpoint = Setpoint<std::int16_t, units::torque::newton_meter_t>;


// --- resolving a setpoint against a mechanism ------------------------------
//
// Each returns the raw value the drive expects. A quantity against an
// unconfigured mechanism has no defined answer, so these report failure
// rather than returning something plausible.

inline bool Resolve(
  const PositionSetpoint & setpoint, const MechanismScale & scale, std::int32_t & out)
{
  if (setpoint.IsRaw()) {
    out = setpoint.GetRaw();
    return true;
  }
  if (!scale.IsValid()) {
    return false;
  }
  out = scale.ToQuadCounts(setpoint.GetQuantity());
  return true;
}

inline bool Resolve(
  const VelocitySetpoint & setpoint, const MechanismScale & scale, std::int32_t & out)
{
  if (setpoint.IsRaw()) {
    out = setpoint.GetRaw();
    return true;
  }
  if (!scale.IsValid()) {
    return false;
  }
  out = scale.ToRpm(setpoint.GetQuantity());
  return true;
}

inline bool Resolve(
  const SpeedSetpoint & setpoint, const MechanismScale & scale, std::uint32_t & out)
{
  if (setpoint.IsRaw()) {
    out = setpoint.GetRaw();
    return true;
  }
  if (!scale.IsValid()) {
    return false;
  }
  // Profile velocity is a magnitude: the direction comes from the target
  // position, not from the sign of the speed.
  const std::int32_t rpm = scale.ToRpm(setpoint.GetQuantity());
  out = static_cast<std::uint32_t>(rpm < 0 ? -rpm : rpm);
  return true;
}

// Against the motor's rated torque rather than the mechanism: the drive's
// torque unit is a fraction of the MOTOR's rating, at the motor shaft. A
// rated torque of zero means the motor data was never configured, and
// there is then no defined answer.
inline bool Resolve(
  const TorqueSetpoint & setpoint, std::uint32_t ratedTorqueMicroNm, std::int16_t & out)
{
  if (setpoint.IsRaw()) {
    out = setpoint.GetRaw();
    return true;
  }
  if (ratedTorqueMicroNm == 0) {
    return false;
  }
  out = ToPerThousand(setpoint.GetQuantity(), ratedTorqueMicroNm);
  return true;
}

}  // namespace epos4
