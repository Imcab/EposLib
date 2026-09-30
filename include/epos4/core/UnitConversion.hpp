#pragma once

#include <cstdint>

#include <ros2units/units.h>

namespace epos4
{

// ---------------------------------------------------------------------------
// Between the drive's raw integers and physical quantities.
//
// Quantities are nholthaus/units types, vendored by the robot_units package -
// the same library WPILib's C++ bindings use - so anything else on the robot
// already speaks them.
//
// WHY THE DRIVE CANNOT DO THIS ITSELF
//
// The EPOS4 has a factor group - «SI unit position» (0x60A8), «SI unit
// velocity» (0x60A9), «SI unit acceleration» (0x60AA) - but it is almost
// entirely fixed:
//
//   0x60A8  position       ONE legal value: 0x00B50000, increments
//   0x60AA  acceleration   ONE legal value: 0x00C00300, rpm/s
//   0x60A9  velocity       rpm, with a prefix from 10^0 down to 10^-6
//
// So it cannot report degrees however it is configured. Position is always
// quadcounts and acceleration always rpm/s. Converting needs the encoder
// resolution and the gear ratio, which are properties of the machine and not
// of the drive.
//
// THE FACTOR OF FOUR
//
// An encoder sold as "500 CPR" produces 500 pulses per turn and 2000
// quadcounts, because the drive counts all four edges of the quadrature pair.
// The manual states it as
//
//     4 x pulses/rev = increments/rev = quadcounts/rev
//
// Everything below is in quadcounts, which is what 0x6064 reports.
// ---------------------------------------------------------------------------
class MechanismScale
{
public:
  MechanismScale() = default;

  // quadCountsPerRevolution is of the MOTOR shaft: four times the encoder's
  // pulses per revolution.
  //
  // gearRatio is output turns per motor turn, so a 1:100 reduction is
  // 1.0/100.0. With one set, every quantity below is at the OUTPUT - the
  // joint - which is what application code cares about. Leave it at 1.0 for
  // a direct drive, or when the encoder is already after the gearbox.
  constexpr MechanismScale(std::uint32_t quadCountsPerRevolution, double gearRatio = 1.0)
  : quadCounts_(quadCountsPerRevolution), gearRatio_(gearRatio) {}

  constexpr bool IsValid() const {return quadCounts_ != 0 && gearRatio_ != 0.0;}
  constexpr std::uint32_t GetQuadCountsPerRevolution() const {return quadCounts_;}
  constexpr double GetGearRatio() const {return gearRatio_;}

  // --- position: 0x6064 and 0x607A are in quadcounts ---

  constexpr units::angle::turn_t ToAngle(std::int32_t quadCounts) const
  {
    return units::angle::turn_t{
      static_cast<double>(quadCounts) / static_cast<double>(quadCounts_) * gearRatio_};
  }

  // Rounds to nearest rather than truncating. Truncation is not a rounding
  // detail here: a command that round-trips through the conversion loses a
  // count every time, and the axis walks.
  std::int32_t ToQuadCounts(units::angle::turn_t angle) const
  {
    return Round(angle.value() / gearRatio_ * static_cast<double>(quadCounts_));
  }

  // --- velocity: 0x606C and 0x60FF are in rpm OF THE MOTOR ---
  //
  // With the default «SI unit velocity» of 0x00B44700 that is plain rpm. If
  // the velocity prefix is changed, scale the raw value first.

  constexpr units::angular_velocity::revolutions_per_minute_t ToAngularVelocity(
    std::int32_t rpm) const
  {
    return units::angular_velocity::revolutions_per_minute_t{
      static_cast<double>(rpm) * gearRatio_};
  }

  std::int32_t ToRpm(units::angular_velocity::revolutions_per_minute_t velocity) const
  {
    return Round(velocity.value() / gearRatio_);
  }

  // --- acceleration: always rpm/s, 0x60AA has no other legal value ---
  //
  // Expressed as revolutions per minute per second. nholthaus has no such
  // unit, and inventing one would be worse than being explicit: the value
  // stays a plain number with the unit named here and in the manual.

  constexpr double ToRpmPerSecondAtOutput(std::uint32_t rpmPerSecond) const
  {
    return static_cast<double>(rpmPerSecond) * gearRatio_;
  }

  std::uint32_t FromRpmPerSecondAtOutput(double rpmPerSecondAtOutput) const
  {
    const double atMotor = rpmPerSecondAtOutput / gearRatio_;
    return static_cast<std::uint32_t>(Round(atMotor < 0.0 ? -atMotor : atMotor));
  }

private:
  static std::int32_t Round(double value)
  {
    return static_cast<std::int32_t>(value < 0.0 ? value - 0.5 : value + 0.5);
  }

  std::uint32_t quadCounts_{0};
  double gearRatio_{1.0};
};


// ---------------------------------------------------------------------------
// Torque, which needs no encoder - only the rated torque.
//
// 0x6071 and 0x6077 are in THOUSANDTHS of «Motor rated torque» (0x6076),
// which the drive computes as Nominal current x Torque constant and reports
// in micronewton metres.
// ---------------------------------------------------------------------------

constexpr units::torque::newton_meter_t ToTorque(
  std::int16_t perThousand, std::uint32_t ratedTorqueMicroNm)
{
  return units::torque::newton_meter_t{
    static_cast<double>(perThousand) * static_cast<double>(ratedTorqueMicroNm) / 1e9};
}

inline std::int16_t ToPerThousand(
  units::torque::newton_meter_t torque, std::uint32_t ratedTorqueMicroNm)
{
  if (ratedTorqueMicroNm == 0) {
    // Zero means the motor data has not been configured. Returning a
    // plausible number would command torque against an unknown scale.
    return 0;
  }
  const double value = torque.value() * 1e9 / static_cast<double>(ratedTorqueMicroNm);
  return static_cast<std::int16_t>(value < 0.0 ? value - 0.5 : value + 0.5);
}


// --- current: the drive reports and takes milliamps ---

constexpr units::current::ampere_t ToCurrent(std::int32_t milliamps)
{
  return units::current::ampere_t{static_cast<double>(milliamps) / 1000.0};
}

inline std::int32_t ToMilliamps(units::current::ampere_t current)
{
  const double value = current.value() * 1000.0;
  return static_cast<std::int32_t>(value < 0.0 ? value - 0.5 : value + 0.5);
}


// --- supply voltage: «Power supply voltage» (0x2200:01), tenths of a volt ---

constexpr units::voltage::volt_t ToVoltage(std::uint16_t decivolts)
{
  return units::voltage::volt_t{static_cast<double>(decivolts) / 10.0};
}


// --- temperature: «Temperature power stage» (0x3201), tenths of a degree ---
//
// Signed on purpose: the object is INTEGER16, and a rover on a cold morning
// starts below zero.

constexpr units::temperature::celsius_t ToTemperature(std::int16_t decidegrees)
{
  return units::temperature::celsius_t{static_cast<double>(decidegrees) / 10.0};
}

}  // namespace epos4
