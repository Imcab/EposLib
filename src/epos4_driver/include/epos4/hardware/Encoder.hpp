#pragma once

#include <cstdint>
#include <system_error>

#include "epos4/configs/EncoderConfigs.hpp"
#include "epos4/signals/StatusSignal.hpp"

namespace epos4
{

class Epos4;

// ---------------------------------------------------------------------------
// The feedback subsystem of one drive, reached through Epos4::GetEncoder().
//
//   auto & encoder = motor.GetEncoder();
//
//   encoder.Apply(epos4::configs::SensorsConfigs{
//     .sensor1 = configs::Sensor1Type::kDigitalIncrementalEncoder1,
//     .sensor3 = configs::Sensor3Type::kDigitalHallSensor});
//
//   epos4::configs::DigitalIncrementalEncoderConfigs enc;
//   enc.pulsesPerRevolution = 1024;
//   enc.type = configs::IncrementalEncoderType{
//     .index = configs::IndexType::kWithIndex};
//   encoder.Apply(enc);
//
//   encoder.GetPosition().Refresh().GetValue();
//   encoder.GetQuadCountsPerRevolution();
//
// It is a view onto the same device, not an owner: it holds a reference to
// the Epos4 and does not outlive it. Separate from Epos4 because feedback is
// its own subject with five sensor types and three slots, and folding all of
// that into the device class would bury the handful of methods anyone
// actually calls day to day.
// ---------------------------------------------------------------------------
class Encoder
{
public:
  explicit Encoder(Epos4 & device);

  // -------------------------------------------------------------------------
  // Configuration
  //
  // Every Apply below refuses to run while the drive is powered and returns
  // std::errc::operation_not_permitted. The manual states for each encoder
  // type word: "Write access is only permitted in device state «Power
  // Disable»". Letting the writes through anyway would produce a partially
  // applied configuration and a pile of SDO aborts, which is worse than not
  // starting.
  // -------------------------------------------------------------------------

  // WARNING: changing which sensors exist clears the absolute position.
  // The manual: "«Position referenced to home position», Position actual
  // value, and Additional position actual values will be cleared." An axis
  // that was homed is no longer referenced afterwards and must be homed
  // again before any stored pose means anything.
  std::error_code Apply(const configs::SensorsConfigs & config);

  std::error_code Apply(const configs::DigitalIncrementalEncoderConfigs & config);
  std::error_code Apply(const configs::AnalogIncrementalEncoderConfigs & config);
  std::error_code Apply(const configs::SsiAbsoluteEncoderConfigs & config);
  std::error_code Apply(const configs::HallSensorConfigs & config);

  // Reads 0x3000:01 back and decodes it into the slot types.
  std::error_code ReadSensorsConfiguration(configs::SensorsConfigs & out);

  // -------------------------------------------------------------------------
  // Readings
  // -------------------------------------------------------------------------

  // Axis position, 0x6064 [position units]. The same signal the device
  // exposes; offered here so feedback code does not have to reach back out
  // to the device for the one value it cares about most.
  signals::StatusSignal<std::int32_t> & GetPosition();

  // Axis velocity, 0x606C [velocity units].
  signals::StatusSignal<std::int32_t> & GetVelocity();

  // Position at the last detected index pulse, 0x3010:04 or 0x3020:04
  // [increments]. Only meaningful on a 3-channel encoder; it is what a
  // homing run against the index actually lands on.
  signals::StatusSignal<std::int32_t> & GetIndexPosition(std::uint8_t encoderNumber = 1);

  // Live state of the three Hall sensors as a pattern, 0x301A:02. An
  // impossible combination here is what 0x7388 "Hall sensor error" reports,
  // so this is the value to look at when commissioning wiring.
  signals::StatusSignal<std::uint16_t> & GetHallPattern();

  // Raw SSI frame position, 0x3012:09, before any scaling. Useful to confirm
  // the bit layout is right: a wrong single-turn count shows up here as a
  // value that jumps or wraps at the wrong place.
  signals::StatusSignal<std::uint32_t> & GetSsiRawPosition();

  // Main sensor resolution as the drive computed it, 0x3000:05
  // [quadcounts/revolution]. Worth reading back after configuring: if it does
  // not match QuadCountsPerRevolution() below, the drive and the application
  // disagree about how far one turn is.
  signals::StatusSignal<std::uint32_t> & GetMainSensorResolution();

  // -------------------------------------------------------------------------
  // Unit helpers
  //
  // The conversion the manual states for incremental encoders:
  //     4 x pulses/rev = increments/rev = quadcounts/rev
  // An encoder sold as "500 CPR" is 500 pulses and 2000 quadcounts. Silently
  // getting this factor of four wrong scales every move on the axis.
  // -------------------------------------------------------------------------
  static constexpr std::uint32_t QuadCountsPerRevolution(std::uint32_t pulsesPerRevolution)
  {
    return pulsesPerRevolution * 4u;
  }

  // Resolution of an analog SinCos encoder, 2^interpolationBits x periods.
  // The manual bounds the result to 64 .. 10'000'000 inc/rev.
  static constexpr std::uint32_t SinCosResolution(
    std::uint32_t periodsPerTurn, std::uint8_t interpolationBits)
  {
    return periodsPerTurn * (1u << interpolationBits);
  }

private:
  std::error_code ApplyWhilePowerDisabled(const configs::ConfigWrites & writes);

  Epos4 & device_;

  // Per-instance, not static. Each axis has its own encoder state; sharing
  // one signal between devices would have every joint reporting whichever
  // one was read last.
  signals::StatusSignal<std::int32_t> indexPosition1_;
  signals::StatusSignal<std::int32_t> indexPosition2_;
  signals::StatusSignal<std::uint16_t> hallPattern_;
  signals::StatusSignal<std::uint32_t> ssiRawPosition_;
  signals::StatusSignal<std::uint32_t> mainSensorResolution_;
};

}  // namespace epos4
