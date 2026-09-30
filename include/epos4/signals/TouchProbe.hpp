#pragma once

#include <cstdint>

namespace epos4::signals
{

// What touch probe 1 has latched, read together so the positions and the
// status describing them come from the same moment.
struct TouchProbeState
{
  bool enabled{false};          // «Touch probe status» (0x60B9) bit 0
  bool positiveEdgeStored{false};  // bit 1
  bool negativeEdgeStored{false};  // bit 2

  // Valid only while the matching *Stored flag is set - the manual is
  // explicit that the status "shall first be evaluated" (6.2.135).
  std::int32_t positiveEdgePosition{0};  // 0x60BA [quadcounts]
  std::int32_t negativeEdgePosition{0};  // 0x60BB [quadcounts]

  // Edges seen since the probe was enabled: 0..1 for a single event,
  // wrapping 0..65535 when continuous (0x60D5 / 0x60D6).
  std::uint16_t positiveEdgeCount{0};
  std::uint16_t negativeEdgeCount{0};
};

// «Touch probe status» (0x60B9, Table 6-165) into the three flags.
constexpr void
DecodeTouchProbeStatus(std::uint16_t status, TouchProbeState & state)
{
  state.enabled = (status & (1u << 0)) != 0;
  state.positiveEdgeStored = (status & (1u << 1)) != 0;
  state.negativeEdgeStored = (status & (1u << 2)) != 0;
}

}  // namespace epos4::signals
