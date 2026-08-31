#pragma once
#include <cstdint>

namespace fal {

// Heading source for the docking sequence.
//
// Deliberately narrow: this is a PLANAR indoor robot, so yaw is the only axis
// the alignment logic cares about. Exposing the full quaternion would invite
// consumers to depend on data the BNO08x cannot always give us -- on a 4-wire
// hookup with no INT and no RST, reports arrive when they arrive.
//
// valid() is not decoration. Without a reset line the only recovery from an SHTP
// desync is a software reset, so callers MUST be able to ask whether the heading
// they are holding is still real rather than a frozen last-good value.
struct IImu {
  virtual ~IImu() = default;

  virtual bool begin() = 0;

  // Pump the transport. Call every control tick; cheap when nothing has arrived.
  virtual void update(uint32_t now_ms) = 0;

  // Heading in degrees, [-180, 180). Meaningless unless valid() is true.
  virtual float yawDeg() const = 0;

  // True while reports are arriving. Goes false when the sensor stops talking,
  // which is a fault the caller must handle -- not a reason to keep integrating.
  virtual bool valid() const = 0;
};

}  // namespace fal
