#pragma once
#include <cstddef>
#include <cstdint>

namespace tb {

// Per-wheel DC bus current.
//
// With the encoders deferred and no ALARM terminal on the Wheel Drive PCB, this
// is the ONLY hardware feedback the drivetrain has. Treat it accordingly: it is
// observation, not protection -- the real trip chain is the driver's P-sv
// overload trim, then the fuse, then software.
//
// It cannot detect a stall on its own. A stalled motor and a loaded one both
// draw hard; separating them needs speed feedback, which is what the encoders
// were for. What it CAN do is spot a corner drawing unlike its peers.
struct ICurrentSense {
  virtual ~ICurrentSense() = default;

  virtual bool begin() = 0;

  // Sample every channel. Call at the control rate, not faster -- the ADS1115
  // needs ~8 ms per single-shot conversion at 128 SPS.
  virtual void update(uint32_t now_ms) = 0;

  // Amps for one wheel, index order matching tb::Corner (FL, FR, RL, RR).
  // Signed: a bidirectional part reads negative when BRK pushes current back
  // toward the supply, and swallowing that sign would hide regeneration.
  virtual float amps(size_t wheel) const = 0;

  virtual bool valid() const = 0;
};

}  // namespace tb
