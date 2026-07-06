// Monotonic millisecond clock port — lets time-based logic (debounce, freshness,
// timeouts) be driven deterministically from tests via a FakeClock.
#pragma once

#include <cstdint>

namespace tb {

struct IClock {
  virtual ~IClock() = default;
  virtual uint32_t millis() const = 0;
};

} // namespace tb
