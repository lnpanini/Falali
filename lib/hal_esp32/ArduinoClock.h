// IClock backed by Arduino millis().
#pragma once

#include <Arduino.h>

#include "IClock.h"

namespace tb {

class ArduinoClock : public IClock {
public:
  uint32_t millis() const override { return ::millis(); }
};

} // namespace tb
