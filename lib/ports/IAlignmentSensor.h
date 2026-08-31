// The upward-facing alignment-sensing layer. Deliberately abstract: it yields a
// frame of zone readings and hides whether that comes from N single-point VL53L0X
// (XSHUT re-addressing — the only backend today) or an array part (VL53L5CX)
// later. A TCA9548A mux backend existed until 2026-08-31.
#pragma once

#include "types.h"

namespace fal {

struct IAlignmentSensor {
  virtual ~IAlignmentSensor() = default;

  // Initialise the hardware. Returns false on failure.
  virtual bool begin() = 0;

  // Latest frame of zone readings (invalid zones flagged, not dropped).
  virtual AlignmentFrame read() = 0;

  // Number of zones this backend reports.
  virtual size_t zoneCount() const = 0;
};

} // namespace fal
