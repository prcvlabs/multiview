
#pragma once

#include <stdint.h>
#include <stdlib.h>

namespace perceive
{
// Canny with adaptive threshold. Returns thresholds used when ptrs not NULL
void canny(const uint8_t* grey,
           uint32_t width,
           uint32_t height,
           uint8_t* edges,
           double smooth_sigma = 2.0,
           double* low_ptr     = nullptr, // NULL implies auto-select
           double* high_ptr    = nullptr);   // NULL implies auto-select

} // namespace perceive
