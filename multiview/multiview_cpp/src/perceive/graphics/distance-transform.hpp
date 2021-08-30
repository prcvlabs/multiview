
#pragma once

#include "perceive/foundation.hpp"

namespace perceive
{
void distance_transform(uint8_t* grey, uint32_t width, uint32_t height,
                        bool l2_norm, uint8_t cap);

// There is code, in barn-owl/cute3d, for doing dist-trans with gradients

} // namespace perceive
