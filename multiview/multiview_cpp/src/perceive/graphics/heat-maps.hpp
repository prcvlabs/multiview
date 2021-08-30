
#pragma once

namespace perceive
{
// @see
// https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
enum class HeatMap : int {
   AUTUMN = 0,
   BONE,
   COOL,
   HOT,
   HSV,
   JET,
   OCEAN,
   PINK,
   RAINBOW,
   SPRING,
   SUMMER,
   WINTER
};

uint32_t heat_map8(uint8_t value, HeatMap hm) noexcept; // value in [0.255]
uint32_t heat_map(double value, HeatMap hm) noexcept;   // value in [0..1]

} // namespace perceive
