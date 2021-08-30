
#include "colour-set.hpp"
#include "heat-maps.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace perceive
{
static int hm_to_cv_type(const HeatMap hm)
{
   switch(hm) {
#define HM_CASE(x) \
   case HeatMap::x: return cv::COLORMAP_##x;
      HM_CASE(AUTUMN);
      HM_CASE(BONE);
      HM_CASE(COOL);
      HM_CASE(HOT);
      HM_CASE(HSV);
      HM_CASE(JET);
      HM_CASE(OCEAN);
      HM_CASE(PINK);
      HM_CASE(RAINBOW);
      HM_CASE(SPRING);
      HM_CASE(SUMMER);
      HM_CASE(WINTER);
#undef HM_CASE
   }
}

static array<uint32_t, 256> make_heatmap(const int hm_type)
{
   array<uint32_t, 256> ret;

   cv::Mat o(int(ret.size()), 1, CV_8UC1);
   for(auto i = 0; i < o.rows; ++i) o.at<uint8_t>(i, 0) = uint8_t(i);
   cv::applyColorMap(o, o, hm_type);
   for(auto i = 0; i < o.rows; ++i)
      ret[size_t(i)] = vec3b_to_rgb(o.at<cv::Vec3b>(i, 0));
   return ret;
}

static auto all_hms()
{
   array<array<uint32_t, 256>, int(HeatMap::WINTER) + 1> all;
   for(int k = 0; k < int(all.size()); ++k)
      all[size_t(k)] = make_heatmap(hm_to_cv_type(HeatMap(k)));
   return all;
}

uint32_t heat_map(double value, HeatMap hm) noexcept
{
   if(!std::isfinite(value)) return heat_map8(255, hm);
   return heat_map8(uint8_t(std::clamp(255.0 * value, 0.0, 255.0)), hm);
}

uint32_t heat_map8(uint8_t value, HeatMap hm) noexcept
{
   static auto all = all_hms();
   return all[size_t(hm)][value];
}

} // namespace perceive
