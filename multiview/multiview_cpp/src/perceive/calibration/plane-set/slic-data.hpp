
#pragma once

#include "perceive/graphics/image-container.hpp"

namespace perceive::calibration
{
struct SlicData
{
   ARGBImage input_image;
   unsigned superpixel_size{0};
   real compactness{0.0};
   unsigned n_labels{0};
   IntImage labels;
   ARGBImage contours;

   struct SpixelInfo
   {
      vector<Point2> inliers; // all inliers to the spixel
      vector<Point2> borders; // just the bordering pixels
      Vector2 center{0, 0};
   };
   vector<SpixelInfo> slic_info;

   bool init(const ARGBImage image,
             unsigned superpixel_size,
             real compactness,
             std::function<bool()> is_canceled) noexcept;
};

} // namespace perceive::calibration
