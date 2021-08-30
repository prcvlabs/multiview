
#pragma once

#include <opencv2/core/core.hpp>

#include "perceive/foundation.hpp"
#include "perceive/graphics/image-container.hpp"
#include "vector.hpp"

namespace perceive
{
class BinocularCamera;
struct ImageFeatures2d;

constexpr unsigned k_label_out_of_range     = 0;
constexpr unsigned k_label_in_range_general = 1;
constexpr unsigned k_label_floor            = 2;

class PointCloud
{
 public:
   CUSTOM_NEW_DELETE(PointCloud)

   vector<Vector3> Xs; // The 3d points
   vector<Point2> xy;  // xy pixel in reference image
   IntImage lookup;    // Maps [x, y] => index, as in X[ind]
   Vector3 C;          // center of gravity of Xs

   unsigned N() const { return unsigned(Xs.size()); }

   void init(const cv::Mat& disparity,
             const bool left_is_reference, // In general, TRUE
             const BinocularCamera& bino_cam);

   size_t memory_usage() const noexcept;
};

GreyImage make_depth_image(const PointCloud&) noexcept(false);
ARGBImage make_depth_heatmap(const PointCloud&) noexcept(false);

} // namespace perceive
