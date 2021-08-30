
#pragma once

#include "stdinc.hpp"

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
class Slic3d

{
 public:
   struct Params final : public MetaCompatible
   {
      // bool operator==(const Params&) const noexcept;
      // bool operator!=(const Params& o) const noexcept { return !(*this == o);
      // } std::string to_string() const;

      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      real selected_threshold = 0.10;
   };

   struct PlaneResult
   {
      vector<int> plane_ind; // -1 for unassigned planes
      vector<Plane> p3s;
      vector<real> scores;
      vector<bool> selected;

      cv::Mat disparity; // disparity map of the above
      bool disparity_l_ref = false;
   };
};

// 'next' returns TRUE until there are no more pixels
// @return average score for plane
real score_slic_p3(const DistortedCamera& cam0,
                   const DistortedCamera& cam1,
                   const LABImage& lab0,
                   const LABImage& lab1,
                   const vector<Point2>& inliers,
                   const Plane& p3) noexcept;

// @return A plane (could be NAN plane) for each superpixel in `f2d0`
Slic3d::PlaneResult set_p3s_to_slic(const Slic3d::Params& params,
                                    const unsigned cam_num,
                                    const DistortedCamera& cam0,
                                    const DistortedCamera& cam1,
                                    const ImageFeatures2d& f2d0,
                                    const ImageFeatures2d& f2d1,
                                    const bool left_is_ref,
                                    const std::vector<Plane> p3s) noexcept;

} // namespace perceive
