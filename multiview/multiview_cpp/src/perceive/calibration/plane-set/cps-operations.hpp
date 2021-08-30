
#pragma once

#include "calib-plane-set.hpp"
#include "plane-opts-data.hpp"
#include "run-calibration.hpp"

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive::calibration
{
struct PlaneOpsCalculator
{
   const PlaneOpsData& ops_data;
   ParallelJobSet& pjobs; // for parallel execution

   // Up to 12 degrees of freedom
   Vector3 C[2];    // Two camera centers
   Quaternion q[2]; // Two rotations, from world to CAM0/1
   vector<real> ds; // 'd' parameters in p3s

   // ---- construction
   PlaneOpsCalculator(PlaneOpsData& in_ops_data);
   void init(const BinocularCameraInfo& bcam_info,
             const EuclideanTransform& et0);

   string to_string() const noexcept;

   // Converts C[2] and q[2] into the relevant parameters in bcam_info
   void update_bcam_info(BinocularCameraInfo& bcam_info) const noexcept;
   EuclideanTransform extract_et0() const noexcept;

   // ----
   Vector3 plane_ray_intersect(int cam_ind,
                               const Plane& p3,
                               const Vector2& D) const noexcept;
   Vector2 project_to_distorted(int cam_ind, const Vector3& X) const noexcept;
   Vector2 project_to_undistorted(int cam_ind, const Vector3& X) const noexcept;

   void make_mapxy(const bool left_to_right,
                   const Plane& p3,
                   const int w,
                   const int h,
                   cv::Mat& mapx,
                   cv::Mat& mapy);

   // From planes from 'ops_data'
   void make_mapxy(const bool left_to_right,
                   const int selected_index, // pass -1 for all planes
                   cv::Mat& mapx,
                   cv::Mat& mapy);

   friend string str(const calibration::PlaneOpsCalculator& pcalc) noexcept
   {
      return pcalc.to_string();
   }
};

real image_match_score(const cv::Mat& ref,
                       const cv::Mat& src,
                       string im_fname = ""s);

// Reproject
ARGBImage reproject_plane_points(const PlaneOpsCalculator& pcalc,
                                 const CalibPlane& cp,
                                 const ImageFeatures2d& f2d0,
                                 const ImageFeatures2d& f2d1,
                                 const bool left_to_right);

// refine-bcaminfo
std::pair<BinocularCameraInfo, EuclideanTransform>
refine_bcaminfo(const BinocularCameraInfo& bcam_info,
                const CalibPlaneSet& cps,
                const EuclideanTransform& extrinsic,
                const ImageFeatures2d& f2d0,
                const ImageFeatures2d& f2d1);

void make_cam_p3_cam_transfer_mapxy(
    const bool left_to_right,
    const Plane& p3,
    const int w,
    const int h,
    const array<CachingUndistortInverse, 2>& cu,
    const bool use_cu,     // Perform distort/undistort operations
    const Vector3 C[2],    // Two camera centers
    const Quaternion q[2], // Two rotations, from world to CAM0/1
    ParallelJobSet& pjobs,
    cv::Mat& mapx,
    cv::Mat& mapy);

} // namespace perceive::calibration
