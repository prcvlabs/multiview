
#include "calib-plane-set.hpp"
#include "cps-operations.hpp"

#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/graphics.hpp"
#include "perceive/utils/create-cv-remap.hpp"

#define This PlaneOpsCalculator

namespace perceive::calibration
{
// ---------------------------------------------------------------- construction
//
This::This(PlaneOpsData& in_ops_data)
    : ops_data(in_ops_data)
    , pjobs(in_ops_data.pjobs)
{
   init(ops_data.bcam_info, ops_data.et0);
}

// ------------------------------------------------------------------------ init
//
void This::init(const BinocularCameraInfo& bcam_info,
                const EuclideanTransform& et0)
{
   const auto et1 = bcam_info.make_et1(et0);

   std::tie(C[0], q[0]) = make_camera_extrinsics(et0);
   std::tie(C[1], q[1]) = make_camera_extrinsics(et1);

   ds.clear();
   std::transform(cbegin(ops_data.cps.p3s),
                  cend(ops_data.cps.p3s),
                  back_inserter(ds),
                  [&](auto& cp) { return cp.p3.w; });
}

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   std::stringstream ss{""};

   ss << format("PlaneOpsCalculator") << endl
      << format("   C[0] = {}", str(C[0])) << endl
      << format("   C[1] = {}", str(C[1])) << endl
      << format("   q[0] = {}", str(q[0])) << endl
      << format("   q[1] = {}", str(q[1])) << endl
      << format("   ds   = [{}]", implode(cbegin(ds), cend(ds), ", ")) << endl
      << endl;

   return ss.str();
}

// --------------------------------------------------------- plane-ray-intersect
//
Vector3 This::plane_ray_intersect(int cam_ind,
                                  const Plane& p3,
                                  const Vector2& D) const noexcept
{
   Expects(cam_ind >= 0 and cam_ind < 2);
   return ::perceive::plane_ray_intersect(
       C[cam_ind], q[cam_ind], ops_data.cu[size_t(cam_ind)], p3, D);
}

// -------------------------------------------------------- project-to-distorted
//
Vector2 This::project_to_distorted(int cam_ind, const Vector3& X) const noexcept
{
   Expects(cam_ind >= 0 and cam_ind < 2);
   return ::perceive::project_to_distorted(
       C[cam_ind], q[cam_ind], ops_data.cu[size_t(cam_ind)], X);
}

Vector2 This::project_to_undistorted(int cam_ind,
                                     const Vector3& X) const noexcept
{
   Expects(cam_ind >= 0 and cam_ind < 2);
   return ::perceive::project_to_undistorted(C[cam_ind], q[cam_ind], X);
}

// ------------------------------------------------------------ update-bcam-info
//
void This::update_bcam_info(BinocularCameraInfo& bcam_info) const noexcept
{
   // EuclideanTransform et0, et1;
   auto et0 = dcam_Cq_to_euclidean_transform(C[0], q[0]);
   auto et1 = dcam_Cq_to_euclidean_transform(C[1], q[1]);
   // et0.translation = C[0];
   // et1.translation = C[1];
   // et0.rotation    = q[0].inverse();
   // et1.rotation    = q[1].inverse();
   auto et01   = (et0 * et1.inverse()).inverse();
   bcam_info.t = -et01.rotation.inverse_rotate(et01.translation.normalised());
   bcam_info.baseline = et01.translation.norm();
   bcam_info.q        = et01.rotation.inverse();
}

// ----------------------------------------------------------------- extract-et0
//
EuclideanTransform This::extract_et0() const noexcept
{
   return dcam_Cq_to_euclidean_transform(C[0], q[0]);
   // EuclideanTransform et0;
   // et0.translation = C[0];
   // et0.rotation    = q[0].inverse();
   // return et0;
}

// ------------------------------------------------------------------ make-mapxy
//
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
    cv::Mat& mapy)
{
   static const Matrix3r H = Matrix3r::Identity();
   static const Matrix3r K = Matrix3r::Identity();

   const int src_cam = left_to_right ? CAM1 : CAM0;
   const int dst_cam = left_to_right ? CAM0 : CAM1;

   auto plane_ray_intersect = [&](int cam_ind, const Vector2& D) {
      auto U = use_cu ? cu[size_t(cam_ind)].undistort(D) : D;
      return ::perceive::plane_ray_intersect(C[cam_ind], q[cam_ind], p3, U);
   };

   auto project_to_undistorted = [&](int cam_ind, const Vector3& X) -> Vector2 {
      return ::perceive::project_to_undistorted(C[cam_ind], q[cam_ind], X);
   };

   auto f = [&](const Vector2& x) -> Vector2 {
      // let's not bother with
      auto X = plane_ray_intersect(src_cam, x);
      auto U = project_to_undistorted(dst_cam, X);
      if(!use_cu) return U;
      auto O = cu[size_t(dst_cam)].in_bounds(U) ? cu[size_t(dst_cam)].distort(U)
                                                : Vector2(-1, -1);
      if(false) {
         INFO(format("x = {}", str(x)));
         INFO(format("X = {}", str(X)));
         INFO(format("U = {}", str(U)));
         INFO(format("O = {}", str(O)));
      }
      return O;
   };

   create_cv_remap_threaded(
       unsigned(w), unsigned(h), H, f, K, mapx, mapy, pjobs, false);
}

void This::make_mapxy(const bool left_to_right,
                      const Plane& p3,
                      const int w,
                      const int h,
                      cv::Mat& mapx,
                      cv::Mat& mapy)
{
   static const Matrix3r H = Matrix3r::Identity();
   static const Matrix3r K = Matrix3r::Identity();

   const int src_cam = left_to_right ? CAM1 : CAM0;
   const int dst_cam = left_to_right ? CAM0 : CAM1;

   auto f = [&](const Vector2& x) -> Vector2 {
      // let's not bother with
      auto X = plane_ray_intersect(src_cam, p3, x);
      auto U = project_to_undistorted(dst_cam, X);
      return ops_data.cu[size_t(dst_cam)].in_bounds(U)
                 ? ops_data.cu[size_t(dst_cam)].distort(U)
                 : Vector2(-1, -1);
   };

   create_cv_remap_threaded(
       unsigned(w), unsigned(h), H, f, K, mapx, mapy, pjobs, false);
}

// ------------------------------------------------------------------ make-mapxy
//
void This::make_mapxy(const bool left_to_right,
                      const int selected_index,
                      cv::Mat& mapx,
                      cv::Mat& mapy)
{
   const auto w = ops_data.w;
   const auto h = ops_data.h;

   static const Matrix3r H = Matrix3r::Identity();
   static const Matrix3r K = Matrix3r::Identity();

   const int src_cam     = left_to_right ? CAM1 : CAM0;
   const int dst_cam     = left_to_right ? CAM0 : CAM1;
   const auto& cp_lookup = ops_data.cp_lookup[size_t(src_cam)];
   const auto& cu        = ops_data.cu[size_t(dst_cam)];

   auto make_plane = [](int type, real d) {
      switch(type) {
      case 0: return Plane(1.0, 0.0, 0.0, d);
      case 1: return Plane(0.0, 1.0, 0.0, d);
      case 2: return Plane(0.0, 0.0, 1.0, d);
      }
      return Plane(0.0, 0.0, 0.0, d);
   };

   auto f = [&](const Vector2& x) -> Vector2 {
      const Point2 p = to_pt2(x);
      if(!cp_lookup.in_bounds(p)) return Vector2(-1, -1);
      const unsigned idx = unsigned(cp_lookup(p));
      if(selected_index != -1 and unsigned(selected_index) != idx)
         return Vector2(-1, -1);
      if(idx >= ops_data.cps.p3s.size()) return Vector2(-1, -1);
      const auto& cp = ops_data.cps.p3s[idx];
      Expects(int(idx) >= 0 and idx < ds.size());
      const auto p3 = make_plane(cp.plane_type, ds[idx]);
      // Plane(cp.p3.xyz(), ds[idx]);

      // let's not bother with
      auto X = plane_ray_intersect(src_cam, p3, x);
      auto U = project_to_undistorted(dst_cam, X);
      return cu.in_bounds(U) ? cu.distort(U) : Vector2(-1, -1);
   };

   create_cv_remap_threaded(
       unsigned(w), unsigned(h), H, f, K, mapx, mapy, pjobs, false);
}

// ----------------------------------------------------------- image-match-score
//
real image_match_score(const cv::Mat& ref, const cv::Mat& src, string im_fname)
{
   Expects(ref.cols == src.cols);
   Expects(ref.rows == src.rows);
   const auto w = ref.cols;
   const auto h = ref.rows;

   GreyImage g;
   if(!im_fname.empty()) {
      g.resize(unsigned(w), unsigned(h));
      g.fill(127);
   }

   auto ret = 0.0;

   const auto black = cv::Vec3b(0, 0, 0);

   int counter = 0;
   for(auto y = 0; y < h; ++y) {
      auto src_ptr = src.ptr<cv::Vec3b>(y);
      auto ref_ptr = ref.ptr<cv::Vec3b>(y);
      for(auto x = 0; x < w; ++x) {
         auto& X = ref_ptr[x];
         auto& Y = src_ptr[x];

         // skip neutral areas
         if(Y == black) continue;
         auto match = cie2000_score(X, Y);
         ret += match;

         if(!im_fname.empty()) g(x, y) = uint8_t(255.0 * (1.0 - match));

         ++counter;
      }
   }

   if(!im_fname.empty()) g.save(im_fname);

   return (counter == 0) ? 1.0 : ret / real(counter);
}

// ------------------------------------------------------ reproject-plane-points
//
ARGBImage reproject_plane_points(const PlaneOpsCalculator& pcalc,
                                 const CalibPlane& cp,
                                 const ImageFeatures2d& f2d0,
                                 const ImageFeatures2d& f2d1,
                                 const bool left_to_right)
{
   const auto& src_im = (left_to_right) ? f2d0.slic_input : f2d1.slic_input;

   auto process_point = [&](const Point2& x) {

   };

   const auto& spixel_inds
       = (left_to_right) ? cp.l_spixel_indices : cp.r_spixel_indices;
   const auto& slic_info = (left_to_right) ? f2d0.slic_info : f2d1.slic_info;

   for(auto spixel_ind : spixel_inds)
      if(spixel_ind >= 0 and spixel_ind < int(slic_info.size()))
         for(auto& x : slic_info[size_t(spixel_ind)].inliers) process_point(x);

   return src_im;
}

// -------------------------------------------------------------
// refine-bcaminfo
//
std::pair<BinocularCameraInfo, EuclideanTransform>
refine_bcaminfo(const BinocularCameraInfo& in_bcam_info,
                const CalibPlaneSet& cps,
                const EuclideanTransform& in_extrinsic,
                const ImageFeatures2d& f2d0,
                const ImageFeatures2d& f2d1)
{
   // 12 degrees of freedom:
   // cam0: rotation, translation (i.e., extrinsic)
   // cam1: rotation, translation, baseline (i.e., binocular)
   auto bcam_info = in_bcam_info;
   auto extrinsic = in_extrinsic;

   //

   return std::make_pair(bcam_info, extrinsic);
}

} // namespace perceive::calibration
