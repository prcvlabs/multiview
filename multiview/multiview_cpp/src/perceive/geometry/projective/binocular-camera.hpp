
#pragma once

#include "json/json.h"
#include <opencv2/core/core.hpp>

#include "caching-undistort-inverse.hpp"
#include "polynomial-model.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/vector.hpp"

#include "triangulation.hpp"

namespace perceive
{
constexpr int CAM0 = 0;
constexpr int CAM1 = 1;

// ------------------------------------------------------- Binocular Camera Info
// For reading/writing cameras to disk
struct BinocularCameraInfo
{
   string camera_id;

   Quaternion q; // from CAM0 to CAM1's point of view
   Vector3 t{-1.0, 0.0, 0.0};
   real baseline{0.15};
   array<DistortionModel, 2> M;

   CUSTOM_NEW_DELETE(BinocularCameraInfo)

   int n_sensors() const noexcept { return int(M.size()); }

   bool operator==(const BinocularCameraInfo&) const noexcept;
   bool operator!=(const BinocularCameraInfo& o) const noexcept
   {
      return !(*this == o);
   }
   string to_string() const noexcept { return to_json_string(); }
   string to_json_string() const noexcept;

   // The euclidean transfrom from sensor 0 to sensor 1
   EuclideanTransform euclid_transform() const noexcept;
   void set_from_et(const EuclideanTransform&) noexcept;

   // If 'et0' is transform for cam0, this returns transform for cam1
   // NOTE: the euclid transform 'et0' gives 'eye-coords' => 'world-coords'.
   // Thus, et0.inverse(X) gives 'world=>eye', which is used for projecting to
   // camera images.
   EuclideanTransform make_et1(const EuclideanTransform& et0) const noexcept;

   // Returns the Essential Matrix
   Matrix3r E() const
   {
      Matrix3r Tx = make_skew_symmetric(t.normalised());
      Matrix3r R  = quaternion_to_rot3x3(q);
      return Tx * R;
   }

   Vector3 C0() const noexcept { return Vector3(0, 0, 0); }
   Vector3 C1() const noexcept
   {
      return -baseline * q.inverse_rotate(t.normalized());
   }
   Vector3 C(int cam) const noexcept { return (cam == 0) ? C0() : C1(); }

   // Rectifies the ideal coordinate (to a homogenous ideal coordinate)
   Vector2 rectify_ideal(int cam, const Vector2& x) const noexcept;

   Vector3 solve3d_from_distorted(const Vector2& d0,
                                  const Vector2& d1) const noexcept;
   real reproj_err_from_distorted(const Vector2& d0,
                                  const Vector2& d1) const noexcept;

   Vector2 project(int cam, const Vector3& X) const noexcept;

   // Find 'X' on plane 'p3' that intersects with ray from distorted
   // point 'D' in cam 'cam'. 'et0' is the extrinsic transform for
   // Cam0. The transform for Cam1 is calculated from 'q', 't', 'baseline', and
   // 'et0'.
   Vector3 plane_ray_intersect(const int cam,
                               const EuclideanTransform& et0,
                               const Plane& p3,
                               const Vector2& D) const noexcept;

   Vector3 to_ray(int cam, const Vector3& X) const noexcept;
};

void load(BinocularCameraInfo& data, const string& fname) noexcept(false);
void save(const BinocularCameraInfo& data, const string& fname) noexcept(false);
void read(BinocularCameraInfo& data, const std::string& in) noexcept(false);
void write(const BinocularCameraInfo& data, std::string& out) noexcept(false);
void read(BinocularCameraInfo& data, const Json::Value& node) noexcept(false);
void write(const BinocularCameraInfo& data, Json::Value& node) noexcept(false);

/// Print debug info on triangulation 3D points P and Q
/// If 'K' is zero, then points [p0, q0] (left image) and
/// [p1, q1] (right image) are considered distorted, and are
/// undistorted using bcam_info.M[cam_ind]. Otherwise 'K'
/// is used to turn them into rays. If the points are already
/// ideal, then set 'K' to the identity matrix.
void print_debug_triangulation(const string& label,
                               const BinocularCameraInfo& bcam_info,
                               const Vector2& p0,
                               const Vector2& q0,
                               const Vector2& p1,
                               const Vector2& q1,
                               const Matrix3r K = Matrix3r::Zero()) noexcept;

/// Print debug info on triangulating 3D points P and Q
/// If 'K' is zero, then points [p0, q0] (left image) and
/// [p1, q1] (right image) are considered distorted, and are
/// undistorted using bcam_info.M[cam_ind]. Otherwise 'K'
/// is used to turn them into rays. If the points are already
/// ideal, then set 'K' to the identity matrix.
void print_debug_p3_intersect(const string& label,
                              const BinocularCameraInfo& bcam_info,
                              const Plane& p3,
                              const Vector2& p0,
                              const Vector2& q0,
                              const Vector2& p1,
                              const Vector2& q1,
                              const Matrix3r K = Matrix3r::Zero()) noexcept;

// 'e0' and 'e1' are estimated euclidean transforms for each camera.
// They are averaged, and then the bino-cam-info is used to calculate
// the output transformtions.
std::pair<EuclideanTransform, EuclideanTransform>
estimate_bino_camera_euclid_transforms(const BinocularCameraInfo& info,
                                       const EuclideanTransform& e0,
                                       const EuclideanTransform& e1);

Vector2
transfer_point_between_images(const Vector2& x0,
                              const Plane& p3_et0,           // et0 coords
                              const EuclideanTransform& et0, // world->cam
                              const EuclideanTransform& et1, // world->cam
                              const CachingUndistortInverse& cu0,
                              const CachingUndistortInverse& cu1) noexcept;

// ------------------------------------------------------------ Binocular Camera

class BinocularCamera
{
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
   string camera_id_{""s};

   unsigned w_{0};
   unsigned h_{0};

   DistortionModel M_[2];

   // array<shared_ptr<CachingUndistortInverse>, 2> cu_;
   CachingUndistortInverse cu_[2];

   Quaternion q_;
   Matrix3r R_;
   Matrix3r R_inv_;
   Vector3 t_;
   real baseline_{0.0};

   Quaternion rect_q_;

   Vector3 C_[2]; // Camera centers

   Matrix3r HR_[2];
   Matrix3r Kn_[2];
   Matrix3r KR_[2];
   Matrix3r KRR_[2];
   Matrix3r KRR_inv_[2];

   bool feedback_{false};

 public:
   BinocularCamera();
   BinocularCamera(const BinocularCamera&) = default;
   BinocularCamera(BinocularCamera&&)      = default;
   ~BinocularCamera()                      = default;
   BinocularCamera& operator=(const BinocularCamera&) = default;
   BinocularCamera& operator=(BinocularCamera&&) = default;

   void init(const Quaternion& q,
             const Vector3& t,
             const real baseline,
             const unsigned distorted_w,
             const unsigned distorted_h,
             const Matrix3r& K,
             const unsigned width,
             const unsigned height,
             const bool use_calib_roi);

   void init(const DistortionModel& M0,
             const DistortionModel& M1,
             const Quaternion& q,
             const Vector3& t,
             const real baseline,
             const unsigned distorted_w,
             const unsigned distorted_h,
             const Matrix3r& K,
             const unsigned width,
             const unsigned height,
             const bool use_calib_roi);

   // @param info           The binocular camera info
   // @param distorted_w/h  The w/h of distorted images
   // @param K, w/h         The parameters for undistorted images
   void init(const BinocularCameraInfo& info,
             const unsigned distorted_w,
             const unsigned distorted_h,
             const Matrix3r& K, // Ignored if 'use-calib-roi' is true
             const unsigned width,
             const unsigned height,
             const bool use_calib_roi);

   void set_baseline(real baseline);

   void set_working_format(const unsigned distorted_w,
                           const unsigned distorted_h);

   // If 'et0' is transform for cam0, this returns transform for cam1
   // NOTE: the euclid transform 'et0' gives 'eye-coords' => 'world-coords'.
   // Thus, et0.inverse(X) gives 'world=>eye', which is used for projecting to
   // camera images.
   EuclideanTransform make_et1(const EuclideanTransform& et0) const noexcept;

   // Solve 3D from points in the rectified images
   Vector3 solve3d(real x0, real x1, real y) const noexcept;
   Vector3 solve3d(const Vector2& x0, const Vector2& x1) const noexcept;
   Vector3 solve3d(real x0, real y0, real x1, real y1) const noexcept;

   // Solve 3D from points in distorted images
   Vector3 solve3d_from_distorted(const Vector2& x0,
                                  const Vector2& x1) const noexcept;

   // Solve 3D from rays, where each ray is from the point of view
   // of the camera. (i.e., undistorted points)
   Vector3 solve3d_from_rays(const Vector3& ray0,
                             const Vector3& ray1) const noexcept;
   Vector3 solve3d_from_undistorted(const Vector2& u0,
                                    const Vector2& u1) const noexcept;

   // 3d error for solve3d using specified values. i.e., using these
   // values, we expect to generate four planar points for ray-ray
   // intersection. This method returns a 3d non-planarity measure
   real solve3d_3d_err(real x0, real y0, real x1, real y1) const noexcept;

   // Reproj error (from rectified)
   real reproj_err(real x0, real y0, real x1, real y1) const noexcept;
   real reproj_err(const Vector2& x0, const Vector2& x1) const noexcept;
   real reproj_err_from_distorted(const Vector2& d0,
                                  const Vector2& d1) const noexcept;

   // Convert a distorted point to a ray -- from individual cam's point of view
   Vector3 to_ray(int cam_ind, const Vector2& distorted) const noexcept;
   Vector3 to_ray(int cam_ind, const Vector3r& distorted) const noexcept;

   // Convert a point in the recitifed image to a ray
   Vector3 rectified_to_ray(int cam_ind, const Vector2& x) const noexcept;
   Vector3 rectified_to_ray(int cam_ind, const Vector3r& x) const noexcept;

   // Project a 3d point to the distorted image
   Vector2 project(int cam_ind, const Vector3& X) const;

   // Project a 3d point to the rectified image
   Vector2 project_to_rectified(int cam_ind, const Vector3& X) const;

   real disparity(const Vector3& X) const; // Distance between projected point

   // Epipolar "ellipses"... actually quadric beziers
   // 'X' is a pixel in the left camera,
   // 'min/max-dist' give the limits of the bezier
   // 'A', 'B', and 'C' are the three control points of the output bezier
   void epipolar_curve(const Vector2& X,
                       real min_dist,
                       real max_dist,
                       Vector2& A,
                       Vector2& B,
                       Vector2& C) const;

   // void calculate_mapxy(cv::Mat mapx[2], cv::Mat mapy[2]) const;
   void get_mapxy(cv::Mat mapx[2], cv::Mat mapy[2]) const; // may calculate
   void
   get_sensor_mapxy(unsigned sensor_num, cv::Mat& mapx, cv::Mat& mapy) const;

   unsigned M_width() const noexcept
   {
      return unsigned(M_[0].calib_format().x);
   }
   unsigned M_height() const noexcept
   {
      return unsigned(M_[0].calib_format().y);
   }
   unsigned w() const noexcept { return w_; }
   unsigned h() const noexcept { return h_; }

   const string& camera_id() const noexcept { return camera_id_; }

   const DistortionModel& model(int ind) const noexcept { return M_[ind]; }
   const CachingUndistortInverse& cu(int ind) const noexcept
   {
      return cu_[ind];
   }

   const Quaternion& q() const noexcept { return q_; } // world->cam for CAM1
   const Matrix3r& R() const noexcept { return R_; }
   const Vector3& t() const noexcept { return t_; }
   const Vector3& C(int ind) const noexcept { return C_[ind]; }
   real baseline() const noexcept { return baseline_; }
   const Matrix3r& Kn(int ind) const noexcept { return Kn_[ind]; }

   // These are the rectification homographies
   const Matrix3r& KR(int ind) const noexcept { return KR_[ind]; }
   const Matrix3r& KRR(int ind) const noexcept { return KRR_[ind]; }
   const Matrix3r& KRR_inv(int ind) const noexcept { return KRR_inv_[ind]; }

   const Quaternion& rectify_rotation() const noexcept { return rect_q_; }

   Vector2
   rectify_distorted(int cam_ind,
                     const Vector2& D) const noexcept; // Distorted->rectified
   Vector2
   rectify_ideal(int cam_ind,
                 const Vector2& U) const noexcept; // Undistorted->rectified
   Vector2
   unrectify_to_ideal(int cam_ind,
                      const Vector2& R) const noexcept; // rectified->ideal
   Vector2 unrectify_to_distorted(int cam_ind, const Vector2& R)
       const noexcept; // rectified->distorted

   // const std::string& cache_filename() const noexcept
   //{return cache_filename_;}

   void set_feedback(bool value) { feedback_ = value; }

   // Takes distorted point 'D' in camera 'cam-ind', and finds the
   // plane-ray intersection with plane 'p3'
   Vector3 plane_ray_intersect(int cam_ind,
                               const EuclideanTransform& extrinsic,
                               const Plane& p3,
                               const Vector2& D) const noexcept;

   Vector2 backproject_to_distorted(int cam_ind, const Vector3& X);

   std::string to_string() const;

   friend std::string str(const BinocularCamera& cam)
   {
      return cam.to_string();
   }
};

// --------------------------------------------------------------------- Project

inline Vector2
project(const Matrix3r& R, const Vector3& t, const Vector3& X) noexcept
{
   return homgen_P2_to_R2(to_vec3(R * to_vec3r(X)) + t);
}

// ---------------------------------------------------------------------

Matrix3r well_calibrated_region(const DistortionModel& M0,
                                const DistortionModel& M1,
                                const Matrix3r& H0,
                                const Matrix3r& H1,
                                const unsigned width,
                                const unsigned height,
                                AABB& inscribed_rect);

// ------------------------------------------------------------------- fit plane
// @param bcam_info Binocular Camera
// @param Xs Contiguous array of image points in Cam0
// @param Ys Corresponding contiguous array of points in Cam1
// @param N Number of points. Xs[N-1] and Ys[N-1] must be valid.
// @param K If non-zero, then points are turned to rays with K.inverse(),
//          otherwise bcam_info.M[cam_id].undistort is used.
// Note: Call print_debug_p3_intersect to examine result.
Plane fit_plane(const BinocularCameraInfo& bcam_info,
                const Vector2* xs,
                const Vector2* ys,
                const unsigned N,
                const bool feedback = false,
                const Matrix3r K    = Matrix3r::Zero()) noexcept;

} // namespace perceive
