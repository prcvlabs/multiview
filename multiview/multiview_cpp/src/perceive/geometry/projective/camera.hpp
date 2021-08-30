
#pragma once

#include "perceive/foundation.hpp"

#include <opencv2/core/core.hpp>

#include "perceive/geometry/aabb.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/geometry/vector.hpp"

#include "json/json.h"

#include "triangulation.hpp"

namespace perceive
{
class Camera
{
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
   unsigned w_{2}, h_{2};
   real fx_{1.0}, fy_{1.0};   // focal lengths
   Vector2 ppt_{0.0, 0.0};    // Principal point
   Vector3 C_{0.0, 0.0, 0.0}; // Camera centre
   Quaternion q_;             // Rotation (defaults to no rotation)

   // Derived terms
   Matrix3r K_;                     // Intrinsic matrix
   Matrix3r K_inv_;                 // Inverse intrinsic matrix
   real hfov_{0.0};                 // Horizontal field of view
   real vfov_{0.0};                 // Vertical field of view
   real f_{0.0};                    // Average of two above
   real fx_inv_{0.0}, fy_inv_{0.0}; // inverse focal lengths
   Matrix3r R_;                     // Rotation matrix
   Matrix3r R_inv_;                 // Inverse rotation matrix
   Matrix3r KR_inv_;                // (KR)^-1
   Vector3 t_;                      // t = -RC
   Matrix34r P_;                    // Camera matrix
   AABB aabb_;                      // Bounding box of camera
   Plane C_plane_;                  // Parallel to image plane, but thru centre

   // These derived items are calculated on demand.
   // calc-derived will set dirty to true.
   mutable bool dirty_{true};
   mutable cv::Mat mapx_, mapy_;

   void calc_derived();

 public:
   // Construction
   Camera();
   Camera(const Camera&) = default;
   Camera(Camera&&)      = default;
   ~Camera()             = default;
   Camera& operator=(const Camera&) = default;
   Camera& operator=(Camera&&) = default;

   // File IO
   void save(std::string& filename) const;
   static Camera load(std::string& filename);

   // Getters, instrinsic
   unsigned w() const { return w_; }
   unsigned h() const { return h_; }
   const Vector2& ppt() const { return ppt_; }
   real f() const { return f_; }
   real fx() const { return fx_; }
   real fy() const { return fy_; }
   real hfov() const { return hfov_; }
   real vfov() const { return vfov_; }
   const Matrix3r& K() const { return K_; }
   const AABB& aabb() const { return aabb_; }

   // Getters, extrinsic
   const Matrix3r& R() const { return R_; }
   const Quaternion& q() const { return q_; }
   const Vector3& C() const { return C_; } // camera centre
   const Vector3& t() const { return t_; } // t = -RC
   const Matrix34r& P() const { return P_; }

   // Getters misc
   bool is_finite() const; //!< Checks all camera terms (no NANs or infinities)

   // @{ Setters
   // Rescales the intrinsic camera parameters.
   // This is not cropping: the principal point must be handled with care.
   void set_format(unsigned w, unsigned h);

   // Sets the intrinsic parameters for a given camera format
   void
   set_intrinsic(unsigned w, unsigned h, real fx, real fy, const Vector2& ppt);

   // Sets the position and rotation of the camera
   void set_extrinsic(const Vector3& C, const Quaternion& q);

   // Sets [R|t], where R is a rotation matrix, and t = -RC
   void set_extrinsic(const Matrix3r& R, const Vector3& t);
   // @}

   // @{ Operations
   Vector2 normalize(const Vector2& x) const;   // Image => normalized coords
   Vector2 unnormalize(const Vector2& x) const; // Normalized => image coords

   Vector2 project(const Vector3& X) const; // project (R3->R2)
   Vector3 project(const Vector4& X) const; // project (P3->P2)

   // Result is the _direction_ vector (in R3) extending out from camera centre
   Vector3 back_project(const Vector2& X) const;   //<! X in R2
   Vector3 back_project(const Vector3& X) const;   //<! X in P2
   Vector3r back_project(const Vector3r& X) const; //<! X in P2

   // Square reprojection error between of 'x' assuming it is evidence for 'X'
   real reproj_error_sq(const Vector2& x, const Vector3& X) const;
   real reproj_error(const Vector2& x, const Vector3& X) const;

   // Is the passed point in front of the camera?
   bool in_front(const Vector3& X) const;
   // @}

   // @{ String. (IO is done in a separate "JSON" header)
   std::string to_string() const;
   std::string to_str() const { return to_string(); }
   // @}

   // String shim
   friend std::string str(const Camera& v) { return v.to_string(); }
   friend std::ostream& operator<<(std::ostream& out, const Camera& v)
   {
      out << v.to_string();
      return out;
   }

   friend void json_load(const Json::Value& node, Camera&);
   friend Json::Value json_save(const Camera&);
};

// Homography that moves points between two cameras
Matrix3r homography_between(const Matrix34r& Po, const Matrix34r& Pn);

// Find a (fairly symmetric) rectification output two new camera
// matrices Pn1 = [R | 0], Pn2 = [R | t]. Camera centre does not change.
// Output cameras must be premultiplied by Kn  .
void find_rectified_cameras(const Matrix3r& K1,
                            const Matrix3r& K2,
                            const Matrix3r& R,
                            const Vector3r& C1d,
                            const Vector3r& C2d,
                            unsigned w,
                            unsigned h,     // output size
                            Matrix34r& Pn1, // two new cameras
                            Matrix34r& Pn2, // rotation between them is I
                            Matrix3r& Kn,   // ouput K matrix (shared)
                            Matrix3r& H1_out,
                            Matrix3r& H2_out);

} // namespace perceive
