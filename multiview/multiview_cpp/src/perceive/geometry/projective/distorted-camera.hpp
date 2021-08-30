
#pragma once

#include "binocular-camera.hpp"
#include "caching-undistort-inverse.hpp"
#include "polynomial-model.hpp"

#include "perceive/geometry/box.hpp"
#include "perceive/geometry/cylinder.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
// ------------------------------------------------------------ Distorted Camera
// Light-weight camera object with extrinsic parameters
struct DistortedCamera
{
   Vector3 C;                  // camera center
   Quaternion q;               // rotation, World to CAM
   CachingUndistortInverse cu; // distort and undistort
   int w = 0;
   int h = 0;

   CUSTOM_NEW_DELETE(DistortedCamera)

   EuclideanTransform et() const noexcept;

   string to_string() const noexcept;

   friend string str(const DistortedCamera& o) noexcept
   {
      return o.to_string();
   }
};

DistortedCamera make_distorted_camera(const EuclideanTransform& et,
                                      const CachingUndistortInverse& cu,
                                      const unsigned w,
                                      const unsigned h);

std::pair<DistortedCamera, DistortedCamera>
make_distorted_camera_pair(const BinocularCameraInfo& bcam_info,
                           const EuclideanTransform& et0,
                           const unsigned w, // to set working format
                           const unsigned h);

std::pair<DistortedCamera, DistortedCamera>
make_distorted_camera_pair(const BinocularCamera& bcam,
                           const EuclideanTransform& et0,
                           const unsigned w, // to set working format
                           const unsigned h);

// -------------------------------------- euclidean-transform to/from extrinsics
// Convert EuclideanTransforms to/from
// camera extrinsics
std::pair<Vector3, Quaternion>
make_camera_extrinsics(const EuclideanTransform& et) noexcept;

EuclideanTransform dcam_Cq_to_euclidean_transform(const Vector3& C,
                                                  const Quaternion& q) noexcept;

// ----------------------------------------------------------- projecting points
// Project 'X' into a camera point
inline Vector2
project_to_undistorted(const Vector3& C,    // cam center
                       const Quaternion& q, // Rotation world to CAM
                       const Vector3& X) noexcept
{
   return homgen_P2_to_R2(q.apply(X - C));
}

inline Vector3
project_to_normalized(const Vector3& C,    // cam center
                      const Quaternion& q, // Rotation world to CAM
                      const Vector3& X) noexcept
{
   return (q.apply(X - C)).normalise_point();
}

inline Vector2
project_to_distorted(const Vector3& C,    // cam center
                     const Quaternion& q, // Rotation world to CAM
                     const CachingUndistortInverse& cu,
                     const Vector3& X) noexcept
{
   return cu.distort(project_to_undistorted(C, q, X));
}

inline Vector3 project_to_normalized(const DistortedCamera& dcam,
                                     const Vector3& X) noexcept
{
   return project_to_normalized(dcam.C, dcam.q, X);
}

inline Vector2 project_to_undistorted(const DistortedCamera& dcam,
                                      const Vector3& X) noexcept
{
   return project_to_undistorted(dcam.C, dcam.q, X);
}

// Ray is in camera space (not world space)
inline Vector3 project_to_ray(const DistortedCamera& dcam,
                              const Vector3& X) noexcept
{
   return homgen_R2_to_P2(project_to_undistorted(dcam, X)).normalised();
}

inline Vector2 project_to_distorted(const DistortedCamera& cam,
                                    const Vector3& X) noexcept
{
   return project_to_distorted(cam.C, cam.q, cam.cu, X);
}

// ------------------------------------------------------------- undistort-to-P2
// Distorted coordinate to homogenous normalized coordinate
inline Vector3 undistort_to_P2(const CachingUndistortInverse& cu,
                               const Vector2& D)
{
   return homgen_R2_to_P2(cu.undistort(D));
}

inline Vector3 undistort_to_P2(const DistortedCamera& dcam, const Vector2& D)
{
   return undistort_to_P2(dcam.cu, D);
}

// ------------------------------------------------------------- backproject-ray

inline Vector3 backproject_ray(const Quaternion& q, // Rotation world to CAM
                               const CachingUndistortInverse& cu,
                               const Vector2& D) noexcept // distorted
{
   return q.inverse_apply(undistort_to_P2(cu, D)).normalised();
}

inline Vector3 backproject_ray(const DistortedCamera& dcam,
                               const Vector2& D) noexcept // distorted
{
   return backproject_ray(dcam.q, dcam.cu, D);
}

// ----------------------------------------------------------------- project-ray

inline Vector2
project_ray_to_distorted(const Quaternion& q, // Rotation world to CAM
                         const CachingUndistortInverse& cu,
                         const Vector3& ray) noexcept
{
   return cu.distort(homgen_P2_to_R2(q.apply(to_vec3(ray))));
}

inline Vector2 project_ray_to_distorted(const DistortedCamera& dcam,
                                        const Vector3& ray) noexcept
{
   return project_ray_to_distorted(dcam.q, dcam.cu, ray);
}

// ----------------------------------------- epipolar-line-in-undistorted-coords

inline Vector3
epipolar_line_in_undistorted_coords(const DistortedCamera& dcam0,
                                    const DistortedCamera& dcam1,
                                    const Vector3 ray0 // world coordinates
                                    ) noexcept
{
   const auto u = project_to_normalized(dcam1, dcam0.C);
   const auto v = project_to_normalized(dcam1, dcam0.C + 100.0 * ray0);
   return to_homgen_line(u, v);
}

inline Vector3
epipolar_line_in_undistorted_coords(const DistortedCamera& dcam0,
                                    const DistortedCamera& dcam1,
                                    const Vector2 u0 // undistorted cam coords
                                    ) noexcept
{
   const auto u = project_to_normalized(dcam1, dcam0.C);
   const auto v = project_to_normalized(
       dcam1, dcam0.C + 100.0 * dcam0.q.inverse_apply(homgen_R2_to_P2(u0)));
   return to_homgen_line(u, v);
}

// ----------------------------------------------------------------------- plane

inline Plane p3_from_two_distorted(const DistortedCamera& dcam,
                                   const Vector2& D0,
                                   const Vector2& D1) noexcept
{
   const auto ray0 = backproject_ray(dcam, D0);
   const auto ray1 = backproject_ray(dcam, D1);
   return Plane(dcam.C, dcam.C + ray0, dcam.C + ray1);
}

// --------------------------------------------------------- plane-ray intersect
// Find 3d point 'X' that lies on plane 'p3'
// given camera point 'U' or 'D'
inline Vector3 plane_ray_intersect(const Vector3& C,    // cam center
                                   const Quaternion& q, // Rotation world to CAM
                                   const Plane& p3, // plane we intersect with
                                   const Vector2& U) noexcept // ideal
{
   const auto cam_ray = homgen_R2_to_P2(U).normalised();
   const auto ray     = q.inverse_apply(cam_ray);
   return plane_ray_intersection(p3, C, C + ray);
}

inline Vector3 plane_ray_intersect(const Vector3& C,    // cam center
                                   const Quaternion& q, // Rotation world to CAM
                                   const CachingUndistortInverse& cu,
                                   const Plane& p3, // plane we intersect with
                                   const Vector2& D) noexcept // distorted
{
   return plane_ray_intersect(C, q, p3, cu.undistort(D));
}

inline Vector3 plane_ray_intersect(const DistortedCamera& cam,
                                   const Plane& p3, // plane we intersect with
                                   const Vector2& D) noexcept // distorted
{
   return plane_ray_intersect(cam.C, cam.q, cam.cu, p3, D);
}

// ---------------------------------------------------------- transfer-distorted
// Take a point in 1 camera, image it onto a 3D plane, and
// then find the corresponding point in the other camera.
inline Vector2 transfer_distorted(const Vector3& C0, // source camera
                                  const Quaternion& q0,
                                  const CachingUndistortInverse& cu0,
                                  const Vector3& C1, // destination camera
                                  const Quaternion& q1,
                                  const CachingUndistortInverse& cu1,
                                  const Plane& p3, // plane
                                  const Vector2& D // point in source camera
                                  ) noexcept
{
   Vector3 X = plane_ray_intersect(C0, q0, cu0, p3, D);
   return project_to_distorted(C1, q1, cu1, X);
}

inline Vector2 transfer_distorted(const DistortedCamera& cam0, // source cam
                                  const DistortedCamera& cam1, // dest cam
                                  const Plane& p3,             // plane
                                  const Vector2& D // point in source camera
                                  ) noexcept
{
   Vector3 X = plane_ray_intersect(cam0.C, cam0.q, cam0.cu, p3, D);
   return project_to_distorted(cam1.C, cam1.q, cam1.cu, X);
}

inline Vector2 transfer_distorted(const bool left_to_right,
                                  const Vector3& C0,
                                  const Quaternion& q0,
                                  const CachingUndistortInverse& cu0,
                                  const Vector3& C1,
                                  const Quaternion& q1,
                                  const CachingUndistortInverse& cu1,
                                  const Plane& p3,
                                  const Vector2& D) noexcept
{
   return (left_to_right) ? transfer_distorted(C0, q0, cu0, C1, q1, cu1, p3, D)
                          : transfer_distorted(C1, q1, cu1, C0, q0, cu0, p3, D);
}

inline Vector2 transfer_distorted(const bool left_to_right,
                                  const DistortedCamera& cam0,
                                  const DistortedCamera& cam1,
                                  const Plane& p3,
                                  const Vector2& D) noexcept
{
   return (left_to_right) ? transfer_distorted(cam0, cam1, p3, D)
                          : transfer_distorted(cam1, cam0, p3, D);
}

// ---------------------------------------------------------- triangulate points

Vector3 triangulate(const array<DistortedCamera, 2>& dcams,
                    const Vector2& u, // distorted point (left)
                    const Vector2& v, // distorted point (right)
                    const bool feedback = false) noexcept;

Vector3 triangulate(const DistortedCamera& dcam0,
                    const DistortedCamera& dcam1,
                    const Vector2& u, // distorted point (left)
                    const Vector2& v, // distorted point (right)
                    const bool feedback = false) noexcept;

// --------------------------------------------------------------- plot-cylinder
//
void plot_cylinder(const DistortedCamera& dcam,
                   const Cylinder& Cy,
                   const AABB& bounds,
                   std::function<void(int, int)> f);

void plot_human_cylinder(const DistortedCamera& dcam,
                         const Cylinder& Cy,
                         const AABB& bounds,
                         std::function<void(int, int)> f);

std::pair<Vector2f, Vector2f>
project_circle(const DistortedCamera& dcam,
               const Vector3f& X,      // Circle centre
               const float radius,     // Circle radius
               const Vector3f& up_n,   // Normal to circle plane
               const bool semi_circle, // to point of view
               const AABB& im_bounds,
               std::function<void(int, int, float)> f) noexcept;

// -------------------------------------------------------------------- plot-box
//
void plot_box(const DistortedCamera& dcam,
              const Box& box,
              const AABB& bounds,
              std::function<void(int, int)> f,
              const uint32_t stipple = 0xffffffffu);

vector<Vector2> box_hull(const DistortedCamera& dcam, const Box& box);

// ------------------------------------------------------------------- plot-axis
//
void plot_axis(ARGBImage& im, const DistortedCamera& dcam) noexcept;

// ------------------------------------------------------------------ debug-dcam
//
const DistortedCamera* get_debug_dcam() noexcept;
void set_debug_dcam(const DistortedCamera*) noexcept;

} // namespace perceive
