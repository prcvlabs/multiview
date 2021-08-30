
#include "hist-cell-plausibility.hpp"

#include "2d-helpers.hpp"
#include "static_math/static_math.h"

namespace perceive
{
// ------------------------------------------------------ Hist Cell Plausibility
//
float HistCellPlausibility::normalized_height_prob() const noexcept
{
   if(!std::isfinite(height)) return 0.0f;
   const float z = float(human::height_z_score(real(height)));
   return std::exp(-0.5f * z * z);
}

float HistCellPlausibility::normalized_pfd_prob() const noexcept
{
   if(!std::isfinite(proj_floor_dist)) return 0.0f;

   // How many radius have we shifted?
   const auto r   = 0.5f * float(human::k_shoulder_width_adult_male);
   const auto n_d = proj_floor_dist / r;
   const auto d   = n_d / n_cam_dist;

   const float z = 2.0f * d; // 1 radius is 2 stdev of error
   return std::exp(-0.5f * z * z);
}

float HistCellPlausibility::probability() const noexcept
{
   return std::clamp<float>(
       std::sqrt(normalized_height_prob() * normalized_pfd_prob()), 0.0f, 1.0f);
}

HistCellPlausibility hist_cell_plausibility(const LocalizationData::Params& p,
                                            const Skeleton2D& p2d,
                                            const DistortedCamera& dcam,
                                            const Vector3& X,
                                            const bool feedback) noexcept
{
   // Plane for this p2d
   const auto p3 = calculate_p2d_plane(p2d, X);
   Expects(std::fabs(p3.side(X)) < 1e-3);
   Expects(std::fabs(X.z) < 1e-3);

   HistCellPlausibility ret;

   const auto cy_ret = p2d.realize_cylinder(dcam, X);
   ret.height        = float(cy_ret.Cy.height);

   { // Take all keypoints, project them onto the floor, average them
      int counter  = 0;
      Vector3 Y1   = {0.0, 0.0, 0.0};
      auto process = [&](const Vector3& Y) {
         if(!Y.is_finite()) return;
         Y1 += Vector3{Y.x, Y.y, 0.0};
         counter++;
      };

      for(const auto& kp : cy_ret.keypoints) {
         process(kp.first);
         process(kp.second);
      }
      Y1 /= real(counter);

      const auto C  = Vector3(dcam.C.x, dcam.C.y, 0.0);
      const auto Yr = (Y1 - C).normalised();

      const auto Y = C + (X - C).norm() * Yr; // X and Y are equidistant
      const auto l = (X - Y).norm();          // This is the distance

      ret.proj_floor_dist = float(l);

      if(feedback) {
         const auto r  = 0.5f * float(human::k_shoulder_width_adult_male);
         const auto d  = ret.proj_floor_dist / r;
         const float z = 2.0f * d; // 1 radius is 2 stdev of error
         const float p = phi_function(z + 0.5f) - phi_function(z - 0.5f);
         TRACE(
             format("X = {}, Y1 = {}, Y = {}, l = {} / {} = {}. z = {}, p = {}",
                    str(X),
                    str(Y1),
                    str(Y),
                    ret.proj_floor_dist,
                    r,
                    d,
                    z,
                    p));
         FATAL("kBAM!");
      }
   }

   const float dist = float(Vector2(dcam.C.x - X.x, dcam.C.y - X.y).norm());
   const float cam_dist
       = std::sqrt(square(dist) + square(float(ret.proj_floor_dist)));
   ret.n_cam_dist = cam_dist / float(p.extrinsic_calibration_error_factor);

   if(feedback) {
      const auto Cr   = to_vec3(p2d.centre_ray()).normalised();
      const auto A_   = plane_ray_intersection(p3, dcam.C, dcam.C + Cr);
      const Vector3 A = Vector3(A_.x, A_.y, 0.0);
      const auto Ar   = (A - dcam.C).normalised();
      const auto Xr   = (X - dcam.C).normalised();
      const auto c    = project_to_distorted(dcam, dcam.C + Cr);
      const auto x    = project_to_distorted(dcam, dcam.C + Xr);
      const auto x_   = project_to_distorted(dcam, X);
      const auto Y    = plane_ray_intersection(
          Plane(0.0, 0.0, 1.0, 0.0), dcam.C, dcam.C + Cr);
      const auto Nr = backproject_ray(dcam, to_vec2(p2d.neck()));
      const auto N  = plane_ray_intersection(
          Plane(0.0, 0.0, 1.0, 0.0), dcam.C, dcam.C + Nr);
      const auto n = project_to_distorted(dcam, N);

      cout << format("Cam = {}", str(dcam.C)) << endl;
      cout << format("nek = {}", str(p2d.neck())) << endl;
      cout << format("n   = {}", str(n)) << endl;
      cout << format("c   = {}", str(c)) << endl;
      cout << format("x   = {}", str(x)) << endl;
      cout << format("x'  = {}", str(x_)) << endl;
      cout << format("rpj = {}", (c - x).norm()) << endl;
      cout << format("Cr  = {}", str(Cr)) << endl;
      cout << format("Xr  = {}", str(Xr)) << endl;
      cout << format("Xr'  = {}", str(backproject_ray(dcam, x_))) << endl;
      cout << format("Nr  = {}", str(Nr)) << endl;
      cout << format("N   = {}", str(N)) << endl;
      cout << format("A   = {}", str(A)) << endl;
      cout << format("X   = {}", str(X)) << endl;

      cout << format("dot = {}", Xr.dot(Ar)) << endl;
      cout << format("phi = {}",
                     to_degrees(acos(std::clamp(Xr.dot(Ar), -1.0, 1.0))))
           << endl;
      cout << format("X-Y = |{} - {}| = {}", str(X), str(Y), (X - Y).norm())
           << endl;
      cout << format("     # = {}", p2d.sensor_no()) << endl;
      cout << format(" ___ C = {}", str(dcam.C)) << endl;
      cout << format(" ___ q = {}", str(dcam.q)) << endl;
      cout << format("----------------") << endl;
   }

   return ret;
}

} // namespace perceive
