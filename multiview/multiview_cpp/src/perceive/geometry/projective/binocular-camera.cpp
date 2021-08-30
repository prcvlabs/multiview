
#include <unordered_map>

#include <boost/algorithm/string/replace.hpp>

#include "binocular-camera.hpp"
#include "camera.hpp"
#include "triangulation.hpp"

#include "perceive/calibration/find-F.hpp"
#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/geometry/projective/camera.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"
#include "perceive/utils/tick-tock.hpp"

#define This BinocularCamera

namespace perceive
{
// ------------------------------------------------------ Inscribed Rect 2 Hulls

static AABB inscribed_rect_two_hulls(const std::vector<Vector2>& chull0,
                                     const std::vector<Vector2>& chull1,
                                     const real delta,
                                     const bool do_trace) noexcept
{
   auto find_bounding_roi = [&](const auto& hull) -> AABB {
      // Find the bounding rectangle
      auto aabb = AABB::minmax();
      for(const auto& X : hull) aabb.union_point(X);
      return aabb;
   };

   array<vector<Vector2>, 2> hull{{chull0, chull1}};
   array<AABB, 2> bounds;
   std::transform(cbegin(hull), cend(hull), begin(bounds), find_bounding_roi);
   array<vector<Vector2>, 2> isects;

   auto hull_hull_line_bounds = [&](const real y) {
      auto l = to_homgen_line(Vector2{0.0, y}, Vector2{1.0, y});
      for(size_t i = 0; i < 2; ++i) {
         hull_line_intersections(cbegin(hull[i]), cend(hull[i]), l, isects[i]);
         if(isects[i].size() < 2) return Vector2::nan();
      }
      auto cmp = [&](const auto& a, const auto& b) { return a.x < b.x; };
      auto [l0, r0]
          = std::minmax_element(cbegin(isects[0]), cend(isects[0]), cmp);
      auto [l1, r1]
          = std::minmax_element(cbegin(isects[1]), cend(isects[1]), cmp);
      return Vector2(std::max(l0->x, l1->x), std::min(r0->x, r1->x));
   };

   auto calc_aabb = [&](const real y0, const real y1) -> AABB {
      auto xx0 = hull_hull_line_bounds(y0);
      auto xx1 = hull_hull_line_bounds(y1);
      AABB aabb;
      aabb.left                       = std::max(xx0(0), xx1(0));
      aabb.right                      = std::min(xx0(1), xx1(1));
      std::tie(aabb.top, aabb.bottom) = std::minmax(y0, y1);

      if(do_trace) {
         cout << format(" (-) y0={}, y1={}, xx0={}, xx1={}, aabb={}",
                        y0,
                        y1,
                        str(xx0),
                        str(xx1),
                        str(aabb))
              << endl;
      }

      if(!aabb.is_finite()) aabb = AABB(0.0, 0.0, 0.0, 0.0);
      return aabb;
   };

   auto extend_aabb = [&](bool direction, real y0, real y1, real delta) {
      auto aabb0 = calc_aabb(y0, y1);
      while(true) {
         y0         = direction ? (y0 - delta) : y0;
         y1         = direction ? y1 : (y1 + delta);
         auto aabb1 = calc_aabb(y0, y1);

         if(do_trace) {
            cout << format(" <*> y0 = {}, y1 = {}, "
                           "area = [{}, {}], 0={}, 1={}",
                           y0,
                           y1,
                           aabb0.area(),
                           aabb1.area(),
                           str(aabb0),
                           str(aabb1))
                 << endl;

            fgetc(stdin);
         }

         if(aabb0.area() > aabb1.area()) break;
         aabb0 = aabb1;
      }
      return aabb0;
   };

   const auto C = 0.5 * (bounds[0].center(), bounds[1].center());

   auto aabb = extend_aabb(true, C.y, C.y, delta);
   auto ret  = extend_aabb(false, aabb.top, aabb.bottom, delta);

   if(ret.area() <= std::fabs(delta)) {
      LOG_ERR("failed to get an inscribed rectangle");
      cout << format("bounds0 = {}", str(bounds[0])) << endl;
      cout << format("bounds1 = {}", str(bounds[1])) << endl;
      cout << format("C0 = {}", str(bounds[0].center())) << endl;
      cout << format("C1 = {}", str(bounds[1].center())) << endl;
      cout << format("C     = {}", str(C)) << endl;
      cout << format("delta = {}", delta) << endl;
      cout << format("aabb  = {}", str(aabb)) << endl;
      cout << format("ret   = {}", str(ret)) << endl;
      // FATAL(format("aborting due to previous errors"));
   }

   return ret;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// --                               Camera Info                               --
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

bool BinocularCameraInfo::operator==(
    const BinocularCameraInfo& o) const noexcept
{
#define NORM_TEST(v) (fabs(v - o.v) < 1e-9)
#define TEST(v) (v == o.v)
   return true && TEST(camera_id) && ((t - o.t).norm() < 1e-9)
          && ((q.normalised() - o.q.normalised()).norm() < 1e-9)
          && NORM_TEST(baseline) && TEST(M[0]) && TEST(M[1]);
#undef TEST
#undef NORM_TEST
}

string BinocularCameraInfo::to_json_string() const noexcept
{
   auto aa = q.to_axis_angle();
   auto tt = t.normalised();
   return format(R"V0G0N(
{{
   "camera-id": "{}",
   "rotation-axis": [{}, {}, {}],
   "rotation-angle": {},
   "t": [{}, {}, {}],
   "baseline": {},
   "sensor0": {},
   "sensor1": {}
}}
)V0G0N",
                 camera_id,
                 str_precise(aa(0)),
                 str_precise(aa(1)),
                 str_precise(aa(2)),
                 str_precise(aa(3)),
                 str_precise(tt(0)),
                 str_precise(tt(1)),
                 str_precise(tt(2)),
                 str_precise(baseline),
                 indent(M[0].to_json_string(), 14),
                 indent(M[1].to_json_string(), 14));
}

void load(BinocularCameraInfo& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}

void save(const BinocularCameraInfo& data, const string& fname) noexcept(false)
{
   std::string s;
   write(data, s);
   file_put_contents(fname, s);
}

void read(BinocularCameraInfo& data, const std::string& in) noexcept(false)
{
   const auto root = parse_json(in);
   read(data, root);
}

void write(const BinocularCameraInfo& data, std::string& out) noexcept(false)
{
   out = data.to_json_string();
}

void read(BinocularCameraInfo& data, const Json::Value& node) noexcept(false)
{
   std::string err = "";

   auto from_json = [&](const Json::Value& root) {
      Vector3 axis;
      real theta{0.0};
      json_load(get_key(root, "camera-id"), data.camera_id);
      json_load(get_key(root, "rotation-axis"), axis);
      json_load(get_key(root, "rotation-angle"), theta);
      json_load(get_key(root, "t"), data.t);
      json_load(get_key(root, "baseline"), data.baseline);
      const auto s0 = time_thunk(
          [&]() { data.M[0].from_json(get_key(root, "sensor0")); });
      const auto s1 = time_thunk(
          [&]() { data.M[1].from_json(get_key(root, "sensor1")); });
      // TRACE(format("s0 = {}s, s1 = {}s", s0, s1));
      data.q.from_axis_angle(axis, theta);
   };

   try {
      const auto s0 = time_thunk([&]() { from_json(node); });
      // TRACE(format("from-json took {}s", s0));
   } catch(std::logic_error& e) {
      err = strlen(e.what()) == 0 ? "logic error" : e.what();
   } catch(std::runtime_error& e) {
      err = strlen(e.what()) == 0 ? "runtime error" : e.what();
   } catch(std::exception& e) {
      // JSON parse error
      err = strlen(e.what()) == 0 ? "exception" : e.what();
   } catch(...) {
      err = format("unknown error");
   }

   if(err != "") throw std::runtime_error(err);

   {
      const auto [bcam_id, suffix] = extract_suffix(data.camera_id, '_');
      if(!string_is_uppercase(bcam_id)) {
         const string new_bcam_id = string_to_uppercase(bcam_id);
         const string full_id     = new_bcam_id + suffix;
         WARN(format("Camera id is not uppercase. BOO!!!. Changing '{}' -> "
                     "'{}'. Suffix is '{}', and full camera-id is '{}'",
                     bcam_id,
                     new_bcam_id,
                     suffix,
                     full_id));
         data.camera_id = std::move(full_id);
      }
   }
}

void write(const BinocularCameraInfo& data, Json::Value& node) noexcept(false)
{
   node = parse_json(data.to_json_string());
}

// --------------------------------------------------------- Euclidean Transform

EuclideanTransform BinocularCameraInfo::euclid_transform() const noexcept
{
   EuclideanTransform et;
   et.scale       = 1.0;
   et.translation = C1();          //
   et.rotation    = q.conjugate(); // Cam to world
   return et;
}

void BinocularCameraInfo::set_from_et(const EuclideanTransform& et01) noexcept
{
   q        = et01.rotation.conjugate();
   baseline = et01.translation.norm();
   t        = -q.apply(et01.translation).normalised();

   if(true) {
      if(!q.is_finite() or !std::isfinite(baseline) or !t.is_finite()) {
         cout << "et01: " << str(et01) << endl;
         cout << "q:    " << q.to_readable_str() << endl;
         cout << "t:    " << str(t) << endl;
         cout << "base: " << baseline << endl;
         FATAL("kBAM!");
      }
   }
}

EuclideanTransform
BinocularCameraInfo::make_et1(const EuclideanTransform& et0) const noexcept
{
   return (et0.inverse() * euclid_transform().inverse()).inverse();
}

// ------------------------------------------------------------ Rectifying Ideal

Vector2 BinocularCameraInfo::rectify_ideal(int cam,
                                           const Vector2& U) const noexcept
{
   const Matrix3r R     = quaternion_to_rot3x3(q);
   const Matrix3r R_inv = R.inverse();
   const Vector3 t      = this->t.normalised();
   const auto baseline  = this->baseline;

   const auto C0 = this->C0(); // Camera center 0
   const auto C1 = this->C1(); // Camera center 1

   // The XYZ co-ordinates of the new camera
   const Vector3 z0 = to_vec3(Vector3r(R.block(2, 0, 1, 3).transpose()));
   const Vector3 X  = (C1 - C0).normalised();
   const Vector3 Y  = cross(z0, X).normalised();
   const Vector3 Z  = cross(X, Y).normalised();

   Matrix3r Rn;
   Rn << X(0), X(1), X(2), Y(0), Y(1), Y(2), Z(0), Z(1), Z(2);

   Matrix3r HR[2];
   HR[0] = Rn;
   HR[1] = Rn * R_inv;

   Expects(cam == CAM0 or cam == CAM1);

   unsigned width = 800, height = 600;

   AABB aabb;
   const auto K
       = well_calibrated_region(M[0], M[1], HR[0], HR[1], width, height, aabb);

   Matrix3r KHR[2];
   KHR[0] = K * HR[0];
   KHR[1] = K * HR[1];

   return homgen_P2_to_R2(to_vec3(KHR[cam] * Vector3r(U.x, U.y, 1.0)));
}

// -------------------------------------------------------------------- Solve 3d

Vector3
BinocularCameraInfo::solve3d_from_distorted(const Vector2& d0,
                                            const Vector2& d1) const noexcept
{
   Camera cam0;
   Camera cam1;
   cam0.set_intrinsic(10.0, 10.0, 1.0, 1.0, Vector2(0.0, 0.0));
   cam1.set_intrinsic(10.0, 10.0, 1.0, 1.0, Vector2(0.0, 0.0));
   cam0.set_extrinsic(Matrix3r::Identity(), Vector3(0.0, 0.0, 0.0));
   Matrix3r R = quaternion_to_rot3x3(q);
   cam1.set_extrinsic(R, t * baseline);

   Matrix3r E = calibration::make_essential_matrix(q, t);

   const auto u0   = M[0].undistort(d0);
   const auto u1   = M[1].undistort(d1);
   const auto ray0 = to_vec3(cam0.back_project(u0));
   const auto ray1 = to_vec3(cam1.back_project(u1));

   const auto r0 = homgen_R2_to_P2(M[0].undistort(d0)).normalised();
   const auto r1
       = q.inverse_rotate(homgen_R2_to_P2(M[1].undistort(d1)).normalised());
   const auto C0 = cam0.C();
   const auto C1 = cam1.C();
   const auto X  = intersect_rays_2(C0, C0 + ray0, C1, C1 + ray1);
   if(false) {
      INFO("solve3d-from-distorted");
      cout << format("working fmt = {}, {}",
                     str(M[0].working_format()),
                     str(M[1].working_format()))
           << endl;
      cout << format("D = {}, {}", str(d0), str(d1)) << endl;
      cout << format("R = {}, {}", str(r0), str(r1)) << endl;
      cout << format("R = {}, {}", str(ray0), str(ray1)) << endl;
      cout << format("C = {}, {}", str(C0), str(C1)) << endl;
      cout << format("e = {}", 400.0 * calibration::xFl_lFx(E, u0, u1)) << endl;
      cout << format("X = {}", str(X)) << endl;
   }
   return X;
}

real BinocularCameraInfo::reproj_err_from_distorted(
    const Vector2& d0,
    const Vector2& d1) const noexcept
{
   const auto C0   = this->C0();
   const auto C1   = this->C1();
   const auto X    = solve3d_from_distorted(d0, d1);
   const auto ray0 = (X - C0).normalised();
   const auto ray1 = (q.rotate((X - C1))).normalised();
   const auto v0   = M[0].distort(homgen_P2_to_R2(ray0));
   const auto v1   = M[1].distort(homgen_P2_to_R2(ray1));
   const auto err  = 0.5 * ((d0 - v0).norm() + (d1 - v1).norm());
   if(false) {
      INFO("reproj-err-from-distorted");
      cout << format("D = {}, {}", str(d0), str(d1)) << endl;
      cout << format("X = {}", str(X)) << endl;
      cout << format("R = {}, {}", str(ray0), str(ray1)) << endl;
      cout << format("V = {}, {}", str(v0), str(v1)) << endl;
      cout << format("e = {}", err) << endl;
   }
   return err;
}

Vector2 BinocularCameraInfo::project(int cam, const Vector3& X) const noexcept
{
   return M[size_t(cam)].distort(homgen_P2_to_R2(to_ray(cam, X)));
   // if(cam == CAM0) return M[0].distort(homgen_P2_to_R2(X - C0()));
   // return M[1].distort(homgen_P2_to_R2(q.rotate(X - C1())));
}

Vector3 BinocularCameraInfo::to_ray(int cam, const Vector3& X) const noexcept
{
   if(cam == CAM0) return (X - C0()).normalised();
   return (q.rotate(X - C1())).normalised();
}

// --------------------------------------------------------- plane ray intersect
// Find 'X' on plane 'p3' that intersects with ray from distorted
// point 'D' in cam 'cam'. 'et0' is the extrinsic transform for
// Cam0. The transform for Cam1 is calculated from 'q', 't', 'baseline', and
// 'et0'.
Vector3
BinocularCameraInfo::plane_ray_intersect(const int cam,
                                         const EuclideanTransform& et0,
                                         const Plane& p3,
                                         const Vector2& D) const noexcept
{
   // Convert to a ray
   auto cam_ray = homgen_R2_to_P2(M[size_t(cam)].undistort(D)).normalised();
   if(cam == CAM1) cam_ray = q.inverse_apply(cam_ray); // cam1 to cam0
   const auto ray = et0.rotation.rotate(cam_ray);      // cam0 to world

   // Camera center
   const auto C = et0.apply(this->C(cam));
   return plane_ray_intersection(p3, C, C + ray);
}

// --------------------------------------------------- print debug triangulation
/// Print debug info on triangulating 3D points P and Q
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
                               const Matrix3r K) noexcept
{
   const bool use_K     = !(K == Matrix3r::Zero());
   const Matrix3r K_inv = use_K ? K.inverse() : K;

   auto undistort = [&](int cam_id, const Vector2& D) -> Vector3 {
      Expects(cam_id >= 0 and cam_id < 2);
      if(use_K) return to_vec3(K_inv * Vector3r(D.x, D.y, 1.0)).normalised();
      const Vector2 U = bcam_info.M[size_t(cam_id)].undistort(D);
      return Vector3(U.x, U.y, 1.0).normalised();
   };

   auto undistort_rotate = [&](int cam_id, const Vector2& D) -> Vector3 {
      const auto ray = undistort(cam_id, D);
      return (cam_id == 0) ? ray : bcam_info.q.conjugate().rotate(ray);
   };

   // Rays
   const Vector3 rp0 = undistort_rotate(0, p0);
   const Vector3 rq0 = undistort_rotate(0, q0);
   const Vector3 rp1 = undistort_rotate(1, p1);
   const Vector3 rq1 = undistort_rotate(1, q1);

   // Cam centres
   const Vector3 C0 = bcam_info.C0();
   const Vector3 C1 = bcam_info.C1();

   // Triangulate
   const Vector3 P = intersect_rays_2(C0, C0 + rp0, C1, C1 + rp1);
   const Vector3 Q = intersect_rays_2(C0, C0 + rq0, C1, C1 + rq1);

   // Errors
   const real P_err = intersect_rays_2_plane_err(C0, C0 + rp0, C1, C1 + rp1);
   const real Q_err = intersect_rays_2_plane_err(C0, C0 + rq0, C1, C1 + rq1);

   // Reproject
   auto project = [&](int cam_id, const Vector3& X) -> Vector2 {
      const auto R = (cam_id == 0) ? X : bcam_info.q.rotate(X - C1);
      if(use_K) return homgen_P2_to_R2(to_vec3(K * to_vec3r(R)));
      return bcam_info.M[size_t(cam_id)].distort(homgen_P2_to_R2(R));
   };

   const auto a0 = project(0, P);
   const auto a1 = project(1, P);
   const auto b0 = project(0, Q);
   const auto b1 = project(1, Q);

   // Now some nice output
   cout << ANSI_COLOUR_WHITE << ANSI_COLOUR_GREEN_BG << string(60, '-')
        << format("  Triangulation Report '{}' ", label) << ANSI_COLOUR_RESET
        << endl;

   cout << format("   p0 = {},  p1 = {}", str(p0), str(p1)) << endl;
   cout << format("   q0 = {},  q1 = {}", str(q0), str(q1)) << endl;
   cout << endl;
   cout << format("  rp0 = {}, rp1 = {}", str(rp0), str(rp1)) << endl;
   cout << format("  rq0 = {}, rq1 = {}", str(rq0), str(rq1)) << endl;
   cout << endl;
   cout << format("   C0 = {}, C1 = {}", str(C0), str(C1)) << endl;
   cout << format("   C0->C1 rot = {}", bcam_info.q.to_readable_str()) << endl;
   if(!use_K) {
      for(auto i = 0; i < 2; ++i)
         cout << format("   Cam[{}] working format: {}",
                        i,
                        str(bcam_info.M[size_t(i)].working_format()))
              << endl;
   }
   cout << endl;
   cout << format("   P = {}  (err = {}mm)", str(P), P_err * 1000.0) << endl;
   cout << format("   Q = {}  (err = {}mm)", str(Q), Q_err * 1000.0) << endl;
   cout << format("   |P - Q| = {}", (P - Q).norm()) << endl;
   cout << endl;
   cout << format("   Reprojections:") << endl;
   cout << format("   p0 |{} - {}| = {}\n", str(p0), str(a0), (p0 - a0).norm());
   cout << format("   p1 |{} - {}| = {}\n", str(p1), str(a1), (p1 - a1).norm());
   cout << format("   q0 |{} - {}| = {}\n", str(q0), str(b0), (q0 - b0).norm());
   cout << format("   q1 |{} - {}| = {}\n", str(q1), str(b1), (q1 - b1).norm());
   cout << "." << endl; // end
}

// ---------------------------------------------------- print debug p3 intersect
void print_debug_p3_intersect(const string& label,
                              const BinocularCameraInfo& bcam_info,
                              const Plane& p3,
                              const Vector2& p0,
                              const Vector2& q0,
                              const Vector2& p1,
                              const Vector2& q1,
                              const Matrix3r K) noexcept
{
   const bool use_K     = !(K == Matrix3r::Zero());
   const Matrix3r K_inv = use_K ? K.inverse() : K;

   auto undistort = [&](int cam_id, const Vector2& D) -> Vector3 {
      Expects(cam_id >= 0 and cam_id < 2);
      if(use_K) return to_vec3(K_inv * Vector3r(D.x, D.y, 1.0)).normalised();
      const Vector2 U = bcam_info.M[size_t(cam_id)].undistort(D);
      return Vector3(U.x, U.y, 1.0).normalised();
   };

   auto undistort_rotate = [&](int cam_id, const Vector2& D) -> Vector3 {
      const auto ray = undistort(cam_id, D);
      return (cam_id == 0) ? ray : bcam_info.q.conjugate().rotate(ray);
   };

   // Rays
   const Vector3 rp0 = undistort_rotate(0, p0);
   const Vector3 rq0 = undistort_rotate(0, q0);
   const Vector3 rp1 = undistort_rotate(1, p1);
   const Vector3 rq1 = undistort_rotate(1, q1);

   // Cam centres
   const Vector3 C0 = bcam_info.C0();
   const Vector3 C1 = bcam_info.C1();

   // Plane-ray intersections
   const Vector3 P0 = plane_ray_intersection(p3, C0, C0 + rp0);
   const Vector3 Q0 = plane_ray_intersection(p3, C0, C0 + rq0);
   const Vector3 P1 = plane_ray_intersection(p3, C1, C1 + rp1);
   const Vector3 Q1 = plane_ray_intersection(p3, C1, C1 + rq1);
   const Vector3 P  = 0.5 * (P1 + P0);
   const Vector3 Q  = 0.5 * (Q1 + Q0);

   // Reproject
   auto project = [&](int cam_id, const Vector3& X) -> Vector2 {
      const auto R = (cam_id == 0) ? X : bcam_info.q.rotate(X - C1);
      if(use_K) return homgen_P2_to_R2(to_vec3(K * to_vec3r(R)));
      return bcam_info.M[size_t(cam_id)].distort(homgen_P2_to_R2(R));
   };

   const auto a0 = project(0, P);
   const auto a1 = project(1, P);
   const auto b0 = project(0, Q);
   const auto b1 = project(1, Q);

   // Now some nice output
   cout << ANSI_COLOUR_WHITE << ANSI_COLOUR_RED_BG << string(60, '-')
        << format("  Plane Intersect Report '{}' ", label) << ANSI_COLOUR_RESET
        << endl;

   cout << format("   p0 = {},  p1 = {}", str(p0), str(p1)) << endl;
   cout << format("   q0 = {},  q1 = {}", str(q0), str(q1)) << endl;
   cout << endl;
   cout << format("  rp0 = {}, rp1 = {}", str(rp0), str(rp1)) << endl;
   cout << format("  rq0 = {}, rq1 = {}", str(rq0), str(rq1)) << endl;
   cout << endl;
   cout << format("   C0 = {}, C1 = {}", str(C0), str(C1)) << endl;
   cout << format("   C0->C1 rot = {}", bcam_info.q.to_readable_str()) << endl;
   if(!use_K) {
      for(auto i = 0; i < 2; ++i)
         cout << format("   Cam[{}] working format: {}",
                        i,
                        str(bcam_info.M[size_t(i)].working_format()))
              << endl;
   }
   cout << endl;
   cout << format("   p3 = {}", str(p3)) << endl;
   cout << format("   P0 = {}, P1 = {}", str(P0), str(P1)) << endl;
   cout << format("   Q0 = {}, Q1 = {}", str(Q0), str(Q1)) << endl;
   cout << endl;
   cout << format("   P  = {},  err = {}", str(P), (P1 - P0).norm()) << endl;
   cout << format("   Q  = {},  err = {}", str(Q), (Q1 - Q0).norm()) << endl;
   cout << format("   |P - Q| = {}", (P - Q).norm()) << endl;
   cout << endl;
   cout << format("   Reprojections:") << endl;
   cout << format("   p0 |{} - {}| = {}\n", str(p0), str(a0), (p0 - a0).norm());
   cout << format("   p1 |{} - {}| = {}\n", str(p1), str(a1), (p1 - a1).norm());
   cout << format("   q0 |{} - {}| = {}\n", str(q0), str(b0), (q0 - b0).norm());
   cout << format("   q1 |{} - {}| = {}\n", str(q1), str(b1), (q1 - b1).norm());
   cout << "." << endl; // end
}

// ----------------------------------------- Estimate Bino Cam Euclid Transforms
// 'e0' and 'e1' are estimated euclidean transforms for each camera.
// The are averaged, and then the bino-cam-info is used to calculate
// the output transformtions.
//
std::pair<EuclideanTransform, EuclideanTransform>
estimate_bino_camera_euclid_transforms(const BinocularCameraInfo& info,
                                       const EuclideanTransform& e0,
                                       const EuclideanTransform& e1)
{
   EuclideanTransform el, er;

   const auto t_av = 0.5 * (e0.translation + e1.translation);
   const auto q_av = average_rotations(e0.rotation, e1.rotation);

   // Vector that travels from cam-center 0 to cam-center 1, from
   // the point of view of the camera
   const auto C01       = -info.q.conjugate().rotate(info.t).normalised();
   const auto direction = q_av.rotate(C01);
   const auto aa        = info.q.to_axis_angle(); // From C0 to C1... halve
   const auto q = axis_angle_to_quaternion(Vector4{aa.xyz(), aa.w * 0.5});

   el.translation = t_av - 0.5 * info.baseline * direction;
   er.translation = t_av + 0.5 * info.baseline * direction;
   el.rotation    = q_av * q.conjugate();
   er.rotation    = q_av * q;
   el.scale = er.scale = 1.0;

   return std::make_pair(el, er);
}

// ----------------------------------------------- transfer point between images

Vector2
transfer_point_between_images(const Vector2& x0,
                              const Plane& p3_et0,           // et0 coords
                              const EuclideanTransform& et0, // world->cam
                              const EuclideanTransform& et1, // world->cam
                              const CachingUndistortInverse& cu0,
                              const CachingUndistortInverse& cu1) noexcept
{
   const auto U = homgen_R2_to_P2(cu0.undistort(x0));
   const auto W
       = et0.inverse_apply(plane_ray_intersection(p3_et0, Vector3{}, U));
   return cu1.distort(homgen_P2_to_R2(et1.apply(W)));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// --                             Binocular Camera                            --
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// ---------------------------------------------------------------- Construction

This::BinocularCamera()
{
   for(auto i = 0; i < 2; ++i) {
      HR_[i] = KRR_[i] = KR_[i] = KRR_inv_[i] = Kn_[i] = Matrix3r::Zero();
   }
}

// ------------------------------------------------------------------------ Init

void This::init(const Quaternion& q,
                const Vector3& t,
                const real baseline,
                const unsigned distorted_w,
                const unsigned distorted_h,
                const Matrix3r& K,
                const unsigned width,
                const unsigned height,
                const bool use_calib_roi)
{
   w_ = width;
   h_ = height;
   set_working_format(distorted_w, distorted_h);

   q_        = q;
   R_        = quaternion_to_rot3x3(q);
   R_inv_    = R_.inverse();
   t_        = t.normalised();
   baseline_ = baseline;

   C_[0] = Vector3(0.0, 0.0, 0.0);
   C_[1] = -to_vec3(R_inv_ * to_vec3r(t_ * baseline_));

   // The XYZ co-ordinates of the new camera
   const Vector3 z0 = to_vec3(Vector3r(R_.block(2, 0, 1, 3).transpose()));
   const Vector3 X  = (C_[1] - C_[0]).normalised();
   const Vector3 Y  = cross(z0, X).normalised();
   const Vector3 Z  = cross(X, Y).normalised();

   Matrix3r Rn;
   Rn << X(0), X(1), X(2), Y(0), Y(1), Y(2), Z(0), Z(1), Z(2);

   rect_q_ = rot3x3_to_quaternion(Rn);

   HR_[0] = Rn;
   HR_[1] = Rn * R_inv_;

   if(use_calib_roi) {
      AABB aabb;
      const auto K = well_calibrated_region(
          M_[0], M_[1], HR_[0], HR_[1], width, height, aabb);

      if(!is_finite(K)) {
         LOG_ERR("attempt to initialize a binocular camera with erroneous "
                 "calibration region of interest:");
         cout << M_[0].to_string() << endl;
         cout << M_[1].to_string() << endl;
         cout << format("width-height = [{}x{}]", width, height) << endl;
         cout << "calculated sensor roi = " << str(aabb) << endl;
         cout << "HR[0] = " << endl << HR_[0] << endl << endl;
         cout << "HR[1] = " << endl << HR_[1] << endl << endl;
         cout << "K = " << endl << K << endl << endl;
         cout << to_vec3(K * Vector3r(aabb.left, aabb.top, 1.0)) << endl;
         cout << to_vec3(K * Vector3r(aabb.right, aabb.top, 1.0)) << endl;
         cout << to_vec3(K * Vector3r(aabb.right, aabb.bottom, 1.0)) << endl;
         cout << to_vec3(K * Vector3r(aabb.left, aabb.bottom, 1.0)) << endl;
         FATAL("Aborting due to previous errors");
      }

      Kn_[0] = Kn_[1] = K;

   } else {
      if(!is_finite(K)) {
         LOG_ERR("attempt to initialize a binocular camera with erroneous 'K' "
                 "matrix:");
         cout << K << endl << endl;
         FATAL("Aborting due to previous errors");
      }
      Kn_[0] = Kn_[1] = K;
   }

   KR_[0] = Kn_[0] * HR_[0];
   KR_[1] = Kn_[1] * HR_[1];

   KRR_[0]     = KR_[0];
   KRR_[1]     = KR_[1] * R_;
   KRR_inv_[0] = KRR_[0].inverse();
   KRR_inv_[1] = KRR_[1].inverse();

   {
      bool has_error   = false;
      auto test_matrix = [&](const auto& matrix, const string_view name) {
         if(is_finite(matrix)) return;
         LOG_ERR(format("attempt to initialize a binocular camera with "
                        "invalid data. Matrix '{}' = ",
                        name));
         cout << matrix << endl << endl;
      };

      test_matrix(Rn, "Rn");
      test_matrix(HR_[0], "HR_[0]");
      test_matrix(HR_[1], "HR_[1]");
      test_matrix(Kn_[0], "Kn_[0]");
      test_matrix(Kn_[1], "Kn_[1]");
      test_matrix(KR_[0], "KR_[0]");
      test_matrix(KR_[1], "KR_[1]");
      test_matrix(KRR_[0], "KRR_[0]");
      test_matrix(KRR_[1], "KRR_[1]");
      test_matrix(KRR_inv_[0], "KRR_inv_[0]");
      test_matrix(KRR_inv_[1], "KRR_inv_[1]");

      if(has_error) FATAL("Aborting due to previous errors");
   }
}

void This::init(const DistortionModel& M0,
                const DistortionModel& M1,
                const Quaternion& q,
                const Vector3& t,
                const real baseline,
                const unsigned distorted_w,
                const unsigned distorted_h,
                const Matrix3r& K,
                const unsigned width,
                const unsigned height,
                const bool use_calib_roi)
{
   if(false) {
      INFO("bcam init:");
      cout << format("wh = {}x{}", width, height) << endl;
      cout << "K = " << endl << K << endl << endl;
   }

   {
      This tmp;
      *this = tmp; // reinitialize all data
   }

   M_[0] = M0;
   M_[1] = M1;

   for(auto i = 0; i < 2; ++i) {
      cu_[i].init(M_[i]);
      M_[i].finalize();
   }

   if(false) {
      auto s = time_thunk([&]() {
         cu_[0].init(M_[0]);
         cu_[1].init(M_[1]);
      });
      // INFO(format("cu.init took {} seconds", s));

      auto test_it
          = [&](const DistortionModel& M,
                const CachingUndistortInverse& cu) { // Compare distort...
               const auto U  = M.undistort(Vector2(237, 84));
               const auto D1 = M.distort(U);
               const auto D2 = cu.distort(U);
               cout << format("<{}> U = {}, |{} - {}| = {}",
                              M.sensor_id(),
                              str(U),
                              str(D1),
                              str(D2),
                              (D1 - D2).norm())
                    << endl;
            };
      test_it(M_[0], cu(0));
      test_it(M_[1], cu(1));
   }

   this->init(q,
              t,
              baseline,
              distorted_w,
              distorted_h,
              K,
              width,
              height,
              use_calib_roi);
}

void This::init(const BinocularCameraInfo& info,
                const unsigned distorted_w,
                const unsigned distorted_h,
                const Matrix3r& K,
                const unsigned width,
                const unsigned height,
                const bool use_calib_roi)
{
   init(info.M[0],
        info.M[1],
        info.q,
        info.t.normalised(),
        info.baseline,
        distorted_w,
        distorted_h,
        K,
        width,
        height,
        use_calib_roi);
}

void This::set_baseline(real baseline)
{
   baseline_ = baseline;
   C_[1]     = -to_vec3(R_inv_ * to_vec3r(t_ * baseline_));
}

void This::set_working_format(const unsigned distorted_w,
                              const unsigned distorted_h)
{
   for(auto i = 0; i < 2; ++i) {
      M_[i].set_working_format(distorted_w, distorted_h);
      M_[i].finalize();
      cu_[i].set_working_format(distorted_w, distorted_h);
   }
}

// -------------------------------------------------------------------- Make Et1

// If 'et0' is transform for cam0, this returns transform for cam1
// NOTE: the euclid transform 'et0' gives 'eye-coords' => 'world-coords'.
// Thus, et0.inverse(X) gives 'world=>eye', which is used for projecting to
// camera images.
EuclideanTransform This::make_et1(const EuclideanTransform& et0) const noexcept
{
   auto et01        = EuclideanTransform();
   et01.translation = C(1);
   et01.scale       = 1.0;
   et01.rotation    = q().conjugate();
   return (et0.inverse() * et01.inverse()).inverse();
}

// --------------------------------------------------------------------- Solve3d

Vector3 This::solve3d(real x0, real x1, real y) const noexcept
{
   return solve3d(x0, y, x1, y);
}

Vector3 This::solve3d(const Vector2& x0, const Vector2& x1) const noexcept
{
   return solve3d(x0.x, x0.y, x1.x, x1.y);
}

Vector3 This::solve3d(real x0, real y0, real x1, real y1) const noexcept
{
   if(false && feedback_)
      INFO(format("Solve3d: {}, {}, {}, {}", x0, y0, x1, y1));

   Vector3r o0 = KRR_inv_[0] * Vector3r(x0, y0, 1.0);
   Vector3r o1 = KRR_inv_[1] * Vector3r(x1, y1, 1.0);
   return intersect_rays_2(
       C_[0], C_[0] + to_vec3(o0), C_[1], C_[1] + to_vec3(o1));
}

real This::solve3d_3d_err(real x0, real y0, real x1, real y1) const noexcept
{
   Vector3r o0 = KRR_inv_[0] * Vector3r(x0, y0, 1.0);
   Vector3r o1 = KRR_inv_[1] * Vector3r(x1, y1, 1.0);
   return intersect_rays_2_plane_err(C_[0],
                                     C_[0] + to_vec3(o0).normalised(),
                                     C_[1],
                                     C_[1] + to_vec3(o1).normalised());
}

real This::reproj_err_from_distorted(const Vector2& d0,
                                     const Vector2& d1) const noexcept
{
   const auto X = solve3d_from_distorted(d0, d1);
   return (d0 - project(CAM0, X)).norm() + (d1 - project(CAM1, X)).norm();

   const auto u0 = to_vec3r(homgen_R2_to_P2(model(0).undistort(d0)));
   const auto u1 = to_vec3r(homgen_R2_to_P2(model(1).undistort(d1)));
   const auto r0 = homgen_P2_to_R2(to_vec3(KR(CAM0) * u0));
   const auto r1 = homgen_P2_to_R2(to_vec3(KR(CAM1) * u1));
   return reproj_err(r0, r1);
}

real This::reproj_err(const Vector2& x0, const Vector2& x1) const noexcept
{
   const auto X = solve3d(x0, x1);
   return (x0 - project(CAM0, X)).norm() + (x1 - project(CAM1, X)).norm();
}

real This::reproj_err(real x0, real y0, real x1, real y1) const noexcept
{
   return reproj_err(Vector2(x0, y0), Vector2(x1, y1));
}

Vector3 This::solve3d_from_distorted(const Vector2& x0,
                                     const Vector2& x1) const noexcept
{
   return solve3d_from_undistorted(M_[0].undistort(x0), M_[1].undistort(x1));
}

Vector3 This::solve3d_from_undistorted(const Vector2& u0,
                                       const Vector2& u1) const noexcept
{
   return solve3d_from_rays(homgen_R2_to_P2(u0).normalized(),
                            homgen_R2_to_P2(u1).normalized());
}

// Solve 3D from rays
Vector3 This::solve3d_from_rays(const Vector3& ray0,
                                const Vector3& ray1) const noexcept
{
   const auto Rray1 = to_vec3(R_inv_ * to_vec3r(ray1));
   return intersect_rays(C_[0], C_[0] + ray0, C_[1], C_[1] + Rray1);
}

// ---------------------------------------------------------------- Back-project

Vector3 This::to_ray(int cam_ind, const Vector2& distorted) const noexcept
{
   Expects(cam_ind == CAM0 || cam_ind == CAM1);
   return homgen_R2_to_P2(M_[cam_ind].undistort(distorted)).normalized();
}

Vector3 This::to_ray(int cam_ind, const Vector3r& D) const noexcept
{
   Expects(cam_ind == CAM0 || cam_ind == CAM1);
   return to_vec3(M_[cam_ind].undistort(D)).normalized();
}

Vector3 This::rectified_to_ray(int cam_ind, const Vector2& x) const noexcept
{
   Expects(cam_ind == CAM0 || cam_ind == CAM1);
   return rectified_to_ray(cam_ind, Vector3r(x.x, x.y, 1.0));
}

Vector3 This::rectified_to_ray(int cam_ind, const Vector3r& x) const noexcept
{
   Expects(cam_ind == CAM0 || cam_ind == CAM1);
   return to_vec3(KRR_inv_[cam_ind] * x).normalized();
}

// --------------------------------------------------------------------- Project

Vector2 This::project(int cam_ind, const Vector3& X) const
{
   Expects(cam_ind == CAM0 || cam_ind == CAM1);
   Vector3r Xr;
   if(cam_ind == CAM0)
      Xr = normalized_P2(to_vec3r(X - C(CAM0)));
   else
      Xr = normalized_P2(R_ * to_vec3r(X - C(CAM1)));
   const auto U = Vector2{Xr(0), Xr(1)};

   return cu(cam_ind).distort(U);
   // if(cu(cam_ind).in_bounds(U)) return cu(cam_ind).distort(U);
   // return M_[cam_ind].distort(U);
}

Vector2 This::project_to_rectified(int cam_ind, const Vector3& X) const
{
   Expects(cam_ind == CAM0 || cam_ind == CAM1);
   return homgen_P2_to_R2(to_vec3(KRR_[cam_ind] * to_vec3r(X - C(cam_ind))));
}

// ------------------------------------------------------------------- Disparity

real This::disparity(const Vector3& X) const
{
   auto xl = project_to_rectified(CAM0, X);
   auto xr = project_to_rectified(CAM1, X);
   return xl.x - xr.x;
}

// -------------------------------------------------------------- Epipolar Curve

// Epipolar "ellipses"... actually quadric beziers
// 'X' is a pixel in the left camera,
// 'min/max-dist' give the limits of the bezier
// 'A', 'B', and 'C' are the three control points of the output bezier
void This::epipolar_curve(const Vector2& x,
                          real min_dist,
                          real max_dist,
                          Vector2& a,
                          Vector2& b,
                          Vector2& c) const
{
   a = b = c = x;

   // First, must back-project 'X'
   const auto Xr = this->to_ray(0, x); // Already normalized
   assert(fabs(Xr.norm() - 1.0) < 1e-6);

   a = project(CAM1, C(CAM0) + min_dist * Xr);
   c = project(CAM1, C(CAM0) + max_dist * Xr);

   Vector2 a1 = project(CAM1, C(CAM0) + (min_dist + 1e-5) * Xr);
   Vector2 c1 = project(CAM1, C(CAM0) + (max_dist - 1e-5) * Xr);

   // 'b' is the intersection of the line [a-a1] and [c-c1]
   b = homgen_P2_to_R2(cross(to_homgen_line(a, a1), to_homgen_line(c, c1)));
}

// ------------------------------------------------------------------- get_mapxy

void This::get_mapxy(cv::Mat mapx[2], cv::Mat mapy[2]) const
{
   for(unsigned i = 0; i < 2; ++i) { get_sensor_mapxy(i, mapx[i], mapy[i]); }
}

// ------------------------------------------------------------ get_sensor_mapxy

void This::get_sensor_mapxy(unsigned sensor_num,
                            cv::Mat& mapx,
                            cv::Mat& mapy) const
{
   Expects(sensor_num < 2);

   static const Matrix3r H = Matrix3r::Identity();
   const auto fmt          = M_[sensor_num].working_format();
   const auto fx           = real(w_) / real(fmt.x);
   const auto fy           = real(h_) / real(fmt.y);

   // get_cached_mapxy(w_, h_, H, cu_[i], KR_[i], mapx[i], mapy[i]);

   auto f = [&](const Vector2& x) {
      assert(x.is_finite());
      const auto d = cu(int(sensor_num)).distort(x);
      assert(d.is_finite());
      return d;
   };

   create_cv_remap(w_, h_, H, f, KR_[sensor_num], mapx, mapy);
}

// ------------------------------------------------------------------- to-string

std::string This::to_string() const
{
   return format(R"V0G0N(
BinocularCamera
   A-dims:      {}x{}
   out-dims:    {}x{}
   C0:          {}
   C1:          {}
   rot:         {} deg about {}
   baseline:    {}
   M0:          {}
   M1:          {}
)V0G0N",
                 M_width(),
                 M_height(),
                 w_,
                 h_,
                 C_[0].to_string(),
                 C_[1].to_string(),
                 to_degrees(q_.to_axis_angle().w),
                 q_.to_axis_angle().xyz().to_string(),
                 baseline_,
                 indent(M_[0].to_string(), 16),
                 indent(M_[1].to_string(), 16));
}

// ------------------------------------------------------ well-calibrated region

Matrix3r well_calibrated_region(const DistortionModel& M0,
                                const DistortionModel& M1,
                                const Matrix3r& H0,
                                const Matrix3r& H1,
                                const unsigned width,
                                const unsigned height,
                                AABB& inscribed_rect)
{
   auto D_to_X = [](const auto& M, const auto& HR, const Vector2& D) {
      auto W = M.calib_to_working_format(D);
      auto R = homgen_R2_to_P2(M.undistort(W));
      auto X = homgen_P2_to_R2(to_vec3(HR * to_vec3r(R)));

      if(false) {
         cout << format("# ------------") << endl
              << format("     D = {}", str(D)) << endl
              << format("     W = {}", str(W)) << endl
              << format("     R = {}", str(R)) << endl
              << format("     X = {}", str(X)) << endl
              << endl;
      }

      return X;
   };

   auto find_hull = [&](const auto& M, const auto& HR) {
      // This is the distorted hull, generated during calibration
      const auto& distort_hull = M.calib_hull();

      auto d_to_x = [&](const Vector2& D) { return D_to_X(M, HR, D); };

      // Find the undistorted and rectified hull
      vector<Vector2> hull;
      hull.resize(distort_hull.size());

      std::transform(
          cbegin(distort_hull), cend(distort_hull), begin(hull), d_to_x);

      return hull;
   };

   auto hull0 = find_hull(M0, H0);
   auto hull1 = find_hull(M1, H1);

   AABB aabb;
   {
      static std::mutex padlock;
      lock_guard<decltype(padlock)> lock(padlock);
      // INFO(format("HULL ON {}, {}", M0.sensor_id(), M1.sensor_id()));
      const bool do_trace = false; // (M0.sensor_id() == "STR00042"s);
      aabb = inscribed_rect_two_hulls(hull0, hull1, 1e-3, do_trace);
   }

   Matrix3r K = Matrix3r::Identity();
   K(0, 0)    = width / aabb.width();
   K(1, 1)    = height / aabb.height();
   K(0, 2)    = -aabb.left * width / aabb.width();
   K(1, 2)    = -aabb.top * height / aabb.height();

   inscribed_rect = aabb;

   {
      auto generate_hull = [&](const auto& M,
                               const auto& HR,
                               const string& fname) {
         ARGBImage argb;
         argb.resize(unsigned(M.calib_format().x),
                     unsigned(M.calib_format().y));
         argb.zero();
         const auto gt_hull = M.calib_hull();
         for(auto i = 0u; i < gt_hull.size(); ++i) {
            const auto A = gt_hull[i];
            const auto B = gt_hull[(i + 1) % gt_hull.size()];
            plot_line_AA(A, B, [&](int x, int y, float a) {
               if(argb.in_bounds(x, y))
                  argb(x, y) = blend(k_white, argb(x, y), a);
            });

            const auto X = D_to_X(M, HR, A);
            render_string(
                argb, format("X = {}", str(X)), to_pt2(A), k_yellow, k_black);
         }
         argb.save(fname);
         INFO(format("Hull for {} saved to {}", M.sensor_id(), fname));
      };

      if(aabb.area() < 1e-3) {
         LOG_ERR("Failed to create an inscripted AABB for two camera sensors.");
         cout << "multiview failed to find an overlapping well-calibrated "
                 "region "
                 " between the two sensors -- considering their relative "
                 "rotation";

         cout << format("hull0 = [{}]",
                        implode(cbegin(hull0), cend(hull0), ", "))
              << endl;
         cout << format("hull1 = [{}]",
                        implode(cbegin(hull1), cend(hull1), ", "))
              << endl;
         cout << format("aabb  = {}, area = {}", str(aabb), aabb.area())
              << endl;

         generate_hull(M0, H0, format("/tmp/hull-{}.png", M0.sensor_id()));
         generate_hull(M1, H1, format("/tmp/hull-{}.png", M1.sensor_id()));

         FATAL("aborting due to previous errors");
      }
   }

   return K;
}

// ------------------------------------------------------- rectify >-> distorted

Vector2 This::rectify_distorted(int cam_ind, const Vector2& D) const noexcept
{
   const auto U = cu(cam_ind).undistort(D);
   if(cam_ind == 0) return rectify_ideal(cam_ind, U);
   const auto Y = q().inverse_apply(homgen_R2_to_P2(U));
   return rectify_ideal(cam_ind, homgen_P2_to_R2(Y));
}

Vector2 This::rectify_ideal(int cam_ind, const Vector2& U) const noexcept
{
   return homgen_P2_to_R2(
       to_vec3(KRR_[cam_ind] * to_vec3r(homgen_R2_to_P2(U))));
}

Vector2 This::unrectify_to_ideal(int cam_ind, const Vector2& R) const noexcept
{
   return homgen_P2_to_R2(
       to_vec3(KRR_inv_[cam_ind] * to_vec3r(homgen_R2_to_P2(R))));
}

Vector2 This::unrectify_to_distorted(int cam_ind,
                                     const Vector2& R) const noexcept
{
   const auto U = unrectify_to_ideal(cam_ind, R);
   if(cam_ind == 0) return cu(cam_ind).distort(U);
   const auto Y = q().apply(homgen_R2_to_P2(U));
   return cu(cam_ind).distort(homgen_P2_to_R2(Y));
}

// --------------------------------------------------------- plane ray intersect

// Takes distorted point 'D' in camera 'cam-ind', and finds the
// plane-ray intersection with plane 'p3'
Vector3 This::plane_ray_intersect(int cam_ind,
                                  const EuclideanTransform& extrinsic,
                                  const Plane& p3,
                                  const Vector2& D) const noexcept
{
   // Convert to a ray
   auto cam_ray = homgen_R2_to_P2(model(cam_ind).undistort(D)).normalised();
   if(cam_ind == CAM1) cam_ray = q().inverse_apply(cam_ray); // cam1 to cam0
   const auto ray = extrinsic.rotation.rotate(cam_ray);      // cam0 to world

   // Camera center
   const auto C = extrinsic.apply(this->C(cam_ind));

   return plane_ray_intersection(p3, C, C + ray);
}

Vector2 This::backproject_to_distorted(int cam_ind, const Vector3& X)
{
   if(cam_ind == CAM0) return cu(cam_ind).distort(homgen_P2_to_R2(X));

   auto Y = q().apply(X - C(cam_ind));
   return cu(cam_ind).distort(homgen_P2_to_R2(Y));
}

// ------------------------------------------------------------------- fit plane
// @param bcam_info Binocular Camera
// @param Xs Contiguous array of image points in Cam0
// @param Ys Corresponding contiguous array of points in Cam1
// @param N Number of points. Xs[N-1] and Ys[N-1] must be valid.
// @param K If non-zero, then points are turned to rays with K.inverse(),
//          otherwise bcam_info.M[cam_id].undistort is used.
// Note: Call print_debug_p3_intersect to examine result.
Plane fit_plane(const BinocularCameraInfo& bcam_info,
                const Vector2* xx,
                const Vector2* yy,
                const unsigned N,
                const bool feedback,
                const Matrix3r K) noexcept
{
   const bool use_K          = !(K == Matrix3r::Zero());
   const Matrix3r K_inv      = K.inverse();
   const bool super_feedback = false;

   // Cam centres
   array<Vector3, 2> Cs = {bcam_info.C0(), bcam_info.C1()};

   auto to_ray = [&](int cam_id, const Vector2& x) {
      Expects(cam_id >= 0 and cam_id < 2);
      Vector3 ray;
      if(use_K)
         ray = to_vec3(K_inv * Vector3r(x.x, x.y, 1.0)).normalised();
      else {
         const Vector2 U = bcam_info.M[size_t(cam_id)].undistort(x);
         ray             = Vector3(U.x, U.y, 1.0).normalised();
      }
      return (cam_id == 0) ? ray : bcam_info.q.conjugate().rotate(ray);
   };

   auto intersect = [&](const Vector2& x, const Vector2& y) {
      return intersect_rays_2(
          Cs[0], Cs[0] + to_ray(0, x), Cs[1], Cs[1] + to_ray(1, y));
   };

   auto resolve_p3 = [&](int cam_id, const Vector2& x, const Plane& p3) {
      return plane_ray_intersection(
          p3, Cs[size_t(cam_id)], Cs[size_t(cam_id)] + to_ray(cam_id, x));
   };

   auto project = [&](int cam_id, const Vector3& X) {
      const auto R
          = (cam_id == 0) ? X : bcam_info.q.rotate(X - Cs[size_t(cam_id)]);
      return (use_K) ? homgen_P2_to_R2(to_vec3(K * to_vec3r(R)))
                     : bcam_info.M[size_t(cam_id)].distort(homgen_P2_to_R2(R));
   };

   auto calc_err = [&](const Plane& p3) {
      Expects(p3.is_finite());
      real err = 0.0;
      for(auto i = 0u; i < N; ++i) {
         const auto A  = resolve_p3(0, xx[i], p3);
         const auto B  = resolve_p3(1, yy[i], p3);
         const auto b0 = project(0, B);
         const auto a1 = project(1, A);
         err += 0.5 * ((b0 - xx[i]).norm() + (a1 - yy[i]).norm());
      }
      return err / real(N);
   };

   auto find_d = [&](const Plane& p3) {
      Plane p = p3;
      auto f  = [&](double x) -> double {
         p.w = x;
         return calc_err(p);
      };
      return golden_section_search(f, -1e3, 1e3, 1e-3);
   };

   // Get initial plane estimate
   std::vector<Vector3> Xs(N);
   for(auto i = 0u; i < N; ++i) Xs[i] = intersect(xx[i], yy[i]);
   Plane p3 = fit_plane(&Xs[0], N);

   auto pack = [&](real* X) {
      const auto s3 = cartesian_to_spherical(p3.xyz());
      *X++          = s3.x;
      *X++          = s3.y;
   };

   auto unpack = [&](const real* X) {
      Vector3 s3;
      s3.x     = *X++;
      s3.y     = *X++;
      s3.z     = 1.0;
      p3.xyz() = spherical_to_cartesian(s3);
      p3.w     = find_d(p3);
      Expects(p3.is_finite());
   };

   int counter             = 0;
   real best_score         = std::numeric_limits<real>::max();
   const unsigned n_params = 2;
   vector<real> best_params(n_params);
   auto fn = [&](const real* X) -> real {
      unpack(X);
      const real ret = calc_err(p3);
      if(ret < best_score) {
         best_score = ret;
         std::copy(X, X + n_params, begin(best_params));
         if(super_feedback) {
            cout << format("   #{:8d} :: {:10.7f} :: p3 = {}",
                           counter,
                           best_score,
                           str(p3))
                 << endl;
         }
      }
      counter++;
      return ret;
   };

   vector<real> start(n_params);
   auto do_refine = [&](bool use_nelder_mead) {
      vector<real> xmin(n_params);
      real ynewlo   = dNAN;
      real ystartlo = dNAN;
      real reqmin   = 1e-2;
      real diffstep = 0.1;
      int kcount    = 10000; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method = nullptr;

      if(!use_nelder_mead) {
         method = "levenberg-marquardt";
         levenberg_marquardt(fn,
                             n_params,
                             &start[0],
                             &xmin[0],
                             reqmin,
                             diffstep,
                             4,
                             kcount / 100,
                             icount,
                             ifault);
         ynewlo = fn(&xmin[0]);

      } else {
         method = "nelder-mead";

         vector<real> step(n_params);
         auto X = &step[0];
         for(auto i = 0; i < 2; ++i) *X++ = 2.0 * one_degree(); //
         //*X++ = 0.01;                                     // 1cm

         nelder_mead(fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     10 * kcount,
                     icount,
                     numres,
                     ifault);
      }
   };

   pack(&start[0]);
   const real ystartlo = fn(&start[0]);
   auto now            = tick();
   for(auto i = 0; i < 1; ++i) {
      do_refine((i % 2) == 0);
      std::copy(cbegin(best_params), cend(best_params), begin(start));
      for(size_t i = 0; i < 2; ++i) // pertube rotation
         start[i] += 0.001 * (uniform() * one_degree() - 0.5 * one_degree());
      // start[2] += 0.001 * (2.0 * uniform() - 1.0);
   }
   const real ynewlo    = fn(&best_params[0]);
   const real elapsed_s = tock(now);

   unpack(&best_params[0]);

   const bool success = ynewlo < 10.0; // TODO, should be in degrees

   if(feedback) { // Print feedback
      std::stringstream ss{""};
      ss << endl;
      ss << format("Feedback fittin plane (binocular)") << endl;
      ss << format("    elapsed time:         {}s", elapsed_s) << endl;
      ss << format("    initial-score:        {}", ystartlo) << endl;
      ss << format("    final-score:          {}", ynewlo) << endl;
      ss << format("    p3:                   {}", str(p3)) << endl;
      cout << ss.str();
   }

   return p3;
}

} // namespace perceive
