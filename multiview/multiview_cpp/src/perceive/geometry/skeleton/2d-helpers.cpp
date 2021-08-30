
#include "stdinc.hpp"

#include "2d-helpers.hpp"

#include "perceive/geometry/human-heights.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/optimization/non-convex-optimization.hpp"
#include "perceive/utils/base64.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
using namespace skeleton;

// -------------------------------------------------------------- make billboard
//
static vector<Vector3> make_billboard(const Plane& floor,
                                      const vector<Vector3>& rays,
                                      const Vector3& C, // camera center
                                      const real height)
{
   const auto N = rays.size();
   vector<Vector3> out(N);

   auto max_height = [&]() {
      real ret = std::numeric_limits<real>::lowest();
      for(const auto& X : out)
         if(ret < floor.side(X)) ret = floor.side(X);
      return ret;
   };

   auto billboard_p3 = [&](const Vector3& ray) -> Plane {
      const auto X  = ray_position_relative_to_plane(floor, C, C + ray, height);
      const auto p3 = Plane(ray, -dot(X, ray.normalised()));

      if(!(std::fabs(p3.side(X)) < 1e-6)) {
         LOG_ERR(format("p3.side(X)"));
         cout << format("p3  = {}", str(p3)) << endl;
         cout << format("X   = {}", str(X)) << endl;
         cout << format("dot = {}", str_precise(p3.side(X))) << endl;
         cout << format("RAY = {}", str(ray)) << endl;
         cout << format("ray = {}", str(ray.normalised())) << endl;
         FATAL("kBAM!");
      }

      Expects(std::fabs(p3.side(X)) < 1e-6);
      Expects(std::fabs(floor.side(X) - height) < 1e-6);
      return p3;
   };

   auto test_billboard = [&](int ind) -> real {
      const auto p3 = billboard_p3(rays[size_t(ind)]);
      for(auto i = 0u; i < N; ++i)
         out[i] = plane_ray_intersection(p3, C, C + rays[i]);
      return max_height();
   };

   for(auto i = 0u; i < N; ++i)
      if(std::fabs(test_billboard(int(i)) - height) < 1e-4) return out;

   // Expects(false);
   return out;
}

// ----------------------------------------------------------- pose to billboard
//
vector<Vector3> pose_to_billboard(const Plane& floor,
                                  const CachingUndistortInverse& cu,
                                  const EuclideanTransform& et_inv,
                                  const Skeleton2D& pose,
                                  const real height) noexcept
{
   auto show_X = [&](const Vector3& X) {
      auto project = [&](const Vector3& Y) -> Vector2 {
         return cu.distort(homgen_P2_to_R2(et_inv.apply(Y)));
      };

      const auto T = et_inv.translation;
      const auto q = et_inv.rotation;
      const auto C = -q.inverse_rotate(T);
      // const auto X = Vector3(3.0, -1.6, 1.60);
      const auto x = project(X);

      const auto ray0 = homgen_R2_to_P2(cu.undistort(x)).normalised();
      const auto ray  = q.inverse_apply(ray0);
      const auto Y    = ray_position_relative_to_plane(floor, C, C + ray, X.z);
      const auto y    = project(Y);

      const auto ray_cam = homgen_R2_to_P2(cu.undistort(x)).normalised();

      cout << format("T     = {}", str(T)) << endl;
      cout << format("C     = {}", str(C)) << endl;
      cout << format("X     = {}", str(X)) << endl;
      cout << format("x     = {}", str(x)) << endl;
      cout << endl;
      cout << format("[0]   = {}", str((q.rotate(X) + T).normalised())) << endl;
      cout << format("[1]   = {}", str(ray_cam)) << endl;
      cout << format("[2]   = {}", str(q.rotate(X - C).normalised())) << endl;
      cout << endl;
      cout << format("[3]   = {}", str((X - C).normalised())) << endl;
      cout << format("[4]   = {}", str(q.inverse_rotate(ray_cam))) << endl;
      cout << endl;

      // cout << format("Y    = {}", str(Y)) << endl;
      // cout << format("y    = {}", str(y)) << endl;

      //

      // FATAL("kBAM!");
   };

   const auto& T = et_inv.translation;
   const auto& q = et_inv.rotation;
   const auto C  = -q.inverse_rotate(T);
   vector<Vector2> keypoints;
   keypoints.reserve(pose.keypoints().size());
   for(const auto& k : pose.keypoints())
      if(k.score > 1e-3f) keypoints.emplace_back(to_vec2(k.xy()));

   vector<Vector2> hull;
   hull.reserve(keypoints.size());
   andrews_convex_hull(begin(keypoints), end(keypoints), hull);
   const auto N = hull.size();

   vector<Vector3> rays;
   rays.reserve(N);
   for(auto i = 0u; i < N; ++i) {
      const auto x   = hull[i];
      const auto ray = homgen_R2_to_P2(cu.undistort(x));
      const auto Xr  = q.inverse_rotate(ray).normalised();
      if(Xr.is_finite()) rays.push_back(Xr);
   }

   if(N < 3) return vector<Vector3>{};

   const auto out = make_billboard(floor, rays, C, height);

   if(false) {
      for(auto X : out) show_X(X);
      FATAL("kBAM!");
   }

   return out;
}

// --------------------------------------------------------- recover gaze vector
//
static Vector3 recover_gaze_vector(const DistortedCamera& dcam,
                                   const Vector2& u,
                                   const Vector2& v) noexcept
{
   if(!u.is_finite() or !v.is_finite()) return Vector3::nan();

   // Get the plane through the two image points
   const auto ray_u = backproject_ray(dcam, u);
   const auto ray_v = backproject_ray(dcam, v);
   const auto U     = dcam.C + ray_u;
   const auto V     = dcam.C + ray_v;
   const auto p3    = Plane(dcam.C, U, V);

   const auto Z = Vector3{0.0, 0.0, 1.0};          // the up vector
   const auto Y = cross(Z, p3.xyz()).normalised(); // the side vector
   const auto X = cross(Y, Z);                     // the forward vector

   // The view was degenerate
   if(!((Z - cross(X, Y)).norm() < 1e-7)) return Vector3::nan();

   // We now have a 3d basis, but we may need to flip the X and Y axis
   // 'u' should be the "right" point, and 'v' the "left"
   //
   // So, let's assume that 'u' is 1 meter away. i.e., U
   const auto bilateral_p3 = Plane(Y, -dot(Y, U));        //
   const auto F = (bilateral_p3.side(V) >= 0.0) ? X : -X; // forward vector

   return F;
}

// ----------------------------------------------------------- recover-orth-gaze
//
static Vector3 recover_orth_gaze(const DistortedCamera& dcam,
                                 const bool is_right_keypoint,
                                 const Vector2& u)
{
   if(!u.is_finite()) return Vector3::nan();
   const auto ray_u = backproject_ray(dcam, u);
   const auto Z     = Vector3{0.0, 0.0, 1.0}; // the up vector
   const auto X     = cross(ray_u, Z).normalised();
   return is_right_keypoint ? -X : X;
}

// --------------------------------------------------------- Calculate p2d plane
//
Plane calculate_p2d_plane(const Skeleton2D& p2d, const Vector3& X) noexcept
{
   const auto& ray = p2d.centre_ray();
   const auto N    = to_vec3(Vector3f(ray.x, ray.y, 0.0f)).normalised();
   const auto p3   = Plane(N, -X.dot(N));
   Expects(!p3.is_finite() || std::fabs(p3.side(X)) < 1e-4);
   return p3;
}

// ----------------------------------------------------- estimate-pose-direction
//
real estimate_pose_direction(const DistortedCamera& dcam,
                             const Skeleton2D& pose) noexcept
{
   auto calc_F = [&](int ind0, int ind1) {
      return recover_gaze_vector(dcam,
                                 to_vec2(pose.keypoints()[size_t(ind0)].xy()),
                                 to_vec2(pose.keypoints()[size_t(ind1)].xy()));
   };

   auto recover_gaze = [&](const Vector2& u, const Vector2& v) {
      return recover_gaze_vector(dcam, u, v);
   };

   auto theta_of = [&](const auto& F) -> real { return atan2(F.y, F.x); };

   // Eyes and ears
   const auto r_eye = to_vec2(pose.r_eye());
   const auto l_eye = to_vec2(pose.l_eye());
   const auto r_ear = to_vec2(pose.r_ear());
   const auto l_ear = to_vec2(pose.l_ear());
   const auto eye   = recover_gaze(r_eye, l_eye);
   const auto ear   = recover_gaze(r_ear, l_ear);

   // Use eye+ear if available...
   if(eye.is_finite() and ear.is_finite()) return theta_of(0.5 * (eye + ear));

   // Shoulders and hips
   const auto rs = to_vec2(pose.r_shoulder());
   const auto ls = to_vec2(pose.l_shoulder());
   const auto rh = to_vec2(pose.r_hip());
   const auto lh = to_vec2(pose.l_hip());
   const auto s  = recover_gaze(rs, ls);
   const auto h  = recover_gaze(rh, lh);

   // Okay, let's fall back to the shoulders and hips...
   if(s.is_finite() and h.is_finite()) return theta_of(0.5 * (s + h));

   // Okay, how about just the shoulders...
   if(s.is_finite()) return theta_of(s);

   // Just this hips?
   if(h.is_finite()) return theta_of(h);

   // Okay, we probably have a degenerate view...
   // If we can see the 'l' or 'r' shoulder, so assume that the view is
   // orthogonal to the ray.
   if(rs.is_finite()) return theta_of(recover_orth_gaze(dcam, true, rs));
   if(ls.is_finite()) return theta_of(recover_orth_gaze(dcam, false, ls));
   if(rh.is_finite()) return theta_of(recover_orth_gaze(dcam, true, rh));
   if(lh.is_finite()) return theta_of(recover_orth_gaze(dcam, false, lh));

   // Can we just go on the head??
   if(r_eye.is_finite()) return theta_of(recover_orth_gaze(dcam, true, r_eye));
   if(l_eye.is_finite()) return theta_of(recover_orth_gaze(dcam, false, l_eye));
   if(r_ear.is_finite()) return theta_of(recover_orth_gaze(dcam, true, r_ear));
   if(l_ear.is_finite()) return theta_of(recover_orth_gaze(dcam, false, l_ear));

   // Give up!!!
   return dNAN;
}

// ----------------------------------------------------- estimate-pose-floor-pos
//
Vector3 estimate_pose_floor_pos(const DistortedCamera& dcam,
                                const Skeleton2D& pose,
                                const real height) noexcept
{
   auto make_z_plane = [&](real height) {
      return Plane{0.0, 0.0, 1.0, -height};
   };

   auto find_xy = [&](const Plane& p3, const Vector2& D) {
      const auto Z = plane_ray_intersect(dcam, p3, D);
      return Vector2{Z.x, Z.y};
   };

   // ------
   // HACK!!!
   // For an adult, the neck should be 13/16 up the body
   //               the pelvis should be 7/16
   // @see https://en.wikipedia.org/wiki/Body_proportions
   // ------

   const auto e = to_vec2(pose.l_eye());
   if(e.is_finite()) return find_xy(make_z_plane(height * 14.0 / 16.0), e);

   const auto p = to_vec2(pose.pelvis());
   if(p.is_finite()) return find_xy(make_z_plane(height * 8.0 / 16.0), p);

   const auto n = to_vec2(pose.neck());
   if(n.is_finite()) return find_xy(make_z_plane(height * 13.0 / 16.0), n);

   // LOG_ERR(format("pose sucks"));
   // cout << str(pose) << endl;

   // FATAL("kBAM!");

   return Vector3::nan();
}

// ------------------------------------------------------------ estimate-floor-X
//
Vector3f estimate_floor_X(const LocalizationData::Params& loc_data_params,
                          const int n_skeletons,
                          const DistortedCamera** dcam_array,
                          const Skeleton2D** s2d_array,
                          const bool feedback) noexcept
{
   Expects(n_skeletons >= 2);
   constexpr int k_n_params = 2;
   const float n_inv        = 1.0f / float(n_skeletons);
   Vector3f O               = {0.0f, 0.0f, 0.0f};

   // Initialize `O` by averaging s2ds...
   for(auto i = 0; i < n_skeletons; ++i)
      O += s2d_array[i]->best_3d_result().X();
   O /= float(n_skeletons);
   O.z = 0.0f;

   constexpr float cos_theta0 = float(
       human::k_adult_male_radius
       / smath::sqrt(
           1.0 + human::k_adult_male_radius * human::k_adult_male_radius));

   if(!O.is_finite()) return Vector3f::nan(); // nothing to be done

   auto init_params = [&](float* X) {
      for(auto i = 0; i < k_n_params; ++i) X[i] = O(i);
   };

   auto init_step = [&](float* X) {
      for(auto i = 0; i < k_n_params; ++i) X[i] = 0.20f; // 5cm
   };

   auto unpack = [&](const float* X) {
      for(auto i = 0; i < k_n_params; ++i) O[i] = X[i];
      Expects(O.is_finite());
   };

   auto theta_err = [&](const auto& s2d) -> float {
      const auto& s3d      = s2d.best_3d_result();
      const auto C         = Vector2f(s3d.C().x, s3d.C().y);
      const auto cos_theta = uv_cos_theta(
          s3d.floor_ray(), (Vector2f(O.x, O.y) - C).normalised());
      const auto score = std::fabs(cos_theta) - cos_theta0;
      Expects(std::isfinite(cos_theta));
      return (score < 0.0f) ? 0.0f : score;
   };

   float best_score = std::numeric_limits<float>::max();
   const auto now   = tick();
   int counter      = 0;
   auto cost_fn     = [&](const float* X) -> float {
      unpack(X);

      auto sum_s = 0.0f;
      for(auto i = 0; i < n_skeletons; ++i) {
         const auto& s2d  = *s2d_array[i];
         const auto& dcam = *dcam_array[i];
         sum_s += hist_cell_plausibility(loc_data_params, s2d, dcam, to_vec3(O))
                      .probability();
      }

      // sum_ln_p <= 0.0, where 0.0 is the best possible score
      const auto score
          = std::clamp(1.0f - (sum_s / float(n_skeletons)), 0.0f, 1.0f);
      Expects(score >= -0.0f);

      if(feedback && score < best_score) {
         best_score    = score;
         const auto ms = 1000.f * float(tock(now));
         cout << format("#{:05d} :: s = {:7.4f} :: {:7.4f}ms :: X = [{}, {}]",
                        counter,
                        best_score,
                        ms,
                        X[0],
                        X[1])
              << endl;
      }

      ++counter;
      return score;
   };

   NonConvexOptimizer opt;
   opt.set_feedback_title(
       format("Estimating floor X for {} skeletons", n_skeletons));
   opt.set_n_params(k_n_params);
   opt.set_use_nelder_mead(true);
   opt.set_kcount(1000);
   opt.set_reqmin(1e-4f);
   opt.set_recursive_invocations(6);
   opt.set_diffstep(0.1f); // levelberg-marquardt
   init_params(opt.start());
   init_step(opt.step());

   static std::mutex padlock;
   non_automatic_lock_guard lock(padlock); // don't attempt to lock
   if(feedback) { lock.perform_lock(); }

   opt.optimize(cost_fn);

   if(feedback) {
      const auto msg = opt.feedback_string(true);
      sync_write([&]() { cout << msg << "\n\n" << endl; });
   }

   unpack(opt.xmin());
   return to_vec3f(O);
}

// -------------------------------------------------------------- trace-skeleton
//
void trace_skeleton(const Skeleton2D& pose,
                    std::function<void(const Point2&)> f) noexcept
{
   auto process_segment = [&](const Vector2& a, const Vector2& b) {
      if(a.is_finite() and b.is_finite())
         bresenham(a, b, [&](int x, int y) { f(Point2(x, y)); });
   };

   for(const auto& t : get_p2d_bones()) {
      process_segment(pose.keypoint_position(t.kp0),
                      pose.keypoint_position(t.kp1));
   }
}

template<typename T> void render_pose_t(T& im, const Skeleton2D& pose) noexcept
{
   auto draw_line = [&](const Vector2& A, const Vector2& B, const uint32_t k) {
      auto hsv      = rgb_to_hsv(kolour_to_vector3(k));
      hsv.z         = 1.0;
      const auto k1 = vector3_to_kolour(hsv_to_rgb(hsv));

      if(A.is_finite() and B.is_finite())
         plot_line_AA(A, B, [&](int x, int y, float a) { set(im, x, y, k1); });
   };

   for(const auto& t : get_p2d_bones()) {
      const auto A = t.kp0;
      const auto B = t.kp1;
      const auto k = t.kolour;
      if(k != 0xFF000000)
         draw_line(pose.keypoint_position(A), pose.keypoint_position(B), k);
   }
}

void render_pose(ARGBImage& im, const Skeleton2D& pose) noexcept
{
   render_pose_t(im, pose);
}

void render_pose(cv::Mat& im, const Skeleton2D& pose) noexcept
{
   render_pose_t(im, pose);
}

// -------------------------------------------------------------- draw billboard
//
template<typename T>
void draw_billboard_t(T& im,
                      const Skeleton2D& pose,
                      const CachingUndistortInverse& cu,
                      const EuclideanTransform& et_inv,
                      const uint32_t kolour) noexcept(false)
{
   const auto floor  = Plane{0.0, 0.0, 1.0, 0.0};
   const auto height = 1.65; // height of a person
   auto billboard    = pose_to_billboard(floor, cu, et_inv, pose, height);

   // Let's draw the billboard
   for(auto i = 0u; i < billboard.size(); ++i) {
      const auto& A = billboard[(i + 0) % billboard.size()];
      const auto& B = billboard[(i + 1) % billboard.size()];
      const auto a  = cu.distort(homgen_P2_to_R2(et_inv.apply(A)));
      const auto b  = cu.distort(homgen_P2_to_R2(et_inv.apply(B)));
      bresenham(a, b, [&](int x, int y) { set(im, x, y, kolour); });
   }
}

void draw_billboard(ARGBImage& im,
                    const Skeleton2D& pose,
                    const CachingUndistortInverse& cu,
                    const EuclideanTransform& et_inv,
                    const uint32_t kolour) noexcept(false)
{
   draw_billboard_t(im, pose, cu, et_inv, kolour);
}

void draw_billboard(cv::Mat& im,
                    const Skeleton2D& pose,
                    const CachingUndistortInverse& cu,
                    const EuclideanTransform& et_inv,
                    const uint32_t kolour) noexcept(false)
{
   draw_billboard_t(im, pose, cu, et_inv, kolour);
}

} // namespace perceive
