
#include "skeleton-3d.hpp"

#include "2d-helpers.hpp"
#include "skeleton-2d.hpp"

#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/geometry/projective/UV-on-same-p3-with-len.hpp"
#include "perceive/geometry/projective/V-from-U-and-UV-len.hpp"
#include "perceive/geometry/projective/ray-sphere-intersection.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/eigen-helpers.hpp"

#define This Skeleton3D

#define TRACE_DEBUG       \
   {                      \
      INFO("step");       \
      fflush(stdout);     \
      usleep(1000 * 500); \
      fgetc(stdin);       \
   }

namespace perceive
{
using namespace skeleton;

template<typename T>
static Vector2 project_ray(const DistortedCamera* dcam_ptr, const T& ray)
{
   if(dcam_ptr == nullptr || !is_finite(ray)) return Vector2::nan();
   return project_ray_to_distorted(*dcam_ptr, to_vec3(ray));
}

static string debug_string(const DistortedCamera* dcam_ptr,
                           const Skeleton2D& p2d,
                           const vector<Vector3f>& Xs)
{
   std::stringstream ss{""};

   ss << format("DEBUG skeleton\n");
   {
      for(auto&& [ray, kp] : views::zip(p2d.rays(), p2d.keypoints())) {
         // for(const auto& kp : p2d.keypoints()) {
         ss << format("{:20s}   {}  ==>  {}  ==>  {}\n",
                      str(KeypointName(kp.part)),
                      str(kp.pos),
                      str(ray),
                      str(project_ray(dcam_ptr, ray)));
      }
   }
   ss << endl;

   {
      int counter = 0;
      for(auto&& [X, ray] : views::zip(Xs, p2d.rays())) {
         const auto x = (dcam_ptr && is_finite(X))
                            ? project_to_distorted(*dcam_ptr, to_vec3(X))
                            : Vector2::nan();
         const auto y = project_ray(dcam_ptr, ray);

         ss << format("{:20s}   {}  ==>  {}  ::  ray  ==>  {}\n",
                      str(KeypointName(counter++)),
                      str(X),
                      str(x),
                      str(y));
      }
   }
   ss << endl;
   return ss.str();
}

template<typename T>
static constexpr std::pair<int, int>
lookup_kps_(const T& kps, KeypointName kp0, KeypointName kp1)
{
   std::pair<int, int> out;
   for(auto i = 0u; i < kps.size(); ++i) {
      if(kps[i] == kp0) out.first = int(i);
      if(kps[i] == kp1) out.second = int(i);
   }
   return out;
}

template<typename T>
static std::pair<T, T> recover_bone_(const Vector3T<T>& u, // u ray
                                     const Vector3T<T>& v, // v ray
                                     const Vector3T<T>& n, // plane normal
                                     const T length)       // Length of (U - V)
{
   return UV_on_same_p3_with_len(u, v, n, length);
}

static void
unpack_torso(const float* X,
             const Vector3f& C,
             const std::array<Vector3f, skeleton::k_n_keypoints>& rays,
             const float recovery_height,
             Plane4f& torso_p3,
             Vector3f& hip_n,
             Vector3f& up_n,
             float& ray_len,
             float& hip_rotation,
             float& pelvis_size,
             vector<Vector3f>& Xs)
{
   std::fill(begin(Xs), end(Xs), Vector3f::nan());

   torso_p3.xyz() = spherical_to_cartesian(X[0], X[1], 1.0f);
   ray_len        = X[2];
   hip_rotation   = X[3];
   pelvis_size    = X[4];

   {
      bool has_error = false;
      for(auto i = 0; i < 5; ++i)
         if(!std::isfinite(X[i])) has_error = true;
      if(has_error) {
         cout << format("nan has crept into parameters:") << endl;
         for(auto i = 0; i < 5; ++i)
            cout << format("X[{}] = {}", i, X[i]) << endl;
         assert(false);
         Expects(false);
      }
   }

   const auto& pelvis_ray    = rays[int(KeypointName::PELVIS)];
   const auto& neck_ray      = rays[int(KeypointName::NOTCH)];
   const bool has_pelvis     = pelvis_ray.is_finite();
   const bool has_neck       = neck_ray.is_finite();
   const bool has_l_hip      = rays[int(KeypointName::L_HIP)].is_finite();
   const bool has_r_hip      = rays[int(KeypointName::R_HIP)].is_finite();
   const bool has_l_shoulder = rays[int(KeypointName::L_SHOULDER)].is_finite();
   const bool has_r_shoulder = rays[int(KeypointName::R_SHOULDER)].is_finite();
   const bool has_l_ankle    = rays[int(KeypointName::L_ANKLE)].is_finite();
   const bool has_r_ankle    = rays[int(KeypointName::R_ANKLE)].is_finite();
   const bool has_nose       = rays[int(KeypointName::NOSE)].is_finite();

   if(!has_pelvis && !has_neck) return; // nothing to do

   const auto& bones = get_p2d_bones();
   const auto shoulder_to_hip_ratio
       = bones[Bone::L_COLLAR].proportion / bones[Bone::L_PELVIS].proportion;
   const auto shoulder_size = pelvis_size * shoulder_to_hip_ratio;

   auto& neck       = Xs[int(KeypointName::NOTCH)];
   auto& pelvis     = Xs[int(KeypointName::PELVIS)];
   auto& l_hip      = Xs[int(KeypointName::L_HIP)];
   auto& r_hip      = Xs[int(KeypointName::R_HIP)];
   auto& l_shoulder = Xs[int(KeypointName::L_SHOULDER)];
   auto& r_shoulder = Xs[int(KeypointName::R_SHOULDER)];

   const auto refX = C + ray_len * (has_pelvis ? pelvis_ray : neck_ray);

   auto recover_Xs = [&](const Plane4f& p3, KeypointName kp) -> Vector3f& {
      Xs[size_t(kp)] = plane_ray_intersection(p3, C, C + rays[size_t(kp)]);
      return Xs[size_t(kp)];
   };

   auto calc_test_left_n = [&]() -> Vector3f {
      if(has_l_hip && has_r_hip)
         return -(l_hip - r_hip).normalised();
      else if(has_l_shoulder && has_r_shoulder)
         return -(l_shoulder - r_shoulder).normalised();
      else if(has_l_shoulder && has_neck)
         return -(l_shoulder - neck).normalised();
      else if(has_neck && has_r_shoulder)
         return -(neck - r_shoulder).normalised();
      else if(has_l_hip && has_pelvis)
         return -(l_hip - pelvis).normalised();
      else if(has_pelvis && has_r_hip)
         return -(pelvis - r_hip).normalised();
      return Vector3f::nan();
   };

   auto calc_test_up_n = [&]() -> Vector3f {
      if(has_pelvis && has_neck)
         return (neck - pelvis).normalised();
      else if(has_l_shoulder && has_l_hip)
         return (l_shoulder - l_hip).normalised();
      else if(has_r_shoulder && has_r_hip)
         return (r_shoulder - r_hip).normalised();
      else if(has_l_ankle && has_pelvis)
         return (pelvis - recover_Xs(torso_p3, KeypointName::L_ANKLE))
             .normalised();
      else if(has_r_ankle && has_pelvis)
         return (pelvis - recover_Xs(torso_p3, KeypointName::R_ANKLE))
             .normalised();
      else if(has_nose && has_pelvis)
         return (recover_Xs(torso_p3, KeypointName::NOSE) - pelvis)
             .normalised();
      else if(has_nose && has_neck)
         return (recover_Xs(torso_p3, KeypointName::NOSE) - neck).normalised();
      return Vector3f::nan();
   };

   // Recover torso
   Vector3f& torso_n = torso_p3.xyz();
   torso_p3.d()      = -dot(torso_p3.xyz(), refX);
   if(!torso_p3.is_finite()) {
      cout << format("torso-p3:    {}\n", str(torso_p3));
      cout << format("ray-len:     {}\n", ray_len);
      cout << format("hip-rot:     {}\n", hip_rotation);
      cout << format("pelvis_sz:   {}\n", pelvis_size);
      cout << format("has-pelivs:  {}\n", str(has_pelvis));
      cout << format("has-neck:    {}\n", str(has_neck));
      cout << format("has-l-hip:   {}\n", str(has_l_hip));
      cout << format("has-r-hip:   {}\n", str(has_r_hip));
      cout << format("has-l-shoul: {}\n", str(has_l_shoulder));
      cout << format("has-r-shoul: {}\n", str(has_r_shoulder));
      cout << format("has-l-ankle: {}\n", str(has_l_ankle));
      cout << format("has-r-ankle: {}\n", str(has_r_ankle));
      cout << format("has-nose:    {}\n", str(has_nose));
      Expects(torso_p3.is_finite());
   }
   const auto& tp3 = torso_p3;
   recover_Xs(torso_p3, KeypointName::NOTCH);
   recover_Xs(torso_p3, KeypointName::PELVIS);
   recover_Xs(torso_p3, KeypointName::L_SHOULDER); // Will update later
   recover_Xs(torso_p3, KeypointName::R_SHOULDER); //
   recover_Xs(torso_p3, KeypointName::L_HIP);      // Will update later
   recover_Xs(torso_p3, KeypointName::R_HIP);      //

   // If we have a spine, then set the
   Vector3f left_n        = Vector3f::nan();
   const auto test_left_n = calc_test_left_n();

   up_n = Vector3f::nan();
   if(has_pelvis && has_neck) { // i.e., we have the spine
      //  INFO("spine");
      up_n   = (neck - pelvis).normalised();
      left_n = cross(torso_n, up_n);

      // WARN(format("up = {}, left = {}", str(up_n), str(left_n)));

      // Do we need to flip torso and up?
      if(dot(left_n, test_left_n) < 0.0f) {
         left_n   = -left_n;
         torso_p3 = -torso_p3;
      }
   }

   if(!up_n.is_finite() && (has_pelvis || has_neck)) { // Use the legs
      const auto test_up_n = calc_test_up_n();
      left_n               = test_left_n;
      up_n                 = cross(left_n, torso_n);

      if(dot(up_n, test_up_n) < 0.0f) {
         up_n     = -up_n;
         torso_p3 = -torso_p3;
      }
   }

   if(!up_n.is_finite()) return; // degenerate case

   // Check handedness of system
   const auto dot_cross = dot(cross(torso_n, up_n), left_n);

   if(dot_cross < 0.0f) {
      const auto x = left_n;
      const auto y = torso_n;
      const auto z = up_n;

      LOG_ERR(format("Handedness error in 3D recovery:"));
      cout << format("torso-p3 = {}", str(torso_p3)) << endl;
      cout << format("left  = x = {} = c(y, z) = {}", str(x), str(cross(y, z)))
           << endl;
      cout << format("torso = y = {} = c(z, x) = {}", str(y), str(cross(z, x)))
           << endl;
      cout << format("up    = z = {} = c(x, y) = {}", str(z), str(cross(x, y)))
           << endl;
      cout << format("x dot cross(y, z) = {}",
                     dot(cross(torso_n, up_n), left_n))
           << endl;

      Expects(false);
   }

   if(!is_close(dot_cross, 1.0f)) return; // degenerate case

   // Recover the leg planes
   const auto q = axis_angle_to_quaternion(
       to_vec4(Vector4f(up_n.x, up_n.y, up_n.z, hip_rotation)));
   hip_n = to_vec3f(q.rotate(to_vec3(torso_n)));
   const auto hip_ll
       = to_vec3f(q.rotate(to_vec3(left_n).normalised())) * 0.5f * pelvis_size;

   { // Recover the legs
      const auto l_leg_p3 = Plane4f{hip_n, -dot(hip_n, refX + hip_ll)};
      recover_Xs(l_leg_p3, KeypointName::L_HIP);
      recover_Xs(l_leg_p3, KeypointName::L_KNEE);
      recover_Xs(l_leg_p3, KeypointName::L_ANKLE);

      const auto r_leg_p3 = Plane4f{hip_n, -dot(hip_n, refX - hip_ll)};
      recover_Xs(r_leg_p3, KeypointName::R_HIP);
      recover_Xs(r_leg_p3, KeypointName::R_KNEE);
      recover_Xs(r_leg_p3, KeypointName::R_ANKLE);
   }

   if(has_neck) { // Redo shoulders
      // l_shoulder = neck + 0.5f * shoulder_size * left_n;
      // r_shoulder = neck - 0.5f * shoulder_size * left_n;
   }

   { // Recover the arms
      auto recover_arm = [&](const auto kp_shoulder,
                             const auto kp_elbow,
                             const auto kp_wrist) {
         Xs[size_t(kp_elbow)] = Xs[size_t(kp_wrist)] = Vector3f::nan();
         const auto& shoulder                        = Xs[size_t(kp_shoulder)];
         auto& elbow                                 = Xs[size_t(kp_elbow)];
         auto& wrist                                 = Xs[size_t(kp_wrist)];
         if(!shoulder.is_finite()) return; // nothing to do
         const auto len_up  = bones[Bone::L_ARM_UPPER].length(recovery_height);
         const auto len_low = bones[Bone::L_ARM_LOWER].length(recovery_height);

         const auto p3
             = Plane4f{torso_p3.xyz(), -dot(torso_p3.xyz(), shoulder)};
         const auto [e0, e1]
             = calc_k_v((shoulder - C).norm(),
                        dot(rays[size_t(kp_shoulder)], rays[size_t(kp_elbow)]),
                        len_up);

         // Choose e0 or e1 that is closest to torso_p3
         const auto E0 = C + e0 * rays[size_t(kp_elbow)];
         const auto E1 = C + e1 * rays[size_t(kp_elbow)];
         elbow = (std::fabs(p3.side(E0)) < std::fabs(p3.side(E1))) ? E0 : E1;

         const auto [w0, w1]
             = calc_k_v((elbow - C).norm(),
                        dot(rays[size_t(kp_elbow)], rays[size_t(kp_wrist)]),
                        len_low);

         // Choose w0 or w1 that makes smallest angle with upper arm
         const auto W0 = C + w0 * rays[size_t(kp_wrist)];
         const auto W1 = C + w1 * rays[size_t(kp_wrist)];
         wrist         = (dot(W0 - elbow, elbow - shoulder)
                  > dot(W1 - elbow, elbow - shoulder))
                             ? W0
                             : W1;
      };

      recover_arm(KeypointName::L_SHOULDER,
                  KeypointName::L_ELBOW,
                  KeypointName::L_WRIST);
      recover_arm(KeypointName::R_SHOULDER,
                  KeypointName::R_ELBOW,
                  KeypointName::R_WRIST);
   }
}

// ---------------------------------------------------------------- construction

This::Skeleton3D() { Xs_.resize(skeleton::k_n_keypoints, Vector3f::nan()); }

// ------------------------------------------------------------------ fit-3d-ret

Skeleton3D This::fit_3d_ret(const DistortedCamera* dcam_ptr,
                            const Skeleton2D& p2d,
                            const bool feedback) noexcept(false)
{
   constexpr float reproj_weight       = 10.0f;
   constexpr float upright_bias_weight = 0.4f;

   static std::mutex padlock;
   non_automatic_lock_guard lock(padlock); // don't attempt to lock

   if(feedback) { lock.perform_lock(); }

   const auto now = tick();

   // (*) Recover the torso and legs:
   //     + 4df: 3 for the plane, 1 for hip rotation.
   //            Pelvis, neck and shoulders from plane
   //            Hips from rotated plane
   //            Legs from intersections with hip plane
   //     + Error is reprojection error
   // (*) Recover the head:
   //     + 3df: 1 for dist to nose, 2 for looking angle
   // (*) Recover each arm:
   //     + Directly calculated

   // ------------------------------------------------ setup
   const auto& bones     = get_p2d_bones();
   const auto& C         = p2d.cam_centre();
   Vector3f base_X       = Vector3f::nan();
   Plane4f torso_p3      = Plane4f::nan();
   Vector3f hip_n        = Vector3f::nan();
   Vector3f up_n         = Vector3f::nan();
   Vector3f head_n       = Vector3f::nan();
   float recovery_height = float(human::k_average_height);
   vector<Vector3f> Xs   = vector<Vector3f>(Skeleton2D::k_n_keypoints);
   std::fill(begin(Xs), end(Xs), Vector3f::nan());

   const auto& rays = p2d.rays();
   Expects(Xs.size() == rays.size());

   // -------------------------------------------- the model

   constexpr std::array<KeypointName, 14> body_model_kps
       = {KeypointName::NOTCH,
          KeypointName::PELVIS,
          KeypointName::L_SHOULDER,
          KeypointName::R_SHOULDER,
          KeypointName::L_HIP,
          KeypointName::R_HIP,
          KeypointName::L_KNEE,
          KeypointName::R_KNEE,
          KeypointName::L_ANKLE,
          KeypointName::R_ANKLE,
          KeypointName::L_ELBOW,
          KeypointName::R_ELBOW,
          KeypointName::L_WRIST,
          KeypointName::R_WRIST};

   constexpr std::array<Bone::Label, 13> body_model_bones = {Bone::SPINE,
                                                             Bone::L_COLLAR,
                                                             Bone::R_COLLAR,
                                                             Bone::L_PELVIS,
                                                             Bone::R_PELVIS,
                                                             Bone::L_LEG_UPPER,
                                                             Bone::L_LEG_LOWER,
                                                             Bone::R_LEG_UPPER,
                                                             Bone::R_LEG_LOWER,
                                                             Bone::L_ARM_UPPER,
                                                             Bone::L_ARM_LOWER,
                                                             Bone::R_ARM_UPPER,
                                                             Bone::R_ARM_LOWER};

   constexpr std::array<Bone::Label, 5> head_bones
       = {Bone::NECK, Bone::L_FACE, Bone::L_SKULL, Bone::R_FACE, Bone::R_SKULL};

   // ------------------------------------ utility functions

   auto bone_is_finite
       = [&p2d](Bone::Label bone) { return has_bone(p2d, get_bone(bone)); };

   const auto& pelvis_ray = rays[int(PELVIS)];
   const auto& notch_ray  = rays[int(NOTCH)];
   const auto& nose_ray   = rays[int(NOSE)];
   const bool has_pelvis  = pelvis_ray.is_finite();
   const bool has_notch   = notch_ray.is_finite();
   const bool has_nose    = nose_ray.is_finite();

   const auto& l_face  = skeleton::get_bone(skeleton::Bone::L_FACE);
   const auto& r_face  = skeleton::get_bone(skeleton::Bone::R_FACE);
   const auto& l_skull = skeleton::get_bone(skeleton::Bone::L_SKULL);
   const auto& r_skull = skeleton::get_bone(skeleton::Bone::R_SKULL);

   { // -------------------------------- torso legs and hips

      const bool has_spine    = has_bone(p2d, get_bone(Bone::SPINE));
      const bool has_l_pelvis = has_bone(p2d, get_bone(Bone::L_PELVIS));
      const bool has_r_pelvis = has_bone(p2d, get_bone(Bone::R_PELVIS));
      const bool has_l_collar = has_bone(p2d, get_bone(Bone::L_COLLAR));
      const bool has_r_collar = has_bone(p2d, get_bone(Bone::R_COLLAR));

      if(!has_spine && !has_l_pelvis && !has_r_pelvis && !has_l_collar
         && !has_r_collar)
         return {};

      Expects(has_pelvis || has_notch);
      const auto ref_ray0_kp
          = has_pelvis ? KeypointName::PELVIS : KeypointName::NOTCH;
      auto get_ref1 = [&]() -> std::pair<KeypointName, Bone::Label> {
         if(has_spine) return {KeypointName::NOTCH, Bone::SPINE};
         if(has_l_pelvis) return {KeypointName::L_HIP, Bone::L_PELVIS};
         if(has_r_pelvis) return {KeypointName::R_HIP, Bone::R_PELVIS};
         if(has_l_collar) return {KeypointName::L_SHOULDER, Bone::L_COLLAR};
         if(has_r_collar) return {KeypointName::R_SHOULDER, Bone::R_COLLAR};
         Expects(false);
         return {};
      };
      const auto [ref_ray1_kp, ref_bone_label] = get_ref1();
      const auto& ref_ray0                     = rays[size_t(ref_ray0_kp)];
      const auto& ref_ray1                     = rays[size_t(ref_ray1_kp)];
      Expects(ref_ray0.is_finite() && ref_ray1.is_finite());

      const auto ref_height      = recovery_height;
      const auto ref_bone        = get_bone(ref_bone_label);
      const auto ref_bone_length = ref_bone.length(ref_height);

      auto calc_P0 = [&]() {
         const auto& ref_ray0 = has_pelvis ? pelvis_ray : notch_ray;
         const auto& n        = ref_ray0;
         const auto [k_u, k_v]
             = recover_bone_(ref_ray1, ref_ray0, n, ref_bone_length);
         const auto U = C + k_u * ref_ray1;
         const auto V = C + k_v * ref_ray0;
         // Expects_is_close(dot(U, n), dot(V, n), fNAN);
         // Expects_is_close((U - V).norm(), ref_bone_length, 1e-1f);

         if(false && !V.is_finite()) {
            LOG_ERR("failed to get finite `P0`:");
            cout << format("C        = {}", str(C)) << endl;
            cout << format("ref_ray0 = {}", str(ref_ray0)) << endl;
            cout << format("ref_ray1 = {}", str(ref_ray1)) << endl;
            cout << format("n        = {}", str(n)) << endl;
            cout << format("ref-blen = {}", ref_bone_length) << endl;
            cout << format("ku kv    = [{}, {}]", k_u, k_v) << endl;
            cout << format("U        = {}", str(U)) << endl;
            cout << format("V        = {}", str(V)) << endl;
         }

         return V;
      };

      const auto n_params = 5;

      const auto P0 = calc_P0();
      if(!P0.is_finite()) return {};

      float ray_len      = (C - P0).norm();
      float hip_rotation = 0.0f;
      float pelvis_size  = 2.0f * get_bone(Bone::L_PELVIS).length(ref_height);
      const auto z_axis  = Vector3f{0.0f, 0.0f, 1.0f};
      const auto y_axis  = Vector3f{0.0f, 1.0f, 1.0f};

      auto init_params = [&](float* X) {
         // May initial ray orthogonal to the floor
         const auto y_axis = cross(z_axis, ref_ray0);
         Vector3f s        = cartesian_to_spherical(cross(y_axis, z_axis));
         X[0]              = s.x; // inclination
         X[1]              = s.y; // azimuth
         X[2]              = ray_len;
         X[3]              = hip_rotation;
         X[4]              = pelvis_size;

         Expects(s.is_finite());
         Expects(std::isfinite(ray_len));
         Expects(std::isfinite(hip_rotation));
         Expects(std::isfinite(pelvis_size));
      };

      auto unpack = [&](const float* X) {
         unpack_torso(X,
                      C,
                      rays,
                      recovery_height,
                      torso_p3,
                      hip_n,
                      up_n,
                      ray_len,
                      hip_rotation,
                      pelvis_size,
                      Xs);
      };

      int counter    = 0;
      float best_err = std::numeric_limits<float>::max();
      auto cost_fn   = [&](const float* X) -> float {
         unpack(X);

         // Get reprojection error of keypoints
         int reproj_counter = 0;
         auto reproj_err    = 0.0f;
         for(const auto kp : body_model_kps) {
            const auto X = Xs[size_t(kp)];
            if(!X.is_finite()) continue;
            const auto x = homgen_P2_to_R2(X - C);
            const auto y = homgen_P2_to_R2(rays[size_t(kp)]);
            reproj_err += (x - y).norm();
            ++reproj_counter;
         }

         if(reproj_counter == 0) return std::numeric_limits<float>::max();
         Expects(reproj_counter > 0);
         reproj_err /= float(reproj_counter);

         // Get the proportions for spine and legs
         int bone_counter = 0;
         auto bone_err    = 0.0f;
         for(const auto bone_label : body_model_bones) {
            const auto bone = bones[size_t(bone_label)];
            if(Xs[size_t(bone.kp0)].is_finite()
               && Xs[size_t(bone.kp1)].is_finite()) {
               const auto dist
                   = (Xs[size_t(bone.kp0)] - Xs[size_t(bone.kp1)]).norm();
               bone_err += std::fabs(dist - bone.length(recovery_height));
               ++bone_counter;
            }
         }

         if(bone_counter == 0) return std::numeric_limits<float>::max();
         Expects(bone_counter > 0);
         bone_err /= float(bone_counter);

         // Get the difference between 'up' and 'z-axis'
         const float cos_z_theta
             = std::clamp(std::fabs(dot(up_n, z_axis)), 0.0f, 1.0f);
         const float stand_score = std::clamp(1.0f - cos_z_theta, 0.0f, 1.0f);
         const float lie_score   = 2.0f * cos_z_theta; // 2.0: bias against
         const float theta_score = std::min(lie_score, stand_score);

         // we probably need a weight
         const auto err = reproj_weight * reproj_err
                          + upright_bias_weight * theta_score + bone_err;

         if(feedback && err < best_err) {
            best_err         = err;
            const float werr = reproj_weight * reproj_err;
            const float terr = upright_bias_weight * theta_score;

            cout << format("  #{: 5d}  reproj = {:6.4f}  theta = {:6.4f}  bone "
                           "= {:6.4f}  err = "
                           "{:6.4f}",
                           counter,
                           werr,
                           to_degrees(std::acos(theta_score)),
                           bone_err,
                           best_err)
                 << endl;
         }

         ++counter;
         return err;
      };

      const bool use_nelder_mead = false;
      vector<float> start(n_params);
      vector<float> xmin(n_params);
      float ynewlo   = NAN;
      float ystartlo = NAN;
      float reqmin   = 1e-7f;
      float diffstep = float(one_degree());
      int kcount     = 1000; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method_str = nullptr;
      const auto now1        = tick();

      init_params(&start[0]);
      ystartlo = cost_fn(&start[0]);

      if(!use_nelder_mead) {
         method_str = "levenberg-marquardt";
         levenberg_marquardt(cost_fn,   // cost function
                             n_params,  // n parameters
                             &start[0], // start
                             &xmin[0],  // xmin
                             reqmin,    // required minimum variance
                             diffstep,  // initial difference step
                             5,         // recursive invocations
                             kcount,    // max interations
                             icount,    // output count
                             ifault);   // output fault
         ynewlo = cost_fn(&xmin[0]);

      } else {
         method_str = "nelder-mead";

         vector<float> step(n_params);
         std::fill(begin(step), end(step), one_degree());
         nelder_mead(cost_fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     20 * kcount,
                     icount,
                     numres,
                     ifault);

         unpack(&xmin[0]);
      }

      if(feedback) {
         sync_write([&]() {
            cout << format("Feedback for finding body model") << endl;
            cout << format("   method               {:s}", string(method_str))
                 << endl;
            cout << format("   n-params             {:d}", n_params) << endl;
            cout << format("   counter              {}", counter) << endl;
            cout << format("   iterations           {}", icount) << endl;
            cout << format("   fault-code           {}", ifault) << endl;
            auto msg = (use_nelder_mead)
                           ? nelder_mead_fault_str(ifault)
                           : levenberg_marquardt_fault_str(ifault);
            cout << format("   fault-string         {}", msg) << endl;
            cout << endl;
            cout << format("   initial-score        {}", ystartlo) << endl;
            cout << format("   final-score          {}", ynewlo) << endl;
            cout << format("   time                 {}ms", 1000.0 * tock(now1))
                 << endl;
            cout << format(
                "   up-n-theta           {}",
                to_degrees(std::acos(up_n.dot(Vector3f(0.0, 0.0, 1.0)))));
            cout << format("   up                   {}", str(up_n)) << endl;
            cout << format("   forward              {}", str(torso_p3.xyz()))
                 << endl;
            cout << endl;
            cout << endl;
         });
      }
   }

   if(has_nose) { // ---------------------------------- head
      constexpr int n_params = 3;

      const skeleton::Bone neck = get_bone(skeleton::Bone::NECK);

      head_n = torso_p3.is_finite() ? torso_p3.xyz() : -nose_ray;

      const float skull_radius
          = recovery_height * skeleton::Bone::adult_height_inv;
      const float nose_len = skull_radius * 0.15f;

      float nose_ray_len = fNAN;
      bool is_init       = false;
      if(has_notch && torso_p3.is_finite()) {
         const auto [N0, N1] = V_from_U_and_UV_len(
             C, Xs[NOTCH], nose_ray, neck.length(recovery_height));
         Expects(nose_ray.is_finite());
         if(N0.is_finite() && N1.is_finite()) {
            // Select N closest to the torso, in front...
            const auto s0 = torso_p3.side(N0);
            const auto s1 = torso_p3.side(N1);

            if(s0 >= 0.0f && s1 >= 0.0f) {
               nose_ray_len = (s0 < s1) ? (N0 - C).norm() : (N1 - C).norm();
            } else if(s0 >= 0.0f) {
               nose_ray_len = (N0 - C).norm();
            } else if(s1 >= 0.0f) {
               nose_ray_len = (N1 - C).norm();
            }
         }
      }

      if(!std::isfinite(nose_ray_len)) {
         const float nose_height
             = float(human::k_height_ratio_nose) * recovery_height;
         nose_ray_len = (nose_height - C.z) / nose_ray.z;
      }

      if(!std::isfinite(nose_ray_len)) { nose_ray_len = 1.0f; }

      auto recover_skull_point = [&](const Vector3f& O,
                                     const float radius,
                                     const skeleton::Bone bone) -> Vector3f {
         const auto& X   = Xs[size_t(bone.kp0)];
         const auto& ray = rays[size_t(bone.kp1)];
         if(!X.is_finite() || !ray.is_finite()) return Vector3f::nan();
         const auto [k0, k1] = ray_sphere_intersection(C, ray, O, radius);
         Expects(std::isfinite(k0) == std::isfinite(k1));
         if(!std::isfinite(k0)) return Vector3f::nan(); // no intersection
         const auto A    = C + k0 * ray;
         const auto B    = C + k1 * ray;
         const auto lenA = (A - X).norm();
         const auto lenB = (B - X).norm();
         const auto len  = bone.length(recovery_height);
         return (std::fabs(len - lenA) < std::fabs(len - lenB)) ? A : B;
      };

      auto init_params = [&](float* X) {
         Vector3f sph = cartesian_to_spherical(head_n);
         X[0]         = sph.x;
         X[1]         = sph.y;
         X[2]         = nose_ray_len;
      };

      auto unpack = [&](const float* X) {
         head_n       = spherical_to_cartesian(X[0], X[1], 1.0f);
         nose_ray_len = X[2];

         Xs[NOSE] = C + nose_ray_len * nose_ray;

         // Get the sphere center
         const auto O = Xs[NOSE] - head_n * (skull_radius + nose_len);

         Xs[L_EYE] = recover_skull_point(O, skull_radius, l_face);
         Xs[R_EYE] = recover_skull_point(O, skull_radius, r_face);
         Xs[L_EAR] = recover_skull_point(O, skull_radius, l_skull);
         Xs[R_EAR] = recover_skull_point(O, skull_radius, r_skull);
      };

      int counter    = 0;
      float best_err = std::numeric_limits<float>::max();
      auto cost_fn   = [&](const float* X) -> float {
         unpack(X);

         float err   = 0.0f;
         int n_bones = 0;
         for(const auto label : head_bones) {
            const auto& bone = bones[size_t(label)];
            if(Xs[size_t(bone.kp0)].is_finite()
               && Xs[size_t(bone.kp1)].is_finite()) {
               const auto est_len
                   = (Xs[size_t(bone.kp0)] - Xs[size_t(bone.kp1)]).norm();
               const auto bone_len = bone.length(recovery_height);
               err += std::fabs(est_len - bone_len);
               ++n_bones;
            }
         }

         err = (n_bones == 0) ? 10.0f : (err / float(n_bones));

         if(feedback && err < best_err) {
            best_err = err;
            cout << format("#{:05d}  err = {}", counter, best_err) << endl;
         }

         ++counter;
         return err;
      };

      const bool use_nelder_mead = false;
      vector<float> start(n_params);
      vector<float> xmin(n_params);
      float ynewlo   = NAN;
      float ystartlo = NAN;
      float reqmin   = 1e-7f;
      float diffstep = float(one_degree());
      int kcount     = 1000; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method_str = nullptr;
      const auto now1        = tick();

      init_params(&start[0]);
      ystartlo = cost_fn(&start[0]);

      if(!use_nelder_mead) {
         method_str = "levenberg-marquardt";
         levenberg_marquardt(cost_fn,   // cost function
                             n_params,  // n parameters
                             &start[0], // start
                             &xmin[0],  // xmin
                             reqmin,    // required minimum variance
                             diffstep,  // initial difference step
                             5,         // recursive invocations
                             kcount,    // max interations
                             icount,    // output count
                             ifault);   // output fault
         ynewlo = cost_fn(&xmin[0]);
      } else {
         method_str = "nelder-mead";

         vector<float> step(n_params);
         std::fill(begin(step), end(step), one_degree());
         nelder_mead(cost_fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     20 * kcount,
                     icount,
                     numres,
                     ifault);
         unpack(&xmin[0]);
      }

      if(feedback) {
         sync_write([&]() {
            cout << format("Feedback for finding head model") << endl;
            cout << format("   method               {:s}", string(method_str))
                 << endl;
            cout << format("   n-params             {:d}", n_params) << endl;
            cout << format("   counter              {}", counter) << endl;
            cout << format("   iterations           {}", icount) << endl;
            cout << format("   fault-code           {}", ifault) << endl;
            auto msg = (use_nelder_mead)
                           ? nelder_mead_fault_str(ifault)
                           : levenberg_marquardt_fault_str(ifault);
            cout << format("   fault-string         {}", msg) << endl;
            cout << endl;
            cout << format("   initial-score        {}", ystartlo) << endl;
            cout << format("   final-score          {}", ynewlo) << endl;
            cout << format("   time                 {}ms", 1000.0 * tock(now1))
                 << endl;
            cout << endl;
            cout << endl;
         });
      }
   }

   { // -------------------------------- 3d location of base
      Expects((has_pelvis || has_notch));

      // Take a plane with 'up' as the normal, and pelvis or neck on it
      const auto& refX = Xs[has_pelvis ? PELVIS : NOTCH];

      Plane4f base(up_n, -dot(up_n, refX));

      float total_weight = 0.0f;
      float total_dist   = 0.0f;
      for(const auto& prop : get_p2d_proportions()) {
         const auto& X = Xs.at(size_t(prop.kp));
         if(!X.is_finite()) continue;
         const auto Y = X - float(prop.height_ratio) * recovery_height * up_n;
         total_dist += base.side(Y) * float(prop.weight);
         total_weight += float(prop.weight);
      };
      const auto av_dist = total_dist / total_weight;

      base.d() = -dot(up_n, refX + av_dist * up_n);

      // Finally, calculate 'baseX'
      base_X = plane_ray_intersection(base, refX, refX + up_n);
   }

   { // ---------------------- Rescale to the optimal height
     // Let's put those feet on the ground!

      const auto base_X0 = base_X;
      const auto Xs0     = Xs;

      auto update_Xs = [&](const double height) {
         auto min_z       = std::numeric_limits<double>::max();
         const auto ratio = float(height) / recovery_height;
         for(auto&& [X, X0] : views::zip(Xs, Xs0)) {
            if(is_finite(X0)) {
               X = C + ratio * (X0 - C);
               if(double(X.z) < min_z) min_z = double(X.z);
            }
         }
         base_X = C + ratio * (base_X0 - C);
         return std::fabs(min_z);
      };

      float best_height = fNAN;
      auto best_score   = std::numeric_limits<double>::max();
      auto calc_Xs      = Xs;
      auto fn           = [&](double height) -> double {
         const auto score = update_Xs(height);

         // const float ratio = height / recovery_height;
         // auto X            = C + ratio * (base_X - C);
         // double score      = std::fabs(X.z);
         if(score < best_score) {
            best_score  = score;
            best_height = float(height);
         }
         return score;
      };

      golden_section_search(fn, human::min_height(), human::max_height(), 1e-3);

      update_Xs(real(best_height));
      recovery_height = best_height;
   }

   if(!base_X.is_finite()) return {};

   Skeleton3D out;

   { // Initialize output
      out.C_          = C;
      out.sensor_no_  = p2d.sensor_no();
      out.X_          = base_X;
      out.torso_p3_   = torso_p3;
      out.up_n_       = up_n;
      out.left_n_     = cross(up_n, torso_p3.xyz());
      out.hip_n_      = hip_n;
      out.head_n_     = head_n;
      out.height_     = recovery_height;
      out.seconds_    = float(tock(now));
      out.Xs_         = std::move(Xs);
      out.reproj_err_ = 0.0f;
      out.floor_line_ // Issue if the person goes under the camera
          = to_homgen_line(Vector2f(C.x, C.y), Vector2f(base_X.x, base_X.y));
      out.floor_ray_ = Vector2f(base_X.x - C.x, base_X.y - C.y).normalised();

      bool has_error = false;
      if(!out.floor_line_.is_finite()) { has_error = true; }

      if(!out.floor_ray_.is_finite()) { has_error = true; }

      if(has_error) {
         LOG_ERR(format(R"V0G0N(
base_X     =  {}
Xs         = [{}]              
floor-line =  homgen-line([{}, {}], [{}, {}]) = {}

floor-ray  =  [{} - {}, {} - {}].normalised = {}
)V0G0N",
                        str(base_X),
                        implode(cbegin(out.Xs_),
                                cend(out.Xs_),
                                ",\n              ",
                                [&](const auto& X) { return str(X); }),
                        C.x,
                        C.y,
                        base_X.x,
                        base_X.y,
                        str(out.floor_line_),
                        base_X.x,
                        C.x,
                        base_X.y,
                        C.y,
                        str(out.floor_ray_)));
         FATAL("kBAM!");
      }
   }

   return out;
}

// ----------------------------------------------------------------- proportions
//
template<typename List>
static float calc_proportion_(const Skeleton3D& s3d, const List& l)
{
   const auto p3  = Plane4f{s3d.up_n(), -dot(s3d.up_n(), s3d.X())};
   const auto& Xs = s3d.Xs();
   int counter    = 0;
   float d        = 0.0f;
   for(const auto& kp : l) {
      if(Xs[size_t(kp)].is_finite()) {
         d += std::fabs(p3.side(Xs[size_t(kp)]));
         counter++;
      }
   }

   return (counter == 0) ? fNAN : (d / (float(counter) * s3d.height()));
}

Vector3f This::Xs_centre() const noexcept
{
   int counter = 0;
   auto sum    = Vector3f{0.0f, 0.0f, 0.0f};
   for(const auto& Xs : Xs_) {
      if(Xs.is_finite()) {
         sum += Xs;
         counter++;
      }
   }

   return (counter == 0) ? Vector3f::nan() : (sum / float(counter));
}

float This::ankle_proportion() const noexcept
{
   std::array<KeypointName, 2> kps = {{skeleton::L_ANKLE, skeleton::R_ANKLE}};
   return calc_proportion_(*this, kps);
}

float This::knee_proportion() const noexcept
{
   std::array<KeypointName, 2> kps = {{skeleton::L_KNEE, skeleton::R_KNEE}};
   return calc_proportion_(*this, kps);
}

float This::pelvis_proportion() const noexcept
{
   std::array<KeypointName, 3> kps
       = {{skeleton::PELVIS, skeleton::L_HIP, skeleton::R_HIP}};
   return calc_proportion_(*this, kps);
}

float This::notch_proportion() const noexcept
{
   std::array<KeypointName, 3> kps
       = {{skeleton::NOTCH, skeleton::L_SHOULDER, skeleton::R_SHOULDER}};
   return calc_proportion_(*this, kps);
}

float This::nose_proportion() const noexcept
{
   std::array<KeypointName, 1> kps = {{skeleton::NOSE}};
   return calc_proportion_(*this, kps);
}

// ---------------------------------------------------------------- memory-usage
//
size_t This::memory_usage() const noexcept
{
   return sizeof(This) + (Xs_.capacity() * sizeof(Vector3f));
}

// ------------------------------------------------------------ plot-skeleton-3d
//
void plot_skeleton_3d(const DistortedCamera& dcam,
                      const Skeleton3D& s3d,
                      const float in_height,
                      const AABB& im_bounds,
                      std::function<void(int, int, float)> f)
{
   if(!s3d.is_valid()) return;

   constexpr unsigned n_divisions = 21;
   // std::array<float, n_divisions + 1> ts;
   // for(auto i = 0u; i < ts.size(); ++i)
   //    ts[i] = 2.0f * float(M_PI) * float(i) / float(n_divisions);
   std::array<Vector2, n_divisions + 1> xs;

   const float height = std::isfinite(in_height) ? in_height : s3d.height();
   Expects(std::isfinite(height));
   const float radius   = s3d.radius(in_height);
   const Vector3f X     = s3d.X(in_height);
   const Vector3f& up_n = s3d.up_n();
   const Vector3f& fd_n = s3d.forward_n();
   const auto Xs        = s3d.calc_Xs(in_height);

   auto render_circle
       = [&](const float z,
             const bool top_bottom) -> std::pair<Vector2f, Vector2f> {
      if(!std::isfinite(z)) return {};
      Expects(X.is_finite());
      Expects(std::isfinite(z));
      Expects(up_n.is_finite());
      const auto Xf = to_vec3f(X + z * up_n);
      Expects(Xf.is_finite());
      const auto ret
          = project_circle(dcam, Xf, radius, up_n, !top_bottom, im_bounds, f);
      return ret;
   };

   const auto [A, B] = render_circle(0.0f, true); // the base
   render_circle(height * s3d.ankle_proportion(), false);
   render_circle(height * s3d.knee_proportion(), false);
   render_circle(height * s3d.pelvis_proportion(), false);
   render_circle(height * s3d.notch_proportion(), false);
   const auto [C, D] = render_circle(height, true);

   plot_line_AA(to_vec2(A), to_vec2(C), im_bounds, f);
   plot_line_AA(to_vec2(B), to_vec2(D), im_bounds, f);

   { // If we a direction, then
      auto project = [&](const Vector3f& X) -> Vector2 {
         return project_to_distorted(dcam, to_vec3(X));
      };

      const auto P = X + up_n * height * s3d.pelvis_proportion();
      const auto Q = P + radius * fd_n;

      if(P.is_finite() && Q.is_finite())
         plot_line_AA(project(P), project(Q), im_bounds, f);
   }
}

// ----------------------------------------------------------------- shape-score
//
float shape_score(const Skeleton3D& s3d, const bool feedback) noexcept
{
   constexpr float k_min_score = 0.01f;

   const float cos_theta = dot(s3d.up_n(), Vector3f{0.0, 0.0, 1.0});
   const auto theta
       = std::acos(std::clamp<float>(std::fabs(cos_theta), 0.0f, 1.0f));
   const auto spine_score = square(1.0f - (2.0f * theta / float(M_PI)));

   return std::max(k_min_score, spine_score);

   if(false) {
      const auto p3         = Plane4f{s3d.up_n(), -dot(s3d.up_n(), s3d.X())};
      const auto height_inv = 1.0f / s3d.height();

      float sum_diffs      = 0.0f;
      float sum_weight     = 0.0f;
      const Vector3f nan3f = Vector3f::nan();

      for(const auto& prop : k_p2d_proportions_) {
         const bool has_xs    = size_t(prop.kp) < s3d.Xs().size();
         const bool xs_finite = has_xs && s3d.Xs()[size_t(prop.kp)].is_finite();
         const Vector3f& X    = xs_finite ? s3d.Xs()[size_t(prop.kp)] : nan3f;
         const float side     = p3.side(X);
         const float ratio    = side * height_inv;
         const float score    = std::fabs(ratio - float(prop.height_ratio))
                             / float(prop.height_ratio);
         if(xs_finite) {
            sum_diffs += float(prop.weight) * ratio;
            sum_weight += float(prop.weight);
         }
      }

      return (sum_weight == 0.0f) ? 0.0f : (sum_diffs / sum_weight);
   }
}

} // namespace perceive
