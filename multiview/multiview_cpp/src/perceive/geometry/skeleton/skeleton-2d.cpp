
#include "stdinc.hpp"

#include "2d-helpers.hpp"
#include "fit-torso.hpp"
#include "skeleton-2d.hpp"

#include "perceive/geometry/human-heights.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/utils/base64.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"

#define This Skeleton2D

namespace perceive
{
using namespace skeleton;

// ------------------------------------------------------------------ pose model
//
This::PoseModel This::int_to_pose_model(int val) noexcept
{
   const auto r = PoseModel(val);
   switch(r) {
#define E(x) \
   case This::x: return This::x;
      E(BODY_25);
      E(COCO_18);
      E(MPI_15);
      E(MPI_15_4);
   default: return This::BODY_25;
#undef E
   }
   return r;
}

const char* str(const This::PoseModel x) noexcept
{
   switch(x) {
#define E(x) \
   case This::x: return #x;
      E(BODY_25);
      E(COCO_18);
      E(MPI_15);
      E(MPI_15_4);
#undef E
   }
}

This::PoseModel to_pose_model(const string_view val) noexcept(false)
{
#define E(x) \
   if(val == #x) return This::x;
   E(BODY_25);
   E(COCO_18);
   E(MPI_15);
   E(MPI_15_4);
#undef E
   throw std::runtime_error(
       format("could not convert string'{}' to a PoseModel", val));
}

// -------------------------------------------------------------- This::Keypoint

bool This::Keypoint::operator==(const Keypoint& o) const noexcept
{
#define TEST(x) (this->x == o.x)
#define TEST_REAL(x) (float_is_same(this->x, o.x, 1e-4f))
   return TEST(part) and TEST_REAL(pos.x) and TEST_REAL(pos.y)
          and TEST_REAL(score);
#undef TEST
#undef TEST_REAL
}

bool This::Keypoint::operator!=(const Keypoint& o) const noexcept
{
   return !(*this == o);
}

// ------------------------------------------------------------------------ init

static AABB get_cylinder_aabb(const Skeleton2D& o)
{
   AABB t_aabb = o.torso_aabb();
   if(!t_aabb.is_finite()) { t_aabb = o.aabb(); }
   return t_aabb;
}

static void
init_rays(const DistortedCamera* dcam_ptr,
          const std::array<Skeleton2D::Keypoint, k_n_keypoints>& keypoints,
          std::array<Vector3f, k_n_keypoints>& rays)
{
   if(dcam_ptr == nullptr) {
      std::fill(begin(rays), end(rays), Vector3f::nan());
   } else {
      for(auto i = 0u; i < rays.size(); ++i) {
         rays[i]
             = to_vec3f(backproject_ray(*dcam_ptr, to_vec2(keypoints[i].xy())));
      }
   }
}

static Vector3f calc_centre_ray(const std::array<Vector3f, k_n_keypoints>& rays)
{
   int counter = 0;
   auto C      = Vector3f{0.0f, 0.0f, 0.0f};
   for(const auto& ray : rays) {
      if(!ray.is_finite()) continue;
      ++counter;
      C += ray;
   }
   return (counter == 0) ? Vector3f::nan() : C.normalised();
}

static std::array<Vector2f, 2> calc_eigen_vecs(
    const std::array<Skeleton2D::Keypoint, k_n_keypoints>& keypoints)
{
   int counter = 0;
   Vector2f C  = Vector2f(0, 0); // to center the data
   for(const auto& kp : keypoints) {
      if(kp.is_valid()) {
         ++counter;
         C += kp.xy();
      }
   }

   if(counter < 4) { return {Vector2f::nan(), Vector2f::nan()}; }

   C /= float(counter);

   MatrixXr A(counter, 2);
   int pos = 0;
   for(const auto& kp : keypoints) {
      if(kp.is_valid()) {
         const auto x = kp.xy() - C;
         A(pos, 0)    = real(x.x);
         A(pos, 1)    = real(x.y);
         Expects(std::isfinite(x.x));
         Expects(std::isfinite(x.y));
         ++pos;
      }
   }

   // Calculate AtA, and do the SVD
   MatrixXr At  = A.transpose();
   MatrixXr AtA = At * A;
   MatrixXr U, V;
   VectorXr D;

   svd_UDV(AtA, U, D, V);
   const auto major_axis = to_vec2f(Vector2(V(0, 0), V(0, 1)));
   const auto minor_axis = to_vec2f(Vector2(V(1, 0), V(1, 1)));

   return {major_axis, minor_axis};
}

static ProjectiveFloorCylinder
calc_proj_cylinder(const DistortedCamera* dcam_ptr,
                   const Vector3f& centre_ray,
                   const std::array<Vector3f, k_n_keypoints>& rays,
                   const AABB& t_aabb)
{
   if(dcam_ptr == nullptr) return {};

   Expects(t_aabb.is_finite());

   const auto& dcam = *dcam_ptr;
   const auto& C    = dcam.C; // All three planes share the camera center

   const auto t = Vector2(0.5 * (t_aabb.left + t_aabb.right), t_aabb.top);
   const auto l = Vector2(t_aabb.left, t_aabb.top);
   const auto r = Vector2(t_aabb.right, t_aabb.top);

   const auto t_ray = backproject_ray(dcam, t);
   const auto l_ray = backproject_ray(dcam, l);
   const auto r_ray = backproject_ray(dcam, r);

   const auto Z0      = Vector3{0.0, 0.0, 1.0};
   auto calc_t_normal = [&]() {
      const auto Y = Z0.cross(t_ray).normalised();
      const auto N = Y.cross(t_ray).normalised();
      if(!(std::fabs(t_ray.dot(N)) < 1e-3)) {
         WARN(format("AABB = {}, t = {}, t-ray = |{}^T {}| = {}",
                     str(t_aabb),
                     str(t),
                     str(t_ray),
                     str(N),
                     std::fabs(t_ray.dot(N))));
      }
      return N;
   };
   const auto t_norm = calc_t_normal();
   const auto l_norm = Z0.cross(l_ray).normalised();
   const auto r_norm = Z0.cross(r_ray).normalised();

   const auto p3l = Plane(l_norm, -l_norm.dot(C));
   const auto p3r = Plane(r_norm, -r_norm.dot(C));
   const auto p3t = Plane(t_norm, -t_norm.dot(C));

   Expects(C.is_finite());
   Expects(p3l.is_finite());
   Expects(p3r.is_finite());
   Expects(p3t.is_finite());

   const auto C_norm = Z0.cross(to_vec3(centre_ray)).normalised();
   const auto C_p3   = Plane(C_norm, -C_norm.dot(C));

   ProjectiveFloorCylinder out = {};
   if(C_p3.is_finite() && p3t.is_finite()) out.init(p3t, C_p3, C);
   return out;
}

void This::init(const Params& params,
                const int frame_no,
                const int sensor_no,
                const DistortedCamera* dcam_ptr,
                const PoseModel model,
                const vector<Keypoint>& keypoints)
{
   Expects(keypoints.size() > 0);

   model_      = model;
   frame_no_   = frame_no;
   sensor_no_  = sensor_no;
   cam_centre_ = to_vec3f(dcam_ptr->C);
   score_      = 0.0;

   { // copy in the keypoints
      std::fill(begin(keypoints_), end(keypoints_), Keypoint{});
      for(const auto& kp : keypoints) {
         Expects(size_t(kp.part) < keypoints_.size());
         keypoints_[size_t(kp.part)] = kp; // DONE!
      }
   }

   theta_ = (dcam_ptr == nullptr) ? dNAN
                                  : estimate_pose_direction(*dcam_ptr, *this);

   { // set the score
      for(const auto& kp : keypoints_) { score_ += real(kp.score); }
      score_ /= real(keypoints_.size());

      if(false && multiview_trace_mode()) {
         static std::mutex padlock_;
         lock_guard lock(padlock_);
         TRACE("keypoints and score:");
         for(const auto& kp : keypoints_) {
            cout << format("[{}, part = {}, xy = {}, score = {}",
                           kp.part,
                           str(kp.part),
                           str(kp.pos),
                           kp.score)
                 << endl;
         }
         cout << endl;
      }
   }

   eigen_vecs_ = calc_eigen_vecs(keypoints_);
   init_rays(dcam_ptr, keypoints_, rays_);
   centre_ray_ = calc_centre_ray(rays_);
   cylinder_   = calc_proj_cylinder(
       dcam_ptr, centre_ray_, rays_, get_cylinder_aabb(*this));

   if(dcam_ptr == nullptr) {
      best_3d_result_ = {};
   } else {
      best_3d_result_ = fit_3d_ret(dcam_ptr, *this, false);
   }
}

vector<LABImage> This::make_image_patches(const int patch_w,
                                          const int patch_h,
                                          const LABImage& im) const
    noexcept(false)
{
   Expects(patch_w > 1);
   Expects(patch_h > 0);

   const auto& skps = get_p2d_bones();

   vector<LABImage> o;
   o.reserve(skps.size());

   for(const auto& tpl : skps) {
      const auto kpnA = tpl.kp0;
      const auto kpnB = tpl.kp1;
      const auto k    = tpl.kolour;
      if(k == 0xFF000000u) continue;

      auto kpa = keypoint_position(kpnA);
      auto kpb = keypoint_position(kpnB);
      o.emplace_back(
          make_patch(unsigned(patch_w), unsigned(patch_h), kpa, kpb, im));
   }

   o.shrink_to_fit();
   return o;
}

// -------------------------------------------------------------------- equality
//
bool This::test_eq(const Skeleton2D& o, string& s, bool report) const noexcept
{
   const real rel = 1e-4;
#define TEST(x) (this->x == o.x)
#define TEST_REAL(x)                  \
   ((std::fabs(this->x - o.x) <= rel) \
    || (!std::isfinite(this->x) && !std::isfinite(o.x)))

   if(report) {
      std::stringstream ss{""};
      ss << "Pose::operator== report" << endl;
      ss << format("   sensor-no        = {}/{}", sensor_no_, o.sensor_no_)
         << endl;
      ss << format("   model-is         = {}", str(TEST(model_))) << endl;
      ss << format("   keypoints-is     = {} ({}/{})",
                   str(TEST(keypoints_)),
                   keypoints_.size(),
                   o.keypoints_.size())
         << endl;
      ss << format("   theta-is         = {}, {}",
                   str(TEST_REAL(theta_)),
                   is_close_report(theta_, o.theta_, rel))
         << endl;
      ss << format("   score-is         = {}, {}",
                   str(TEST_REAL(score_)),
                   is_close_report(score_, o.score_, rel))
         << endl;
      // ss << format("   image-patches-is = {}", str(TEST(image_patches_)))
      //    << endl;
      if(!TEST(keypoints_)) {
         const int N = int(std::min(keypoints_.size(), o.keypoints_.size()));
         for(auto i = 0; i < N; ++i) {
            const auto& A = keypoints_[size_t(i)];
            const auto& B = o.keypoints_[size_t(i)];
            ss << format("   + [{}, {}, {}, {}] == [{}, {}, {}, "
                         "{}] => {}\n",
                         A.part,
                         A.xy().x,
                         A.xy().y,
                         A.score,
                         B.part,
                         B.xy().x,
                         B.xy().y,
                         B.score,
                         str(A == B));
         }
      }
      s = ss.str();
   }

   return TEST(model_) and TEST(sensor_no_) and TEST(keypoints_)
          and TEST_REAL(theta_) and TEST_REAL(score_);
#undef TEST
#undef TEST_REAL
}

bool This::operator==(const Skeleton2D& o) const noexcept
{
   string s;
   return test_eq(o, s, false);
}

bool This::operator!=(const Skeleton2D& o) const noexcept
{
   return !(*this == o);
}

// ---------------------------------------------------------------- memory-usage
//
size_t This::memory_usage() const noexcept
{
   return sizeof(Skeleton2D)
          + (best_3d_result().memory_usage() - sizeof(Skeleton3D));
}

// --------------------------------------------------------------------- to-json
//
Json::Value This::to_json() const noexcept
{
   auto make_keypoints_json = [&]() {
      auto a = Json::Value{Json::arrayValue};
      a.resize(unsigned(keypoints_.size()));
      for(auto i = 0u; i < keypoints_.size(); ++i) {
         const auto& k = keypoints_[i];
         auto v        = Json::Value{Json::arrayValue};
         v.resize(4);
         v[0] = real(k.part);
         v[1] = real(k.xy().x);
         v[2] = real(k.xy().y);
         v[3] = real(k.score);
         a[i] = v;
      }
      return a;
   };

   auto make_rays_json = [&]() {
      auto a = Json::Value{Json::arrayValue};
      a.resize(unsigned(rays_.size()));
      for(auto i = 0u; i < rays_.size(); ++i) {
         const auto& k = rays_[i];
         auto v        = Json::Value{Json::arrayValue};
         v.resize(3);
         v[0] = json_save(k.x);
         v[1] = json_save(k.y);
         v[2] = json_save(k.z);
         a[i] = v;
      }
      return a;
   };

   auto save_proj_floor_cylinder = [&](const ProjectiveFloorCylinder& Cy) {
      auto o    = Json::Value{Json::objectValue};
      o["top"]  = json_save(Cy.top());
      o["C-p3"] = json_save(Cy.C_p3());
      o["C"]    = json_save(Cy.C());
      return o;
   };

   const string model_s = str(model_);
   Expects(model_ == to_pose_model(model_s));

   auto o                = Json::Value{Json::objectValue};
   o["model"]            = model_s;
   o["frame-no"]         = frame_no_;
   o["sensor-no"]        = sensor_no_;
   o["keypoints"]        = make_keypoints_json();
   o["cam-centre"]       = json_save(cam_centre_);
   o["rays"]             = make_rays_json();
   o["centre-ray"]       = json_save(centre_ray_);
   o["theta"]            = json_save(theta_);
   o["score"]            = json_save(score_);
   o["eigen-vec0"]       = json_save(eigen_vecs_[0]);
   o["eigen-vec1"]       = json_save(eigen_vecs_[1]);
   o["cylinder"]         = save_proj_floor_cylinder(cylinder_);
   o["is-interpolation"] = json_save(is_interpolation_);

   // `best_3d_result_` is derived

   return o;
}

// ------------------------------------------------------------------------ read
//
void This::read(const Json::Value& o) noexcept(false)
{
   auto x = Skeleton2D{};

   auto load_model = [&](const Json::Value& o) {
      Skeleton2D::PoseModel ret;
      string model_s = ""s;
      json_load(o, model_s);
      try {
         ret = to_pose_model(model_s);
      } catch(std::exception& e) {
         FATAL(format("{}", e.what()));
      }
      return ret;
   };

   auto load_keypoints = [](const Json::Value& a) {
      std::array<Keypoint, k_n_keypoints> keypoints;

      if(a.type() != Json::arrayValue)
         throw std::runtime_error("expected array value for keypoints");
      if(a.size() != keypoints.size())
         throw std::runtime_error(format("expected {} keypoints, but found {}",
                                         keypoints.size(),
                                         a.size()));
      for(auto i = 0u; i < keypoints.size(); ++i) {
         auto& k = keypoints[i];
         if(a[i].type() != Json::arrayValue)
            throw std::runtime_error("keypoints must be arrays too!");
         if(a[i].size() != 4)
            throw std::runtime_error("keypoints must be arrays of size 4");
         json_load(a[i][0], k.part);
         json_load(a[i][1], k.pos.x);
         json_load(a[i][2], k.pos.y);
         json_load(a[i][3], k.score);
         if(unsigned(k.part) != i)
            throw std::runtime_error("expected keypoints in order!");
      }

      return keypoints;
   };

   auto load_rays = [](const Json::Value& a) {
      std::array<Vector3f, k_n_keypoints> rays;

      if(a.type() != Json::arrayValue)
         throw std::runtime_error("expected array value for rays");
      if(a.size() != rays.size())
         throw std::runtime_error(format(
             "expected {} keypoints, but found {}", rays.size(), a.size()));
      for(auto i = 0u; i < rays.size(); ++i) {
         auto& k = rays[i];
         if(a[i].type() != Json::arrayValue)
            throw std::runtime_error("rays must be arrays too!");
         if(a[i].size() != 3)
            throw std::runtime_error("rays must be arrays of size 3");
         json_load(a[i][0], k[0]);
         json_load(a[i][1], k[1]);
         json_load(a[i][2], k[2]);
      }

      return rays;
   };

   auto load_proj_floor_cylinder
       = [](const Json::Value& o) -> ProjectiveFloorCylinder {
      Plane top, C_p3;
      Vector3 C;
      json_load(o["top"], top);
      json_load(o["C-p3"], C_p3);
      json_load(o["C"], C);
      ProjectiveFloorCylinder Cy;
      Cy.init(top, C_p3, C);
      return Cy;
   };

   x.model_ = load_model(o["model"]);
   json_load(o["frame-no"], x.frame_no_);
   json_load(o["sensor-no"], x.sensor_no_);
   x.keypoints_ = load_keypoints(o["keypoints"]);
   json_load(o["cam-centre"], x.cam_centre_);
   x.rays_ = load_rays(o["rays"]);
   json_load(o["centre-ray"], x.centre_ray_);
   json_load(o["theta"], x.theta_);
   json_load(o["score"], x.score_);
   json_load(o["eigen-vec0"], x.eigen_vecs_[0]);
   json_load(o["eigen-vec1"], x.eigen_vecs_[1]);
   x.cylinder_       = load_proj_floor_cylinder(o["cylinder"]);
   x.best_3d_result_ = fit_3d_ret(nullptr, x, false);
   json_load(o["is-interpolation"], x.is_interpolation_);

   *this = x;
}

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   const string model_s = str(model_);
   Expects(model_ == to_pose_model(model_s));

   return format(
       R"V0G0N(
{{
   "model":      {},
   "sensor-no":  {},
   "theta":      {},
   "score":      {},
   "rays":      [{}],
   "keypoints": [{}]
}})V0G0N",
       json_encode(model_s),
       sensor_no_,
       theta_,
       score_,
       implode(cbegin(rays_),
               cend(rays_),
               ",\n        ",
               [&](const auto& ray) {
                  return format("[{}, {}, {}]", ray.x, ray.y, ray.z);
               }),
       implode(cbegin(keypoints_),
               cend(keypoints_),
               ",\n        ",
               [&](const auto& k) {
                  return format(
                      "[{}, {}, {}, {}]", k.part, k.xy().x, k.xy().y, k.score);
               }));
}

// ----------------------------------------------------------------- to-json-str
//
string This::to_json_str() const noexcept
{
   std::stringstream ss{""};
   ss << this->to_json();
   return ss.str();
}

// ---------------------------------------------------------------------- ret-kp
// 'ind' is the BODY_25 index.
static int get_kp_index(const Skeleton2D::PoseModel model,
                        const int ind0) noexcept
{
   static constexpr array<int8_t, 25> k_body_25
       = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12,
          13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
   static constexpr array<int8_t, 25> k_coco
       = {0,  1,  2,  3,  4,  5,  6,  7,  -1, 8,  9,  10, 11,
          12, 13, 14, 15, 16, 17, 13, 13, 13, 10, 10, 10};

   Expects(ind0 >= 0 and ind0 < int(k_body_25.size()));
   Expects(ind0 >= 0 and ind0 < int(k_coco.size()));

   if(model == Skeleton2D::BODY_25) {
      return k_body_25[size_t(ind0)];
   } else if(model == Skeleton2D::COCO_18) {
      if(ind0 == 8) { // Special case... average '8' and '11'
         return 8;
      } else {
         return k_coco[size_t(ind0)];
      }
   } else {
      FATAL(format("model not supported. A mapping must be entered "
                   "by hand at this point of code. @see "
                   "https://github.com/CMU-Perceptual-Computing-Lab/openpose/"
                   "blob/master/doc/output.md for hints"));
   }

   return -1;
}

static Vector2 ret_kp(const Skeleton2D& pose, const int ind0) noexcept
{
   auto get_kp = [&](int ind) {
      if(unsigned(ind) > pose.keypoints().size()) {
         LOG_ERR(
             format("WHAT? ind = {}, sz  = {}", ind, pose.keypoints().size()));
      }
      Expects(ind >= 0 and ind < int(pose.keypoints().size()));
      const auto x = to_vec2(pose.keypoints()[size_t(ind)].xy());
      return x.quadrance() > 0.0 ? x : Vector2::nan();
   };

   if(pose.model() == Skeleton2D::BODY_25) {
      return get_kp(get_kp_index(pose.model(), ind0));
   } else if(pose.model() == Skeleton2D::COCO_18) {
      if(ind0 == 8) { // Special case... average '8' and '11'
         return 0.5 * (get_kp(8) + get_kp(11));
      } else {
         return get_kp(get_kp_index(pose.model(), ind0));
      }
   } else {
      FATAL(format("model not supported. A mapping must be entered "
                   "by hand at this point of code. @see "
                   "https://github.com/CMU-Perceptual-Computing-Lab/openpose/"
                   "blob/master/doc/output.md for hints"));
   }

   return Vector2::nan();
}

static const Vector3f& ret_ray(const Skeleton2D& pose, const int ind0) noexcept
{
   auto get_kp_ray = [&](int ind) -> const Vector3f& {
      Expects(unsigned(ind) < pose.rays().size());
      return pose.rays()[size_t(ind)];
   };

   if(pose.model() == Skeleton2D::BODY_25) {
      return get_kp_ray(get_kp_index(pose.model(), ind0));
   } else if(pose.model() == Skeleton2D::COCO_18) {
      return get_kp_ray(get_kp_index(pose.model(), ind0));
   } else {
      FATAL(format("model not supported. A mapping must be entered "
                   "by hand at this point of code. @see "
                   "https://github.com/CMU-Perceptual-Computing-Lab/openpose/"
                   "blob/master/doc/output.md for hints"));
   }

   static Vector3f Z = Vector3f::nan();
   return Z;
}

// ------------------------------------------------------------------------ aabb
//
AABB This::aabb() const noexcept
{
   AABB aabb = AABB::minmax();
   for(auto i = 0; i < 25; ++i) {
      const auto X = ret_kp(*this, i);
      if(X.is_finite()) aabb.union_point(X);
   }
   return aabb;
}

AABB This::head_aabb() const noexcept
{
   AABB aabb = AABB::minmax();
   aabb.union_point(to_vec2(nose()));
   aabb.union_point(to_vec2(neck()));
   aabb.union_point(to_vec2(l_eye()));
   aabb.union_point(to_vec2(r_eye()));
   aabb.union_point(to_vec2(l_ear()));
   aabb.union_point(to_vec2(r_ear()));
   return aabb;
}

AABB This::torso_aabb() const noexcept
{
   // TRACE("HERE");
   AABB aabb = AABB::minmax();
   aabb.union_point(to_vec2(neck()));
   aabb.union_point(to_vec2(l_shoulder()));
   aabb.union_point(to_vec2(r_shoulder()));
   aabb.union_point(to_vec2(pelvis()));
   aabb.union_point(to_vec2(l_hip()));
   aabb.union_point(to_vec2(r_hip()));
   // TRACE("HERE");
   if(aabb.top > aabb.bottom || aabb.left > aabb.right) return AABB::nan();
   //   TRACE("HERE");
   return aabb;
}

// ---------------------------------------------------------------- set keypoint
//
void This::set_keypoint(KeypointName kp_name,
                        const Vector2f& xy,
                        float score) noexcept
{
   Expects(size_t(kp_name) < keypoints_.size());
   auto& kp = keypoints_[size_t(kp_name)];
   kp.part  = int32_t(kp_name);
   kp.pos   = xy;
   kp.score = score;
}

// ------------------------------------------------------------------------ head
//
Vector2f This::head() const noexcept
{
   // nose, eyes, ears in that order
   Vector2f av = {};
   int counter = 0;
   if(nose().is_finite()) {
      av += nose();
      ++counter;
   }
   if(l_eye().is_finite()) {
      av += l_eye();
      ++counter;
   }
   if(r_eye().is_finite()) {
      av += r_eye();
      ++counter;
   }
   if(l_ear().is_finite()) {
      av += l_ear();
      ++counter;
   }
   if(r_ear().is_finite()) {
      av += r_ear();
      ++counter;
   }

   return (counter == 0) ? Vector2f::nan() : av / float(counter);
}

// ------------------------------------------------------------------------ feet
//
Vector2f This::feet() const noexcept
{
   if(model() == BODY_25) {
      auto average_kps = [&](array<KeypointName, 4> kps) {
         int counter = 0;
         Vector2f av{0.0f, 0.0f};
         for(const auto ind : kps) {
            if(keypoints_[size_t(ind)].pos.is_finite()) {
               av += keypoints_[size_t(ind)].pos;
               ++counter;
            }
         }
         return (counter == 0) ? Vector2f::nan() : (av / float(counter));
      };

      Vector2f l_foot = average_kps({KeypointName::L_ANKLE,
                                     KeypointName::L_BIG_TOE,
                                     KeypointName::L_SMALL_TOE,
                                     KeypointName::L_HEAL});
      Vector2f r_foot = average_kps({KeypointName::R_ANKLE,
                                     KeypointName::R_BIG_TOE,
                                     KeypointName::R_SMALL_TOE,
                                     KeypointName::R_HEAL});
      return 0.5 * (l_foot + r_foot);
   } else if(model() == COCO_18) {
      return 0.5f * (l_ankle() + r_ankle());
   } else {
      FATAL("not implemented");
   }
   return Vector2f::nan();
}

Vector2 This::keypoint_position(KeypointName kp) const noexcept
{
   return ret_kp(*this, int(kp));
}

const Vector3f& This::keypoint_ray(KeypointName kp) const noexcept
{
   return ret_ray(*this, int(kp));
}

// -------------------------------------------------------------------- head roi
//
AABBi This::head_roi() const noexcept
{
   auto method_1 = [&]() {
      AABBi aabb = AABBi::minmax();
#define TRY_ADD(prt)                                          \
   {                                                          \
      const auto X = (prt);                                   \
      if(X.is_finite()) aabb.union_point(int(X.x), int(X.y)); \
   }

      TRY_ADD(nose());
      TRY_ADD(l_eye());
      TRY_ADD(r_eye());
      TRY_ADD(l_ear());
      TRY_ADD(r_ear());
      TRY_ADD(neck());
#undef TRY_ADD

      if(aabb.left > aabb.right) aabb.left = aabb.right;
      if(aabb.top > aabb.bottom) aabb.top = aabb.bottom;

      if(aabb.area() == 0) return aabb;

      { // Get the top of the head
         const auto C = aabb.centre();
         aabb.union_point(C.x, C.y - aabb.height());
      }

      aabb.grow(10); // a margin

      return aabb;
   };

   auto method_2 = [&]() {
      AABBi aabb{0, 0, 0, 0};
      const auto o = head();
      if(!o.is_finite()) return aabb;

      auto calc_w = [&]() -> float {
#define TEST_IT(x, y)                            \
   {                                             \
      const auto len = (x - y).norm();           \
      if(std::isfinite(len)) return 1.33f * len; \
   }
         TEST_IT(neck(), l_shoulder());
         TEST_IT(neck(), r_shoulder());
         TEST_IT(l_shoulder(), l_elbow());
         TEST_IT(r_shoulder(), r_elbow());
         TEST_IT(l_shoulder(), r_shoulder());
#undef TEST_IT
         return dNAN;
      };

      const float w = calc_w();
      if(!std::isfinite(w)) return aabb;

      const int x = int(o.x - 0.5f * w);
      const int y = int(o.y - 0.5f * 1.8f * w);

      aabb = AABBi{x, y, x + int(w), int(o.y + 0.5f * w)};
      return aabb;
   };

   const auto aabb = method_1();
   if(aabb.area() > 0) return aabb;
   return method_2();
}

// ------------------------------------------------------------------------- str

std::string str(const Skeleton2D& pose) noexcept
{
   std::stringstream ss{""};
   ss << format("Person, score = {}\n", pose.score());
   for(const auto& k : pose.keypoints())
      ss << format(
          "   {:2d}: {{}, {}} = {}\n", k.part, k.xy().x, k.xy().y, k.score);
   return ss.str();
}

std::string str(const vector<Skeleton2D>& poses) noexcept
{
   std::stringstream ss{""};

   for(auto i = 0u; i < poses.size(); ++i) {
      const auto& pose = poses[i];
      ss << format("Person #{}, score = {}\n", i, pose.score());
      for(const auto& k : pose.keypoints())
         ss << format(
             "   {:2d}: {{}, {}} = {}\n", k.part, k.xy().x, k.xy().y, k.score);
   }

   return ss.str();
}

Skeleton2D Skeleton2D::interpolate(const Params& params,
                                   const Skeleton2D& in_A,
                                   const Skeleton2D& in_B,
                                   const int frame_no,
                                   const DistortedCamera* dcam_ptr)
{
   const Skeleton2D& A = (in_A.frame_no() < in_B.frame_no()) ? in_A : in_B;
   const Skeleton2D& B = (in_A.frame_no() < in_B.frame_no()) ? in_B : in_A;

   Expects(A.sensor_no_ == B.sensor_no_);
   Expects(A.model_ == B.model_);
   Expects(A.frame_no() < frame_no);
   Expects(frame_no < B.frame_no());
   Expects(dcam_ptr != nullptr);

   auto interp_keypoints = [&](auto& keypoints) { // keypoints
      const float delta
          = float(frame_no - A.frame_no()) / float(B.frame_no() - A.frame_no());
      Expects(keypoints.size() == A.keypoints_.size());
      for(auto i = 0u; i < A.keypoints_.size(); ++i) {
         const auto& kA = A.keypoints_[i];
         const auto& kB = B.keypoints_[i];
         auto& kC       = keypoints[i];

         if(kA.is_valid() && kB.is_valid()) {
            kC       = kA;
            kC.pos   = kA.pos + delta * (kB.pos - kA.pos);
            kC.score = (kA.score + (kB.score - kA.score) * delta);
         }
      }
   };

   Skeleton2D C = A;

   // TRACE("HERE");
   C.model_     = A.model_;
   C.frame_no_  = frame_no;
   C.sensor_no_ = A.sensor_no_;
   interp_keypoints(C.keypoints_);
   C.cam_centre_ = A.cam_centre_;
   init_rays(dcam_ptr, C.keypoints_, C.rays_);
   C.centre_ray_ = calc_centre_ray(C.rays_);
   C.theta_
       = (dcam_ptr == nullptr) ? dNAN : estimate_pose_direction(*dcam_ptr, C);
   C.score_      = 0.0;
   C.eigen_vecs_ = calc_eigen_vecs(C.keypoints_);
   C.cylinder_   = calc_proj_cylinder(
       dcam_ptr, C.centre_ray_, C.rays_, get_cylinder_aabb(C));
   C.best_3d_result_   = fit_3d_ret(nullptr, C, false);
   C.is_interpolation_ = true;
   //  TRACE("HERE");

   return C;
}

// -------------------------------------------------------------------- torso-p3
//
const Plane4f& This::torso_p3() const noexcept
{
   return best_3d_result().torso_p3();
}

// ------------------------------------------------------------ realize-cylinder
//
This::CylinderResult This::realize_cylinder(const DistortedCamera& dcam,
                                            const real height,
                                            const bool feedback) const noexcept
{
   // auto Cy   = cylinder_.realize(height * human::k_height_ratio_shoulder);
   // Cy.height = height;
   // Cy.radius = 0.2;

   if(!cylinder_.is_init()) { TRACE("attempt to realize a non-init cylinder"); }

   // X0 is the closest
   // X1 is the furthest along the ray
   Vector3 X0 = cylinder_.realize(2.5 * human::k_height_ratio_shoulder).X;
   Vector3 X1 = cylinder_.realize(0.5 * human::k_height_ratio_shoulder).X;

   // Find 'X' for given height that minimizes deviation
   CylinderResult best_ret;
   real best_score = std::numeric_limits<double>::max();

   auto cost_fun = [&](double r) -> double {
      Expects(r >= 0 && r <= 1.0);
      const auto X     = X0 + r * (X1 - X0);
      auto cy_ret      = this->realize_cylinder(dcam, X);
      const auto score = std::fabs(cy_ret.Cy.height - height);
      if(score < best_score) {
         best_score = score;
         best_ret   = std::move(cy_ret);
      }
      return score;
   };

   const auto min_x = golden_section_search(cost_fun, 0.0, 1.0, 1e-3);

   best_ret.p3        = Plane4f(torso_p3().xyz(),
                         -dot(torso_p3().xyz(), to_vec3f(best_ret.Cy.X)));
   best_ret.sensor_no = this->sensor_no();

   return best_ret;
}

This::CylinderResult This::realize_cylinder(const DistortedCamera& dcam,
                                            const Vector3& X,
                                            const bool feedback) const noexcept
{
   CylinderResult ret;

   Expects(X.is_finite());
   Expects(std::fabs(X.z) < 1e-6); // `X` should be on the floor

   auto& Cy  = ret.Cy;
   Cy.X      = X; // This is the floor
   Cy.radius = human::k_shoulder_width_adult_male * 0.5;

   // We have two "camera facing" planes for each `Skeleton2DProportions`
   // The planes are at Cy.X + Cy.radius, and Cy.X - Cy.radius
   const Vector3 Z_axis       = {0.0, 0.0, 1.0};
   auto calc_billboard_normal = [&]() -> Vector3 {
      // Project the ray onto the floor
      const auto Y = Z_axis.cross(to_vec3(this->centre_ray())).normalised();
      return Z_axis.cross(Y).normalised();
   };
   const Vector3 N  = calc_billboard_normal();
   const Plane p3_0 = Plane(N, -N.dot(X + Cy.radius * N)); // near
   const Plane p3_1 = Plane(N, -N.dot(X - Cy.radius * N)); // far

   Expects(p3_0.is_finite());
   Expects(p3_1.is_finite());

   // Now iterate over each Human keypoint point, and calculate their
   // near/far 3D points
   const auto& props = get_p2d_proportions();
   ret.keypoints.resize(props.size());
   std::transform(
       cbegin(props),
       cend(props),
       begin(ret.keypoints),
       [&](const Skeleton2DProportions& prop) -> std::pair<Vector3, Vector3> {
          const auto x = to_vec2(prop.get(*this));
          return !x.is_finite() ? std::pair<Vector3, Vector3>{Vector3::nan(),
                                                              Vector3::nan()}
                                : std::pair<Vector3, Vector3>{
                                    plane_ray_intersect(dcam, p3_0, x),
                                    plane_ray_intersect(dcam, p3_1, x)};
       });

   if(feedback) { TRACE("FEEDBACK"); }

   { // Calculate height, based on the weighted sum
      auto get_i = [&](int i, bool use_feedback) {
         const auto& prop = props[size_t(i)];
         const auto& X0   = ret.keypoints[size_t(i)].first;
         const auto& X1   = ret.keypoints[size_t(i)].second;
         real hgt         = dNAN;
         if(X0.is_finite() && X1.is_finite())
            hgt = 0.5 * (X0.z + X1.z) / prop.height_ratio;
         if(use_feedback) {
            cout << format(
                "   #{:02d}     = 0.5 * ({:4.2f} + {:4.2f}) = {:4.2f}, "
                "wgt = {:3.2f}, ratio = {:3.2f}, hgt = {:4.2f} :: {} "
                "-> {}, {}",
                i,
                X0.z,
                X1.z,
                0.5 * (X0.z + X1.z),
                prop.weight,
                prop.height_ratio,
                hgt,
                str(prop.get(*this)),
                str(X0),
                str(X1))
                 << endl;
         }
         return std::pair<real, real>(hgt, prop.weight);
      };

      const int sz = int(props.size());
      real sum_hgt = 0.0;
      real sum_wgt = 0.0;
      for(auto i = 0; i < sz; ++i) {
         const auto [hgt, wgt] = get_i(i, feedback && true);
         if(std::isfinite(hgt)) {
            sum_hgt += hgt * wgt;
            sum_wgt += wgt;
         }
      }
      Cy.height = sum_hgt / sum_wgt;

      // Now the stddev
      real sum_cs = 0.0;
      for(auto i = 0; i < sz; ++i) {
         const auto [hgt, wgt] = get_i(i, false);
         if(std::isfinite(hgt)) sum_cs = wgt * square(hgt - Cy.height);
      }
      ret.height_stddev = std::sqrt(sum_cs / sum_wgt);

      if(feedback) {
         cout << format("   ------------------") << endl;
         cout << format("   sum-hgt = {}", sum_hgt) << endl;
         cout << format("   sum-wgt = {}", sum_wgt) << endl;
         cout << format("   height  = {}", Cy.height) << endl;
         cout << format("   stddev  = {}", ret.height_stddev) << endl;
         cout << format("   ------------------") << endl;
         cout << format("   X = {}", str(X)) << endl;
      }
   }

   // if(feedback) { FATAL("kBAM!"); }

   ret.p3 = Plane4f(torso_p3().xyz(), -dot(torso_p3().xyz(), to_vec3f(Cy.X)));
   ret.sensor_no = this->sensor_no();

   return ret;
}

// ------------------------------------------------------- parallel-make-patches
//
vector<vector<LABImage>>
parallel_make_patches(const int patch_w,
                      const int patch_h,
                      const vector<shared_ptr<const Skeleton2D>>& p2ds,
                      std::function<const LABImage*(int sensor_no)> get_lab)
{
   ParallelJobSet pjobs;

   vector<vector<LABImage>> ret;
   ret.resize(p2ds.size());

   auto process_make_patch = [&](size_t i) {
      const auto& p2d_ptr = p2ds[i];
      ret[i]              = p2d_ptr->make_image_patches(
          patch_w, patch_h, *get_lab(p2d_ptr->sensor_no()));
   };
   for(auto i = 0u; i < p2ds.size(); ++i)
      pjobs.schedule([i, &process_make_patch]() { process_make_patch(i); });
   pjobs.execute();

   return ret;
}

} // namespace perceive
