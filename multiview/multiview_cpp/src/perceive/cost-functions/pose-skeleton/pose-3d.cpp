
#include "pose-3d.hpp"

#include "perceive/geometry/polygon.hpp"
#include "perceive/geometry/skeleton/2d-helpers.hpp"
#include "perceive/optimization/golden-section-search.hpp"

#define This Pose3D

namespace perceive
{
// ---------------------------------------------------------------------- Params
//
const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
#define ThisParams This::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, REAL, foot_dist_radius_factor, true));
      m.push_back(MAKE_META(ThisParams, REAL, skeleton_disparity_weight, true));
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// ------------------------------------------------------ SensorPose::operator==
//
bool This::SensorPose::test_eq(const SensorPose& o,
                               string& s,
                               bool report) const noexcept
{
   if(report) {
      const bool is_pose = (pose == o.pose);

      std::stringstream ss{""};
      ss << format("sensor-no = {}/{}", sensor_no, o.sensor_no) << endl;
      ss << format("detect-no = {}/{}", detect_no, o.detect_no) << endl;
      ss << format("pose-is   = {}", str(is_pose)) << endl;
      if(!is_pose) {
         string s1;
         pose->test_eq(*o.pose, s1, true);
         ss << indent(s1, 4) << endl;
      }
      s = ss.str();
   }

#define TEST(x) (this->x == o.x)
   return TEST(sensor_no) and TEST(detect_no) and TEST(pose);
#undef TEST
}

bool This::SensorPose::operator==(const SensorPose& o) const noexcept
{
   std::string s;
   return test_eq(o, s, false);
}

bool This::SensorPose::operator!=(const SensorPose& o) const noexcept
{
   return !(*this == o);
}

// ---------------------------------------------------- SensorPose::memory-usage
//
size_t This::SensorPose::memory_usage() const noexcept
{
   return sizeof(SensorPose) + pose->memory_usage() - sizeof(Skeleton2D);
}

// -------------------------------------------------------------- SensorPose::et
//
const EuclideanTransform&
This::SensorPose::et(const SceneDescription& scene_desc) const noexcept
{
   Expects(is_valid_index(sensor_no, scene_desc.sensor_transforms));
   return scene_desc.sensor_transforms[size_t(sensor_no)];
}

// -------------------------------------------------------------- SensorPose::IO
//
Json::Value This::SensorPose::to_json() const noexcept
{
   Json::Value o{Json::objectValue};
   o["sensor_no"] = sensor_no;
   o["detect_no"] = detect_no;
   o["pose"]      = pose->to_json();
   return o;
}

void This::SensorPose::read(const Json::Value& o) noexcept(false)
{
   SensorPose x;
   if(o.type() != Json::objectValue)
      throw std::runtime_error(
          "expected 'objectValue' when reading SensorPose");
   json_load(get_key(o, "sensor_no"s), x.sensor_no);
   json_load(get_key(o, "detect_no"s), x.detect_no);
   auto ptr = make_shared<Skeleton2D>();
   ptr->read(get_key(o, "pose"s));
   x.pose = std::move(ptr);
   *this  = x;
}

string This::SensorPose::to_json_str() const noexcept
{
   return format(R"V0G0N(
{{
   "sensor_no": {},
   "detect_no": {},
   "pose": {}
}}
)V0G0N",
                 sensor_no,
                 detect_no,
                 indent(pose->to_json_str(), 6));
}

// ------------------------------------------------------------------------ This
//
This::This()
{
   std::fill(begin(Cs_), end(Cs_), Vector2f(0.0f, 0.0f));
   std::fill(begin(ius_), end(ius_), 0.0f);
}

// ---------------------------------------------------------------- memory-usage
//
size_t This::memory_usage() const noexcept
{
   Expects(poses_.size() <= poses_.capacity());
   const auto sz1 = std::accumulate(cbegin(poses_),
                                    cend(poses_),
                                    size_t(0),
                                    [](size_t sz, const SensorPose& pose) {
                                       return sz + pose.memory_usage();
                                    })
                    + (poses_.capacity() - poses_.size()) * sizeof(SensorPose);

   if(false) {
      INFO(format("breakdown"));
      cout << format(R"V0G0N(

   params_      = {}
   poses_       = {}
   i_           = {}
   gaze_theta_  = {}
   is_valid_    = {}
   Cs_          = {}
   ius_         = {}
   -----------------
   sizeof(This) = {}
   sensor-poses = {}
   -----------------
          Total = {}
)V0G0N",
                     sizeof(decltype(params_)),
                     sizeof(decltype(poses_)),
                     sizeof(decltype(i_)),
                     sizeof(decltype(gaze_theta_)),
                     sizeof(decltype(is_valid_)),
                     sizeof(decltype(Cs_)),
                     sizeof(decltype(ius_)),
                     sizeof(This),
                     sz1,
                     sizeof(This) + sz1)
           << endl;
   }

   return sizeof(This) + sz1;
}

// ------------------------------------------------------------------------ init
//
void This::init(const SceneDescription* scene_desc,
                std::function<const PointCloud*(int sensor_no)> get_point_cloud,
                const PoseSkeletonExec::Result* op_ret,
                const Params& params,
                const int sensor_num,
                const int openpose_detection_num) noexcept
{
   vector<Point2> poses(1);
   poses[0] = Point2(sensor_num, openpose_detection_num);
   *this    = combine(scene_desc, get_point_cloud, params, op_ret, poses);
}

// -------------------------------------------------------------- update-best-iu
//
void This::update_best_iu() noexcept
{
   float sum        = 0.0f;
   unsigned max_ind = 0;
   for(auto i = 0u; i < ius_.size(); ++i) {
      if(ius_[i] > ius_[size_t(max_ind)]) max_ind = i;
      sum += ius_[i];

      Expects(std::isfinite(ius_[i]));
      Expects(ius_[i] >= 0.0f);
   }

   if(std::fabs<float>(sum) > 1e-9f)
      for(auto& iu : ius_) iu /= sum;

   i_ = max_ind;
}

// ------------------------------------------------------------------ operator==
//
template<typename T> static bool T_is_same(T a, T b) noexcept
{
   return float_is_same(a, b) or std::fabs(a - b) < T(1e-4);
}

bool This::test_eq(const Pose3D& o, const bool report) const noexcept
{
   auto arr_eq = [&](const auto& A, const auto& B, auto f) -> bool {
      const auto [ii, jj] = std::mismatch(cbegin(A), cend(A), cbegin(B), f);
      const bool ret      = (ii == cend(A)) and (jj == cend(B));
      return ret;
   };

   auto fs     = T_is_same(gaze_theta_, o.gaze_theta_);
   auto ius_is = arr_eq(
       ius_, o.ius_, [&](auto& a, auto& b) { return T_is_same(a, b); });
   auto Cs_is = arr_eq(Cs_, o.Cs_, [&](auto& a, auto& b) {
      for(auto i = 0; i < int(a.size()); ++i)
         if(!T_is_same(a[i], b[i])) { return false; }
      return true;
   });

   const auto poses_is = (poses_ == o.poses_);
   const auto i_is     = (i_ == o.i_);
   const auto valid_is = (is_valid_ == o.is_valid_);
   const auto gaze_is  = T_is_same(gaze_theta_, o.gaze_theta_);

   const auto ret
       = poses_is and i_is and valid_is and gaze_is and Cs_is and ius_is;

   if(report) {
      INFO("Pose3D::operator== report");
      cout << format("   poses-is = {} ({}/{})",
                     str(poses_is),
                     poses_.size(),
                     o.poses_.size())
           << endl;
      cout << format("   poses-is = {}", str(poses_is)) << endl;
      if(!poses_is) {
         auto N = std::max(poses_.size(), o.poses_.size());
         for(auto i = 0u; i < N; ++i) {
            cout << format("       SensorPose[{}]", i) << endl;
            if(i < poses_.size() && i < o.poses_.size()) {
               string s;
               poses_[i].test_eq(o.poses_[i], s, true);
               cout << indent(s, 12) << endl;
            } else if(i < poses_.size()) {
               cout << indent("o not available", 12) << endl;
            } else {
               cout << indent("pose not available", 12) << endl;
            }
         }
      }
      cout << format("   i-is     = {}", str(i_is)) << endl;
      cout << format("   valid-is = {}", str(valid_is)) << endl;
      cout << format("   gaze-is  = {}", str(gaze_is)) << endl;
      cout << format("   Cs-is    = {}", str(Cs_is)) << endl;
      cout << format("   ius-is   = {}", str(ius_is)) << endl;

      if(!ret) FATAL("kBAM!");
   }

   return ret;
}

bool This::operator==(const Pose3D& o) const noexcept
{
   const bool ret = this->test_eq(o, false);
   if(!ret && multiview_trace_mode()) { this->test_eq(o, true); }
   return ret;
}

bool This::operator!=(const Pose3D& o) const noexcept { return !(*this == o); }

// --------------------------------------------------------------------- combine
//

Pose3D
This::combine(const SceneDescription* scene_desc,
              std::function<const PointCloud*(int sensor_no)> get_point_cloud,
              const Params& params,
              vector<SensorPose>&& in_poses) noexcept
{
   Expects(scene_desc);
   Expects(in_poses.size() > 0);

   Pose3D ret;
   ret.params_ = params;
   ret.poses_  = std::move(in_poses);
   auto& poses = ret.poses_;

   // remove_duplicates(poses);
   for(const auto& xy : poses)
      Expects(unsigned(xy.sensor_no) < unsigned(scene_desc->n_sensors()));

   auto dcam = [&](unsigned ind) -> const DistortedCamera& {
      return scene_desc->dcam(int(ind));
   };

   auto rescale_score = [](real score) -> real {
      static constexpr real min_score = 0.000;
      return (score * (1.0 - min_score)) + min_score;
   };

   vector<Vector3f> floor_pos(poses.size());
   auto update_floor_pos = [&](double height) {
      Expects(std::isfinite(height));
      for(auto i = 0u; i < poses.size(); ++i) {
         auto C = to_vec3f(estimate_pose_floor_pos(
             dcam(unsigned(poses[i].sensor_no)), *poses[i].pose, height));
         if(!C.is_finite()) {
            cout << poses[i].pose->to_string() << endl;
            Expects(false);
         }
         floor_pos[i] = C;
      }
   };

   auto average_C = [&]() -> Vector3f {
      return std::accumulate(
                 cbegin(floor_pos), cend(floor_pos), Vector3f{0.0f, 0.0f, 0.0f})
             / float(floor_pos.size());
   };

   auto calc_iu = [&](const Vector3f& C0) -> real {
      const auto min_iu_0 = std::numeric_limits<real>::max();
      auto min_iu         = min_iu_0;
      for(auto i = 0u; i < poses.size(); ++i) {
         // What is the intersection/union?
         const Vector3f& C1 = floor_pos[i];
         const auto iu = circles_intersection_on_union(real((C1 - C0).norm()),
                                                       k_pose_radius);
         if(!std::isfinite(iu)) {
            min_iu = 0.0;
            continue;
         }
         if(iu < min_iu) min_iu = iu;
      }
      return (min_iu == min_iu_0) ? 0.0 : min_iu; // want _maximum_ overlap
   };

   // Returns [0..1] or NAN if no feet are found amongst all the poses
   const auto XT       = Vector2{293.935, 296.229};
   const Plane z_p3    = Plane{0.0, 0.0, 1.0, 0.0};
   auto calc_foot_dist = [&](const Vector3f& C0f) -> real {
      const auto C0 = to_vec3(C0f);
      const auto r  = params.foot_dist_radius_factor * k_pose_radius;
      auto ret      = 0.0;
      int counter   = 0;
      for(auto i = 0u; i < poses.size(); ++i) {
         const auto& pose    = poses[i].pose;
         const int sensor_no = poses[i].sensor_no;
         const Vector2 v
             = to_vec2(pose->feet()); // Projected location of the feet
         // const bool is_test  = (v - XT).norm() < 1.0;
         if(v.is_finite()) { // Locate 'v' on the floor plane
            const auto C
                = plane_ray_intersect(dcam(unsigned(sensor_no)), z_p3, v);
            const auto d = std::clamp<real>((C0 - C).norm(), 0.0, r) / r;
            if(ret < d) ret = d;
            counter++;
         }
      }
      return (counter == 0) ? 1.0 : (1.0 - ret);
   };

   auto calc_skel_dist = [&](const Vector3f& C0) -> real {
      if(!get_point_cloud) return 1.0; // point-cloud not supplied
      const auto r = params.skeleton_disparity_weight * k_pose_radius;
      real ret     = 0.0;
      int counter  = 0;
      for(auto i = 0u; i < poses.size(); ++i) {
         const auto sensor_no = poses[i].sensor_no;
         const PointCloud* pc = get_point_cloud(poses[i].sensor_no);
         if(pc == nullptr) continue; // Not the primary sensor
         const auto& et0 = scene_desc->sensor_transforms[size_t(sensor_no)];

         const auto& pose   = poses[i].pose;
         const auto& lookup = pc->lookup;
         trace_skeleton(*pose, [&](const Point2& x) {
            if(!lookup.in_bounds(x)) return;
            const int ind = lookup(x);
            if(ind < 0) return; // no 3D point cloud point
            auto X = to_vec3f(et0.apply(pc->Xs[size_t(ind)]));
            Expects(X.is_finite());
            X.z          = 0.0;
            const auto d = std::clamp<real>(real((X - C0).norm()), 0.0, r) / r;
            Expects(std::isfinite(d));
            ret += d;
            counter++;
         });
      }
      return (counter == 0) ? 1.0 : (1.0 - (ret / real(counter)));
   };

   auto calc_score = [&](const Vector3f& C0) -> real {
      Expects(C0.is_finite());
      const real iu = calc_iu(C0);
      const real foot_dist
          = (params.foot_dist_radius_factor > 0.0) ? calc_foot_dist(C0) : 1.0;
      const real skel_dist
          = (params.skeleton_disparity_weight > 0.0) ? calc_skel_dist(C0) : 1.0;
      const real score = rescale_score(iu) * rescale_score(foot_dist)
                         * rescale_score(skel_dist);
      Expects(std::isfinite(score));
      return score;
   };

   auto calc_gaze_theta = [&]() {
      Vector2 F{0.0, 0.0};
      for(auto i = 0u; i < poses.size(); ++i) { // gazes
         const real gaze_theta = poses[i].pose->theta();
         F += Vector2(cos(gaze_theta), sin(gaze_theta));
      }
      return atan2(F.y, F.x);
   };

   auto calc_is_valid = [&](const real gaze_theta) {
      return std::isfinite(gaze_theta)
             and std::all_of(cbegin(poses), cend(poses), [](const auto& xy) {
                    const auto neck   = xy.pose->neck();
                    const auto pelvis = xy.pose->pelvis();
                    return neck.is_finite() and pelvis.is_finite();
                 });
   };

   ret.gaze_theta_ = calc_gaze_theta();
   ret.is_valid_   = calc_is_valid(ret.gaze_theta_);

   if(ret.is_valid_) {
      for(auto i = 0; i < k_n_heights; ++i) {
         update_floor_pos(k_height(i));
         const auto C0 = average_C();
         Expects(C0.is_finite());
         ret.Cs_[size_t(i)]  = Vector2f(C0.x, C0.y);
         ret.ius_[size_t(i)] = float(calc_score(C0));
         Expects(std::isfinite(ret.ius_[size_t(i)]));
      }

      // Normalize the 'ius' scores
      ret.update_best_iu();
   }

   Expects(std::isfinite(ret.gaze_theta_) or !ret.is_valid());
   Expects(ret.i_ < ret.Cs_.size());

   return ret;
}

Pose3D
This::combine(const SceneDescription* scene_desc,
              std::function<const PointCloud*(int sensor_no)> get_point_cloud,
              const Params& params,
              const PoseSkeletonExec::Result* op_ret,
              const vector<Point2>& posez) noexcept
{
   Expects(scene_desc);
   Expects(op_ret);
   Expects(posez.size() > 0);

   // remove_duplicates(posez);
   for(const auto& xy : posez)
      Expects(unsigned(xy.x) < unsigned(scene_desc->n_sensors()));

   vector<SensorPose> poses(posez.size());
   std::transform(
       cbegin(posez), cend(posez), begin(poses), [&](const auto& xy) {
          return SensorPose(xy.x, xy.y, op_ret->pose(xy.x)[size_t(xy.y)]);
       });

   return combine(scene_desc, get_point_cloud, params, std::move(poses));
}

Pose3D
This::combine(const SceneDescription* scene_desc,
              std::function<const PointCloud*(int sensor_no)> get_point_cloud,
              const Params& params,
              const Pose3D& A,
              const Pose3D& B) noexcept
{
   vector<SensorPose> poses;
   poses.reserve(A.poses_.size() + B.poses_.size());
   poses.insert(end(poses), cbegin(A.poses_), cend(A.poses_));
   poses.insert(end(poses), cbegin(B.poses_), cend(B.poses_));
   return combine(scene_desc, get_point_cloud, params, std::move(poses));
}

Pose3D This::combine(const SceneDescription* scene_desc,
                     const Params& params,
                     const Pose3D& A,
                     const Pose3D& B) noexcept
{
   return combine(scene_desc, nullptr, params, A, B);
}

// -------------------------------------------------------- Pose 3D::to_string()
//
string This::to_string(const SceneDescription* scene_desc) const noexcept
{
   std::stringstream ss{""};
   for(auto i = 0u; i < size(); ++i) {
      if(i > 0) ss << ", ";
      const auto sensor_num = poses()[i].sensor_no;
      const auto& pose      = poses()[i].pose;
      const auto neck       = pose->neck();
      const auto pelvis     = pose->pelvis();
      const auto feet       = pose->feet();
      ss << format("{{}, neck=[{}, {}] pelvis=[{}, {}] feet=[{}, {}]}",
                   scene_desc->sensor_ids[size_t(sensor_num)],
                   neck.x,
                   neck.y,
                   pelvis.x,
                   pelvis.y,
                   feet.x,
                   feet.y);
   }

   return format(R"V0G0N(
Pose3D:
   C:        {}
   radius:   {}
   height:   {}
   gaze:     {}
   iu:       {}
   poses:   [{}]
{})V0G0N",
                 str(C()),
                 radius(),
                 height(),
                 to_degrees(gaze_theta()),
                 iu(),
                 ss.str(),
                 "");
}

string This::to_json_string() const noexcept
{
   return format(
       R"V0G0N(
{{
   "best_i":     {},
   "gaze_theta": {},
   "is_valid":   {},
   "poses":     [{}],
   "Cs":        [{}],
   "ius":       [{}]
}}
%s)V0G0N",
       i_,
       (std::isfinite(gaze_theta_) ? str(gaze_theta_) : "null"),
       str(is_valid_),
       implode(cbegin(poses_),
               cend(poses_),
               "      ,",
               [](auto& a) { return trim_copy(indent(a.to_json_str(), 6)); }),
       implode(cbegin(Cs_),
               cend(Cs_),
               ", ",
               [](auto& a) { return format("[{}, {}]", a.x, a.y); }),
       implode(cbegin(ius_), cend(ius_), ", "),
       "");
}

Json::Value This::to_json() const noexcept
{
   Json::Value o{Json::objectValue};
   o["best_i"]     = i_;
   o["gaze_theta"] = std::isfinite(gaze_theta_) ? Json::Value(gaze_theta_)
                                                : Json::Value(nullptr);
   o["is_valid"]   = is_valid_;
   o["poses"]      = json_save_t(
       cbegin(poses_), cend(poses_), [&](const SensorPose& o) -> Json::Value {
          return o.to_json();
       });
   o["Cs"]  = json_save(cbegin(Cs_), cend(Cs_));
   o["ius"] = json_save(cbegin(ius_), cend(ius_));
   return o;
}

void This::read_with_defaults(const Json::Value& o,
                              const Pose3D* defaults) noexcept(false)
{
   Pose3D x           = (defaults) ? *defaults : *this;
   const bool do_warn = false;
   const string op    = "lose Pose3D"s;

   vector<Vector2f> Cs;
   vector<float> ius;

   json_try_load_key(x.i_, o, "best_i"s, op, do_warn);
   json_try_load_key(x.gaze_theta_, o, "gaze_theta"s, op, do_warn);
   json_try_load_key(x.is_valid_, o, "is_valid"s, op, do_warn);

   json_try_load_t<SensorPose>(
       x.poses_,
       o,
       "poses"s,
       [](const Json::Value& node, SensorPose& sp) { sp.read(node); },
       op,
       do_warn);

   if(o["poses"].type() == Json::arrayValue) {
      if(x.poses_.size() != o["poses"].size())
         throw std::runtime_error(
             format("failed to load poses: json had {} object, but loaded {}",
                    o["poses"].size(),
                    x.poses_.size()));
   }

   json_try_load_key(Cs, o, "Cs"s, op, do_warn);
   json_try_load_key(ius, o, "ius"s, op, do_warn);

   if(Cs.size() == x.Cs_.size()) std::copy(cbegin(Cs), cend(Cs), begin(x.Cs_));
   if(ius.size() == x.ius_.size())
      std::copy(cbegin(ius), cend(ius), begin(x.ius_));

   *this = x;
}

// ------------------------------------------------------------------------ read
//
Pose3D This::read(const Json::Value& node,
                  const Pose3D* defaults) noexcept(false)
{
   Pose3D out;
   out.read_with_defaults(node, defaults);
   return out;
}

// ---------------------------------------------------------------- make_pose_3d
//
Pose3D
make_pose_3d(const SceneDescription* scene_desc,
             std::function<const PointCloud*(int sensor_no)> get_point_cloud,
             const PoseSkeletonExec::Result* op_ret,
             const This::Params& params,
             const int sensor_num,
             const int openpose_detection_num) noexcept
{
   Expects(scene_desc);
   Expects(op_ret);
   Expects(scene_desc->n_sensors() == int(op_ret->size()));
   Expects(unsigned(sensor_num) < unsigned(scene_desc->n_sensors()));

   Pose3D p3d;
   p3d.init(scene_desc,
            get_point_cloud,
            op_ret,
            params,
            sensor_num,
            openpose_detection_num);
   Expects(p3d.poses().size() == 1);
   Expects(p3d.poses()[0].sensor_no == sensor_num);
   return p3d;
}

} // namespace perceive
