
#include "op-inc.hpp"

#define OPENPOSE_FLAGS_DISABLE_POSE
#include <openpose/headers.hpp>
#include <openpose/utilities/errorAndLog.hpp>

#include <opencv2/opencv.hpp>

#define This OpExec

using TDatum    = op::Datum;
using TDatums   = std::vector<shared_ptr<TDatum>>;
using TDatumsSP = shared_ptr<TDatums>;

namespace perceive
{
// ----------------------------------------------------------- render mode to op
//
static op::RenderMode render_mode_to_op(This::RenderMode x) noexcept
{
   switch(x) {
   case This::NONE: return op::RenderMode::None;
   case This::CPU: return op::RenderMode::Cpu;
   case This::GPU:
      return (k_cuda_is_available) ? op::RenderMode::Gpu : op::RenderMode::Cpu;
   }
   return op::RenderMode::None;
}

// ----------------------------------------------------------_- pose model to op
//
static op::PoseModel pose_model_to_op(This::PoseModel x) noexcept
{
   switch(x) {
#define E(x) \
   case This::x: return op::PoseModel::x;
      E(BODY_25);
      E(COCO_18);
      E(MPI_15);
      E(MPI_15_4);
#undef E
   }
   return op::PoseModel::BODY_25;
}

static op::WrapperStructPose set_op_wrapper_struct_pos(const This::Params& p)
{
   static const op::String k_models_path
       = op::String(format("{:s}/models", k_openpose_path).c_str());

   // FATAL(format("k-models-path = {:s}", k_models_path.getStdString()));

   op::WrapperStructPose pose_ops;
   pose_ops.poseMode
       = p.pose_enabled ? op::PoseMode::Enabled : op::PoseMode::Disabled;
   pose_ops.netInputSize = op::Point(p.net_input_size.x, p.net_input_size.y);
   if(k_cuda_is_available) {
      pose_ops.gpuNumberStart
          = (p.gpu_start == 0) ? perceive_cuda_gpu0_default() : p.gpu_start;
      pose_ops.gpuNumber
          = (p.num_gpus == -1) ? perceive_cuda_num_gpus_default() : p.num_gpus;
      pose_ops.renderMode = op::RenderMode::Gpu;
   } else {
      pose_ops.gpuNumberStart = 0;
      pose_ops.gpuNumber      = 0;
      pose_ops.renderMode     = op::RenderMode::Cpu;
   }
   pose_ops.scalesNumber       = p.scales_num;
   pose_ops.scaleGap           = p.scale_gap;
   pose_ops.renderMode         = render_mode_to_op(p.render_mode);
   pose_ops.poseModel          = pose_model_to_op(p.pose_model);
   pose_ops.blendOriginalFrame = true;
   pose_ops.alphaHeatMap       = 1.0; // only render heat map
   pose_ops.modelFolder        = k_models_path;
   pose_ops.heatMapTypes.clear();
   if(p.bg_heatmap)
      pose_ops.heatMapTypes.push_back(op::HeatMapType::Background);
   if(p.body_heatmap) pose_ops.heatMapTypes.push_back(op::HeatMapType::Parts);
   if(p.PAF_heatmap) pose_ops.heatMapTypes.push_back(op::HeatMapType::PAFs);
   pose_ops.heatMapScaleMode    = op::ScaleMode::ZeroToOne;
   pose_ops.addPartCandidates   = p.add_part_candidates;
   pose_ops.renderThreshold     = p.render_threshold;
   pose_ops.numberPeopleMax     = p.max_n_people;
   pose_ops.maximizePositives   = p.maximize_recall;
   pose_ops.enableGoogleLogging = false;

   return pose_ops;
}

// ---------------------------------------------------------------- unpack-poses
//
static vector<This::Pose> unpack_poses(const TDatums* datums) noexcept
{
   vector<This::Pose> ret;
   if(datums == nullptr) return ret;

   if(datums->size() == 0) return ret;

   const auto& poseKeypoints = datums->at(0)->poseKeypoints;
   const auto n_persons      = poseKeypoints.getSize(0);
   const auto n_parts        = poseKeypoints.getSize(1);
   const auto dim3           = poseKeypoints.getSize(2);

   Expects(dim3 == 3 or n_persons == 0);

   ret.resize(n_persons);

   for(auto person = 0; person < n_persons; ++person) {
      auto& p = ret[person].keypoints;
      p.reserve(n_parts);
      for(auto k = 0; k < n_parts; ++k) {
         p.emplace_back();
         p.back().part  = k;
         p.back().x     = poseKeypoints[{person, k, 0}];
         p.back().y     = poseKeypoints[{person, k, 1}];
         p.back().score = poseKeypoints[{person, k, 2}];
         ret[person].score += p.back().score;
      }
      ret[person].score /= real(n_parts);
   }

   return ret;
}

// ----------------------------------------------------------------------- Pimpl

struct This::Pimpl
{
 private:
   unique_ptr<op::Wrapper> op_wrapper_{nullptr};
   Params last_params_;
   bool is_first_ = true;

   // Not reentrant
   op::Wrapper& get_op_wrapper_() noexcept(false)
   {
      if(!op_wrapper_)
         op_wrapper_
             = make_unique<op::Wrapper>(op::ThreadManagerMode::Asynchronous);
      return *op_wrapper_;
   }

 public:
   op::Wrapper& configured_op(const Params& p) noexcept(false)
   {
      auto& wrapper = get_op_wrapper_();
      if(is_first_ or p != last_params_) force_configure_op(p, is_first_);
      return wrapper;
   }

   void force_configure_op(const Params& p,
                           const bool never_feedback = false) noexcept(false)
   {
      if(is_first_ or (p.feedback and !never_feedback))
         INFO(format("Configuring openpose:\n{:s}", indent(p.to_string(), 4)));
      is_first_    = false;
      last_params_ = p;
      auto& op     = get_op_wrapper_();
      op.stop();
      if(p.single_threaded) {
         WARN(format("disabling multi-threading in openpose"));
         op.disableMultiThreading();
      }
      op.configure(set_op_wrapper_struct_pos(p));
      op.setDefaultMaxSizeQueues(1);
      op.start();
   }
};

// ----------------------------------------------------------------- render mode
//
static This::RenderMode to_render_mode(const string_view val) noexcept(false)
{
#define E(x) \
   if(val == #x) return This::x;
   E(NONE);
   E(CPU);
   E(GPU);
#undef E
   throw std::runtime_error(
       format("could not convert integer '{:s}' to a RenderMode", val));
}

const char* str(const This::RenderMode x) noexcept
{
   switch(x) {
#define E(x) \
   case This::x: return #x;
      E(NONE);
      E(CPU);
      E(GPU);
#undef E
   }
}

This::RenderMode This::int_to_render_mode(int val) noexcept
{
   const auto r = RenderMode(val);
   switch(r) {
#define E(x) \
   case This::x: return This::x;
      E(NONE);
      E(CPU);
      E(GPU);
   default: return This::NONE;
#undef E
   }
   return r;
}

// ------------------------------------------------------------------ pose model
//

static This::PoseModel to_pose_model(const string_view val) noexcept(false)
{
#define E(x) \
   if(val == #x) return This::x;
   E(BODY_25);
   E(COCO_18);
   E(MPI_15);
   E(MPI_15_4);
#undef E
   throw std::runtime_error(
       format("could not convert integer '{:s}' to a PoseModel", val));
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

const vector<MemberMetaData>& OpExec::Params::meta_data() const noexcept
{
#define ThisParams OpExec::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, BOOL, single_threaded, false));
      m.push_back(MAKE_META(ThisParams, BOOL, pose_enabled, true));

      m.push_back({meta_type::JSON_VALUE,
                   "net_input_size"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      Json::Value x{Json::arrayValue};
                      x.resize(2);
                      x[0] = o.net_input_size.x;
                      x[1] = o.net_input_size.y;
                      return std::any(x);
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o          = *reinterpret_cast<ThisParams*>(ptr);
                      const auto& s    = std::any_cast<const Json::Value>(x);
                      o.net_input_size = Point2(s[0].asInt(), s[1].asInt());
                   }});

      m.push_back(MAKE_META(ThisParams, INT, gpu_start, false));
      m.push_back(MAKE_META(ThisParams, INT, num_gpus, false));
      m.push_back(MAKE_META(ThisParams, INT, scales_num, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, scale_gap, true));
      m.push_back({meta_type::STRING,
                   "render_mode"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      return std::any(string(str(o.render_mode)));
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o         = *reinterpret_cast<ThisParams*>(ptr);
                      const string& s = std::any_cast<const string>(x);
                      o.render_mode   = to_render_mode(s);
                   }});
      m.push_back({meta_type::STRING,
                   "pose_model"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      return std::any(string(str(o.pose_model)));
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o         = *reinterpret_cast<ThisParams*>(ptr);
                      const string& s = std::any_cast<const string>(x);
                      o.pose_model    = to_pose_model(s);
                   }});
      m.push_back(MAKE_META(ThisParams, BOOL, bg_heatmap, true));
      m.push_back(MAKE_META(ThisParams, BOOL, body_heatmap, true));
      m.push_back(MAKE_META(ThisParams, BOOL, PAF_heatmap, true));
      m.push_back(MAKE_META(ThisParams, BOOL, add_part_candidates, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, render_threshold, true));
      m.push_back(MAKE_META(ThisParams, INT, max_n_people, true));
      m.push_back(MAKE_META(ThisParams, BOOL, maximize_recall, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// -------------------------------------------------------------- This::Keypoint

bool This::Keypoint::operator==(const Keypoint& o) const noexcept
{
#define TEST(x) (this->x == o.x)
#define TEST_REAL(x) (float_is_same(this->x, o.x))
   return TEST(part) and TEST_REAL(x) and TEST_REAL(y) and TEST_REAL(score);
#undef TEST
#undef TEST_REAL
}

bool This::Keypoint::operator!=(const Keypoint& o) const noexcept
{
   return !(*this == o);
}

// ------------------------------------------------------------------ This::Pose

bool This::Pose::operator==(const Pose& o) const noexcept
{
#define TEST(x) (this->x == o.x)
#define TEST_REAL(x) (float_is_same(this->x, o.x))
   return TEST(model) and TEST(keypoints) and TEST_REAL(theta)
          and TEST_REAL(score);
#undef TEST
#undef TEST_REAL
}

bool This::Pose::operator!=(const Pose& o) const noexcept
{
   return !(*this == o);
}

Json::Value This::Pose::to_json() const noexcept
{
   auto a = Json::Value{Json::arrayValue};
   a.resize(keypoints.size());
   for(auto i = 0u; i < keypoints.size(); ++i) {
      const auto& k = keypoints[i];
      auto v        = Json::Value{Json::arrayValue};
      v.resize(4);
      v[0] = float(k.part);
      v[1] = k.x;
      v[2] = k.y;
      v[3] = k.score;
      a[i] = v;
   }

   auto o         = Json::Value{Json::objectValue};
   o["keypoints"] = a;
   o["score"]     = score;
   o["model"]     = json_encode(str(model));
   o["theta"]     = theta;

   return o;
}

void This::Pose::read(const Json::Value& o) noexcept(false)
{
   auto x = Pose{};

   try {
      json_load(o["score"], x.score);

      // What type of model?
      if(o["model"].type() == Json::stringValue)
         x.model = to_pose_model(o["model"].asString());
      else
         WARN(format("Pose did not have 'string' \"model\" value"));

      // Load the keypoints
      const auto& a = o["keypoints"];
      if(a.type() != Json::arrayValue)
         throw std::runtime_error("expected array value");
      x.keypoints.resize(a.size());
      for(auto i = 0u; i < x.keypoints.size(); ++i) {
         auto& k = x.keypoints[i];
         if(a[i].type() != Json::arrayValue)
            throw std::runtime_error("keypoints must be arrays too!");
         if(a[i].size() != 4)
            throw std::runtime_error("keypoints must be arrays of size 4");
         json_load(a[i][0], k.part);
         json_load(a[i][1], k.x);
         json_load(a[i][2], k.y);
         json_load(a[i][3], k.score);
      }

      json_load(o["theta"], x.theta);

      *this = x;
   } catch(std::exception& e) {
      throw e;
   }
}

string This::Pose::to_json_str() const noexcept
{
   const string model_s = str(model);

   return format(
       R"V0G0N(
{
   "model":      {:s},
   "theta":      {},
   "score":      {},    
   "keypoints": [{:s}]
}
{:s})V0G0N",
       json_encode(model_s),
       theta,
       score,
       implode(cbegin(keypoints),
               cend(keypoints),
               ",\n        ",
               [&](const auto& k) {
                  return format(
                      "[{}, {}, {}, {}]", k.part, k.x, k.y, k.score);
               }),
       "");
}

// 'ind' is the BODY_25 index.
static Vector2 ret_kp(const This::Pose& pose, const int ind0) noexcept
{
   static constexpr array<int, 25> k_body_25
       = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12,
          13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
   static constexpr array<int, 25> k_coco
       = {0,  1,  2,  3,  4,  5,  6,  7,  -1, 8,  9,  10, 11,
          12, 13, 14, 15, 16, 17, 13, 13, 13, 10, 10, 10};

   Expects(ind0 >= 0 and ind0 < int(k_body_25.size()));
   Expects(ind0 >= 0 and ind0 < int(k_coco.size()));

   auto get_kp = [&](int ind) {
      Expects(ind >= 0 and ind < int(pose.keypoints.size()));
      const auto x = to_vec2(pose.keypoints[ind].xy());
      return x.quadrance() > 0.0 ? x : Vector2::nan();
   };

   if(pose.model == OpExec::BODY_25) {
      return get_kp(k_body_25[ind0]);
   } else if(pose.model == OpExec::COCO_18) {
      if(ind0 == 8) { // Special case... average '8' and '11'
         return 0.5 * (get_kp(8) + get_kp(11));
      } else {
         return get_kp(k_coco[ind0]);
      }
   } else {
      FATAL(format("model not supported. A mapping must be entered "
                   "by hand at this point of code. @see "
                   "https://github.com/CMU-Perceptual-Computing-Lab/openpose/"
                   "blob/master/doc/output.md for hints"));
   }

   return Vector2::nan();
}

Vector2 This::Pose::nose() const noexcept { return ret_kp(*this, 0); }
Vector2 This::Pose::neck() const noexcept { return ret_kp(*this, 1); }
Vector2 This::Pose::pelvis() const noexcept { return ret_kp(*this, 8); }
Vector2 This::Pose::l_eye() const noexcept { return ret_kp(*this, 16); }
Vector2 This::Pose::r_eye() const noexcept { return ret_kp(*this, 15); }
Vector2 This::Pose::l_ear() const noexcept { return ret_kp(*this, 18); }
Vector2 This::Pose::r_ear() const noexcept { return ret_kp(*this, 17); }
Vector2 This::Pose::l_shoulder() const noexcept { return ret_kp(*this, 5); }
Vector2 This::Pose::r_shoulder() const noexcept { return ret_kp(*this, 2); }
Vector2 This::Pose::l_elbow() const noexcept { return ret_kp(*this, 6); }
Vector2 This::Pose::r_elbow() const noexcept { return ret_kp(*this, 3); }
Vector2 This::Pose::l_wrist() const noexcept { return ret_kp(*this, 7); }
Vector2 This::Pose::r_wrist() const noexcept { return ret_kp(*this, 4); }
Vector2 This::Pose::l_hip() const noexcept { return ret_kp(*this, 12); }
Vector2 This::Pose::r_hip() const noexcept { return ret_kp(*this, 9); }
Vector2 This::Pose::l_knee() const noexcept { return ret_kp(*this, 13); }
Vector2 This::Pose::r_knee() const noexcept { return ret_kp(*this, 10); }
Vector2 This::Pose::l_ankle() const noexcept { return ret_kp(*this, 14); }
Vector2 This::Pose::r_ankle() const noexcept { return ret_kp(*this, 11); }
Vector2 This::Pose::l_heal() const noexcept { return ret_kp(*this, 21); }
Vector2 This::Pose::r_heal() const noexcept { return ret_kp(*this, 24); }
Vector2 This::Pose::l_big_toe() const noexcept { return ret_kp(*this, 19); }
Vector2 This::Pose::r_big_toe() const noexcept { return ret_kp(*this, 22); }
Vector2 This::Pose::l_small_toe() const noexcept { return ret_kp(*this, 20); }
Vector2 This::Pose::r_small_toe() const noexcept { return ret_kp(*this, 23); }

Vector2 This::Pose::feet() const noexcept
{
   if(model == BODY_25) {
      auto average_kps = [&](array<int, 4> kps) {
         int counter = 0;
         Vector2 av{0.0, 0.0};
         for(const auto ind : kps) {
            auto x = ret_kp(*this, ind);
            if(x.is_finite()) {
               av += x;
               counter++;
            }
         }
         return (counter == 0) ? Vector2::nan() : (av / real(counter));
      };

      Vector2 l_foot = average_kps({14, 19, 20, 21});
      Vector2 r_foot = average_kps({11, 22, 23, 24});
      return 0.5 * (l_foot + r_foot);
   } else if(model == COCO_18) {
      return 0.5 * (l_ankle() + r_ankle());
   } else {
      FATAL("not implemented");
   }
   return Vector2::nan();
}

// ------------------------------------------------------------------------- str
//
string str(const vector<This::Pose>& xs) noexcept
{
   auto pose_to_json = [](const This::Pose& x) { return x.to_json_str(); };
   return format("{{{:s}}}", implode(cbegin(xs), cend(xs), ", ", pose_to_json));
}

// ------------------------------------------------------------------------- run
//
This::This()
    : pimpl_(make_unique<Pimpl>())
{
   static std::once_flag flag1;
   std::call_once(flag1, []() {
      // op::ConfigureLog::setPriorityThreshold(op::Priority::None);
      // op::ConfigureLog::setPriorityThreshold(op::Priority::Max);
      op::ConfigureLog::setPriorityThreshold(op::Priority::NoOutput);

      std::vector<op::LogMode> modes = {op::LogMode::StdCout};
      op::ConfigureLog::setLogModes(modes);
   });
}
This::~This() = default;

// ------------------------------------------------------------------------- run
//
vector<This::Pose> This::run(const cv::Mat& image,
                             const This::Params& p) noexcept
{
   vector<This::Pose> ret;

   const bool is_trace_mode = true;
   const bool output_images = false;

   const auto now = tick();

   static int frame_no = 0;

   try {
      auto& op_wrapper    = pimpl_->configured_op(p);
      TDatumsSP processed = nullptr;

      if(output_images) {
         cv::imwrite(format("/tmp/op-in-{:4d}.png", frame_no), image);
      }

      const bool was_pushed = op_wrapper.waitAndPush(OP_CV2OPMAT(image));
      if(!was_pushed) FATAL(format("failed to push image... not sure why."));

      const bool was_popped = op_wrapper.waitAndPop(processed);
      if(!was_popped) FATAL(format("failed to pop results... not sure why."));

      if(processed == nullptr) {
         WARN("failed to process sensor image");
      } else if(processed->empty()) {
         WARN(format("empty data for sensor image"));
      } else {
         ret = unpack_poses(processed.get());

         if(output_images) {
            cv::Mat out = OP_OP2CVMAT(processed->at(0)->cvOutputData);
            cv::imwrite(format("/tmp/op-out-{:4d}.png", frame_no), out);
         }
      }

      const auto seconds = tock(now);

      if(is_trace_mode) { INFO(format("times = {}s", seconds)); }

   } catch(const std::exception& e) {
      FATAL(format("exception: {:s}", e.what()));
   }

   ++frame_no;

   return ret;
}

} // namespace perceive
