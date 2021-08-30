
#include "hyperpose-helpers.hpp"

#include "ctre.hpp"

#ifdef WITH_HYPERPOSE
#include <hyperpose/hyperpose.hpp>
#endif

#define This HyperposeWrapper

#ifndef WITH_HYPERPOSE

// -----------------------------------------------------------------------------
// -------------------------------------------------------- WithOUT HYPERPOSE --
// -----------------------------------------------------------------------------

namespace perceive
{
struct This::Pimpl
{
   //
};

This::This()
    : pimpl_(new Pimpl)
{}
This::~This() = default;

vector<vector<shared_ptr<const Skeleton2D>>>
This::run(const SceneDescription* scene_desc_ptr,
          const int frame_no,
          const int n_images,
          std::function<const cv::Mat*(int i)> get_image,
          const pose_skeleton::HyperposeParams& p,
          const string_view outdir,
          ParallelJobSet& pjobs,
          std::function<bool()> is_cancelled) noexcept
{
   WARN(format("running hyperpose, however, it wasn't compiled in!"));
   vector<vector<shared_ptr<const Skeleton2D>>> out((size_t(n_images)));
   return out;
}

void clear_hyperpose_registry() noexcept {}

} // namespace perceive

#else

// -----------------------------------------------------------------------------
// ----------------------------------------------------------- WITH HYPERPOSE --
// -----------------------------------------------------------------------------

#include <gflags/gflags.h>
#include <hyperpose/hyperpose.hpp>
#include <variant>

namespace hp = hyperpose;

namespace perceive
{
// ----------------------------------------------------------------

static string to_str(const hp::human_t& hh, const int w, const int h) noexcept
{
   int counter          = 0;
   const string parts_s = implode(
       begin(hh.parts),
       end(hh.parts),
       ",\n   ",
       [&](const hp::body_part_t& part) -> string {
          return format("[{:02d}, xy = {{{:3d}, {:3d}}}, score = {:5.3f}]",
                        counter++,
                        int(float(w) * part.x),
                        int(float(h) * part.y),
                        part.score);
       });

   return format(R"V0G0N(
HumanDetection
   {}
)V0G0N",
                 parts_s);
}

// ---------------------------------------------------------------- unpack-poses
// @see
// https://hyperpose.readthedocs.io/en/latest/cpp/human_8hpp.html#a205ca2b8f07ba1397fbeeb55614c4064
static vector<shared_ptr<const Skeleton2D>>
unpack_poses(const DistortedCamera* dcam_ptr,
             const int frame_no,
             const int sensor_no,
             const vector<hp::human_t>& data,
             const pose_skeleton::HyperposeParams& params,
             const cv::Mat& im) noexcept
{
   vector<shared_ptr<const Skeleton2D>> out;
   out.resize(data.size());

   static constexpr std::array<int32_t, hp::COCO_N_PARTS> convert_hp_ids
       = {{int32_t(skeleton::KeypointName::NOSE),
           int32_t(skeleton::KeypointName::NOTCH),
           int32_t(skeleton::KeypointName::R_SHOULDER),
           int32_t(skeleton::KeypointName::R_ELBOW),
           int32_t(skeleton::KeypointName::R_WRIST),
           int32_t(skeleton::KeypointName::L_SHOULDER),
           int32_t(skeleton::KeypointName::L_ELBOW),
           int32_t(skeleton::KeypointName::L_WRIST),
           int32_t(skeleton::KeypointName::R_HIP),
           int32_t(skeleton::KeypointName::R_KNEE),
           int32_t(skeleton::KeypointName::R_ANKLE),
           int32_t(skeleton::KeypointName::L_HIP),
           int32_t(skeleton::KeypointName::L_KNEE),
           int32_t(skeleton::KeypointName::L_ANKLE),
           int32_t(skeleton::KeypointName::R_EYE),
           int32_t(skeleton::KeypointName::L_EYE),
           int32_t(skeleton::KeypointName::R_EAR),
           int32_t(skeleton::KeypointName::L_EAR)}};

   const auto fw = float(im.cols);
   const auto fh = float(im.rows);

   auto body_part_to_kp = [&](const int32_t id, const hp::body_part_t& part) {
      Skeleton2D::Keypoint kp;
      kp.part  = id;
      kp.pos   = (part.has_value) ? Vector2f(part.x * fw, part.y * fh)
                                  : Vector2f::nan();
      kp.score = (part.has_value) ? part.score : 0.0f;
      return kp;
   };

   auto make_keypoints = [&](const hp::human_t& human) {
      Skeleton2D::Params p2d_params;
      vector<Skeleton2D::Keypoint> kps;
      kps.resize(skeleton::k_n_keypoints);
      for(auto i = 0; i < skeleton::k_n_keypoints; ++i)
         kps.at(size_t(i)).part = i;

      for(size_t ind = 0; ind < human.parts.size(); ++ind) {
         Expects(ind < convert_hp_ids.size());
         const int32_t part_id = convert_hp_ids.at(ind);
         Expects(size_t(part_id) <= kps.size());
         kps.at(size_t(part_id)) = body_part_to_kp(part_id, human.parts[ind]);
      }

      { // Make the pelvis, as the average of the two hips...
         const auto& lhip = kps.at(size_t(skeleton::L_HIP));
         const auto& rhip = kps.at(size_t(skeleton::R_HIP));
         auto& pelvis     = kps.at(size_t(skeleton::PELVIS));
         pelvis.pos       = 0.5f * (lhip.xy() + rhip.xy());
         pelvis.score     = 0.5f * (lhip.score + rhip.score);
      }

      return kps;
   };

   auto make_p2d
       = [&](const hp::human_t& human) -> shared_ptr<const Skeleton2D> {
      Skeleton2D::Params p2d_params;
      auto p2d_ptr = make_shared<Skeleton2D>();
      p2d_ptr->init(p2d_params,
                    frame_no,
                    sensor_no,
                    dcam_ptr,
                    Skeleton2D::BODY_25,
                    make_keypoints(human));
      return p2d_ptr;
   };

   // Execute in parallel
   static thread_local ParallelJobSet pjobs;
   for(size_t i = 0; i < data.size(); ++i)
      pjobs.schedule(
          [i, &out, &data, &make_p2d]() { out.at(i) = make_p2d(data.at(i)); });
   pjobs.execute();

   // Return
   return out;
}

// ------------------------------------------------------ resolve-model-filename
//
static string resolve_model_filename(string_view name) noexcept
{
   return format("{}/models/{}", multiview_hyperpose_path(), name);
}

// -------------------------------------------------------------- model registry
//
constexpr std::string_view onnx_suffix = ".onnx";
constexpr std::string_view uff_suffix  = ".uff";

enum class ModelType : int { NONE = 0, ONNX, UFF };

static constexpr ModelType string_to_model_type(string_view name) noexcept
{
   if(ends_with(name, onnx_suffix)) return ModelType::ONNX;
   if(ends_with(name, uff_suffix)) return ModelType::UFF;
   return ModelType::NONE;
}

static constexpr const char* str(const ModelType m) noexcept
{
   switch(m) {
   case ModelType::NONE: return "NONE";
   case ModelType::ONNX: return "ONNX";
   case ModelType::UFF: return "UFF";
   }
   return "NONE";
}

static Point2 calc_network_resolution(Point2 x, string_view name) noexcept
{
   static constexpr auto pattern
       = ctll::fixed_string("^.*?([0-9]+)x([0-9]+)\\.([^\\.]*)$");
   if(x.x < 0 || x.y < 0) {
      const auto match = ctre::match<pattern>(name);
      if(!match) FATAL(format("failed to match name '{}' to regex", name));
      const auto ec0 = lexical_cast(match.get<1>().to_view(), x.x);
      const auto ec1 = lexical_cast(match.get<2>().to_view(), x.y);
      if(ec0 || ec1)
         FATAL(format("failed to convert either '{}' or '{}' to integers",
                      match.get<1>().to_view(),
                      match.get<2>().to_view()));
   }
   return x;
}

static cv::Size to_cv_size(const Point2 x) noexcept { return {x.y, x.x}; }

static SpinLock padlock_;

struct HyperposeModelRegistry
{
 private:
   HyperposeModelRegistry()
   {
      register_exit_function([this]() { reset(); });
   }

   struct RegistryData
   {
      ModelType type                       = ModelType::NONE;
      hp::dnn::uff uff_model               = {};
      hp::dnn::onnx onnx_model             = {};
      int max_batch_size                   = 0;
      unique_ptr<hp::dnn::tensorrt> engine = nullptr;
   };

   std::unordered_map<string, RegistryData> registry_ = {};

   /// @precondition: `padlock_` is locked.
   /// @precondition: `name` is not in the registry.
   bool make_engine_(const pose_skeleton::HyperposeParams& p,
                     const int n_images,
                     const string_view outdir) noexcept
   {
      const auto& name = p.model_name;
      Expects(registry_.count(name) == 0);

      const auto filename = resolve_model_filename(name);
      if(!is_regular_file(filename))
         FATAL(format("failed to find hyperpose-model file '{}'", filename));

      Point2 network_resolution
          = calc_network_resolution(p.network_resolution, p.model_name);

      RegistryData data;
      data.type           = string_to_model_type(name);
      data.max_batch_size = (n_images > 0) ? n_images : 1;

      TRACE(format("attempting to load HyperposeModel '{}' with outdir set to "
                   "'{}', type = {}, and network resolution = [{}x{}]",
                   filename,
                   outdir,
                   str(data.type),
                   network_resolution.x,
                   network_resolution.y));

      if(data.type == ModelType::NONE)
         FATAL(format(
             "precondition failed: model '{}' neither `onyx` nor `uff`", name));

      try {
         if(data.type == ModelType::ONNX) {
            data.onnx_model.model_path = filename;
            data.engine
                = make_unique<hp::dnn::tensorrt>(data.onnx_model,
                                                 to_cv_size(network_resolution),
                                                 data.max_batch_size);
         } else if(data.type == ModelType::UFF) {
            data.uff_model.model_path = filename;
            data.uff_model.input_name = "image"s;
            data.uff_model.output_names
                = {format("{}/hyperpose-uff/conf", outdir),
                   format("{}/hyperpose-uff/paf", outdir)};
            for(const auto& d : data.uff_model.output_names) {
               if(!is_directory(d)) // Make sure output directories exist
                  if(!mkdir_p(d))
                     throw std::runtime_error(
                         format("could not make directory '{}'", d));
            }
            data.engine
                = make_unique<hp::dnn::tensorrt>(data.uff_model,
                                                 to_cv_size(network_resolution),
                                                 data.max_batch_size);
         } else {
            FATAL("kBAM!"); // precondition failed
         }

         registry_[name] = std::move(data);
         return true;
      } catch(std::exception& e) {
         FATAL(format("failed to load mode '{}': {}", name, e.what()));
      }

      return false;
   }

 public:
   static HyperposeModelRegistry& instance() noexcept
   {
      static HyperposeModelRegistry registry;
      return registry;
   }

   hp::dnn::tensorrt* get_engine(const pose_skeleton::HyperposeParams& p,
                                 const int n_images,
                                 const string_view outdir) noexcept
   {
      lock_guard lock(padlock_);
      auto ii = registry_.find(p.model_name);

      const bool exists  = !(ii == cend(registry_));
      const bool do_make = !exists || ii->second.max_batch_size < n_images;
      if(do_make) {
         if(exists) registry_.erase(ii);
         if(!make_engine_(p, n_images, outdir))
            FATAL(format("failed to load hyperpose engine, aborting"));
         ii = registry_.find(p.model_name);
         Expects(ii != cend(registry_));
      }

      return ii->second.engine.get();
   }

   void clear_engine(const string& name) noexcept
   {
      lock_guard lock(padlock_);
      registry_.erase(name);
   }

   void reset() noexcept
   {
      lock_guard lock(padlock_);
      registry_.clear();
   }
};

// ----------------------------------------------------------------------- Pimpl
//
struct This::Pimpl
{
   vector<hp::parser::paf> parsers;
};

// ----------------------------------------------------------------- Constructor
//
This::This()
    : pimpl_(new Pimpl)
{}
This::~This() = default;

// ------------------------------------------------------------------------- Run
//
vector<vector<shared_ptr<const Skeleton2D>>>
This::run(const SceneDescription* scene_desc_ptr,
          const int frame_no,
          const int n_images,
          std::function<const cv::Mat*(int i)> get_image,
          const pose_skeleton::HyperposeParams& p,
          const string_view outdir,
          ParallelJobSet& pjobs,
          std::function<bool()> is_cancelled) noexcept
{
   Expects(n_images >= 0);
   if(n_images == 0) return {};

   hp::dnn::tensorrt* engine_ptr = nullptr;
   vector<cv::Mat> images;
   vector<size_t> indices;
   vector<hp::internal_t> raw;
   vector<vector<hp::human_t>> pose_vectors;
   vector<vector<shared_ptr<const Skeleton2D>>> out;
   auto& parsers = pimpl_->parsers;

   const auto s0 = time_thunk([&]() { // Grab the engine
      engine_ptr
          = HyperposeModelRegistry::instance().get_engine(p, n_images, outdir);
      Expects(engine_ptr != nullptr);
   });

   const auto s1 = time_thunk([&]() { // Prepare the input
      images.reserve(size_t(n_images));
      indices.reserve(size_t(n_images));
      for(int i = 0; i < n_images; ++i) {
         if(auto ptr = get_image(i); ptr != nullptr) {
            images.push_back(*ptr);
            indices.push_back(size_t(i));
         }
      }

      if(false) {
         const auto fname = "/tmp/zzz_envista_test_v1_28_2.png"s;
         images.clear();
         indices.clear();
         images.push_back(cv::imread(fname));
         indices.push_back(0);
         TRACE(format("pushed: {}", fname));
      }

      if(false && multiview_trace_mode() && scene_desc_ptr != nullptr) {
         const auto scene_name = scene_desc_ptr->scene_info.scene_key;
         TRACE(format("HYPERPOSE, scene {}, frame {}", scene_name, frame_no));
         for(int i = 0; i < n_images; ++i) {
            if(auto ptr = get_image(i); ptr != nullptr) {
               const auto fname
                   = format("/tmp/zzz_{}_{}_{}.png", scene_name, frame_no, i);
               cv::imwrite(fname, *ptr);
            }
         }
      }
   });

   const auto s2 = time_thunk([&]() { // Run the engine
      lock_guard lock(padlock_);      //
      raw = engine_ptr->inference(images);

   });

   const auto s3 = time_thunk([&]() { // Parse the output
      if(parsers.size() < raw.size()) parsers.resize(raw.size());
      pose_vectors.resize(raw.size());

      auto process_i = [&](size_t i) {
         auto& packet    = raw[i];
         pose_vectors[i] = parsers[i].process(packet[0], packet[1]);
         //= std::move(humans);

         if(false) {
            cv::Mat im = images[i].clone();
            for(auto&& human : pose_vectors[i]) {
               hp::draw_human(im, human);
               INFO(format("pose: {}", to_str(human, im.cols, im.rows)));
               break;
            }
            cv::imwrite(format("/tmp/out_{:04d}_{}.png", frame_no, i), im);
            FATAL("kBAM!");
         }
      };

      for(size_t i = 0; i < raw.size(); ++i)
         pjobs.schedule([i, &process_i]() { process_i(i); });
      pjobs.execute();

   });

   const auto s4 = time_thunk([&]() {
      auto process_i = [&](size_t i) {
         Expects(i < indices.size());
         const auto sensor_no = int(indices.at(i));
         Expects(size_t(sensor_no) < out.size());
         const DistortedCamera* dcam_ptr
             = (scene_desc_ptr == nullptr) ? nullptr
                                           : &scene_desc_ptr->dcam(sensor_no);
         out.at(size_t(sensor_no)) = unpack_poses(
             dcam_ptr, frame_no, sensor_no, pose_vectors[i], p, images[i]);
      };

      out.resize(size_t(n_images));
      for(size_t i = 0; i < raw.size(); ++i)
         pjobs.schedule([i, &process_i]() { process_i(i); });
      pjobs.execute();
   });

   if(multiview_trace_mode()) {
      TRACE(format("hyperpose inference timings, with {} images:",
                   images.size()));
      cout << format(R"V0G0N(    
   load engine:    {:5.3f}ms
   prepare input:  {:5.3f}ms
   run engine:     {:5.3f}ms
   parse features: {:5.3f}ms
   unpack poses:   {:5.3f}ms

)V0G0N",
                     s0 * 1000.0,
                     s1 * 1000.0,
                     s2 * 1000.0,
                     s3 * 1000.0,
                     s4 * 1000.0);
   }

   return out;
}

void clear_hyperpose_registry() noexcept
{
   HyperposeModelRegistry::instance().reset();
}

} // namespace perceive

#endif

#undef This
