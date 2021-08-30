
#include "stdinc.hpp"

#include "op-helpers.hpp"

//#include "pose-skeleton-exec.hpp"

#ifndef WITH_OPENPOSE

namespace perceive
{
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

struct OpenposeWrapper::Pimpl
{};

OpenposeWrapper::OpenposeWrapper()
    : pimpl_(new Pimpl)
{}

OpenposeWrapper::~OpenposeWrapper() = default;

vector<vector<shared_ptr<const Skeleton2D>>>
OpenposeWrapper::run(const SceneDescription* scene_desc_ptr,
                     const int frame_no,
                     const int n_images,
                     std::function<const cv::Mat*(int i)> get_image,
                     const pose_skeleton::OpenposeParams& p,
                     const string_view outdir,
                     ParallelJobSet& pjobs,
                     std::function<bool()> is_cancelled) noexcept
{
   WARN(format("running openpose, but it wasn't compiled in!"));
   vector<vector<shared_ptr<const Skeleton2D>>> out;
   out.resize(size_t(n_images));
   return out;
}

} // namespace perceive

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

#else

#include "perceive/geometry/skeleton/2d-helpers.hpp"

#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/utils/file-system.hpp"

#define OPENPOSE_FLAGS_DISABLE_POSE
#include <openpose/headers.hpp>
#include <openpose/utilities/errorAndLog.hpp>

#include <opencv2/imgproc.hpp>

namespace perceive
{
using TDatum    = op::Datum;
using TDatums   = std::vector<shared_ptr<TDatum>>;
using TDatumsSP = shared_ptr<TDatums>;

// ------------------------------------------------------------ wrapper-instance
//
static op::Wrapper& op_wrapper_instance()
{
   static unique_ptr<op::Wrapper> op_wrapper_
       = make_unique<op::Wrapper>(op::ThreadManagerMode::Asynchronous);
   return *op_wrapper_;
}

// ----------------------------------------------------------- render mode to op
//
static op::RenderMode render_mode_to_op(pose_skeleton::RenderMode x) noexcept
{
   switch(x) {
   case pose_skeleton::RenderMode::NONE: return op::RenderMode::None;
   case pose_skeleton::RenderMode::CPU: return op::RenderMode::Cpu;
   case pose_skeleton::RenderMode::GPU:
      return (k_cuda_is_available) ? op::RenderMode::Gpu : op::RenderMode::Cpu;
   }
   return op::RenderMode::None;
}

// ----------------------------------------------------------_- pose model to op
//
static op::PoseModel pose_model_to_op(Skeleton2D::PoseModel x) noexcept
{
   switch(x) {
#define E(x)           \
   case Skeleton2D::x: \
      return op::PoseModel::x;
      E(BODY_25);
      E(COCO_18);
      E(MPI_15);
      E(MPI_15_4);
#undef E
   }
   return op::PoseModel::BODY_25;
}

// ------------------------------------------------------ extract image from net
//
static cv::Mat extract_image_from_net(const TDatums* datums)
{
   auto& poseHeatMaps = datums->at(0)->poseHeatMaps;

   // Read desired channel
   const auto numberChannels = poseHeatMaps.getSize(0);
   const auto height         = poseHeatMaps.getSize(1);
   const auto width          = poseHeatMaps.getSize(2);

   // Read image used from OpenPose body network (same resolution than
   //   heatmaps)
   auto& inputNetData = datums->at(0)->inputNetData[0];
   const cv::Mat inputNetDataB(
       height, width, CV_32F, &inputNetData.getPtr()[0]);
   const cv::Mat inputNetDataG(
       height, width, CV_32F, &inputNetData.getPtr()[height * width]);
   const cv::Mat inputNetDataR(
       height, width, CV_32F, &inputNetData.getPtr()[2 * height * width]);

   cv::Mat netInputImage;
   cv::merge(std::vector<cv::Mat>{inputNetDataB, inputNetDataG, inputNetDataR},
             netInputImage);
   netInputImage = (netInputImage + 0.5) * 255;

   // Turn into uint8 cv::Mat
   cv::Mat netInputImageUint8;
   netInputImage.convertTo(netInputImageUint8, CV_8UC1);
   return netInputImageUint8;
};

// ------------------------------------------------------------ collapse-heatmap
//
static cv::Mat collapse_heatmap(const TDatums* datums)
{
   if(datums == nullptr or datums->size() == 0) return cv::Mat{};
   auto& poseHeatMaps = datums->at(0)->poseHeatMaps;
   const auto nc      = poseHeatMaps.getSize(0);
   const auto h       = poseHeatMaps.getSize(1);
   const auto w       = poseHeatMaps.getSize(2);
   const cv::Mat net_data(nc, h * w, CV_32F, poseHeatMaps.getPtr());
   cv::Mat heat(1, h * w, CV_32F);
   cv::reduce(net_data, heat, 0, cv::REDUCE_SUM);
   heat = heat.reshape(1, h);
   return heat;
}

// ---------------------------------------------------------------- heatmap-view
//
static cv::Mat heatmap_view(const TDatums* datums)
{
   if(datums == nullptr or datums->size() == 0) return cv::Mat{};

   // Read desired channel
   cv::Mat cp = collapse_heatmap(datums);
   cp         = cp * 255;

   // Get a uint8 colormap
   cv::Mat heatmap_uint8;
   cp.convertTo(heatmap_uint8, CV_8UC1);
   cv::applyColorMap(heatmap_uint8, heatmap_uint8, cv::COLORMAP_JET);

   // Combining both images
   cv::Mat out_im;
   cv::addWeighted(
       extract_image_from_net(datums), 0.1, heatmap_uint8, 0.9, 0., out_im);
   return out_im;
}

// -------------------------------------------------------------- billboard-view
//
// cv::Mat draw_billboard(const cv::Mat& in,
//                        const vector<shared_ptr<const Skeleton2D>>& poses,
//                        const CachingUndistortInverse& cu,
//                        const EuclideanTransform& et_inv,
//                        const uint32_t kolour)
// {
//    if(in.empty()) return in;

//    const auto floor  = Plane(0, 0, 1, 0);
//    const auto height = 1.65; // height of a person

//    ARGBImage argb = cv_to_argb(in);
//    argb(0, 0)     = k_red;
//    for(const auto& pose_ptr : poses) {
//       auto billboard = pose_to_billboard(floor, cu, et_inv, *pose_ptr,
//       height);

//       // Let's draw the billboard
//       for(auto i = 0u; i < billboard.size(); ++i) {
//          const auto& A = billboard[i];
//          const auto& B = billboard[(i + 1) % billboard.size()];
//          const auto a  = cu.distort(homgen_P2_to_R2(et_inv.apply(A)));
//          const auto b  = cu.distort(homgen_P2_to_R2(et_inv.apply(B)));
//          bresenham(a, b, [&](int x, int y) {
//             if(argb.in_bounds(x, y)) argb(x, y) = kolour;
//          });
//       }
//    }

//    return argb_to_cv(argb);
// }

// ----------------------------------------------------------------- render mode
//
static cv::Mat calc_heatmap(const TDatums* datumsPtr, const int desiredChannel)
{
   auto& poseHeatMaps = datumsPtr->at(0)->poseHeatMaps;

   // Read desired channel
   const auto numberChannels = poseHeatMaps.getSize(0);
   const auto height         = poseHeatMaps.getSize(1);
   const auto width          = poseHeatMaps.getSize(2);
   const cv::Mat desiredChannelHeatMap(
       height,
       width,
       CV_32F,
       &poseHeatMaps
            .getPtr()[desiredChannel % numberChannels * height * width]);
   cv::Mat cp = desiredChannelHeatMap.clone();
   cp         = cp * 255;

   cv::Mat desiredChannelHeatMapUint8;
   cp.convertTo(desiredChannelHeatMapUint8, CV_8UC1);

   // Combining both images
   cv::applyColorMap(desiredChannelHeatMapUint8,
                     desiredChannelHeatMapUint8,
                     cv::COLORMAP_JET);

   cv::Mat out_im;
   cv::addWeighted(extract_image_from_net(datumsPtr),
                   0.1,
                   desiredChannelHeatMapUint8,
                   0.9,
                   0.,
                   out_im);
   return out_im;
}

// --------------------------------------------------- set-op-wrapper-struct-pos
//
static op::WrapperStructPose
set_op_wrapper_struct_pos(const pose_skeleton::OpenposeParams& p)
{
   static const op::String k_models_path
       = op::String(format("{}/models", multiview_openpose_path()).c_str());

   op::WrapperStructPose pose_ops;
   pose_ops.poseMode
       = p.pose_enabled ? op::PoseMode::Enabled : op::PoseMode::Disabled;
   pose_ops.netInputSize = op::Point(p.net_input_size.x, p.net_input_size.y);
   // pose_ops.netInputSize = op::Point(sz.x, sz.y);
   if(k_cuda_is_available) {
      pose_ops.gpuNumberStart
          = (p.gpu_start == 0) ? multiview_cuda_gpu0() : p.gpu_start;
      pose_ops.gpuNumber
          = (p.num_gpus == -1) ? multiview_cuda_num_gpus() : p.num_gpus;
      pose_ops.renderMode = op::RenderMode::Gpu;
   } else {
      pose_ops.gpuNumberStart = 0;
      pose_ops.gpuNumber      = 0;
      pose_ops.renderMode     = op::RenderMode::Cpu;
   }
   pose_ops.scalesNumber = p.scales_num;
   pose_ops.scaleGap     = p.scale_gap;
   pose_ops.renderMode   = render_mode_to_op(p.render_mode);
   pose_ops.poseModel    = pose_model_to_op(p.pose_model);
   pose_ops.blendOriginalFrame
       = (p.render_mode != pose_skeleton::RenderMode::NONE);
   pose_ops.alphaHeatMap = 1.0; // only render heat map
   pose_ops.modelFolder  = k_models_path;
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

// ---------------------------------------------------------------- unpack-poses
//
static void unpack_pose(const DistortedCamera* dcam_ptr, // could be nullptr
                        const int frame_no,
                        const int sensor_no,
                        const TDatums* datums,
                        const pose_skeleton::OpenposeParams& p,
                        const cv::Mat& im,
                        const int detection_index,
                        Skeleton2D& p2d) noexcept
{
   Expects(datums != nullptr && datums->size() > 0);

   const auto& poseKeypoints = datums->at(0)->poseKeypoints;
   const auto n_persons      = poseKeypoints.getSize(0);
   const auto n_parts        = poseKeypoints.getSize(1);
   const auto dim3           = poseKeypoints.getSize(2);

   Expects(dim3 == 3);
   Expects(n_persons >= 0 && unsigned(detection_index) < unsigned(n_persons));

   Skeleton2D::Params p2d_params;

   vector<Skeleton2D::Keypoint> kps;
   kps.reserve(size_t(n_parts));

   for(auto k = 0; k < n_parts; ++k) {
      auto pos = Vector2f{poseKeypoints[{detection_index, k, 0}],
                          poseKeypoints[{detection_index, k, 1}]};
      if(pos.x == 0.0f && pos.y == 0.0f) pos = Vector2f::nan();

      kps.emplace_back();
      kps.back().part  = get_kp_index(p.pose_model, k);
      kps.back().pos   = pos;
      kps.back().score = poseKeypoints[{detection_index, k, 2}];
   }

   p2d.init(
       p2d_params, frame_no, sensor_no, dcam_ptr, p.pose_model, std::move(kps));
}

static vector<shared_ptr<const Skeleton2D>>
unpack_poses(const DistortedCamera* dcam_ptr,
             const int frame_no,
             const int sensor_no,
             const TDatums* datums,
             const pose_skeleton::OpenposeParams& params,
             const cv::Mat& im) noexcept
{
   if(datums == nullptr or datums->size() == 0) return {};

   const auto& poseKeypoints = datums->at(0)->poseKeypoints;
   const auto n_persons      = poseKeypoints.getSize(0);

   vector<shared_ptr<const Skeleton2D>> ret;
   ret.reserve(size_t(n_persons));
   for(auto person = 0; person < n_persons; ++person) {
      if(!(person == 0 && sensor_no == 0)) continue;

      auto ptr = make_shared<Skeleton2D>();
      unpack_pose(
          dcam_ptr, frame_no, sensor_no, datums, params, im, person, *ptr);
      ret.emplace_back(std::move(ptr));
   }

   return ret;
}

// ------------------------------------------------------ OpenposeWrapper::Pimpl
//
struct OpenposeWrapper::Pimpl
{
 private:
   pose_skeleton::OpenposeParams last_params_;
   unsigned op_n_sensors_ = 0;
   bool is_first_         = true;

   // Not reentrant
   op::Wrapper& get_op_wrapper_() noexcept(false)
   {
      return op_wrapper_instance();
   }

 public:
   Pimpl()
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

   op::Wrapper& configured_op(const pose_skeleton::OpenposeParams& p,
                              const unsigned n_sensors) noexcept(false)
   {
      auto& wrapper = get_op_wrapper_();
      if(is_first_ or p != last_params_ or op_n_sensors_ != n_sensors)
         force_configure_op(p, n_sensors, is_first_);
      return wrapper;
   }

   void force_configure_op(const pose_skeleton::OpenposeParams& p,
                           const unsigned n_sensors,
                           const bool never_feedback = false) noexcept(false)
   {
      if(is_first_ or (p.feedback and !never_feedback))
         INFO(format("Configuring openpose:\n    n-sensors = {}\n{}",
                     n_sensors,
                     indent(p.to_string(), 4)));
      is_first_     = false;
      last_params_  = p;
      op_n_sensors_ = n_sensors;
      auto& op      = get_op_wrapper_();
      op.stop();
      if(p.single_threaded) {
         WARN(format("disabling multi-threading in openpose"));
         op.disableMultiThreading();
      }
      op.configure(set_op_wrapper_struct_pos(p));
      op.setDefaultMaxSizeQueues(n_sensors);
      op.start();
   }
};

// ------------------------------------------------------------- OpenposeWrapper

OpenposeWrapper::OpenposeWrapper()
    : pimpl_(new Pimpl)
{}

OpenposeWrapper::~OpenposeWrapper() = default;

// ------------------------------------------------------------- OpenposeWrapper
//
static std::mutex run_padlock; // Openpose is not reentrant

vector<vector<shared_ptr<const Skeleton2D>>>
OpenposeWrapper::run(const SceneDescription* scene_desc_ptr,
                     const int frame_no,
                     const int n_images,
                     std::function<const cv::Mat*(int i)> get_image,
                     const pose_skeleton::OpenposeParams& p,
                     const string_view outdir,
                     ParallelJobSet& pjobs,
                     std::function<bool()> is_cancelled) noexcept
{
   std::lock_guard lock(run_padlock);
   const bool is_serial = multiview_openpose_serial();

   // Grab the images
   vector<const cv::Mat*> images;
   images.reserve(size_t(n_images));
   for(auto i = 0; i < n_images; ++i) images.push_back(get_image(i));

   // Set up the return result
   vector<vector<shared_ptr<const Skeleton2D>>> poses((size_t(n_images)));

   // Where openpose puts results
   vector<TDatumsSP> processed((size_t(n_images)));

   auto unpack_pose_i_j = [&](unsigned i, unsigned j) {
      if(images.at(i) == nullptr) return;
      shared_ptr<Skeleton2D> p2d_ptr = make_shared<Skeleton2D>();

      const DistortedCamera* dcam_ptr = (scene_desc_ptr == nullptr)
                                            ? nullptr
                                            : &scene_desc_ptr->dcam(int(i));
      unpack_pose(dcam_ptr,
                  frame_no,
                  int(i),
                  processed.at(i).get(),
                  p,
                  *images.at(i),
                  int(j),
                  *p2d_ptr);
      poses[i][j] = std::move(p2d_ptr);
   };

   auto process_i = [&](unsigned i) {
      if(images.at(i) == nullptr) return;

      unsigned sz_i = 0;
      auto& x       = processed.at(i);
      if(x == nullptr) {
         WARN(format("failed to process sensor image {}", i));
      } else if(x->empty()) {
         WARN(format("empty data for sensor image {}", i));
      } else {
         const TDatums* datums = processed[i].get();
         Expects(datums != nullptr && datums->size() > 0);
         sz_i = unsigned(datums->at(0)->poseKeypoints.getSize(0));
      }

      poses[i].resize(sz_i);
      for(auto j = 0u; j < sz_i; ++j)
         pjobs.schedule([&unpack_pose_i_j, i, j]() { unpack_pose_i_j(i, j); });
   };

   try {
      auto& op_wrapper = pimpl_->configured_op(p, unsigned(n_images));

      auto push_i = [&](int i) {
         const auto im = images.at(size_t(i));
         if(im != nullptr) {
            const bool was_pushed = op_wrapper.waitAndPush(OP_CV2OPMAT(*im));
            if(!was_pushed)
               FATAL(format("failed to push image... not sure why."));
         }
      };

      auto pop_i = [&](int i) {
         if(images.at(size_t(i)) == nullptr) return; // nothing to do
         const bool was_popped = op_wrapper.waitAndPop(processed[size_t(i)]);
         if(!was_popped)
            FATAL(format("failed to pop results... not sure why."));
      };

      // Push all images
      if(!is_cancelled()) {
         for(auto i = 0; i < n_images; ++i) {
            push_i(i);
            if(is_serial) pop_i(i);
         }
      }

      if(!is_serial)
         for(auto i = 0; i < n_images; ++i) pop_i(i);

      // Okay, what results do we have??
      poses.resize(size_t(n_images));
      for(auto i = 0; i < n_images; ++i) process_i(unsigned(i));
      pjobs.execute();
   } catch(const std::exception& e) {
      FATAL(format("exception: {}", e.what()));
   }

   return poses;
}

} // namespace perceive

#endif
