
#include "calc_rectified.hpp"

#include <opencv2/opencv.hpp>

#include "bcam_init.hpp"
#include "input_images.hpp"
#include "perceive/pipeline/detail/helpers.hpp"
#include "perceive/utils/cuda-spec.hpp"

namespace perceive::pipeline::calc_rectified
{
// ---------------------------------------------------------------------- Params
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, sensor_index, true));
      m.push_back(MAKE_META(ThisParams, BOOL, use_cuda_if_available, true));
      m.push_back({meta_type::STRING,
                   "interpolation_method"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      return std::any(str(o.interpolation_method));
                   },
                   [](void* ptr, const std::any& value) {
                      auto& o            = *reinterpret_cast<ThisParams*>(ptr);
                      const string& name = std::any_cast<const string>(value);
                      o.interpolation_method = to_interpolation_method(name);
                   }});
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

string str(Params::InterpolationMethod method) noexcept
{
   switch(method) {
   case Params::NEAREST: return "NEAREST"s;
   case Params::LINEAR: return "LINEAR"s;
   case Params::CUBIC: return "CUBIC"s;
   case Params::LANCZOS4: return "LANCZOS4"s;
   default: Expects(false); return ""s;
   }
}

Params::InterpolationMethod
to_interpolation_method(const string_view name) noexcept
{
   if(name == "NEAREST"s) return Params::NEAREST;
   if(name == "LINEAR"s) return Params::LINEAR;
   if(name == "CUBIC"s) return Params::CUBIC;
   if(name == "LANCZOS4"s) return Params::LANCZOS4;
   FATAL(format("bad interpolation method value: '{}'", name));
   return Params::NEAREST;
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret{make_shared<Result>()};

   ret->image_result = data.match_result<input_images::Result>(
       format("input_images[{}]", params.sensor_index));
   if(!ret->image_result) {
      LOG_ERR(format("input_images[{}] not found", params.sensor_index));
      return nullptr;
   }

   ret->mapping_result = data.match_result<get_xy_mapping::Result>(
       format("get_xy_mapping[{}]", params.sensor_index));
   if(!ret->mapping_result) {
      LOG_ERR(format("get_xy_mapping[{}]", params.sensor_index));
      return nullptr;
   }

   const auto& scene_desc = ret->scene_desc();
   auto n_cameras{scene_desc.n_cameras()};

   if(is_cancelled()) return nullptr;

   const auto now = tick();

   if(is_cancelled()) return nullptr;

   const cv::Mat& in = ret->image_result->equal;

   if(!in.empty()) {
      if(params.use_cuda_if_available and cuda::cuda_is_available()) {
#ifdef WITH_CUDA
         ret->rectified = cuda_remapper.remap(ret->rectified,
                                              ret->mapping_result->mapx_gpu,
                                              ret->mapping_result->mapy_gpu,
                                              int(params.interpolation_method));
#else
         LOG_ERR("Cuda is not available, rectified not done!");
         return nullptr;
#endif
      } else {
         cv::remap(in,
                   ret->rectified,
                   ret->mapping_result->mapx,
                   ret->mapping_result->mapy,
                   int(params.interpolation_method),
                   cv::BORDER_CONSTANT,
                   cv::Scalar(255, 255, 255));
      }
   }

   const auto elapsed = tock(now);

   if(params.feedback) {
      if(!is_cancelled() && !ret->rectified.empty()) {
         cv::imwrite(format("{}/{:2d}_tw-03-rectified.png",
                            params.out_dir,
                            params.sensor_index),
                     ret->rectified);
      }
      print_timing(format("#{} Rectify: {}s", params.sensor_index, elapsed));
   }

   return is_cancelled() ? nullptr : ret;
}
} // namespace perceive::pipeline::calc_rectified
