#include "convert_to_lab.hpp"
#include "input_images.hpp"
#include "perceive/pipeline/detail/helpers.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "stdinc.hpp"

namespace perceive::pipeline::convert_to_lab
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
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result> Task::execute(const RunData& data,
                                       const Params& params,
                                       std::function<bool()> is_cancelled) const
    noexcept
{
   auto begin = tick();
   auto ret{make_shared<Result>()};

   ret->input_image_result = data.match_result<input_images::Result>(
       format("input_images[{}]", params.sensor_index));
   if(!ret->input_image_result) {
      LOG_ERR("Dependency not found: input_images");
      return nullptr;
   }

   if(is_cancelled()) return nullptr;

   auto initial_start        = tick();
   const cv::Mat& cv_rgb_img = ret->input_image_result->equal;
   ret->argb_image = make_shared<const ARGBImage>(cv_to_argb(cv_rgb_img));

   auto start = tick();

   try {
      // Runs cuda if available
      ret->lab_image = make_shared<const LABImage>(
          LAB_vec3f_im_to_LAB(argb_to_LAB_vec3f_im(*ret->argb_image)));
   } catch(const std::runtime_error& e) {
      LOG_ERR(format("Exception in convert_to_lab: {}", e.what()));
      return nullptr;
   }

   auto elapsed = tock(start);

   if(params.feedback) {
      if(!is_cancelled()) {
         print_timing(
             format("#{} Convert to LAB: {}s", params.sensor_index, elapsed));
      }
   }

   return ret;
}

} // namespace perceive::pipeline::convert_to_lab
