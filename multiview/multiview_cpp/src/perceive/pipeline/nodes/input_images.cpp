
#include "input_images.hpp"

#include "perceive/graphics/hist-equalize.hpp"
#include "perceive/pipeline/detail/helpers.hpp"

namespace perceive::pipeline::input_images
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
      m.push_back(MAKE_META(ThisParams, BOOL, raw_image_equalize, true));
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
   Expects(is_cancelled);

   auto ret{make_shared<Result>()};

   ret->copy_image_result
       = data.match_result<copy_sensor_images::Result>("copy_sensor_images");
   if(!ret->copy_image_result) {
      LOG_ERR("Dependency not found: copy_sensor_images");
      return nullptr;
   }

   const bool is_loaded = ret->copy_image_result->images.frame_loaded;
   if(!is_loaded) {
      WARN(format("FRAME was not loaded, input-images[{}], frame={}",
                  params.sensor_index,
                  ret->copy_image_result->images.frame_no));
   }

   unsigned sensor_index{params.sensor_index};

   const auto& scene_desc{ret->scene_desc()};

   if(is_cancelled()) return nullptr;

   if(is_cancelled()) return nullptr;
   auto s = time_thunk([&]() {
      Expects(sensor_index >= 0);
      Expects(static_cast<int>(sensor_index) < scene_desc.n_sensors());

      if(is_cancelled()) return;

      ret->raw
          = ret->copy_image_result->images.sensor_images[params.sensor_index];

      if(is_cancelled()) return;

      if(params.raw_image_equalize && !ret->raw.empty())
         hist_equalize(ret->raw, ret->equal);
      else
         ret->equal = ret->raw;

      if(is_cancelled()) return;
      if(!ret->equal.empty())
         cv::cvtColor(ret->equal, ret->grey, cv::COLOR_BGR2GRAY);
   });

   if(is_cancelled()) return nullptr;

   if(params.feedback) {
      ParallelJobSet jobs;
      auto schedule_im = [&](const std::string& fname, const cv::Mat& im) {
         jobs.schedule([&, fname]() {
            if(is_cancelled() || im.empty()) return;
            cv::imwrite(format("{}/{}", params.out_dir, fname), im);
         });
      };

      schedule_im(format("tw-01-dist_im_{}.png", sensor_index), ret->raw);
      schedule_im(format("tw-02-equal_im_{}.png", sensor_index), ret->equal);
      schedule_im(format("tw-03-grey_im_{}.png", sensor_index), ret->grey);

      jobs.execute();
      print_timing(format("#{} Input images: {}s", params.sensor_index, s));
   }

   return ret;
}

} // namespace perceive::pipeline::input_images
