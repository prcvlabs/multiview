
#include "copy_sensor_images.hpp"

#include "perceive/graphics/hist-equalize.hpp"
#include "perceive/movie/debug-movie.hpp"
#include "perceive/utils/file-system.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace perceive::pipeline::copy_sensor_images
{
// ---------------------------------------------------------------------- Params
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, frame_num, true));
      m.push_back(MAKE_META(ThisParams, BOOL, export_frames, false));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret{make_shared<Result>()};
   ret->scene_result = data.match_result<load_scene_description::Result>(
       "load_scene_description");
   if(!ret->scene_result) {
      LOG_ERR("Dependency not found: load_scene_description");
      return nullptr;
   }

   const auto& scene_desc{ret->scene_desc()};
   ret->images = scene_desc.get_images_for_frame(int(params.frame_num));

   //
   if(params.export_frames) {
      const string frames_dir = format("{}/frames", params.out_dir);

      // Make sure the directory exists, and is empty
      if(!is_directory(frames_dir)) mkdir(frames_dir);
      if(!is_directory(frames_dir))
         FATAL(format("failed to create frames-directory: '{}'", frames_dir));

      const int sz = int(ret->images.raw_images.size());
      for(auto i = 0; i < sz; ++i) {
         const int n = scene_desc.n_sensors_for(i);
         for(auto j = 0; j < n; ++j) {
            const int sensor_id = scene_desc.sensor_lookup(i, j);
            cv::imwrite(
                movie::make_cam_fname(frames_dir, int(params.frame_num), i, j),
                ret->images.sensor_images[size_t(sensor_id)]);
         }
      }
   }

   return ret;
}
} // namespace perceive::pipeline::copy_sensor_images
