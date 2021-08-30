
#include "calc_calibration_mask.hpp"

#include <opencv2/opencv.hpp>

#include "bcam_init.hpp"
#include "input_images.hpp"
#include "perceive/pipeline/detail/helpers.hpp"

namespace perceive::pipeline::calc_calibration_mask
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

// bool Params::operator==(const Params& o) const noexcept
// {
//    return sensor_index == o.sensor_index;
// }

// Json::Value Params::to_json() const noexcept
// {
//    auto root{Json::Value{Json::objectValue}};
//    root["sensor_index"] = sensor_index;
//    return root;
// }

// void Params::read(const Json::Value& o) noexcept(false)
// {
//    const string op{"reading 'calc_calibration_mask' params"s};
//    json_try_load_key(sensor_index, o, "sensor_index", op);

//    Expects(test_json_serialization(*this));
// }

// ---------------------------------------------------------------------
// Execute
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

   const auto& scene_desc{ret->scene_desc()};

   if(is_cancelled()) return nullptr;

   auto s{time_thunk([&]() {
      const DistortionModel& M = scene_desc.model(int(params.sensor_index));
      if(is_cancelled() or params.feedback) return;
      auto& im{ret->calib_mask};
      const auto& hull{M.calib_hull()};
      im.resize(ret->image_result->equal.cols,
                ret->image_result->equal.rows); // w, h, w
      for(auto y{0u}; y < im.height; ++y) {
         if((y % 20 == 0) and is_cancelled()) return;
         for(auto x = 0u; x < im.width; ++x) {
            auto X   = M.working_to_calib_format(Vector2(x, y));
            im(x, y) = point_in_polygon(X, cbegin(hull), cend(hull));
         }
      }
   })};

   if(params.feedback) {
      print_timing(format(
          "#{} Calculate calibration mask: {}s", params.sensor_index, s));
   }

   return is_cancelled() ? nullptr : ret;
}
} // namespace perceive::pipeline::calc_calibration_mask
