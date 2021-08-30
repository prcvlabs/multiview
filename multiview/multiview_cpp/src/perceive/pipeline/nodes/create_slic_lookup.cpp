
#include "create_slic_lookup.hpp"

#include <opencv2/opencv.hpp>

#include "input_images.hpp"
#include "perceive/pipeline/detail/helpers.hpp"
#include "run_f2d.hpp"

namespace perceive::pipeline::create_slic_lookup
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

// ---------------------------------------------------------------------
// Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret{make_shared<Result>()};

   ret->mapping_result = data.match_result<get_xy_mapping::Result>(
       format("get_xy_mapping[{}]", params.sensor_index));
   if(!ret->mapping_result) {
      LOG_ERR(format("get_xy_mapping[{}] not found", params.sensor_index));
      return nullptr;
   }

   ret->f2d_result = data.match_result<run_f2d::Result>(
       format("run_f2d[{}]", params.sensor_index));
   if(!ret->f2d_result) {
      LOG_ERR(format("run_f2d[{}] not found", params.sensor_index));
      return nullptr;
   }

   auto s{time_thunk([&]() {
      if(ret->f2d_result->f2d.is_empty) return;

      const auto& mapx{ret->mapping_result->mapx};
      const auto& mapy{ret->mapping_result->mapy};
      const auto w{mapx.cols};
      const auto h{mapx.rows};
      const auto& src{ret->f2d_result->f2d.slic_labels};
      IntImage& lookup{ret->slic_lookup};
      lookup.resize(w, h);
      lookup.fill(-1);

      auto find_label = [&](int x, int y) {
         auto p = to_pt2(Vector2f(mapx.at<float>(y, x), mapy.at<float>(y, x)));
         // Expects(src.in_bounds(p));
         return src.in_bounds(p) ? src(p) : -1;
      };

      for(auto y = 0; y < h; ++y)
         for(auto x = 0; x < w; ++x) lookup(x, y) = find_label(x, y);
   })};

   if(params.feedback) {
      print_timing(format(
          "#{} Calculate rectified slic labels: {}s", params.sensor_index, s));
   }

   return is_cancelled() ? nullptr : ret;
}
} // namespace perceive::pipeline::create_slic_lookup
