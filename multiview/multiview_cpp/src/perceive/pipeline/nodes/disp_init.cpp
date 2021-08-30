
#include "disp_init.hpp"

#include "bcam_init.hpp"
#include "perceive/pipeline/detail/helpers.hpp"

namespace perceive::pipeline::disp_init
{
// ---------------------------------------------------------------------- Params
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, cam_num, true));
      m.push_back(MAKE_META(ThisParams, COMPATIBLE_OBJECT, disp_params, true));
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
   auto ret = make_shared<Result>();

   ret->bcam_result = data.match_result<bcam_init::Result>(
       format("bcam_init[{}]", params.cam_num));
   if(!ret->bcam_result) {
      LOG_ERR("Dependency not found: bcam_init");
      return nullptr;
   }

   auto& scene_desc{ret->scene_desc()};
   Expects(params.cam_num < unsigned(scene_desc.n_cameras()));
   auto& bcam_info{scene_desc.bcam_infos[params.cam_num]};
   Expects(2 == bcam_info.n_sensors());

   for(unsigned i{0}; i < 2; ++i) {
      auto s_ind  = scene_desc.sensor_lookup(int(params.cam_num), int(i));
      auto result = data.match_result<calc_rectified::Result>(
          format("calc_rectified[{}]", s_ind));
      if(!result) {
         LOG_ERR(format("Dependency not found; calc_rectified[{}]", s_ind));
         return nullptr;
      }
      ret->rectified_result.push_back(result);
   }

   if(is_cancelled()) return nullptr;

   const auto& bino_cam{ret->bcam_result->bcam};

   const unsigned n_disparities{1};
   ret->set_size(n_disparities);

   ParallelJobSet jobs;

   real s1 = 0.0;
   real s2 = 0.0;

   // Don't calculate point-clouds
   if(scene_desc.is_no_stereo()) {
      // Do nothing
      for(auto& pt : ret->point_cloud) pt = make_shared<PointCloud>();
   } else {
      auto calc = [&](int ind, bool l_to_r, auto disp_method) {
         ret->disparity_l_ref[size_t(ind)] = l_to_r;
         Disparity::calculate(params.disp_params,
                              l_to_r,
                              disp_method,
                              ret->rectified_result[0]->rectified,
                              ret->rectified_result[1]->rectified,
                              ret->disparity[size_t(ind)],
                              ret->disparity_confidence[size_t(ind)]);
      };

      auto make_disp_im = [&](unsigned ind) {
         ret->disparity_image[ind] = cv::Mat(
             ret->disparity[ind].rows, ret->disparity[ind].cols, CV_8UC1);
         double minVal;
         double maxVal;
         minMaxLoc(ret->disparity[ind], &minVal, &maxVal);
         minVal = 0.0;
         maxVal = 100.0;
         ret->disparity[ind].convertTo(
             ret->disparity_image[ind], CV_8UC1, 255.0 / (maxVal - minVal));
      };

      auto make_pt_cloud = [&](unsigned ind) {
         const auto l_to_r = ret->disparity_l_ref[ind];
         auto ptr          = make_shared<PointCloud>();
         ptr->init(ret->disparity[ind], l_to_r, bino_cam);
         ret->point_cloud[ind] = std::move(ptr);
      };

      s1 = time_thunk(
          [&]() { calc(0, true, params.disp_params.disparity_method); });

      s2 = time_thunk([&]() {
         for(auto i{0u}; i < ret->disparity.size(); ++i) {
            jobs.schedule([make_disp_im, i]() { make_disp_im(i); });
            jobs.schedule([make_pt_cloud, i]() { make_pt_cloud(i); });
         }
         jobs.execute();
      });
   }

   if(params.feedback) {
      auto out_disp = [&](unsigned ind) {
         if(!ret->disparity_image[ind].empty()) {
            string fname{format("{}/{:2d}_tw-04-disparity-image_{}.png",
                                params.out_dir,
                                params.cam_num,
                                ind)};
            cv::imwrite(fname, ret->disparity_image[ind]);
         }
      };

      for(auto i{0u}; i < ret->disparity.size(); ++i)
         jobs.schedule([out_disp, i]() { out_disp(i); });
      jobs.execute();

      print_timing(format("#{} Disparity: {}s", params.cam_num, s1));
      print_timing(format(
          "#{} Point Cloud + Disparity images: {}s", params.cam_num, s2));
   }

   return is_cancelled() ? nullptr : ret;
}
} // namespace perceive::pipeline::disp_init
