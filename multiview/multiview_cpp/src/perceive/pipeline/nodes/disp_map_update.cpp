
#include "disp_map_update.hpp"

#include "perceive/cost-functions/smooth-point-cloud.hpp"

namespace perceive::pipeline::disp_map_update
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
      m.push_back(MAKE_META(ThisParams, BOOL, apply_smoothing, true));
      m.push_back(MAKE_META(ThisParams, INT, smooth_polynomial_order, true));
      m.push_back(MAKE_META(ThisParams, REAL, smooth_search_radius, true));
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

   ret->bcam_result = data.match_result<bcam_init::Result>(
       format("bcam_init[{}]", params.cam_num));
   if(!ret->bcam_result) {
      LOG_ERR("Dependency not found: bcam_init");
      return nullptr;
   }

   const auto& scene_desc{ret->bcam_result->scene_desc()};

   ret->disp_result = data.match_result<disp_init::Result>(
       format("disp_init[{}]", params.cam_num));
   if(!ret->disp_result) {
      LOG_ERR("Dependency not found: disp_init");
      return nullptr;
   }

   auto s_ind{scene_desc.sensor_lookup(int(params.cam_num), 0)};
   ret->f2d_result
       = data.match_result<run_f2d::Result>(format("run_f2d[{}]", s_ind));
   if(!ret->f2d_result) {
      LOG_ERR("Dependency not found: run_f2d");
      return nullptr;
   }

   ret->slic_lookup_result = data.match_result<create_slic_lookup::Result>(
       format("create_slic_lookup[{}]", s_ind));
   if(!ret->slic_lookup_result) {
      LOG_ERR("Dependency not found: create_slic_lookup");
      return nullptr;
   }

   const auto& bcam{ret->bcam_result->bcam};
   const auto& disp{*ret->disp_result};
   const auto& f2d{ret->f2d_result->f2d};
   const auto& lookup{ret->slic_lookup_result->slic_lookup};

   Expects(disp.disparity.size() > 0);
   const bool is_empty = disp.disparity[0].empty();

   if(is_empty) {
      ret->disparity       = disp.disparity[0];
      ret->ref_disparity   = scene_desc.ref_disparities.at(params.cam_num);
      ret->point_cloud     = disp.point_cloud[0];
      ret->ref_point_cloud = ret->point_cloud;

   } else {
      ret->disparity = disp.disparity[0];

      if(params.cam_num < scene_desc.ref_disparities.size()) {
         ret->ref_disparity = scene_desc.ref_disparities.at(params.cam_num);

         auto ptr = make_shared<PointCloud>();
         ptr->init(ret->ref_disparity, true, bcam);
         ret->ref_point_cloud = std::move(ptr);
      } else {
         // nothing to combine
         ret->point_cloud = disp.point_cloud[0];
         return ret;
      }
      const auto w = ret->ref_disparity.cols;
      const auto h = ret->ref_disparity.rows;

      for(auto y = 0; y < h; ++y) {
         float* src = ret->ref_disparity.ptr<float>(y);
         float* dst = ret->disparity.ptr<float>(y);
         for(auto x = 0; x < w; ++x) {
            Expects(lookup.in_bounds(x, y));
            auto label = lookup(x, y);
            if(!std::isfinite(src[x])) continue;
            if(label < 0) continue;
            const auto& info = f2d.slic_info[size_t(label)];

            if(!info.still_cuttoff or !std::isfinite(dst[x])
               or (dst[x] <= 0.0f)) {
               dst[x] = src[x];
            }
         }
      }

      {
         auto ptr = make_shared<PointCloud>();
         ptr->init(ret->disparity, true, bcam);
         ret->point_cloud = std::move(ptr);
      }

      if(params.apply_smoothing) {
         SmoothPointCloudParams opts;
         opts.polynomial_order = params.smooth_polynomial_order;
         opts.search_radius    = params.smooth_search_radius;

         auto ptr = make_shared<PointCloud>(
             smooth_pointcloud(*ret->point_cloud, opts));
         ret->point_cloud = std::move(ptr);
      }
   }

   return ret;
}
} // namespace perceive::pipeline::disp_map_update
