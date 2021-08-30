
#include "slic_3d_init.hpp"

#include "bcam_init.hpp"
#include "perceive/cost-functions/slic-3d/slic-3d.hpp"
#include "perceive/pipeline/detail/helpers.hpp"
#include "run_f2d.hpp"

namespace perceive::pipeline::slic_3d_init
{
// ------------------------------------------------------------------- meta data

const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, cam_num, true));
      m.push_back(MAKE_META(ThisParams, COMPATIBLE_OBJECT, slic3d, true));
      return m;
   };
#undef ThisParams
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result> Task::execute(const RunData& data,
                                       const Params& params,
                                       std::function<bool()> is_cancelled) const
    noexcept
{
   auto ret{make_shared<Result>()};

   ret->bcam_result = data.match_result<bcam_init::Result>(
       format("bcam_init[{}]", params.cam_num));
   if(!ret->bcam_result) {
      LOG_ERR("Dependency not found: bcam_init");
      return nullptr;
   }

   for(unsigned sensor{0}; sensor < 2; ++sensor) {
      auto f2d_result = data.match_result<run_f2d::Result>(
          format("run_f2d[{}]", params.cam_num));
      if(!f2d_result) {
         LOG_ERR("Dependency not found: run_f2d");
         return nullptr;
      }
      ret->f2d_result.push_back(f2d_result);
   }

   const auto& scene_desc{ret->scene_desc()};
   auto n_cameras{scene_desc.n_cameras()};

   const auto& bcam{ret->bcam_result->bcam};
   const auto& et0{scene_desc.cam_transforms[params.cam_num]};
   DistortedCamera cam0, cam1;
   std::tie(cam0, cam1)
       = make_distorted_camera_pair(bcam, et0, bcam.w(), bcam.h());
   const auto& f2d0{ret->f2d_result[0]->f2d};
   const auto& f2d1{ret->f2d_result[1]->f2d};

   real s = 0.0;
   if(!f2d1.is_empty) {
      const bool left_is_ref{true};
      const auto& np3s{scene_desc.scene_info.known_planes};
      vector<Plane> p3s(np3s.size());
      std::transform(cbegin(np3s), cend(np3s), begin(p3s), [](auto x) {
         return x.second;
      });

      s = time_thunk([&]() {
         ret->plane_result[params.cam_num] = set_p3s_to_slic(params.slic3d,
                                                             params.cam_num,
                                                             cam0,
                                                             cam1,
                                                             f2d0,
                                                             f2d1,
                                                             left_is_ref,
                                                             p3s);
      });
   }

   if(params.feedback) {
      print_timing(format("#{} slic-3d-init: {}s", params.cam_num, s));
   }

   return ret;
}
} // namespace perceive::pipeline::slic_3d_init
