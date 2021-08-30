
#include "localization-task.hpp"

#include "perceive/pipeline/detail/helpers.hpp"

namespace perceive::pipeline::localization
{
// ---------------------------------------------------------------------- Params
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(
          MAKE_META(ThisParams, COMPATIBLE_OBJECT, localization_params, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// ---------------------------------------------------- get_point_cloud_for_bcam
//
const PointCloud* Result::get_point_cloud_for_bcam(int bcam_ind) const noexcept
{
   const auto& disps = floor_hist_result->disp_result;
   if(unsigned(bcam_ind) >= disps.size()) {
      LOG_ERR(format("bcam_ind = {}, but disps.size() = {}. (ret={:p})",
                     bcam_ind,
                     disps.size(),
                     reinterpret_cast<const void*>(floor_hist_result.get())));
      Expects(false);
   }
   return disps[size_t(bcam_ind)]->point_cloud.get();
}

int Result::frame_no() const noexcept
{
   if(!pose_skeleton_result) return -1;
   if(!pose_skeleton_result->copy_images_result) return -1;
   return pose_skeleton_result->copy_images_result->images.frame_no;
}

// ------------------------------------------------------------ Calc Motion Hist
//
FloatImage Result::calc_motion_hist() const noexcept
{
   const auto& p    = this->p.localization_params;
   const auto& hist = floor_hist_result->hist.hist;
   // const auto& stats = movie_stats_result->stats;
   return make_motion_histogram(hist, p.use_median, p.n_deviations);
}

// ---------------------------------------------------------------- Calc Prob BG
//
FloatImage Result::calc_prob_bg() const noexcept
{
   const auto& p   = this->p.localization_params;
   const auto& hp  = floor_hist_result->hist_params;
   const auto hist = calc_motion_hist();
   // const auto& stats = movie_stats_result->stats;
   return make_bg_histogram(hist, p, real(hp.hist_sz));
}

// ------------------------------------------------------- Calc Prob FG Openpose
//
std::tuple<FloatImage, FloatImage, vector<vector<Skeleton2DInfo>>>
Result::calc_prob_fg_openpose() const noexcept
{
   const auto& scene_desc   = pose_skeleton_result->scene_desc();
   const auto& p            = this->p.localization_params;
   const auto& hist         = floor_hist_result->hist;
   const auto& openpose_ret = pose_skeleton_result->op;
   const auto fg0           = calc_prob_fg_3d();
   const auto ref_num       = 0; // '0' is reference sensor
   auto get_pcloud          = [&](int sensor_no) {
      const auto xy = scene_desc.bcam_lookup(sensor_no);
      return (xy.y == ref_num) ? get_point_cloud_for_bcam(xy.x) : nullptr;
   };
   auto get_f2d = [&](int sensor_no) {
      return &(floor_hist_result->f2d_result.at(size_t(sensor_no))->f2d);
   };
   auto get_bcam = [&](int cam_no) -> const BinocularCamera* {
      auto bcam_ptr = floor_hist_result->bcam_result(cam_no);
      Expects(bcam_ptr != nullptr);
      return &(bcam_ptr->bcam);
   };
   return make_fg_openpose_histogram(
       scene_desc, openpose_ret, hist, get_pcloud, get_f2d, get_bcam, fg0, p);
}

// ------------------------------------------------------------- Calc Prob FG 3d
//
FloatImage Result::calc_prob_fg_3d() const noexcept
{
   const auto& p   = this->p.localization_params;
   const auto& hp  = floor_hist_result->hist_params;
   const auto hist = calc_motion_hist();
   // const auto& stats = movie_stats_result->stats;
   return make_fg_3d_histogram(hist, p, real(hp.hist_sz));
}

// ---------------------------------------------------------------- Calc Prob FG
//
FloatImage Result::calc_prob_fg() const noexcept
{
   const auto fg0                  = calc_prob_fg_3d();
   const auto [fg1, heights, p3ds] = calc_prob_fg_openpose();
   Expects(fg0.width == fg1.width and fg0.height == fg1.height);
   const auto& p = this->p.localization_params;
   return make_fg_histogram(fg0, fg1, p);
}

// ------------------------------------------------------------ Calc Motion Hist
//
FloatImage Result::calc_fowlkes_hist() const noexcept
{
   const auto& p  = this->p.localization_params;
   const auto& hp = floor_hist_result->hist_params;
   return make_fowlkes_histogram(
       calc_prob_bg(), calc_prob_fg(), p, real(hp.hist_sz));
}

// --------------------------------------------------------------------- Execute
//
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   if(is_cancelled()) return nullptr;
   auto ret{make_shared<Result>()};

   ret->p = params;

#define MATCH_DEPENDENCY(x, y)                   \
   {                                             \
      ret->x = data.match_result<y::Result>(#y); \
      if(!ret->x) {                              \
         LOG_ERR("Can't find dependency: " #y);  \
         return nullptr;                         \
      }                                          \
   }

   MATCH_DEPENDENCY(load_scene_result, load_scene_description);
   MATCH_DEPENDENCY(pose_skeleton_result, pose_skeleton_init);
   // MATCH_DEPENDENCY(movie_stats_result, movie_stats);
   MATCH_DEPENDENCY(floor_hist_result, floor_hist);
#undef MATCH_DEPENDENCY

   const auto& scene_desc   = ret->pose_skeleton_result->scene_desc();
   const auto& p            = params.localization_params;
   const auto& hist_ret     = ret->floor_hist_result;
   const auto& openpose_ret = ret->pose_skeleton_result->op;
   const auto& f2d_vec_ret  = hist_ret->f2d_result;
   if(is_cancelled()) return nullptr;

   if(is_cancelled()) return nullptr;

   const bool is_no_stereo = scene_desc.is_no_stereo();

   auto get_pcloud = [&](int sensor_no) -> const PointCloud* {
      if(is_no_stereo) return nullptr;
      const int ref_num = 0; // left sensor always reference
      const auto xy     = scene_desc.bcam_lookup(sensor_no);
      return (xy.y == ref_num) ? ret->get_point_cloud_for_bcam(xy.x) : nullptr;
   };

   auto get_f2d = [&](int sensor_no) -> const ImageFeatures2d* {
      Expects(size_t(sensor_no) < f2d_vec_ret.size());
      return &f2d_vec_ret[size_t(sensor_no)]->f2d;
   };

   auto get_bcam = [&](int cam_no) -> const BinocularCamera* {
      auto cam_ptr = hist_ret->bcam_result(cam_no);
      Expects(cam_ptr != nullptr);
      return &(cam_ptr->bcam);
   };

   const auto now        = tick();
   ret->data             = LocalizationData::calculate(scene_desc,
                                           &openpose_ret,
                                           hist_ret->hist,
                                           get_pcloud,
                                           get_f2d,
                                           get_bcam,
                                           p,
                                           params.feedback,
                                           ret->frame_no());
   const auto elapsed_ms = tock(now);

   if(is_cancelled()) return nullptr;

   if(params.feedback) print_timing(format("localization: {}s", elapsed_ms));

   if(is_cancelled()) { return nullptr; }

   return ret;
}

} // namespace perceive::pipeline::localization
