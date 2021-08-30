
#pragma once

#include "stdinc.hpp"

#include "floor_hist.hpp"
#include "movie-stats-task.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include "pose-skeleton-task.hpp"

namespace perceive::pipeline::localization
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"s};

   LocalizationData::Params localization_params;
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   Params p;

   shared_ptr<const load_scene_description::Result> load_scene_result;
   shared_ptr<const pose_skeleton_init::Result> pose_skeleton_result;
   shared_ptr<const floor_hist::Result> floor_hist_result;
   const PointCloud* get_point_cloud_for_bcam(int bcam_ind) const noexcept;

   int frame_no() const noexcept;

   FloatImage calc_motion_hist() const noexcept;
   FloatImage calc_prob_bg() const noexcept;
   std::tuple<FloatImage, FloatImage, vector<vector<Skeleton2DInfo>>>
   calc_prob_fg_openpose() const noexcept;
   FloatImage calc_prob_fg_3d() const noexcept;
   FloatImage calc_prob_fg() const noexcept;
   FloatImage calc_fowlkes_hist() const noexcept;

   LocalizationData data; // The final localization
};

class Task : public PipelineTask<Params, Result>
{
 public:
   CUSTOM_NEW_DELETE(Task)

 protected:
   shared_ptr<const Result> execute(const RunData& data,
                                    const Params& params,
                                    std::function<bool()> is_cancelled) const
       noexcept override;
};

}; // namespace perceive::pipeline::localization

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::localization::Params)
} // namespace perceive
