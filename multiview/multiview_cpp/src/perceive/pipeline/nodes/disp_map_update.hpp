
#pragma once

#include "stdinc.hpp"

#include "bcam_init.hpp"
#include "create_slic_lookup.hpp"
#include "disp_init.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"
#include "run_f2d.hpp"

namespace perceive::pipeline::disp_map_update
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned cam_num{0};
   bool apply_smoothing{false};
   int smooth_polynomial_order{3};
   real smooth_search_radius{0.15};
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   shared_ptr<const bcam_init::Result> bcam_result;
   shared_ptr<const disp_init::Result> disp_result;
   shared_ptr<const run_f2d::Result> f2d_result;
   shared_ptr<const create_slic_lookup::Result> slic_lookup_result;

   cv::Mat ref_disparity;
   cv::Mat disparity;
   shared_ptr<const PointCloud> ref_point_cloud;
   shared_ptr<const PointCloud> point_cloud;

   const SceneDescription& scene_desc() const
   {
      return disp_result->scene_desc();
   }
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

}; // namespace perceive::pipeline::disp_map_update

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::disp_map_update::Params)
} // namespace perceive
