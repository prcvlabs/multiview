
#pragma once

#include "stdinc.hpp"

#include "bcam_init.hpp"
#include "calc_rectified.hpp"
#include "perceive/cost-functions/disparity/disparity.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline::disp_init
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   unsigned cam_num{0};
   Disparity::Params disp_params;
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   Result(int sz = 3) { set_size(sz); }

   shared_ptr<const bcam_init::Result> bcam_result;
   vector<shared_ptr<const calc_rectified::Result>> rectified_result;

   vector<cv::Mat> disparity;
   vector<bool> disparity_l_ref;
   vector<cv::Mat> disparity_image;
   vector<cv::Mat> disparity_confidence;
   vector<shared_ptr<const PointCloud>> point_cloud;

   void set_size(int i_sz)
   {
      const size_t sz = size_t(i_sz);
      disparity.resize(sz);
      disparity_l_ref.resize(sz, true);
      disparity_image.resize(sz);
      disparity_confidence.resize(sz);
      point_cloud.resize(sz);
   }

   const SceneDescription& scene_desc() const
   {
      return bcam_result->scene_desc();
   }
};

class Task : public PipelineTask<Params, Result>
{
 public:
   CUSTOM_NEW_DELETE(Task)

 protected:
   shared_ptr<const Result>
   execute(const RunData& data,
           const Params& params,
           std::function<bool()> is_cancelled) const noexcept override;
};

}; // namespace perceive::pipeline::disp_init

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::disp_init::Params)
} // namespace perceive
