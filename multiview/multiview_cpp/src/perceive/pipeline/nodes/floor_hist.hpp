
#pragma once

#include "stdinc.hpp"

#include "convert_to_lab.hpp"
#include "create_slic_lookup.hpp"
#include "disp_map_update.hpp"
#include "run_f2d.hpp"

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::pipeline::floor_hist
{
struct Params final : public MetaCompatible
{
   CUSTOM_NEW_DELETE(Params)

   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};
   std::string out_dir{"/tmp"};

   FloorHistogram::Params hist_params;
};

struct Result
{
   CUSTOM_NEW_DELETE(Result)

   std::vector<shared_ptr<const convert_to_lab::Result>> lab_result;
   std::vector<shared_ptr<const run_f2d::Result>> f2d_result;
   std::vector<shared_ptr<const create_slic_lookup::Result>> slic_lookup_result;

   std::vector<shared_ptr<const disp_map_update::Result>> disp_result;

   const bcam_init::Result* bcam_result(const int cam_index) const noexcept
   {
      return (size_t(cam_index) < disp_result.size())
                 ? disp_result[size_t(cam_index)]->bcam_result.get()
                 : nullptr;
   }

   const calc_rectified::Result*
   rect_result(const int cam_index, const int sensor_num) const noexcept
   {
      if(size_t(cam_index) >= disp_result.size()) return nullptr;
      const auto& vec
          = disp_result[size_t(cam_index)]->disp_result->rectified_result;
      if(size_t(sensor_num) >= vec.size()) return nullptr;
      return vec[size_t(sensor_num)].get();
   }

   FloorHistogram::Params hist_params;
   FloorHistogram hist;
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

}; // namespace perceive::pipeline::floor_hist

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::floor_hist::Params)
} // namespace perceive
