
#pragma once

#include "stdinc.hpp"

#include "load-image_example.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::stereobm::stereobm
{
struct Params
{
   bool operator==(const Params& o) const noexcept;
   bool operator!=(const Params& o) const noexcept { return !(*this == o); }
   Json::Value to_json() const noexcept;
   void read(const Json::Value&) noexcept(false);

   enum : int { CPU = 0, CUDA } implementation = CPU;
};

struct Result
{
   using load_result = shared_ptr<const load_image::Result>;
   array<load_result, 2> images;
   cv::Mat disparity;
};

class Task : public PipelineTask<Params, Result>
{
 protected:
   shared_ptr<const Result>
   execute(const RunData& data,
           const Params& params,
           std::function<bool()> is_cancelled) const noexcept override;
};

} // namespace perceive::stereobm::stereobm
