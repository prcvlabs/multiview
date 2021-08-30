
#pragma once

#include "stdinc.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

namespace perceive::stereobm::load_image
{
struct Params
{
   bool operator==(const Params& o) const noexcept;
   bool operator!=(const Params& o) const noexcept { return !(*this == o); }
   Json::Value to_json() const noexcept;
   void read(const Json::Value&) noexcept(false);

   string filename = ""s;
};

struct Result
{
   cv::Mat image;
};

class Task : public PipelineTask<Params, Result>
{
 protected:
   shared_ptr<const Result>
   execute(const RunData& data,
           const Params& params,
           std::function<bool()> is_cancelled) const noexcept override;
};

}; // namespace perceive::stereobm::load_image
