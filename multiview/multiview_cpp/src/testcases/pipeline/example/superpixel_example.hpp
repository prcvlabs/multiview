
#pragma once

#include "stdinc.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/pipeline-task.hpp"

#include "split-image_example.hpp"

namespace perceive::example::superpixels
{
struct Params
{
   bool operator==(const Params& o) const noexcept;
   bool operator!=(const Params& o) const noexcept { return !(*this == o); }
   Json::Value to_json() const noexcept;
   void read(const Json::Value&) noexcept(false);

   enum : int { SLIC = 0, SLICO, MSLIC } algorithm = SLIC;
   int region_size                                 = 10;
   float ruler                                     = 10.0f;
   int iterations                                  = 10;
   bool feedback{false};
};

struct Result
{
   shared_ptr<const split_image::Result> image;
   cv::Mat labels; // type is: CV_32SC1 (i.e., int32)
   unsigned n_labels = 0;

   cv::Mat make_label_image() const noexcept;
};

class Task : public PipelineTask<Params, Result>
{
 protected:
   shared_ptr<const Result>
   execute(const RunData& data,
           const Params& params,
           std::function<bool()> is_cancelled) const noexcept override;
};

} // namespace perceive::example::superpixels
