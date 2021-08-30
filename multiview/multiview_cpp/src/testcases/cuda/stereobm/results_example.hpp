
#pragma once

#include "load-image_example.hpp"
#include "stereobm.hpp"

namespace perceive::stereobm
{
class FrameResults
{
 public:
   FrameResults(const std::string& left_image, const std::string& right_image);

   string outdir = "/tmp"; // Should be set (ulimately) by command-line params

   array<load_image::Task, 2> input_image;
   stereobm::Task stereobm_cpu;
   stereobm::Task stereobm_cuda;
};

} // namespace perceive::stereobm
