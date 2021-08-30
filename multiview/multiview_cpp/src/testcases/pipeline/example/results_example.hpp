
#pragma once

#include "load-image_example.hpp"
#include "split-image_example.hpp"
#include "superpixel_example.hpp"

namespace perceive::example
{
class FrameResults
{
 public:
   FrameResults();

   string outdir = "/tmp"; // Should be set (ulimately) by command-line params

   load_image::Task input_image;
   array<split_image::Task, 2> split_image;
   array<superpixels::Task, 2> superpixels;
};

} // namespace perceive::example
