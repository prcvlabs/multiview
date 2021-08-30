
// This is an EXAMPLE

#include <opencv2/opencv.hpp>

#include "split-image_example.hpp"

namespace perceive::example::split_image
{
// ---------------------------------------------------------------------- Params
bool Params::operator==(const Params& o) const noexcept
{
   return is_left_image == o.is_left_image;
}

Json::Value Params::to_json() const noexcept
{
   auto root             = Json::Value{Json::objectValue};
   root["is_left_image"] = is_left_image;
   return root;
}

void Params::read(const Json::Value& o) noexcept(false)
{
   const string op = "reading 'split-image' params"s;
   is_left_image   = json_load_key<bool>(o, "is_left_image", op);
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   if(params.feedback) INFO(format("starting '{:s}'", taskname()));

   // ---- Load Dependencies ----
   auto src_result = data.match_result<load_image::Result>("input-image");

   // ---- Sanity Checks ----
   if(!src_result or src_result->image.empty()) {
      LOG_ERR(format("input image was nullptr"));
      return nullptr;
   }

   const auto& src = src_result->image;
   int w           = src.cols;
   int h           = src.rows;

   if(w < 2) {
      LOG_ERR(format("attempt to split image with only {} pixel-width", w));
      return nullptr;
   }

   if(w % 2 == 1) {
      WARN(format("split image task on image with odd pixel-width (width = {})",
                  w));
   }

   // ---- Create Result ----
   auto ret = make_shared<Result>();

   int l = 0, r = w / 2;
   if(!params.is_left_image) {
      l = w / 2;
      r = w;
   }

   ret->image = src(cv::Range(0, h), cv::Range(l, r));

   // ---- Feedback ----
   if(params.feedback) {
      const auto fname = format("{:s}/1_{:s}.png", data.outdir, taskname());
      INFO(format("writing '{:s}'", fname));
      cv::imwrite(fname, ret->image);
   }

   return ret;
}
} // namespace perceive::example::split_image
