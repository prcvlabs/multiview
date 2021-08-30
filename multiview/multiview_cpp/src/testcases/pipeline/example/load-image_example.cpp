
// This is an EXAMPLE

#include <opencv2/opencv.hpp>

#include "load-image_example.hpp"

namespace perceive::example::load_image
{
// ---------------------------------------------------------------------- Params
bool Params::operator==(const Params& o) const noexcept
{
   return filename == o.filename;
}

Json::Value Params::to_json() const noexcept
{
   auto root        = Json::Value{Json::objectValue};
   root["filename"] = filename;
   return root;
}

void Params::read(const Json::Value& o) noexcept(false)
{
   const string op = "reading 'load-image' params"s;
   filename        = json_load_key<string>(o, "filename", op);
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret = make_shared<Result>();

   ret->image = cv::imread(params.filename, cv::IMREAD_COLOR);
   if(ret->image.empty()) {
      LOG_ERR(format("failed to load image '{:s}'", params.filename));
      ret = nullptr;
   }

   if(data.feedback) {
      auto fname = format("{:s}/0_{:s}.png", data.outdir, taskname());
      // INFO(format("loaded '{:s}', saving to '{:s}'", params.filename, fname));
      cv::imwrite(fname, ret->image);
   }

   return ret;
}
} // namespace perceive::example::load_image
