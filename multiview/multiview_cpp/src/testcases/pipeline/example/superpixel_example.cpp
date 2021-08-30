
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc/slic.hpp>

#include "superpixel_example.hpp"

namespace perceive::example::superpixels
{
// ---------------------------------------------------------------------- Params
bool Params::operator==(const Params& o) const noexcept
{
#define TEST(x) (x == o.x)
#define TEST_DIST(x) (float_is_same(x, o.x))
   return TEST(algorithm) and TEST(region_size) and TEST_DIST(ruler)
          and TEST(iterations);
#undef TEST
#undef TEST_DIST
}

Json::Value Params::to_json() const noexcept
{
   auto root           = Json::Value{Json::objectValue};
   root["algorithm"]   = int(algorithm);
   root["region_size"] = region_size;
   root["ruler"]       = real(ruler);
   root["iterations"]  = iterations;
   return root;
}

void Params::read(const Json::Value& o) noexcept(false)
{
   const string op = "reading 'superpixel' params"s;
   region_size     = json_load_key<int>(o, "region_size", op);
   ruler           = json_load_key<float>(o, "ruler", op);
   iterations      = json_load_key<int>(o, "iterations", op);

   int a = json_load_key<int>(o, "algorithm", op);
   switch(a) {
   case 0: algorithm = SLIC;
   case 1: algorithm = SLICO;
   case 2: algorithm = MSLIC;
   default:
      throw std::runtime_error(
          format("invalid algorithm constant '{}' while {:s}", a, op));
   }
}

// ---------------------------------------------------------------------- Result
cv::Mat Result::make_label_image() const noexcept
{
   if(labels.empty() or !image) return cv::Mat{};

   auto w = labels.cols;
   auto h = labels.rows;

   cv::Mat out = image->image.clone();

   Expects(w == out.cols);
   Expects(h == out.rows);
   Expects(labels.type() == CV_32SC1); // Blergh. CV_32SC1 is int32_t
   Expects(out.type() == CV_8UC3);     // Blergh. CV_8UC3 is cv::Vec3b

   // OpenCV sucks. This just grabs the label at position (x, y)
   auto label_at = [&](int x, int y) {
      if(x < 0 or x >= w) return -1;
      if(y < 0 or y >= h) return -1;
      return labels.at<int32_t>(y, x);
   };

   // Draw a border wherever two different labels meet.
   for(auto y = 0; y < h; ++y) {
      for(auto x = 0; x < w; ++x) {
         int label = label_at(x, y);
         for(const auto& dxy : eight_connected) {
            if(label != label_at(x + dxy.first, y + dxy.second)) {
               out.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // Draw
               break;
            }
         }
      }
   }

   return out;
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret = make_shared<Result>();

   // ---- Load Dependencies ----
   ret->image = data.match_result<split_image::Result>("split-image_");

   // ---- Sanity Checks ----
   if(!ret->image or ret->image->image.empty()) {
      LOG_ERR(format("split image was nullptr"));
      return nullptr;
   }

   const auto& src = ret->image->image;

   // ---- Create Result ----
   const auto algorithm = params.algorithm == Params::SLIC ? cv::ximgproc::SLIC
                          : params.algorithm == Params::SLICO
                              ? cv::ximgproc::SLICO
                              : cv::ximgproc::MSLIC;
   auto spixel          = cv::ximgproc::createSuperpixelSLIC(
       src, algorithm, params.region_size, params.ruler);

   spixel->iterate(params.iterations);

   // ---- Save Result ----
   ret->n_labels = unsigned(spixel->getNumberOfSuperpixels());
   spixel->getLabels(ret->labels);

   if(params.feedback) {
      const auto fname = format("{:s}/2_{:s}.png", data.outdir, taskname());
      INFO(format("writing '{:s}'", fname));
      cv::imwrite(fname, ret->make_label_image());
   }

   return ret;
}

} // namespace perceive::example::superpixels
