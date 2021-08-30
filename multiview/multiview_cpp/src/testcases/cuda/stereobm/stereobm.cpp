
#include <algorithm>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef WITH_CUDA
#include <opencv2/cudastereo.hpp>
#undef WITH_CUDA
#endif // WITH_CUDA

#include "stereobm.hpp"

static void init_stereobm(cv::StereoBM& sbm)
{
   sbm.setBlockSize(25);
   sbm.setDisp12MaxDiff(126);
   sbm.setMinDisparity(1);
   sbm.setNumDisparities(96);
   sbm.setSpeckleRange(111);
   sbm.setSpeckleWindowSize(185);
   sbm.setPreFilterCap(63);
   sbm.setPreFilterSize(7);
   sbm.setPreFilterType(1);
   sbm.setSmallerBlockSize(15);
   sbm.setTextureThreshold(0);
   sbm.setUniquenessRatio(9);
}

namespace perceive::stereobm::stereobm
{
constexpr float max_disparity = 150.0f;

// ---------------------------------------------------------------------- Params
bool Params::operator==(const Params& o) const noexcept
{
   return o.implementation == implementation;
}

Json::Value Params::to_json() const noexcept
{
   auto root              = Json::Value{Json::objectValue};
   root["implementation"] = int(implementation);
   return root;
}

void Params::read(const Json::Value& o) noexcept(false)
{
   const string op = "reading 'StereoBM' params"s;
   //   implementation  =
   int code = json_load_key<int>(o, "implementation", op);
   switch(code) {
   case Params::CPU: implementation = Params::CPU; break;
   case Params::CUDA: implementation = Params::CUDA; break;
   default:
      throw std::runtime_error(format("Invalid implementation: {}", code));
   }
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   auto ret = make_shared<Result>();

   // ---- Load Dependencies ----
   for(unsigned i = 0; i < ret->images.size(); ++i) {
      ret->images[i]
          = data.match_result<load_image::Result>(format("load-image_[{}]", i));
   }

   // ---- Sanity Checks ----
   for(unsigned i = 0; i < ret->images.size(); ++i) {
      if(!ret->images[i] or ret->images[i]->image.empty()) {
         LOG_ERR(format("image {} was nullptr", i));
         return nullptr;
      }
   }

   // ---- Sanity Checks ----
   for(unsigned i = 0; i < ret->images.size(); ++i) {
      if(!ret->images[i] or ret->images[i]->image.empty()) {
         LOG_ERR(format("image {} was nullptr", i));
         return nullptr;
      }
   }

   const auto& left  = ret->images[0]->image;
   const auto& right = ret->images[1]->image;

   // ---- Create Result ----
   if(Params::CPU == params.implementation) {
      cv::Mat result;
      // INFO("Using cv::StereoBM");
      auto impl = cv::StereoBM::create();
      init_stereobm(*impl);
      auto duration = time_thunk([&]() { impl->compute(left, right, result); });
      // INFO(format(
      //     "{:s}: Calculated disparity map in {} seconds", taskname(),
      //     duration));

      Expects(CV_16S == result.type());
      Expects(left.cols == result.cols);
      Expects(left.rows == result.rows);

      ret->disparity = cv::Mat(left.rows, left.cols, CV_32F);
      for(int y = 0; y < result.rows; ++y) {
         int16_t* src_ptr = result.ptr<int16_t>(y);
         float* dst_ptr   = ret->disparity.ptr<float>(y);
         for(int x = 0; x < result.cols; ++x) {
            auto val   = std::fabs(float(*src_ptr++)) / 16.0f;
            *dst_ptr++ = val > max_disparity ? 0.0f : val;
         }
      }
   } else if(Params::CUDA == params.implementation) {
#ifdef WITH_CUDA
      INFO("Using cv::cuda::StereoBM");
      auto impl = cv::cuda::createStereoBM();
      init_stereobm(*impl);

      // Create GPU matrices
      cv::cuda::GpuMat gpu_left, gpu_right, gpu_result;
      gpu_left.upload(left);
      gpu_right.upload(right);

      // Do CUDA StereoBM
      auto duration = time_thunk(
          [&]() { impl->compute(gpu_left, gpu_right, gpu_result); });

      // Download the result
      // gpu_result.download(result);

      Expects(CV_8U == gpu_result.type());
      Expects(left.cols == gpu_result.cols);
      Expects(left.rows == gpu_result.rows);

      // gpu_result.download(result);
      cv::cuda::GpuMat gpu_result_float;
      gpu_result.convertTo(gpu_result_float, CV_32F);
      gpu_result_float.download(ret->disparity);
      INFO(format("{:s}: Calculated disparity map in {} seconds",
                  taskname(),
                  duration));
#else // not WITH_CUDA
      // FATAL("attempt to run CUDA code, when compiled without CUDA");
      WARN("skipping stereobm::cuda, because of some bug with recent compute "
           "capabilities");
#endif // WITH_CUDA
   } else {
      LOG_ERR(format("Unknown implementation: {}", params.implementation));
      return nullptr;
   }

   // ---- Save Result ----
   if(data.feedback) {
      const auto fname = format("{:s}/2_{:s}.png", data.outdir, taskname());
      // INFO(format("writing '{:s}'", fname));
      cv::imwrite(fname, ret->disparity);
   }

   return ret;
}

} // namespace perceive::stereobm::stereobm
