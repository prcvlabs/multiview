
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/cost-functions/pose-skeleton/hyperpose-helpers.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"

#include <opencv2/imgcodecs.hpp>

namespace perceive
{
#ifdef WITH_HYPERPOSE

// Runs pose detection on the image in MULTIVIEW_HYPERPOSE_TEST_IMAGE, with
// the model in MULTIVIEW_HYPERPOSE_DIR/models (see
// build/download-hyperpose-models.sh).
CATCH_TEST_CASE("Hyperpose", "[hyperpose]")
{
   CATCH_SECTION("hyperpose-image")
   {
      if(!cuda::cuda_is_available()) {
         WARN("no CUDA device found, skipping Hyperpose");
         return;
      }

      const char* image_fname = getenv("MULTIVIEW_HYPERPOSE_TEST_IMAGE");
      if(image_fname == nullptr) {
         WARN("MULTIVIEW_HYPERPOSE_TEST_IMAGE not set, skipping Hyperpose");
         return;
      }

      pose_skeleton::HyperposeParams p;
      p.model_name = "openpose-thin-V2-HW=368x432.onnx"s;
      const auto model_fname
          = format("{}/models/{}", multiview_hyperpose_path(), p.model_name);
      if(!is_regular_file(model_fname)) {
         WARN(format("model '{}' not found, skipping Hyperpose", model_fname));
         return;
      }

      const cv::Mat image = cv::imread(image_fname);
      CATCH_REQUIRE(!image.empty());

      HyperposeWrapper hyperpose;
      ParallelJobSet pjobs;
      const auto poses = hyperpose.run(
          nullptr,
          0,
          1,
          [&image](int) { return &image; },
          p,
          ""s,
          pjobs,
          []() { return false; });

      CATCH_REQUIRE(poses.size() == 1);
      INFO(format("hyperpose found {} skeletons in '{}'",
                  poses[0].size(),
                  image_fname));
      CATCH_REQUIRE(poses[0].size() > 0);
   }
}

#endif
} // namespace perceive
