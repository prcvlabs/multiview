
#ifdef WITH_CUDA

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "stereobm/results_example.hpp"
#include <cstdlib>
#include <cuda_runtime.h>

namespace perceive
{
CATCH_TEST_CASE("StereoBM", "[stereo_bm]")
{
   if(false) {
      CATCH_SECTION("stereo_bm")
      {
         // INFO("Hello world!");

         char* pdata = std::getenv("PERCEIVE_DATA");
         if(!pdata) { throw std::runtime_error("PERCEIVE_DATA not set"); }

         std::string left_fname  = "left.png";
         std::string right_fname = "right.png";

         stereobm::FrameResults frame(
             format("{:s}/multiview/test/stereobm/{:s}", pdata, left_fname),
             format("{:s}/multiview/test/stereobm/{:s}", pdata, right_fname));

         // This forces the tasks onto the thread pool, leaving the
         // main thread (this thread) alone.
         std::atomic<int> counter = 0;

         schedule([&counter, &frame]() {
            frame.stereobm_cpu.result([&counter](auto) { ++counter; });
            frame.stereobm_cuda.result([&counter](auto) { ++counter; });
         });

         // Wait until jobs are done... we DON'T do this in the pipeline.
         // In Qt, we can use signals and slots to signal back to the main
         // thread.
         while(counter != 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
         }
      }
   }
}
} // namespace perceive

#endif // WITH_CUDA
