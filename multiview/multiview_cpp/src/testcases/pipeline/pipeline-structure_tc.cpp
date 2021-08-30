
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "example/results_example.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/utils/threads.hpp"

namespace perceive
{
CATCH_TEST_CASE("PipelineStructure", "[pipeline_structure]")
{
   //
   // ----------------------------------------------------- pipeline-structure-1
   //
   CATCH_SECTION("pipeline-structure-1")
   {
      // Generate a test image, and save it to disk
      const string outdir     = "/tmp";
      const string test_fname = format("{:s}/test-input.png", outdir);
      ARGBImage im;
      im.resize(1600, 600);
      for(auto y = 0u; y < im.height; ++y)
         for(auto x = 0u; x < im.width; ++x)
            im(x, y) = hsv_to_rgb_uint32(
                (real(x) / im.width) * 360.0, 1.0 - (real(y) / im.height), 1.0);
      im.save(test_fname);

      example::FrameResults frame;
      example::load_image::Params p;
      p.filename = test_fname;
      frame.input_image.set_params(p);

      // This forces the tasks onto the thread pool, leaving the
      // main thread (this thread) alone.
      std::atomic<int> counter = 0;
      auto exec                = [&](int i) {
         frame.superpixels[size_t(i)].result([&counter](auto) { ++counter; });
      };

      for(auto i = 0; i < 2; ++i) schedule([&exec, i]() { exec(i); });

      // Wait until jobs are done... we DON'T do this in the pipeline.
      // In Qt, we can use signals and slots to signal back to the main thread.
      while(counter != 2) {
         std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
   }
}

} // namespace perceive
