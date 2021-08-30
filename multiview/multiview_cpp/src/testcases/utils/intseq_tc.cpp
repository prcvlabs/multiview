
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/scene/scene-description.hpp"
#include "perceive/utils/base64.hpp"

namespace perceive
{
vector<unsigned>
target_frame_to_video_frame_mapping(const real video_fps,
                                    const real target_fps,
                                    const int n_video_frames) noexcept
{
   const auto xys = calc_frame_pairs(video_fps, target_fps, n_video_frames);
   vector<unsigned> ret;
   ret.reserve(xys.size());

   int next_frame = 0;
   for(const auto& xy : xys) {
      if(xy.y == next_frame) {
         ret.push_back(unsigned(xy.x));
         next_frame++;
      }
   }

   return ret;
}

static void run_sim(const real in_fps, const real target_fps, const int N)
{
   INFO(format("SIM: fps {} ==> {}, N = {}", in_fps, target_fps, N));

   const auto pairs = calc_frame_pairs(in_fps, target_fps, N);
   for(const auto ij : pairs)
      cout << format("{:2d}/{:2d} :: |{:8.5f} - {:8.5f}| = {:8.5f}",
                     ij.x,
                     ij.y,
                     ij.x / in_fps,
                     ij.y / target_fps,
                     std::fabs(ij.x / in_fps - ij.y / target_fps))
           << endl;

   {
      const auto ffs
          = target_frame_to_video_frame_mapping(in_fps, target_fps, N);
      int t_frame = 0;
      cout << "*" << endl;
      for(const auto o_frame : ffs) {
         cout << format("{:2d} => {:2d}", t_frame++, o_frame) << endl;
      }
   }
}

CATCH_TEST_CASE("Intseq", "[intseq]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("intseq")
   {
      // run_sim(15.0, 15.0, 20);
      // run_sim(15.0, 7.5, 20);
      // run_sim(15.0, 10.0, 20);
      // run_sim(15.0, 20.0, 20);
   }
}

} // namespace perceive
