
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/cost-functions/pose-skeleton/pose-skeleton-exec.hpp"
#include "perceive/geometry/skeleton/p2d-affinity.hpp"

namespace perceive
{
static string pose_json_sample = R"V0G0N(
{
   "model":      "BODY_25",
   "sensor-no":  0,
   "theta":      2.265183,
   "score":      0.647702,    
   "keypoints": [[0, 521.609, 327.842, 0.925759],
        [1, 512.409, 344.229, 0.890728],
        [2, 486.803, 344.218, 0.882927],
        [3, 466.714, 373.428, 0.809998],
        [4, 448.447, 399.073, 0.835346],
        [5, 536.26, 344.2, 0.89116],
        [6, 548.929, 386.209, 0.882668],
        [7, 548.937, 424.694, 0.831504],
        [8, 501.443, 430.117, 0.806641],
        [9, 483.193, 430.07, 0.760873],
        [10, 490.503, 488.673, 0.831889],
        [11, 494.19, 563.667, 0.730961],
        [12, 517.88, 430.151, 0.814154],
        [13, 490.515, 474.048, 0.530531],
        [14, 490.568, 548.982, 0.165301],
        [15, 517.876, 325.867, 0.914163],
        [16, 525.242, 322.269, 0.817905],
        [17, 501.455, 325.913, 0.876867],
        [18, 0, 0, 0],
        [19, 0, 0, 0],
        [20, 0, 0, 0],
        [21, 492.323, 556.288, 0.0850829],
        [22, 505.134, 602.013, 0.684841],
        [23, 497.787, 594.75, 0.624274],
        [24, 495.976, 572.779, 0.598968]],
   "rays":   []
}
)V0G0N"s;

CATCH_TEST_CASE("OpenPose", "[openpose]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("openpose")
   {
      // const auto json_dat = parse_json(pose_json_sample);

      // using Pose = Skeleton2D;
      // Pose p1;
      // try {
      //    p1.read(json_dat);
      // } catch(std::exception& e) {
      //    FATAL(format("read error: {}", e.what()));
      // }

      // const string p1_str = p1.to_json_str();

      // CATCH_REQUIRE(p1_str == pose_json_sample);
   }
}

} // namespace perceive
