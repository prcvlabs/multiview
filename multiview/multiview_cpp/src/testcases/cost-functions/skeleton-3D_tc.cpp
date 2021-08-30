
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/geometry/skeleton/skeleton-2d.hpp"
#include "perceive/geometry/skeleton/skeleton-3d.hpp"
#include "perceive/io/lazy-s3.hpp"

namespace perceive
{
static const string aws_prefix
    = "s3://perceive-multiview/test-data/p2d-skeletons"s;
static const std::array<string, 5> k_skeleton_fnames
    = {{"frame-0029_sensor-00_skeleton-00.json"s,
        "frame-0029_sensor-00_skeleton-01.json"s,
        "frame-0029_sensor-00_skeleton-02.json"s,
        "frame-0029_sensor-00_skeleton-03.json"s,
        "frame-0029_sensor-00_skeleton-04.json"s}};

static shared_ptr<const Skeleton2D> load_p2d(const int ind)
{
   Expects(unsigned(ind) < k_skeleton_fnames.size());
   string raw_data;
   lazy_load(format("{}/{}", aws_prefix, k_skeleton_fnames[size_t(ind)]),
             raw_data);
   Json::Value data = parse_json(raw_data);
   auto p2d_ptr     = make_shared<Skeleton2D>();
   p2d_ptr->read(data);
   return p2d_ptr;
}

static DistortedCamera load_dcam()
{
   const string key = "C0001019_v2";
   BinocularCameraInfo bcam_info;
   const auto s0 = time_thunk([&]() { fetch(bcam_info, key); });
   CachingUndistortInverse cu;
   const auto s1 = time_thunk([&]() { cu.init(bcam_info.M[0]); });
   const auto et = EuclideanTransform{Vector3{-6.691225178041149135311e-01,
                                              2.157871165011404190892e+00,
                                              2.454118742677926068296e+00},
                                      Quaternion{-3.036845037075900366474e-01,
                                                 7.620380153053342153768e-01,
                                                 -5.238878008363446747708e-01,
                                                 2.293803774786336291402e-01},
                                      1.0};
   const int w   = 896;
   const int h   = 672;
   return make_distorted_camera(et, cu, w, h); // [wxh] of image
}

CATCH_TEST_CASE("Skeleton3D", "[skeleton-3d]")
{
   CATCH_SECTION("skeleton-3d")
   { //
     // INFO("Hello world!");
     // CATCH_REQUIRE(true);

      // const auto& dcam = load_dcam();
      // auto test_it     = [&](shared_ptr<const Skeleton2D> p2d_ptr) {
      //    // cout << p2d_ptr->to_string() << endl << endl;
      //    Skeleton3D::Params params;
      //    Skeleton3D::Solution sol;
      //    const auto height  = 1.70;
      //    const auto ref_kp  = Skeleton2D::NECK;
      //    const auto k_ref   = 2.0;
      //    const auto s3d_ptr = fit_skeleton_3d(params, dcam, p2d_ptr);
      //    INFO(format("n-solutions = {}", s3d_ptr->n_solutions()));
      //    s3d_ptr->realize_solution(params, 0, height, ref_kp, k_ref, sol,
      //    true); for(const auto& X : sol.Xs) cout << format("X = {}", str(X))
      //    << endl; cout << p2d_ptr->to_string() << endl;
      // };
      // test_it(load_p2d(0));
   }
}

} // namespace perceive
