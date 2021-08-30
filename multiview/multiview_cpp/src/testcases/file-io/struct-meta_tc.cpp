
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/pipeline/cli-args.hpp"
#include "perceive/pipeline/nodes/nodes.hpp"
#include "perceive/pipeline/pipeline-input.hpp"
#include "perceive/pipeline/pipeline-output.hpp"

namespace perceive
{
template<typename T> static void test_eq(const T& u, const Json::Value& o)
{
   T v, z;
   v.read_with_defaults(o, &z);
   CATCH_REQUIRE(u == v);
}

template<typename T> static void test_it_eq()
{
   T u;
   Json::Value packed        = u.to_json();
   const vector<string> keys = packed.getMemberNames();
   for(auto i = 0u; i < keys.size(); i += 2) packed.removeMember(keys[i]);
   test_eq<T>(u, packed);
}

template<typename T> static void test_read_eq()
{
   T u, v;
   Json::Value packed        = u.to_json();
   const vector<string> keys = packed.getMemberNames();
   for(auto i = 0u; i < keys.size(); ++i) {
      Json::Value p2 = packed;
      p2.removeMember(keys[i]);
      read(v, p2);
      CATCH_REQUIRE(u == v);
   }
   test_it_eq<T>();
}

CATCH_TEST_CASE("STRUCT-META", "[struct_meta]")
{
   CATCH_SECTION("struct-meta_pipeline-config")
   {
      pipeline::CliArgs x;
      x.outdir           = "foo"s;
      Json::Value packed = x.to_json();
      packed.removeMember("start_frame_no");
      packed.removeMember("in_stats_filenames");
      test_eq<pipeline::CliArgs>(x, packed);
   }

   CATCH_SECTION("struct-meta_openpose")
   {
      pose_skeleton::OpenposeParams p, q;
      p.pose_enabled   = !p.pose_enabled;
      p.net_input_size = Point2{-1, -1};
      p.render_mode    = pose_skeleton::RenderMode::CPU;
      const auto s     = p.to_json_string();
      q.read(parse_json(s));
      CATCH_REQUIRE(p == q);
   }

   CATCH_SECTION("struct-meta")
   {
      // cost functions
      test_it_eq<fowlkes::Params>();
      test_it_eq<pose_skeleton::OpenposeParams>();
      test_it_eq<Tracklet::Params>();
      test_it_eq<Tracks::Params>();

      // pipeline
      test_read_eq<pipeline::CliArgs>();
      test_read_eq<pipeline::PipelineInput>();
      test_read_eq<PipelineOutput>();

      // pipeline nodes
      test_read_eq<pipeline::bcam_init::Params>();
      test_read_eq<pipeline::calc_calibration_mask::Params>();
      test_read_eq<pipeline::calc_rectified::Params>();
      test_read_eq<pipeline::convert_to_lab::Params>();
      test_read_eq<pipeline::copy_sensor_images::Params>();
      test_read_eq<pipeline::create_slic_lookup::Params>();
      test_read_eq<pipeline::disp_init::Params>();
      test_read_eq<pipeline::disp_map_update::Params>();
      test_read_eq<pipeline::floor_hist::Params>();
      test_read_eq<pipeline::get_xy_mapping::Params>();
      test_read_eq<pipeline::input_images::Params>();
      test_read_eq<pipeline::load_scene_description::Params>();
      test_read_eq<pipeline::localization::Params>();
      test_read_eq<pipeline::movie_stats::Params>();
      // test_read_eq<pipeline::movie::Params>(); // Not necessary
      test_read_eq<pipeline::pose_skeleton_init::Params>();
      test_read_eq<pipeline::run_f2d::Params>();
      test_read_eq<pipeline::slic_3d_init::Params>();
      test_read_eq<pipeline::tracklet::Params>();
      test_read_eq<pipeline::tracks::Params>();
   }
}
} // namespace perceive
