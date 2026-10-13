
#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/cost-functions/localization/envelope-ops.hpp"
#include "perceive/utils/file-system.hpp"
#include "testcases/testcase-data.hpp"

namespace perceive::ldata
{
static constexpr real k_frame_duration = 1.0 / 15.0;

// --------------------------------------------------------------- make-envelope
//
static LocalizationData make_frame(const int frame_no)
{
   LocalizationData ldat;
   ldat.loc_min  = -20.0f;
   ldat.loc_max  = 20.0f;
   ldat.frame_no = frame_no;
   ldat.hist_sz  = 0.1;
   ldat.bounds   = AABB(0.0, 0.0, 0.8, 0.6);
   ldat.loc_hist.resize(8, 6);
   for(auto y = 0u; y < ldat.loc_hist.height; ++y)
      for(auto x = 0u; x < ldat.loc_hist.width; ++x)
         ldat.loc_hist(x, y) = uint16_t(frame_no * 7 + int(x) + int(y));
   ldat.p2ds.resize(2); // two sensors, no detections
   return ldat;
}

static LocalizationDataEnvelope make_envelope(const int start, const int n)
{
   LocalizationDataEnvelope env;
   env.start_frame    = unsigned(start);
   env.frame_duration = k_frame_duration;
   for(auto t = start; t < start + n; ++t)
      env.loc_data.push_back(make_frame(t));
   return env;
}

static bool frames_equal(const LocalizationDataEnvelope& a,
                         const LocalizationDataEnvelope& b)
{
   if(a.start_frame != b.start_frame) return false;
   if(a.loc_data.size() != b.loc_data.size()) return false;
   for(size_t i = 0; i < a.loc_data.size(); ++i)
      if(a.loc_data[i] != b.loc_data[i]
         or a.loc_data[i].frame_no != b.loc_data[i].frame_no)
         return false;
   return true;
}

static void save_envelope(const string& fname,
                          const LocalizationDataEnvelope& env)
{
   save_localization_data(
       fname,
       env.start_frame,
       unsigned(env.loc_data.size()),
       env.frame_duration,
       [&](unsigned t) { return &env.loc_data.at(t - env.start_frame); },
       false);
}

// --------------------------------------------------------------- cat-envelopes
//
CATCH_TEST_CASE("CatEnvelopes", "[ldata]")
{
   CATCH_SECTION("adjacent envelopes, default keep ranges")
   {
      const auto ret
          = cat_envelopes({make_envelope(0, 20), make_envelope(20, 20)}, {});
      CATCH_REQUIRE(ret.error.empty());
      CATCH_REQUIRE(ret.overlap_reports.empty());
      CATCH_REQUIRE(frames_equal(ret.envelope, make_envelope(0, 40)));
      CATCH_REQUIRE(ret.envelope.frame_duration == k_frame_duration);
   }

   CATCH_SECTION("overlapping envelopes, explicit keep ranges")
   {
      const auto ret = cat_envelopes(
          {make_envelope(0, 30), make_envelope(10, 30)}, {{0, 20}, {20, 20}});
      CATCH_REQUIRE(ret.error.empty());
      CATCH_REQUIRE(frames_equal(ret.envelope, make_envelope(0, 40)));
      CATCH_REQUIRE(ret.overlap_reports.size() == 1);
      const auto& report = ret.overlap_reports.front();
      CATCH_REQUIRE(report["first_frame"].asInt() == 10);
      CATCH_REQUIRE(report["n_frames"].asInt() == 20);
      CATCH_REQUIRE(report["n_equal"].asInt() == 20);
      CATCH_REQUIRE(report["boundary_frame"].asInt() == 20);
   }

   CATCH_SECTION("overlapping envelopes, default keep ranges")
   {
      const auto ret
          = cat_envelopes({make_envelope(0, 30), make_envelope(10, 30)}, {});
      CATCH_REQUIRE(ret.error.empty());
      CATCH_REQUIRE(frames_equal(ret.envelope, make_envelope(0, 40)));
      CATCH_REQUIRE(ret.overlap_reports.size() == 1);
   }

   CATCH_SECTION("errors")
   {
      auto require_error = [](const CatResult& ret) {
         CATCH_REQUIRE(!ret.error.empty());
         CATCH_REQUIRE(ret.envelope.loc_data.empty());
      };

      const auto a = make_envelope(0, 30);
      const auto b = make_envelope(10, 30);

      // No envelopes; wrong number of keep ranges
      require_error(cat_envelopes({}, {}));
      require_error(cat_envelopes({a, b}, {{0, 20}}));

      // The keep ranges leave a gap, or overlap
      require_error(cat_envelopes({a, b}, {{0, 20}, {25, 15}}));
      require_error(cat_envelopes({a, b}, {{0, 20}, {15, 25}}));

      // A keep range outside its envelope, or empty
      require_error(cat_envelopes({a, b}, {{0, 35}, {35, 5}}));
      require_error(cat_envelopes({a, b}, {{0, 20}, {20, 25}}));
      require_error(cat_envelopes({a, b}, {{0, 0}, {0, 20}}));

      // Out of order
      require_error(
          cat_envelopes({make_envelope(20, 20), make_envelope(0, 20)}, {}));

      // A gap between envelopes
      require_error(
          cat_envelopes({make_envelope(0, 20), make_envelope(30, 20)}, {}));

      { // Different frame durations
         auto c           = make_envelope(20, 20);
         c.frame_duration = 1.0 / 30.0;
         require_error(cat_envelopes({make_envelope(0, 20), c}, {}));
      }

      { // A frame number that is not the frame's position
         auto c                 = make_envelope(20, 20);
         c.loc_data[3].frame_no = 99;
         require_error(cat_envelopes({make_envelope(0, 20), c}, {}));
      }

      { // A different histogram geometry
         auto c                = make_envelope(20, 20);
         c.loc_data[0].hist_sz = 0.2;
         require_error(cat_envelopes({make_envelope(0, 20), c}, {}));
      }
   }
}

// -------------------------------------------------------------- diff-envelopes
//
CATCH_TEST_CASE("DiffEnvelopes", "[ldata]")
{
   CATCH_SECTION("identical")
   {
      const auto diff
          = diff_envelopes(make_envelope(0, 20), make_envelope(0, 20));
      CATCH_REQUIRE(diff.error.empty());
      CATCH_REQUIRE(diff.identical());
      CATCH_REQUIRE(diff.n_frames == 20);
      CATCH_REQUIRE(diff.n_equal == 20);
      CATCH_REQUIRE(diff.n_bytes_equal == 20);
      CATCH_REQUIRE(diff.frames.empty());
      CATCH_REQUIRE(diff.to_json(false)["identical"].asBool());
   }

   CATCH_SECTION("one histogram cell differs")
   {
      const auto a = make_envelope(0, 20);
      auto b       = make_envelope(0, 20);
      b.loc_data[7].loc_hist(3, 2) += 5;
      const auto diff = diff_envelopes(a, b);
      CATCH_REQUIRE(!diff.identical());
      CATCH_REQUIRE(diff.headers_equal);
      CATCH_REQUIRE(diff.n_equal == 19);
      CATCH_REQUIRE(diff.n_bytes_equal == 19);
      CATCH_REQUIRE(diff.frames.size() == 1);
      CATCH_REQUIRE(diff.worst.frame_no == 7);
      CATCH_REQUIRE(!diff.worst.equal);
      CATCH_REQUIRE(!diff.worst.bytes_equal);
      CATCH_REQUIRE(diff.worst.md5_a.size() == 32);
      CATCH_REQUIRE(diff.worst.md5_a != diff.worst.md5_b);
      CATCH_REQUIRE(diff.worst.hist_cells_differ == 1);
      CATCH_REQUIRE(diff.worst.hist_max_abs == 5);
      CATCH_REQUIRE(std::fabs(diff.worst.hist_l1_mean - 5.0 / 48.0) < 1e-12);
      CATCH_REQUIRE(diff.worst.p2ds_equal);
      CATCH_REQUIRE(diff.worst.header_equal);
      CATCH_REQUIRE(diff.to_json(true)["frames"].size() == 1);
   }

   CATCH_SECTION("the 2D poses differ")
   {
      const auto a = make_envelope(0, 20);
      auto b       = make_envelope(0, 20);
      Skeleton2DInfo info;
      info.id = 1;
      b.loc_data[4].p2ds[1].push_back(info);
      auto c                          = b;
      c.loc_data[4].p2ds[1].back().id = 2;

      // A 2D pose without a skeleton cannot be written, so it is never
      // bytes-equal
      auto diff = diff_envelopes(a, b);
      CATCH_REQUIRE(diff.n_equal == 19);
      CATCH_REQUIRE(diff.n_bytes_equal == 19);
      CATCH_REQUIRE(!diff.identical());
      CATCH_REQUIRE(diff.worst.frame_no == 4);
      CATCH_REQUIRE(diff.worst.md5_b.empty());
      CATCH_REQUIRE(diff.worst.hist_cells_differ == 0);
      CATCH_REQUIRE(!diff.worst.p2ds_equal);
      CATCH_REQUIRE(diff.worst.p2d_count_a == 0);
      CATCH_REQUIRE(diff.worst.p2d_count_b == 1);
      CATCH_REQUIRE(diff.worst.p2d_sensors_differ == 1);

      diff = diff_envelopes(b, c);
      CATCH_REQUIRE(diff.n_equal == 19);
      CATCH_REQUIRE(!diff.worst.p2ds_equal);
      CATCH_REQUIRE(diff.worst.p2d_count_a == 1);
      CATCH_REQUIRE(diff.worst.p2d_count_b == 1);
   }

   CATCH_SECTION("different frame ranges")
   {
      auto diff = diff_envelopes(make_envelope(0, 30), make_envelope(10, 30));
      CATCH_REQUIRE(diff.error.empty());
      CATCH_REQUIRE(!diff.headers_equal);
      CATCH_REQUIRE(!diff.identical());
      CATCH_REQUIRE(diff.first_frame == 10);
      CATCH_REQUIRE(diff.n_frames == 20);
      CATCH_REQUIRE(diff.n_equal == 20);
      CATCH_REQUIRE(diff.n_bytes_equal == 20);

      diff = diff_envelopes(make_envelope(0, 10), make_envelope(10, 10));
      CATCH_REQUIRE(!diff.error.empty());
      CATCH_REQUIRE(!diff.identical());
   }
}

// --------------------------------------------------------------- envelope-info
//
CATCH_TEST_CASE("EnvelopeInfo", "[ldata]")
{
   const string tmpd = make_temp_directory("/tmp/envelope-ops-tc.XXXXXX");
   const auto env    = make_envelope(30, 20);

   const string fname0 = format("{}/a.data", tmpd);
   const string fname1 = format("{}/b.data", tmpd);
   save_envelope(fname0, env);
   save_envelope(fname1, env);
   CATCH_REQUIRE(!is_regular_file(fname0 + ".tmp"));

   const auto loaded = load_localization_data(fname0);
   CATCH_REQUIRE(frames_equal(loaded, env));
   CATCH_REQUIRE(diff_envelopes(loaded, env).identical());

   const auto info0 = envelope_info(loaded, fname0);
   const auto info1 = envelope_info(load_localization_data(fname1), fname1);
   CATCH_REQUIRE(info0["start_frame"].asInt() == 30);
   CATCH_REQUIRE(info0["n_frames"].asInt() == 20);
   CATCH_REQUIRE(info0["end_frame"].asInt() == 50);
   CATCH_REQUIRE(info0["hist_w"].asInt() == 8);
   CATCH_REQUIRE(info0["hist_h"].asInt() == 6);
   CATCH_REQUIRE(info0["n_sensors"].asInt() == 2);
   CATCH_REQUIRE(info0["p2d_count_total"].asInt() == 0);
   CATCH_REQUIRE(info0["frame_no_contiguous"].asBool());
   CATCH_REQUIRE(info0["bytes"].asUInt64() > 0);
   CATCH_REQUIRE(info0["md5"].asString().size() == 32);
   CATCH_REQUIRE(info0["md5"].asString() == info1["md5"].asString());
   CATCH_REQUIRE(info0["frame_md5"].size() == 20);
   CATCH_REQUIRE(info0["frame_md5"][0].asString().size() == 32);
   CATCH_REQUIRE(info0["frame_md5"][0] != info0["frame_md5"][1]);
   CATCH_REQUIRE(info0["frames_md5"] == info1["frames_md5"]);
   CATCH_REQUIRE(info0["frames_md5"] == envelope_info(env, "")["frames_md5"]);

   remove_all(tmpd);
}

// ------------------------------------------------------ real localization data
//
CATCH_TEST_CASE("CatEnvelopesTestcase007", "[ldata]")
{
   const string fname = format("{}/cloud-golden/007/ref-120.data",
                               test::testcase_data_dir());
   if(!is_regular_file(fname)) {
      WARN(format("file '{}' not found (tools/cloud/tests/make_reference.sh "
                  "writes it), skipping test",
                  fname));
      return;
   }

   const auto env = load_localization_data(fname);
   CATCH_REQUIRE(env.loc_data.size() > 2);
   const int start = int(env.start_frame);
   const int mid   = start + int(env.loc_data.size()) / 2;

   // Split it in two, overlapping by 2 frames, then concatenate again
   LocalizationDataEnvelope a;
   LocalizationDataEnvelope b;
   a.start_frame    = env.start_frame;
   b.start_frame    = unsigned(mid - 2);
   a.frame_duration = b.frame_duration = env.frame_duration;
   for(const auto& ldat : env.loc_data) {
      if(ldat.frame_no < mid) a.loc_data.push_back(ldat);
      if(ldat.frame_no >= mid - 2) b.loc_data.push_back(ldat);
   }

   const auto ret = cat_envelopes({a, b}, {});
   CATCH_REQUIRE(ret.error.empty());
   CATCH_REQUIRE(frames_equal(ret.envelope, env));
   CATCH_REQUIRE(diff_envelopes(ret.envelope, env).identical());
   CATCH_REQUIRE(ret.overlap_reports.size() == 1);
   CATCH_REQUIRE(ret.overlap_reports[0]["n_equal"].asInt() == 2);
   CATCH_REQUIRE(envelope_info(env, fname)["frame_no_contiguous"].asBool());

   // operator== skips `prob_xy`; the bytes do not
   auto changed               = env;
   LocalizationData* ldat_ptr = nullptr;
   for(auto& ldat : changed.loc_data)
      for(auto& p2d_vec : ldat.p2ds)
         for(auto& info : p2d_vec)
            if(ldat_ptr == nullptr and info.prob_xy.size() > 0) {
               info.prob_xy(0, 0) += 1.0f;
               ldat_ptr = &ldat;
            }
   CATCH_REQUIRE(ldat_ptr != nullptr);
   const auto diff = diff_envelopes(env, changed);
   CATCH_REQUIRE(diff.n_equal == diff.n_frames);
   CATCH_REQUIRE(diff.n_bytes_equal == diff.n_frames - 1);
   CATCH_REQUIRE(!diff.identical());
   CATCH_REQUIRE(diff.worst.frame_no == ldat_ptr->frame_no);
   CATCH_REQUIRE(diff.worst.equal);
   CATCH_REQUIRE(!diff.worst.bytes_equal);
}

} // namespace perceive::ldata
