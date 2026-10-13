
#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/pipeline/cli-args.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive::pipeline
{
// -------------------------------------------------------------- write-envelope
//
static void write_envelope(const string& fname, const int start, const int n)
{
   vector<LocalizationData> frames;
   frames.resize(size_t(n));
   for(auto i = 0; i < n; ++i) {
      auto& ldat    = frames[size_t(i)];
      ldat.frame_no = start + i;
      ldat.hist_sz  = 0.1;
      ldat.loc_hist.resize(4, 3);
      ldat.loc_hist.fill(uint16_t(start + i));
   }
   save_localization_data(
       fname,
       unsigned(start),
       unsigned(n),
       1.0 / 15.0,
       [&](unsigned t) { return &frames.at(t - unsigned(start)); },
       false);
}

static CliArgs parse(const vector<string>& args)
{
   vector<string> strs = args;
   vector<char*> argv;
   for(auto& s : strs) argv.push_back(s.data());
   argv.push_back(nullptr);
   return parse_command_line(int(strs.size()), argv.data());
}

// ---------------------------------------------------------- parse-command-line
//
CATCH_TEST_CASE("CliArgsShards", "[cli-args]")
{
   const string tmpd = make_temp_directory("/tmp/cli-args-tc.XXXXXX");
   const string a    = format("{}/a.data", tmpd);
   const string b    = format("{}/b.data", tmpd);
   write_envelope(a, 0, 10);
   write_envelope(b, 10, 15);

   CATCH_SECTION("repeated --load-loc-data concatenates")
   {
      const auto config = parse({"pipeline",
                                 "-d",
                                 tmpd,
                                 "--load-loc-data",
                                 a,
                                 "--load-loc-data",
                                 b,
                                 "--still-warmup",
                                 "5",
                                 "--require-aligned"});
      CATCH_REQUIRE(!config.has_error);
      CATCH_REQUIRE(config.ldata_env_ptr != nullptr);
      CATCH_REQUIRE(config.ldata_env_ptr->start_frame == 0);
      CATCH_REQUIRE(config.ldata_env_ptr->loc_data.size() == 25);
      for(auto i = 0; i < 25; ++i)
         CATCH_REQUIRE(config.ldata_env_ptr->loc_data[size_t(i)].frame_no == i);
      CATCH_REQUIRE(config.start_frame_no == 0);
      CATCH_REQUIRE(config.n_frames_to_process == 25);
      CATCH_REQUIRE(std::fabs(config.target_fps - 15.0) < 1e-9);
      CATCH_REQUIRE(config.still_warmup_frames == 5);
      CATCH_REQUIRE(config.require_aligned);
      CATCH_REQUIRE(config.n_loc_data_fnames() == 2);
      CATCH_REQUIRE(config.localization_data_fname == a);
      CATCH_REQUIRE(is_regular_file(format("{}/run-object.json", tmpd)));
      CATCH_REQUIRE(!is_regular_file(format("{}/run-object.json.tmp", tmpd)));
   }

   CATCH_SECTION("one --load-loc-data")
   {
      const auto config = parse({"pipeline", "-d", tmpd, "--load-loc-data", b});
      CATCH_REQUIRE(!config.has_error);
      CATCH_REQUIRE(config.ldata_env_ptr->start_frame == 10);
      CATCH_REQUIRE(config.ldata_env_ptr->loc_data.size() == 15);
      CATCH_REQUIRE(config.n_frames_to_process == 15);
   }

   CATCH_SECTION("errors")
   {
      // Files out of order, the same file twice, a missing file
      CATCH_REQUIRE(parse({"pipeline",
                           "-d",
                           tmpd,
                           "--load-loc-data",
                           b,
                           "--load-loc-data",
                           a})
                        .has_error);
      CATCH_REQUIRE(parse({"pipeline",
                           "-d",
                           tmpd,
                           "--load-loc-data",
                           a,
                           "--load-loc-data",
                           a})
                        .has_error);
      CATCH_REQUIRE(parse({"pipeline",
                           "-d",
                           tmpd,
                           "--load-loc-data",
                           format("{}/missing.data", tmpd)})
                        .has_error);
      CATCH_REQUIRE(
          parse({"pipeline", "-d", tmpd, "--still-warmup", "-1"}).has_error);
   }

   remove_all(tmpd);
}

} // namespace perceive::pipeline
