
#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/pipeline/shard-range.hpp"

namespace perceive::pipeline
{
// ------------------------------------------------------- shard-alignment-error
//
CATCH_TEST_CASE("ShardAlignmentError", "[shard]")
{
   auto error_contains = [](const std::optional<string>& err, const char* s) {
      return err.has_value() and err->find(s) != string::npos;
   };

   CATCH_SECTION("aligned ranges")
   {
      CATCH_REQUIRE(!shard_alignment_error(0, 120, 499, 10, nullptr));
      CATCH_REQUIRE(!shard_alignment_error(60, 60, 499, 10, nullptr));
      CATCH_REQUIRE(!shard_alignment_error(300, 199, 499, 10, nullptr));
      CATCH_REQUIRE(!shard_alignment_error(0, 499, 499, 10, nullptr));
      CATCH_REQUIRE(!shard_alignment_error(0, 2, 2, 10, nullptr));
   }

   CATCH_SECTION("unaligned ranges")
   {
      CATCH_REQUIRE(error_contains(
          shard_alignment_error(305, 100, 499, 10, nullptr), "start"));
      CATCH_REQUIRE(error_contains(
          shard_alignment_error(300, 95, 499, 10, nullptr), "end"));
      CATCH_REQUIRE(error_contains(
          shard_alignment_error(0, 1, 499, 10, nullptr), "2 frames"));
      CATCH_REQUIRE(error_contains(
          shard_alignment_error(490, 20, 499, 10, nullptr), "outside"));
      CATCH_REQUIRE(error_contains(
          shard_alignment_error(-10, 20, 499, 10, nullptr), "outside"));
      CATCH_REQUIRE(error_contains(
          shard_alignment_error(80, 70, 149, 10, nullptr), "outside"));
      CATCH_REQUIRE(shard_alignment_error(0, 120, 499, 0, nullptr));
      CATCH_REQUIRE(shard_alignment_error(0, 120, 499, -10, nullptr));
   }

   CATCH_SECTION("ranges inside a localization-data envelope")
   {
      LocalizationDataEnvelope env;
      env.start_frame    = 300;
      env.frame_duration = 1.0 / 15.0;
      env.loc_data.resize(100);

      CATCH_REQUIRE(!shard_alignment_error(300, 100, 499, 10, &env));
      CATCH_REQUIRE(!shard_alignment_error(320, 50, 499, 10, &env));
      CATCH_REQUIRE(error_contains(
          shard_alignment_error(290, 110, 499, 10, &env), "localization"));
      CATCH_REQUIRE(error_contains(
          shard_alignment_error(300, 110, 499, 10, &env), "localization"));
      CATCH_REQUIRE(error_contains(shard_alignment_error(0, 200, 499, 10, &env),
                                   "localization"));
   }
}

} // namespace perceive::pipeline
