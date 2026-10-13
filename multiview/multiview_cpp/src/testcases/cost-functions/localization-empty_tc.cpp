
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"

namespace perceive
{
// ------------------------------------------------------------------- make-like
// A frame with the geometry of a real one, and some histogram content
//
static LocalizationData make_like(const LocalizationData::Params& p)
{
   LocalizationData like;
   like.frame_no = 3;
   like.hist_sz  = 0.1;
   like.bounds   = AABB(-6.0, -3.0, 3.0, 10.0);
   like.loc_min  = float(p.loc_min());
   like.loc_max  = float(p.loc_max());
   like.loc_hist.resize(90, 130);
   uint16_t counter = 0;
   for(auto y = 0u; y < like.loc_hist.height; ++y)
      for(auto x = 0u; x < like.loc_hist.width; ++x)
         like.loc_hist(x, y) = counter++;
   like.p2ds.resize(4);
   return like;
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("LocalizationDataMakeEmpty", "[localization_empty]")
{
   CATCH_SECTION("make-empty-geometry-and-value")
   {
      const LocalizationData::Params p;
      const auto like = make_like(p);
      const auto dat  = LocalizationData::make_empty(like, p, 17);

      CATCH_REQUIRE(dat.frame_no == 17);
      CATCH_REQUIRE(dat.hist_sz == like.hist_sz);
      CATCH_REQUIRE(dat.bounds == like.bounds);
      CATCH_REQUIRE(dat.loc_min == like.loc_min);
      CATCH_REQUIRE(dat.loc_max == like.loc_max);
      CATCH_REQUIRE(dat.loc_hist.width == like.loc_hist.width);
      CATCH_REQUIRE(dat.loc_hist.height == like.loc_hist.height);
      CATCH_REQUIRE(dat.invariant_check());

      CATCH_REQUIRE(dat.p2ds.size() == like.p2ds.size());
      for(const auto& sensor_p2ds : dat.p2ds)
         CATCH_REQUIRE(sensor_p2ds.empty());

      // The zero-evidence value: 4.0 with the default parameters
      const float range = float(p.loc_max() - p.loc_min());
      const float value
          = 0.5f * (1.0f - float(p.zero_value)) * range + float(p.loc_min());
      CATCH_REQUIRE(std::fabs(value - 4.0f) < 1e-5f);

      constexpr float k_max_grey16 = 65535.0f;
      const float step             = range / k_max_grey16;
      for(auto y = 0u; y < dat.loc_hist.height; ++y) {
         for(auto x = 0u; x < dat.loc_hist.width; ++x) {
            const float cell
                = dat.loc_min
                  + range * float(dat.loc_hist(x, y)) / k_max_grey16;
            CATCH_REQUIRE(std::fabs(cell - value) <= step);
         }
      }

      // Every cell holds the same grey16 value
      CATCH_REQUIRE(dat.loc_hist(0, 0) == dat.loc_hist(89, 129));
   }

   CATCH_SECTION("make-empty-read-write")
   {
      const LocalizationData::Params p;
      const auto dat = LocalizationData::make_empty(make_like(p), p, 5);

      vector<char> buffer(1024 * 1024);
      FILE* fp = fmemopen(&buffer[0], buffer.size(), "w");
      dat.write(fp);
      fclose(fp);

      LocalizationData rhs;
      fp = fmemopen(&buffer[0], buffer.size(), "r");
      rhs.read(fp);
      fclose(fp);

      CATCH_REQUIRE(dat == rhs);
      CATCH_REQUIRE(rhs.frame_no == 5);
      CATCH_REQUIRE(rhs.p2ds.size() == dat.p2ds.size());
   }
}

} // namespace perceive
