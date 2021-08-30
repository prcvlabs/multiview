
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/utils/timestamp.hpp"

namespace perceive
{
CATCH_TEST_CASE("timestamp_test", "[timestamp_test]")
{
   CATCH_SECTION("is-leap-year")
   {
      CATCH_REQUIRE(is_leap_year(2000));
      CATCH_REQUIRE(is_leap_year(1972));
      CATCH_REQUIRE(is_leap_year(1968));
      CATCH_REQUIRE(!is_leap_year(1900));
      CATCH_REQUIRE(!is_leap_year(2001));
   }

   CATCH_SECTION("calc-day-of-week")
   {
      CATCH_REQUIRE(day_of_week(2018, 1, 1) == 1);
      CATCH_REQUIRE(day_of_week(2018, 3, 30) == 5);
      CATCH_REQUIRE(day_of_week(2018, 6, 26) == 2);
      CATCH_REQUIRE(day_of_week(2018, 9, 22) == 6);
      CATCH_REQUIRE(day_of_week(2018, 12, 19) == 3);
      CATCH_REQUIRE(day_of_week(2000, 1, 1) == 6);
      CATCH_REQUIRE(day_of_week(2000, 3, 29) == 3);
      CATCH_REQUIRE(day_of_week(2000, 6, 25) == 0);
      CATCH_REQUIRE(day_of_week(2000, 9, 21) == 4);
      CATCH_REQUIRE(day_of_week(2000, 12, 18) == 1);
   }

   CATCH_SECTION("calc-day-of-year")
   {
      CATCH_REQUIRE(day_of_year(2000, 1, 1) == 0);
      CATCH_REQUIRE(day_of_year(2000, 1, 28) == 27);
      CATCH_REQUIRE(day_of_year(2000, 2, 24) == 54);
      CATCH_REQUIRE(day_of_year(2000, 3, 22) == 81);
      CATCH_REQUIRE(day_of_year(2000, 4, 18) == 108);
      CATCH_REQUIRE(day_of_year(2000, 5, 15) == 135);
      CATCH_REQUIRE(day_of_year(2000, 6, 11) == 162);
      CATCH_REQUIRE(day_of_year(2000, 7, 8) == 189);
      CATCH_REQUIRE(day_of_year(2000, 8, 4) == 216);
      CATCH_REQUIRE(day_of_year(2000, 8, 31) == 243);
      CATCH_REQUIRE(day_of_year(2000, 9, 27) == 270);
      CATCH_REQUIRE(day_of_year(2000, 10, 24) == 297);
      CATCH_REQUIRE(day_of_year(2000, 11, 20) == 324);
      CATCH_REQUIRE(day_of_year(2000, 12, 17) == 351);
      CATCH_REQUIRE(day_of_year(1971, 1, 1) == 0);
      CATCH_REQUIRE(day_of_year(1971, 3, 30) == 88);
      CATCH_REQUIRE(day_of_year(1971, 6, 26) == 176);
      CATCH_REQUIRE(day_of_year(1971, 9, 22) == 264);
      CATCH_REQUIRE(day_of_year(1971, 12, 19) == 352);
   }

   CATCH_SECTION("TimestampTest1")
   {
      Timestamp ts;

      auto test_it = [&](int year,
                         int mon,
                         int day,
                         int hour,
                         int min,
                         int sec,
                         int micros) {
         if(mon == 2 and day > 28) return;
         if(day > 30 and (mon == 4 or mon == 6 or mon == 9 or mon == 11))
            return;

         auto s = format("{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}.{:06d}",
                         year,
                         mon,
                         day,
                         hour,
                         min,
                         sec,
                         micros);
         const auto ts = Timestamp::parse(s);
         CATCH_REQUIRE(s == str(ts));
         CATCH_REQUIRE(ts.year() == year);
         CATCH_REQUIRE(ts.month() == mon);
         CATCH_REQUIRE(ts.day() == day);
         CATCH_REQUIRE(ts.hour() == hour);
         CATCH_REQUIRE(ts.minute() == min);
         CATCH_REQUIRE(ts.second() == sec);
         CATCH_REQUIRE(ts.micros() == micros);
      };

      for(auto y = 0; y < 10000; y += 50)
         for(auto m = 1; m <= 12; ++m)
            for(auto d = 1; d <= 31; ++d) test_it(y, m, d, 0, 0, 0, 0);
   }

   CATCH_SECTION("timestamp-ymd")
   {
      auto test_it = [&](int year, int mon, int day) {
         const auto s  = format("{:04d}-{:02d}-{:02d}", year, mon, day);
         const auto ts = Timestamp::parse(s);
         CATCH_REQUIRE(ts.year() == year);
         CATCH_REQUIRE(ts.month() == mon);
         CATCH_REQUIRE(ts.day() == day);
      };

      test_it(1999, 12, 1);
      test_it(2000, 2, 21);
   }

   CATCH_SECTION("extract-timestamp")
   {
      auto test_a = [&](const Timestamp ts,
                        const string& s,
                        const string& prefix,
                        const string& suffix) {
         const auto full_s = format("{:s}{:s}{:s}", s, prefix, suffix);
         const auto s2     = extract_timestamp(full_s);
         CATCH_REQUIRE(s == s2);
         CATCH_REQUIRE(ts == Timestamp::parse(s2));
      };

      //
      auto test_it = [&](const string& s) {
         const auto ts = Timestamp::parse(s);
         test_a(ts, s, ""s, ""s);
         test_a(ts, s, format("{:s}/", s), format("/{:s}", s));
         test_a(ts, s, "abs"s, "_"s);
         test_a(ts, s, ""s, "_"s);
         test_a(ts, s, "_"s, ""s);
      };

      test_it("2000-02-28"s);
      test_it("2000-02-28t"s);
      test_it("2000-02-28T"s);
      test_it("2000-02-28t21:30:56"s);
      test_it("2000-02-28t21:30-56"s);
      test_it("2000-02-28t21-30:56"s);
      test_it("2000-02-28t21-30-56"s);
      test_it("2000-02-28t21:30:56."s);
      test_it("2000-02-28t21:30:56.1"s);
      test_it("2000-02-28t21:30:56.12"s);
      test_it("2000-02-28t21:30:56.123"s);
      test_it("2000-02-28t21:30:56.1234"s);
      test_it("2000-02-28t21:30:56.12345"s);
      test_it("2000-02-28t21:30:56.123456"s);
      test_it("2000-02-28t21:30:56.123456+"s);
      test_it("2000-02-28t21:30:56.123456+9"s);
      test_it("2000-02-28t21:30:56.123456+98"s);
      test_it("2000-02-28t21:30:56.123456+987"s);
      test_it("2000-02-28t21:30:56.123456+9876"s);
   }
}

} // namespace perceive
