
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/utils/string-utils.hpp"

namespace perceive
{
CATCH_TEST_CASE("StrReplace", "[str-replace]")
{
   CATCH_SECTION("str-replace")
   {
      const auto s = str_replace(
          "Spain", "Germany", "The rain in Spain falls mainly in the plains.");
      CATCH_REQUIRE(s == "The rain in Germany falls mainly in the plains."s);

      CATCH_REQUIRE(str_replace("abc", "xyz", "abcabc") == "xyzxyz"s);
      CATCH_REQUIRE(str_replace("", "xyz", "abcabc") == "abcabc"s);
      CATCH_REQUIRE(str_replace("abc", "xyz", "") == ""s);
      CATCH_REQUIRE(str_replace("abc", "", "abcabc") == ""s);
   }
}

} // namespace perceive
