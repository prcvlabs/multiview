
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/utils/file-system.hpp"

using namespace perceive;

CATCH_TEST_CASE("DirnameSV", "[dirname-sv]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("dirname-sv")
   {
      CATCH_REQUIRE(dirname_sv("foo") == ".");
      CATCH_REQUIRE(dirname_sv("s3://") == "s3://");
      CATCH_REQUIRE(dirname_sv("/") == "/");
      CATCH_REQUIRE(dirname_sv("//") == "//");
      CATCH_REQUIRE(dirname_sv("s3://foo") == "s3://");
      CATCH_REQUIRE(dirname_sv("s3://foo/") == "s3://");
      CATCH_REQUIRE(dirname_sv("s3://foo/bar") == "s3://foo");
      CATCH_REQUIRE(dirname_sv("//foo") == "//");
      CATCH_REQUIRE(dirname_sv("/foo") == "/");
   }
}
