
#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
// ---------------------------------------------------- file-put-contents-atomic
//
CATCH_TEST_CASE("FilePutContentsAtomic", "[fs]")
{
   const string tmpd = make_temp_directory("/tmp/atomic-file-tc.XXXXXX");

   CATCH_SECTION("writes the content and leaves no temporary")
   {
      const string fname = format("{}/a.txt", tmpd);
      const string dat   = "The quick brown fox\0jumped"s;
      CATCH_REQUIRE(!file_put_contents_atomic(fname, dat));
      CATCH_REQUIRE(file_get_contents(fname) == dat);
      CATCH_REQUIRE(!is_regular_file(fname + ".tmp"));
   }

   CATCH_SECTION("overwrites, including with an empty payload")
   {
      const string fname = format("{}/b.txt", tmpd);
      CATCH_REQUIRE(!file_put_contents_atomic(fname, "first"));
      CATCH_REQUIRE(file_get_contents(fname) == "first");
      CATCH_REQUIRE(!file_put_contents_atomic(fname, "second, longer"));
      CATCH_REQUIRE(file_get_contents(fname) == "second, longer");
      CATCH_REQUIRE(!file_put_contents_atomic(fname, ""));
      CATCH_REQUIRE(is_regular_file(fname));
      CATCH_REQUIRE(file_get_contents(fname).empty());
      CATCH_REQUIRE(!is_regular_file(fname + ".tmp"));
   }

   CATCH_SECTION("fails into a missing directory and writes nothing")
   {
      const string fname = format("{}/no-such-dir/c.txt", tmpd);
      CATCH_REQUIRE(file_put_contents_atomic(fname, "data"));
      CATCH_REQUIRE(!is_regular_file(fname));
      CATCH_REQUIRE(!is_regular_file(fname + ".tmp"));
   }

   CATCH_SECTION("a failed rename keeps the old file and removes the temporary")
   {
      // The target is a non-empty directory, so the rename fails after the
      // temporary is written.
      const string dname = format("{}/d", tmpd);
      CATCH_REQUIRE(mkdir(dname));
      CATCH_REQUIRE(!file_put_contents(format("{}/keep.txt", dname), "keep"s));
      CATCH_REQUIRE(file_put_contents_atomic(dname, "data"));
      CATCH_REQUIRE(is_directory(dname));
      CATCH_REQUIRE(file_get_contents(format("{}/keep.txt", dname)) == "keep");
      CATCH_REQUIRE(!is_regular_file(dname + ".tmp"));
   }

   remove_all(tmpd);
}

} // namespace perceive
