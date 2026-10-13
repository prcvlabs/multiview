
#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/utils/file-system.hpp"

#include <filesystem>
#include <thread>

namespace perceive
{
// ---------------------------------------------------- file-put-contents-atomic
//
CATCH_TEST_CASE("FilePutContentsAtomic", "[fs]")
{
   const string tmpd = make_temp_directory("/tmp/atomic-file-tc.XXXXXX");

   // The names in `tmpd`, sorted: a leftover temporary shows up here
   auto ls_tmpd = [&]() {
      vector<string> names;
      for(const auto& e : std::filesystem::directory_iterator(tmpd))
         names.push_back(e.path().filename().string());
      std::sort(begin(names), end(names));
      return names;
   };

   CATCH_SECTION("writes the content and leaves no temporary")
   {
      const string fname = format("{}/a.txt", tmpd);
      const string dat   = "The quick brown fox\0jumped"s;
      CATCH_REQUIRE(!file_put_contents_atomic(fname, dat));
      CATCH_REQUIRE(file_get_contents(fname) == dat);
      CATCH_REQUIRE(ls_tmpd() == vector<string>{"a.txt"});
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
      CATCH_REQUIRE(ls_tmpd() == vector<string>{"b.txt"});
   }

   CATCH_SECTION("fails into a missing directory and writes nothing")
   {
      const string fname = format("{}/no-such-dir/c.txt", tmpd);
      CATCH_REQUIRE(file_put_contents_atomic(fname, "data"));
      CATCH_REQUIRE(!is_regular_file(fname));
      CATCH_REQUIRE(ls_tmpd().empty());
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
      CATCH_REQUIRE(ls_tmpd() == vector<string>{"d"});
   }

   CATCH_SECTION("concurrent writers of one file never mix their payloads")
   {
      // As processes sharing a lazy-s3 cache directory do: each writer must
      // have its own temporary, or one renames the other's half-written file
      constexpr size_t k_n_writers = 8;
      constexpr int k_n_rounds     = 50;
      const string fname           = format("{}/e.bin", tmpd);
      vector<string> payloads;
      for(auto i = 0u; i < k_n_writers; ++i)
         payloads.push_back(string(size_t(1 << 20), char('a' + i)));

      vector<std::error_code> ecs(k_n_writers);
      vector<std::thread> writers;
      for(auto i = 0u; i < k_n_writers; ++i)
         writers.emplace_back([&, i]() {
            for(auto j = 0; j < k_n_rounds; ++j)
               if(const auto ec = file_put_contents_atomic(fname, payloads[i]))
                  ecs[i] = ec;
         });
      for(auto& t : writers) t.join();

      for(const auto& ec : ecs) CATCH_REQUIRE(!ec);
      const auto dat = file_get_contents(fname);
      CATCH_REQUIRE(std::find(cbegin(payloads), cend(payloads), dat)
                    != cend(payloads));
      CATCH_REQUIRE(ls_tmpd() == vector<string>{"e.bin"});
   }

   remove_all(tmpd);
}

} // namespace perceive
