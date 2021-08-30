
#include <algorithm>
#include <filesystem>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
static std::string sentence = "The quick brown fox jumped over the lazy dog.";

// Data for test cases
static std::vector<char> bin_data0(0);
static std::vector<char> bin_data1(1);
static std::vector<char> bin_data2(512);
static std::vector<char> bin_data3(512); // ASCII string data
static std::vector<char> bin_data4;

// Method to access all test cases
static std::vector<const decltype(bin_data0)*> all_data{
    {&bin_data0, &bin_data1, &bin_data2, &bin_data3, &bin_data4}};

static void init_testcase_data()
{
   static bool done = false;
   if(done) return;

   // Initialize these data buckets
   bin_data1[0] = '\0';
   std::iota(begin(bin_data2), end(bin_data2), 0);
   for(auto i = 0u; i < bin_data3.size(); ++i) bin_data3[i] = 1 + (i % 127);

   bin_data4.resize(sentence.size());
   std::copy(cbegin(sentence), cend(sentence), begin(bin_data4));
   done = true;
}

static string as_string(const vector<char>& data)
{
   string ret;
   ret.resize(data.size());
   std::copy(cbegin(data), cend(data), begin(ret));
   return ret;
}

template<typename U, typename V> static bool is_equal(const U& s, const V& data)
{
   if(s.size() != data.size()) return false;

   for(auto i = 0u; i < s.size(); ++i)
      if(s[i] != data[i]) return false;

   return true;
}

// ----------------------------------------------------- Point in/out of Polygon

CATCH_TEST_CASE("file-get/put-contents binary safe", "[file_getput_contents]")
{
   // Sometimes we want a string version of the data
   CATCH_SECTION("file-putget-contents_string_binary-safe")
   {
      init_testcase_data();
      vector<string> fnames;
      auto counter = 0;
      for(const auto& data : all_data) {
         string fname = format("/tmp/file-contents.test-{}", counter++);
         file_put_contents(fname, as_string(*data));
         fnames.emplace_back(std::move(fname));
      }

      for(auto i = 0u; i < all_data.size(); ++i) {
         const auto& data = *all_data[i];
         string fdat      = file_get_contents(fnames[i]);
         CATCH_REQUIRE(is_equal(fdat, data));
      }

      // Cleanup
      for(const auto& fname : fnames) delete_file(fname);

      fnames.clear();
   }

   // Sometimes we want a string version of the data
   CATCH_SECTION("file-putget-contents_vector_binary-safe")
   {
      init_testcase_data();
      vector<string> fnames;

      auto counter = 0;
      for(const auto& data : all_data) {
         string fname = format("/tmp/file-contents.test-{}", counter++);
         file_put_contents(fname, *data);
         fnames.emplace_back(std::move(fname));
      }

      for(auto i = 0u; i < all_data.size(); ++i) {
         const auto& data = *all_data[i];
         vector<char> out;
         file_get_contents(fnames[i], out);
         CATCH_REQUIRE(is_equal(out, data));
      }

      // Cleanup
      for(const auto& fname : fnames) {
         auto p = std::filesystem::path(fname);
         std::filesystem::remove(p);
      }

      fnames.clear();
   }
}

} // namespace perceive
