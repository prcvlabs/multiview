
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/io/s3.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
static std::string sentence = "The quick brown fox jumped over the lazy dog."s;

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
   // Initialize these data buckets
   bin_data1[0] = '\0';
   std::iota(begin(bin_data2), end(bin_data2), 0);
   for(auto i = 0u; i < bin_data3.size(); ++i) bin_data3[i] = 1 + (i % 127);

   bin_data4.resize(sentence.size());
   std::copy(cbegin(sentence), cend(sentence), begin(bin_data4));
}

CATCH_TEST_CASE("data-source", "[test_data_source]")
{
   CATCH_SECTION("lazy-s3")
   {
	return;
      if(hostname() != "hermes"s) { // multiview_trace_mode()
         init_testcase_data();

         auto test_it = [](const vector<char>& data) {
            const string path = "s3://perceive-multiview/TMP/test-file.data"s;
            const string local_p = lazy_s3_cache_filename(path);

            if(false) {
               INFO(format("path  = {:s}", path));
               INFO(format("local = {:s}", local_p));
               cout << str(&data[0], data.size()) << endl;
            }

            if(is_regular_file(local_p)) delete_file(local_p);
            try {
               s3_remove(path);
            } catch(...) {}

            CATCH_REQUIRE(lazy_exists(path) == LazyExists::NOWHERE);
            CATCH_REQUIRE(!s3_exists(path));

            lazy_s3_store(path, data);
            CATCH_REQUIRE(lazy_exists(path) == LazyExists::IN_CACHE);
            CATCH_REQUIRE(s3_exists(path));

            delete_file(local_p);
            CATCH_REQUIRE(lazy_exists(path) == LazyExists::S3_ONLY);

            vector<char> out;
            lazy_s3_load(path, out);
            CATCH_REQUIRE(lazy_exists(path) == LazyExists::IN_CACHE);
            CATCH_REQUIRE(out == data);

            s3_remove(path);
            CATCH_REQUIRE(lazy_exists(path) == LazyExists::IN_CACHE);

            out.clear();
            lazy_s3_load(path, out);
            CATCH_REQUIRE(lazy_exists(path) == LazyExists::IN_CACHE);
            CATCH_REQUIRE(out == data);

            delete_file(local_p);
            CATCH_REQUIRE(lazy_exists(path) == LazyExists::NOWHERE);
            CATCH_REQUIRE(!s3_exists(path));
         };

         for(const auto& data_ptr : all_data) test_it(*data_ptr);
      }
   }
}

} // namespace perceive
