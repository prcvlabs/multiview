
#include "perceive/utils/md5.hpp"

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

namespace perceive
{
CATCH_TEST_CASE("Md5Sum_", "[md5_sum]")
{
   //
   // -------------------------------------------------------
   //
   CATCH_SECTION("md5")
   {
      auto test_str = [&](const string& s, const string& val) {
         CATCH_REQUIRE(md5(s) == val);
      };

      test_str("The rain in Spain falls mainly on the plains.",
               "a7a5a692ff3af6078c52465015dbebba");

      array<char, 30> dat{{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                           0x16, 0x17, 0x18, 0x30, 0x31, 0x30, 0x32, 0x33,
                           0x34, 0x35, 0x36, 0x37, 0x38, 0x39}};
      const string digest = "3271e81510b4854cff43e57b103d3dd1";

      {
         MD5 m;
         m.update(&dat[0], dat.size());
         m.finalize();
         CATCH_REQUIRE(m.hexdigest() == digest);
      }
   }

   CATCH_SECTION("extract-md5")
   {
      auto test_it = [](const string_view in, const string_view m) {
         CATCH_REQUIRE(extract_md5_hexdigest(in) == m);
      };

      test_it("a7a5a692ff3af6078c52465015dbebba",
              "a7a5a692ff3af6078c52465015dbebba");

      test_it("aa7a5a692ff3af6078c52465015dbebba", "");
      test_it("7a5a692ff3af6078c52465015dbebba", "");

      test_it("a_a7a5a692ff3af6078c52465015dbebba",
              "a7a5a692ff3af6078c52465015dbebba");
      test_it("a7a5a692ff3af6078c52465015dbebba_a",
              "a7a5a692ff3af6078c52465015dbebba");
   }
}

} // namespace perceive
