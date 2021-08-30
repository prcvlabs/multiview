
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/utils/base64.hpp"

namespace perceive
{
static void test_base64()
{
   auto test_len = [&](unsigned len) {
      string buf(len, '\0');
      for(size_t i = 0; i < len; ++i) buf[i] = char(i % 256);

      const auto enc_len = base64_encode_length(len);

      const string b64 = base64_encode(buf.data(), buf.size());
      const string bd  = base64_decode(b64);

      if(false) {
         CATCH_REQUIRE(buf.size() == len);
         CATCH_REQUIRE(b64.size() == base64_encode_length(len));
         CATCH_REQUIRE(is_base64(b64));
         CATCH_REQUIRE(base64_decode_length(b64) == len);
         CATCH_REQUIRE(bd == buf);
         CATCH_REQUIRE(bd.size() == len);
      }

      if(true) {
         string c64 = b64;
         c64.push_back('\0'); // should fail
         CATCH_REQUIRE(!is_base64(c64));

         c64.pop_back();
         CATCH_REQUIRE(is_base64(c64));

         if(c64.size() > 10) {
            c64[c64.size() - 4] = '\0'; // should fail
            CATCH_REQUIRE(!is_base64(c64));
         }
      }

      if(false) {
         if(len > 512 - 3) {
            cout << format("len = {}, enc_len = {}: ", len, enc_len) << endl;
            const auto b64len = base64_encode_length(len);
            cout << str(&buf[0], buf.size()) << endl;
            cout << str(&b64[0], b64.size()) << endl;
         }
      }
   };

   test_len(30);

   for(uint len = 0; len <= 512; ++len) test_len(len);
}

CATCH_TEST_CASE("Base64", "[base64]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("base64") { test_base64(); }
}

} // namespace perceive
