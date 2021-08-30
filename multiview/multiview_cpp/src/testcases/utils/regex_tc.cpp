

#define CATCH_CONFIG_PREFIX_ALL
#include "ctre.hpp"
#include "perceive/contrib/catch.hpp"

namespace perceive
{
CATCH_TEST_CASE("Regex", "[regex]")
{
   CATCH_SECTION("regex")
   {
      static constexpr auto pattern
          = ctll::fixed_string("^.*?([0-9]+)x([0-9]+)\\.([^\\.]*)$");

      if(const auto m = ctre::match<pattern>("TinyVGG-V1-HW=256x384.uff")) {
         CATCH_REQUIRE(m.get<1>().to_view() == "256");
         CATCH_REQUIRE(m.get<2>().to_view() == "384");
         CATCH_REQUIRE(m.get<3>().to_view() == "uff");

         int width = 0, height = 0;
         const auto ec1 = lexical_cast(m.get<1>().to_view(), width);
         const auto ec2 = lexical_cast(m.get<2>().to_view(), height);

         CATCH_REQUIRE((ec1.value() == 0 && width == 256));
         CATCH_REQUIRE((ec2.value() == 0 && height == 384));
      } else {
         FATAL("mismatch!");
      }
   }
}

} // namespace perceive
