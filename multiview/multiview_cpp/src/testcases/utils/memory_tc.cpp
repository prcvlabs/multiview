
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"
#include "perceive/utils/memory.hpp"

namespace perceive
{
CATCH_TEST_CASE("MemoryStuff", "[memory_stuff]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("memory-stuff")
   {
      vector<uint64_t> buffer(16);
      const char* p = reinterpret_cast<const char*>(&buffer[0]);
      CATCH_REQUIRE(memory_is_aligned<uint64_t>(p));
      CATCH_REQUIRE(!memory_is_aligned<uint64_t>(p + 1));
   }
}

} // namespace perceive
