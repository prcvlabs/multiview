
#ifdef WITH_CUDA

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "stereobm/results_example.hpp"
#include <cstdlib>
#include <cuda_runtime.h>

namespace perceive
{
CATCH_TEST_CASE("CudaASAN", "[cuda_asan]")
{
   CATCH_SECTION("cuda_asan")
   {
      // INFO("Test if we have the CUDA ASAN bug...");

      int device = -42;
      int code   = cudaGetDevice(&device);

      CATCH_REQUIRE(code != cudaErrorMemoryAllocation);
   }
}
} // namespace perceive

#endif
