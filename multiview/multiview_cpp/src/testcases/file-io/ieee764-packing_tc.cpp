
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/io/ieee754-packing.hpp"
#include "perceive/utils/memory.hpp"

static const bool feedback = false;

namespace perceive
{
// using float16 = _Float16;
using float32 = float;
using float64 = double;

template<typename T> static std::ostream& print_binary(T x, std::ostream& ss)
{
   constexpr int n_bits = sizeof(T) * 8;
   for(int i = n_bits - 1; i >= 0; --i) { ss << ((x & (1 << i)) ? '1' : '0'); }
   return ss;
}

// Convert a float to an IEEE 754 half-precision
inline uint16_t pack_f16(float x)
{
   uint32_t packed = pack_f32(x);
   uint16_t f      = ((packed & 0x80000000u) >> 16) // sign bit
                | ((packed & 0x7c000000u) >> 16)    // exponent
                | ((packed & 0x007fe000u) >> 13)    // fraction
       ;
   return f;
}

template<typename F> void test_ieee764(const F x)
{
   using I = typename std::conditional_t<sizeof(F) == 8, uint64_t, uint32_t>;

   constexpr int m = (sizeof(F) == 8) ? 11 : 8;

   const I y = pack_float(x);
   const F b = unpack_float(y);

#ifdef __STDC_IEC_559__
   const I a = bit_cast<I, F>(x);
   CATCH_REQUIRE(a == y);
   if(std::isfinite(x)) CATCH_REQUIRE(x == bit_cast<F, I>(y));
   const I c = bit_cast<I, F>(b);
   CATCH_REQUIRE(a == c);
#endif

   CATCH_REQUIRE(std::isfinite(x) == std::isfinite(b));
   if(std::isfinite(x)) CATCH_REQUIRE(x == b);
}

template<typename F> static void test_ieee764_blub()
{
   using II =
       typename std::conditional<sizeof(F) == 8, uint64_t, uint32_t>::type;

   std::mt19937 g(0);
   std::uniform_int_distribution<II> dis(0, std::numeric_limits<II>::max());

   auto rand_f = [&]() -> F {
      while(true) {
         II i  = dis(g);
         F ret = bit_cast<F, II>(i);
         if(std::isnormal(ret)) return ret;
      }
      return F(0.0);
   };

   for(auto i = 0; i < 1000000; ++i) { test_ieee764<F>(rand_f()); }

   test_ieee764<F>(F(0.0));
   test_ieee764<F>(F(-0.0));
   test_ieee764<F>(F(1.0));
   test_ieee764<F>(F(-1.0));
   test_ieee764<F>(F(dNAN));
   test_ieee764<F>(std::numeric_limits<F>::infinity());
   test_ieee764<F>(-std::numeric_limits<F>::infinity());
}

} // namespace perceive

namespace perceive::pipeline
{
CATCH_TEST_CASE("FloatTypes", "[float_types]")
{
   CATCH_SECTION("float-types")
   {
      if(false) {
         cout << " 2 = ";
         print_binary(uint16_t(2), cout) << endl;

         cout << " 2 = ";
         print_binary(pack_f32(2), cout) << endl;

         cout << " 2 = ";
         print_binary(pack_f16(2), cout) << endl;

         cout << "-2 = ";
         print_binary(pack_f16(-2), cout) << endl;

         cout << "1/3= ";
         print_binary(pack_f32(1.0f / 3.0f), cout) << endl;

         // cout << "1/3= ";
         // print_binary(pack_f16(1. / 3), cout) << endl;
      }
      CATCH_REQUIRE(true);
   }

   CATCH_SECTION("ieee754-packing")
   {
      std::mt19937 g(0);
      std::uniform_int_distribution<uint32_t> dis(
          0, std::numeric_limits<uint32_t>::max());

      auto rand_f32 = [&]() -> float {
         while(true) {
            uint32_t I = dis(g);
            float ret  = bit_cast<float, uint32_t>(I);
            if(std::isnormal(ret)) return ret;
         }
         return 0.0f;
      };

      auto rand_f64 = [&]() -> double {
         while(true) {
            uint64_t I = (uint64_t(dis(g)) << 32) | (uint64_t(dis(g)) << 0);
            double ret = bit_cast<double, uint64_t>(I);
            if(std::isnormal(ret)) return ret;
         }
         return 0.0;
      };

      CATCH_REQUIRE(is_cpu_ieee754_packing());

      test_ieee764_blub<float>();
      test_ieee764_blub<double>();
   }
}
} // namespace perceive::pipeline
