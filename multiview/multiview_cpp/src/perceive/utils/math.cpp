
#include <random>

#include "math.hpp"
#include "perceive/foundation.hpp"

namespace perceive
{
// these are thread-local
std::random_device::result_type pure_random() noexcept
{
   static thread_local std::random_device rd; // uses RDRND or /dev/urandom
   return rd();
}

size_t make_random_seed() noexcept
{
   struct Seeder
   {
      std::mt19937 gen;
      std::uniform_int_distribution<size_t> dist;
      Seeder()
          : gen(std::random_device()())
          , dist(size_t(0), std::numeric_limits<size_t>::max())
      {}
      size_t operator()() noexcept { return dist(gen); }
   };
   static thread_local Seeder seeder;
   return seeder();
}

// --------------------------------------------------------------------- uniform

double uniform() noexcept
{
   struct TLParams
   {
      bool first{true};
      std::mt19937 gen;
      std::uniform_real_distribution<double> distribution{0.0, 1.0};
   };

   static thread_local TLParams tl;

   if(tl.first) {
      tl.first = false;
      tl.gen.seed(pure_random());
   }

   return tl.distribution(tl.gen);
}

// ---------------------------------------------- sample statistics :: to_string

std::string SampleStatistics::to_string() const noexcept
{
   return format(R"V0G0N(
  N                =   {}
  [min, med, max]  =  [{}, {}, {}]
  abs deviation    =   {}
  average          =   {} 
  stddev           =   {}
  variance         =   {}
  95% interval    =  [{}, {}]
  

)V0G0N",
                 N,
                 min,
                 median,
                 max,
                 absdev,
                 average,
                 stddev,
                 square(stddev),
                 average - 2.98 * stddev,
                 average + 2.98 * stddev);
   // "\u00b1",
   // 2.98 * stddev);
}

} // namespace perceive
