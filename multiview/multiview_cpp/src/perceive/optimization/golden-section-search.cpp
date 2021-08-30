
#include <algorithm>
#include <functional>

#include "golden-section-search.hpp"
#include "static_math/static_math.h"

namespace perceive
{
static inline double clamp(double in, double min, double max)
{
   if(in < min) return min;
   if(in > max) return max;
   return in;
}

double golden_section_search(std::function<double(double)> f,
                             double in_a,
                             double in_b,
                             double tolerance)
{
   constexpr bool feedback  = false;
   constexpr double inv_phi = 2.0 / (1.0 + smath::sqrt(5.0)); // about 0.618

   double a = in_a;
   double b = in_b;

   if(a == b) return a;
   if(a > b) std::swap(a, b);

   double c  = clamp(b - (b - a) * inv_phi, in_a, in_b);
   double d  = clamp(a + (b - a) * inv_phi, in_a, in_b);
   double fc = f(c);
   double fd = f(d);

   unsigned counter{0};

   auto printit = [&]() {
      printf("#% 6d [%f, %f, %f, %f] => [%f%s, %f%s]\n",
             counter++,
             a,
             c,
             d,
             b,
             fc,
             (fc < fd ? "*" : ""),
             fd,
             (fc >= fd ? "*" : ""));
   };

   if(feedback) printit();

   if(fabs(c - d) > tolerance) {
      while(true) {
         if(feedback) printit();
         if(fc < fd) {
            b  = d;
            d  = c;
            fd = fc;
            c  = clamp(b - (b - a) * inv_phi, in_a, in_b);
            if(fabs(c - d) <= tolerance) break;
            fc = f(c);
         } else {
            a  = c;
            c  = d;
            fc = fd;
            d  = clamp(a + (b - a) * inv_phi, in_a, in_b);
            if(fabs(c - d) <= tolerance) break;
            fd = f(d);
         }
      }
   }

   return 0.5 * (c + d);
}

} // namespace perceive
