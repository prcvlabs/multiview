
#include "kz-filter.hpp"

#define This KzFilter

namespace perceive
{
This::This(const int window_size, const int coefficient) noexcept
{
   init(window_size, coefficient);
}

// ------------------------------------------------------------------------ init
//
void This::init(const int m, const int k) noexcept
{
   Expects(m > 0);
   Expects(m % 2 == 1);
   Expects(k > 0);

   k_ = k;

   auto sum_ranges = [](int N, const real* A, real* dest) {
      for(int i = 0; i < N; ++i) *dest++ += *A++;
   };

   // Iteratively calculate coefficients by convolving
   vector<real> coefs((size_t(m)));
   std::fill(begin(coefs), end(coefs), 1.0); // base case
   for(int k = 1; k < k_; ++k) {
      vector<real> t((size_t(m + k * (m - 1))));
      std::fill(begin(t), end(t), 0.0);
      for(int km = 0; km < m; ++km)
         sum_ranges(int(coefs.size()), &coefs[0], &t[size_t(km)]);
      coefs = std::move(t);
   }

   coefficients_ = std::move(coefs);
}

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   return format(R"V0G0N(
KzFilter
   m             =  {}
   k             =  {}
   t             = [-{}..{}]
   coefficients  = [{}]
{})V0G0N",
                 m(),
                 k(),
                 delta_t(),
                 delta_t(),
                 implode(cbegin(coefficients_),
                         cend(coefficients_),
                         ", ",
                         [](real c) { return format("{}", c); }),
                 "");
}

string str(const KzFilter& o) noexcept { return o.to_string(); }

} // namespace perceive
