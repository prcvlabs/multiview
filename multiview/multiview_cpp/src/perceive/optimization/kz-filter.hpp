
#pragma once

#include <functional>
#include <iterator>

namespace perceive
{
// Implementation of Kolmogorov-Zurbenko Filter
// @see https://en.wikipedia.org/wiki/Kolmogorov%E2%80%93Zurbenko_filter
//
// m:    sets window size: must be odd
// k:    smooth (polynomial generation coefficient)
//
struct KzFilter
{
 private:
   int k_                     = 0; // polynomial coefficient generator
   vector<real> coefficients_ = {};

 public:
   // Window size is (m + k(m-1))
   KzFilter(const int m = 5,           // must be odd
            const int k = 3) noexcept; // must be > 0
   KzFilter(const KzFilter&) = delete;
   KzFilter(KzFilter&&)      = default;
   ~KzFilter()               = default;
   KzFilter& operator=(const KzFilter&) = delete;
   KzFilter& operator=(KzFilter&&) = default;

   void init(const int m, const int k) noexcept;

   int k() const noexcept { return k_; }
   int m() const noexcept { return (int(window_size()) + k() - 1) / k(); }
   size_t window_size() const noexcept { return coefficients_.size(); }
   int delta_t() const noexcept { return int((window_size() - 1) / 2); }

   template<typename T> struct is_finite_func
   {
      bool operator()(const T& o) { return is_finite(o); }
   };

   template<typename T> struct scalar_mult_func
   {
      T operator()(const real k, const T& o) { return T(k * real(o)); }
   };

   template<typename T>
   T smooth(std::function<std::optional<T>(int)> get_sample_t,
            const int sample_offset,
            std::function<T(real, const T&)> scalar_mult
            = scalar_mult_func<T>{},
            std::function<T(const T&, const T&)> plus = std::plus<T>{},
            std::function<bool(const T&)> is_finite
            = is_finite_func<T>{}) const noexcept
   {
      real coefficient_sum = 0.0;
      T sum{};
      const int dt = -delta_t();

      int counter = 0;
      for(const auto coefficient : coefficients_) {
         const auto o = get_sample_t(sample_offset + dt + counter++);
         if(o.has_value()) {
            sum = plus(sum, scalar_mult(coefficient, o.value()));
            coefficient_sum += coefficient;
         }
      }

      if(coefficient_sum == 0.0) return T{};
      return scalar_mult(1.0 / coefficient_sum, sum);
   }

   string to_string() const noexcept;
   friend string str(const KzFilter&) noexcept;
};

} // namespace perceive
