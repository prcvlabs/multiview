
#pragma once

#include <algorithm>
#include <random>

//#include "detail/fastexp.hpp"
#include "static_math/cmath.h"
#include "string-utils.hpp"

namespace perceive
{
constexpr float fNAN  = std::numeric_limits<float>::quiet_NaN();
constexpr double dNAN = std::numeric_limits<double>::quiet_NaN();

// ----------------------------------------------------------- Inclusive Between

template<typename T, class less_eq = std::less_equal<T>>
inline constexpr bool inclusive_between(const T low_bound,
                                        const T value,
                                        const T high_bound,
                                        less_eq leq = std::less_equal<T>{})
{
   return leq(value, high_bound) && leq(low_bound, value);
}

// -------------------------------------------------------------------- Float-Eq

namespace detail
{
   template<typename T> inline T default_is_close_epsilon() noexcept
   {
      if constexpr(sizeof(T) == 4)
         return 1e-4f;
      else
         return 1e-6;
   }
} // namespace detail

template<typename T>
inline T relative_epsilon(T a, T b, T relative_tolerance = T(NAN)) noexcept
{
   if(std::isnan(relative_tolerance))
      relative_tolerance = detail::default_is_close_epsilon<T>();
   return relative_tolerance * T(std::max(std::fabs(a), std::fabs(b)));
}

template<typename T>
inline bool is_close(T a, T b, T relative_tolerance = T(NAN)) noexcept
{
   return std::isfinite(a) and std::isfinite(b)
          and T(std::fabs(a - b)) <= relative_epsilon(a, b, relative_tolerance);
}

template<typename T>
inline string is_close_report(T a, T b, T relative_tolerance = T(NAN)) noexcept
{
   if(std::isnan(relative_tolerance))
      relative_tolerance = detail::default_is_close_epsilon<T>();
   const T epsilon = relative_epsilon(a, b, relative_tolerance);
   return format("is({}) = {}, is({}) = {}, rel = {:.10e}, |{} - {}| "
                 "= {} <= {} == {}",
                 a,
                 str(std::isfinite(a)),
                 b,
                 str(std::isfinite(b)),
                 relative_tolerance,
                 a,
                 b,
                 T(std::fabs(a - b)) / relative_tolerance,
                 epsilon / relative_tolerance,
                 str(T(std::fabs(a - b)) <= epsilon));
}

template<typename T>
inline bool is_close_gt(T a, T b, T relative_tolerance = T(NAN)) noexcept
{
   return (a > b) or is_close(a, b, relative_tolerance);
}

template<typename T>
inline bool is_close_lt(T a, T b, T relative_tolerance = T(NAN)) noexcept
{
   return (a < b) or is_close(a, b, relative_tolerance);
}

template<typename T>
inline bool
is_close_between(T x, T a, T b, T relative_tolerance = T(NAN)) noexcept
{
   return is_close_gt(x, a, relative_tolerance)
          and is_close_lt(x, b, relative_tolerance);
}

template<typename T> inline constexpr bool float_eq(T a, T b) noexcept
{
   static_assert(std::is_floating_point<T>::value,
                 "must be floating point type");
   constexpr T eps = std::numeric_limits<T>::epsilon() * T(8.0);
   return fabs(a - b) < eps;
}

template<typename T>
inline constexpr bool
float_is_same(T a, T b, T relative_tolerance = T(NAN)) noexcept
{
   return (std::isfinite(a) and std::isfinite(b)
           and is_close(a, b, relative_tolerance))
          or (std::isnan(a) and std::isnan(b))
          or (a == b); // covers +/- infinity
}

// ----------------------------------------------------------------------- Clamp

template<typename T> constexpr T clamp(T in, T min, T max) noexcept
{
   if(in < min) return min;
   if(in > max) return max;
   return in;
}

// ----------------------------------------------------------------------- Round

template<typename T> T round(const T& number, const int decimal_places) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   if(decimal_places > 0) {
      T val = number;
      for(int i = 0; i < decimal_places; ++i) val *= T(10.0);
      val = std::round(val);
      for(int i = 0; i < decimal_places; ++i) val /= T(10.0);
      return val;
   } else if(decimal_places < 0) {
      T val = number;
      for(int i = 0; i < -decimal_places; ++i) val /= T(10.0);
      val = std::round(val);
      for(int i = 0; i < -decimal_places; ++i) val *= T(10.0);
      return val;
   }
   return std::round(number);
}

// ------------------------------------------------------------- integer-ceiling
// Integer division that truncates AWAY from zero, as opposed
// to the default behavior.
// So: 5/2 == 2, but int_div2(5, 2) == 3
template<std::integral T> T int_div2(T numerator, T denominator) noexcept
{
   static_assert(std::is_integral<T>::value);
   const T N = numerator / denominator;
   const T r = numerator - (N * denominator); // remainder
   // There's probably some nice branchless way to do this

   if constexpr(std::is_signed<T>::value) {
      return (r == T(0)) ? N : (N + (N < 0 ? -1 : 1));
   } else {
      return T(int_div2(int64_t(numerator), int64_t(denominator)));
   }
}

// ---------------------------------------------------------------------- Logish

template<typename T> T logish(T value) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   if(value >= 0.0) return log(value + 1.0);
   return -log(1.0 - value);
}

template<typename T> T logish_prob(T value) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   constexpr double log_2_inv = 1.0 / smath::log(2.0);
   Expects(value >= T(0.0) && value <= T(1.0));
   return T(log(double(value) + 1.0) * log_2_inv);
}

// ---------------------------------------------------------------------- Square

template<typename T> constexpr T square(T x) noexcept { return x * x; }

template<typename T> constexpr T cube(T x) noexcept { return x * x * x; }

// ---------------------------------------------------- Uniform Random Variables

// thread-safe, uniform random variable in the range [0..1]
double uniform() noexcept;

// thread-safe, uniform random variable in the range [min()..max()]
// It is NOT mathematically sound to modulo this value to generate
// smaller ranges, example, [1..6]. Use std::uniform_int_distribution,
// and seed such with this value.
std::random_device::result_type pure_random() noexcept;

size_t make_random_seed() noexcept;

// ---------------------------------------------------------------------- Angles

template<std::floating_point T>
inline constexpr T angle_normalise(T theta) noexcept // [-pi..pi)
{
   if(theta >= T(M_PI))
      return std::fmod(theta + T(M_PI), T(2.0 * M_PI)) - T(M_PI);
   if(theta < T(-M_PI))
      return std::fmod(theta - T(M_PI), T(2.0 * M_PI)) + T(M_PI);
   return theta;
}

template<std::floating_point T>
inline constexpr T angle_normalise2(T theta) noexcept // [0..2pi)
{
   auto t1 = angle_normalise(theta);
   if(t1 < T(0.0)) t1 += T(2.0 * M_PI);
   if(t1 >= T(2.0 * M_PI)) return T(0.0);
   return t1;
}

template<std::floating_point T>
inline constexpr T angle_normalize(T theta) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   return angle_normalise(theta);
}

template<std::floating_point T>
inline constexpr T angle_normalize2(T theta) noexcept
{
   return angle_normalise2(theta);
}

template<std::floating_point T> inline constexpr T to_radians(T theta) noexcept
{
   return theta * (T(M_PI) / T(180.0));
}

template<std::floating_point T> inline constexpr T to_degrees(T theta) noexcept
{
   return theta * (T(180.0) / T(M_PI));
}

template<std::integral T> inline constexpr double to_radians(T theta) noexcept
{
   return to_radians(double(theta));
}

template<std::integral T> inline constexpr double to_degrees(T theta) noexcept
{
   return to_degrees(double(theta));
}

// Let delta = angle_diff(phi, theta)
// THEN:
//      normatile(phi  ) = normalize(theta + delta)
//      normalize(theta) = normalize(phi   - delta)
// AND
//     delta is never larger than pi
// WHERE
//     normalize find the equivalent angle in [0..2pi)
template<std::floating_point T>
inline constexpr T angle_diff(T phi, T theta) noexcept
{
   const auto t0 = angle_normalise2(phi);
   auto t1       = angle_normalise2(theta);
   if(t1 < t0) t1 += T(2.0 * M_PI);
   auto delta = t0 - t1;
   if(delta < -T(M_PI)) delta += T(2.0 * M_PI);
   return delta;
}

// The average of two angles is the angle that minimizes the difference
// to either
template<std::floating_point T>
inline constexpr T angle_average(T phi, T theta) noexcept
{
   const auto diff = angle_diff(phi, theta);
   const auto ret  = theta + T(0.5) * diff;
   return angle_normalise2(ret);
}

template<std::floating_point T>
inline constexpr T short_angle_diff(T phi, T theta) noexcept
{
   auto delta = angle_normalise(phi) - angle_normalise(theta);
   delta += (delta > T(M_PI_2))    ? -T(M_PI)
            : (delta < -T(M_PI_2)) ? T(M_PI)
                                   : T(0.0);
   return delta;
}

template<typename InputIt>
inline auto average_angles(InputIt start, InputIt finish) noexcept
{
   using T = typename std::iterator_traits<InputIt>::value_type;
   static_assert(std::is_floating_point<T>::value);

   T sum_x     = T(0);
   T sum_y     = T(0);
   int counter = 0;

   for(auto ii = start; ii != finish; ++ii) {
      sum_x += std::cos(*ii);
      sum_y += std::sin(*ii);
      ++counter;
   }

   return (counter == 0) ? T(NAN) : std::atan2(sum_y, sum_x);
}

template<typename InputIt>
inline auto average_short_angles(InputIt start, InputIt finish) noexcept
{
   using T = typename std::iterator_traits<InputIt>::value_type;
   static_assert(std::is_floating_point<T>::value);

   T sum_x     = T(0);
   T sum_y     = T(0);
   int counter = 0;
   int flips   = 0;

   for(auto ii = start; ii != finish; ++ii) {
      T x      = std::cos(*ii);
      T y      = std::sin(*ii);
      auto dot = (sum_x * x + sum_y * y);
      if(dot < T(0)) {
         sum_x -= x;
         sum_y -= y;
         ++flips;
      } else {
         sum_x += x;
         sum_y += y;
      }

      ++counter;
   }

   if(2 * flips < counter) {
      sum_x = -sum_x;
      sum_y = -sum_y;
   }

   return (counter == 0) ? T(NAN) : std::atan2(sum_y, sum_x);
}

/**
 * We have cos(theta) = value, but acos(value) requires value in [-1..1]
 * So this function converts the real number-line into a period function [-1..1]
 */
inline double cos_theta_normalise(double value) noexcept;

inline constexpr double one_degree() { return 1.0 * M_PI / 180.0; }

// -- This is about 20% faster than calling atan2
// x*x + y*y must be 1.0 ==> [-pi..pi]
inline double unit_atan2(double y, double x) noexcept
{
   return (y >= 0.0) ? acos(x) : -acos(x);
}

// x*x + y*y must be 1.0 ==> [0..2pi]
inline double unit_atan2_2pi(double y, double x) noexcept
{
   return (y >= 0.0) ? acos(x) : 2.0 * M_PI - acos(x);
}

inline double fast_atan2(double y, double x) noexcept
{
   double norm_inv = 1.0 / sqrt(y * y + x * x);
   return unit_atan2(y * norm_inv, x * norm_inv);
}

// ---------------------------------------------------------------- phi function

// CDF for normal distribution
template<typename T> inline T phi_function(T z_score)
{
   static constexpr T minus_inv_sqrt_2 = T(-1.0 / smath::sqrt(2.0));
   if constexpr(sizeof(T) == 4) {
      return T(0.5) * std::erfcf(minus_inv_sqrt_2 * z_score);
   } else {
      return T(0.5) * std::erfc(minus_inv_sqrt_2 * z_score);
   }
}

// ------------------------------------------------------------ combine mu-sigma

template<typename T>
inline std::pair<T, T>
combine_mu_sigma(unsigned n0, T u0, T s0, unsigned n1, T u1, T s1) noexcept
{
   const auto n   = n0 + n1;
   const auto v0  = square(s0);
   const auto v1  = square(s1);
   const auto sum = u0 * T(n0) + u1 * T(n1);
   const auto ss  = T(n0) * (v0 + u0 * u0) + T(n1) * (v1 + u1 * u1);
   const auto u   = sum / T(n);
   const auto s   = std::sqrt(ss / T(n) - square(u));
   return std::pair<T, T>(u, s);
}

// ----------------------------------------------------------- sample statistics

struct SampleStatistics
{
   unsigned N{0};
   double min{0.0};
   double max{0.0};
   double median{0.0};
   double average{0.0};
   double stddev{0.0};
   double absdev{0.0}; // average absolute deviation

   std::string to_string() const noexcept;

   friend std::string str(const SampleStatistics& stats) noexcept
   {
      return stats.to_string();
   }
};

template<typename InputItr>
SampleStatistics calc_sample_statistics(InputItr begin, InputItr end) noexcept
{
   using T_ = typename std::iterator_traits<InputItr>::value_type;
   using T =
       typename std::conditional_t<std::is_floating_point_v<T_>, T_, double>;

   SampleStatistics s;
   s.N                = unsigned(std::distance(begin, end));
   s.min              = double(std::numeric_limits<T>::max());
   s.max              = double(std::numeric_limits<T>::lowest());
   const double N     = double(s.N);
   const double N_inv = 1.0 / N;
   for(auto ii = begin; ii != end; ++ii) {
      s.average += double(*ii) * N_inv;
      const auto val = double(*ii);
      if(val < s.min) s.min = val;
      if(val > s.max) s.max = val;
   }

   for(auto ii = begin; ii != end; ++ii) {
      const auto val = double(*ii);
      s.stddev += square(val - s.average) * N_inv;
      s.absdev += fabs(val - s.average) * N_inv;
   }
   s.stddev = sqrt(s.stddev);

   // Median
   if(std::distance(begin, end) > 0) {
      auto mid = begin + s.N / 2;
      std::nth_element(begin, mid, end);
      s.median = double(*mid);
   }

   return s;
}

template<typename InputItr>
auto calc_median(InputItr begin, InputItr end) noexcept
{
   const auto N = std::distance(begin, end);
   Expects(N > 0);
   auto mid = begin + N / 2;
   std::nth_element(begin, mid, end);
   return *mid;
}

template<typename InputItr>
real calc_average(InputItr begin, InputItr end) noexcept
{
   const auto N = std::distance(begin, end);
   Expects(N > 0);
   real sum = 0.0;
   for(auto ii = begin; ii != end; ++ii) sum += real(*ii);
   return sum / real(N);
}

// ---------------------------------------------------------------------- modulo
/**
 * @ingroup math
 * @brief The modular arithemtic operator introduced by Gauss, and used in
 *        math. Differs from the `remainder` operator `%`.
 *
 * Modular arithemtic is arithmetic that "wraps around". For integers
 * `mod 5`, we have:
 *
 * ~~~~~~~~~~~~~~~~~
 * -8 -7 -6 | -5 -4 -3 -2 -1 | 0  1  2  3  4  | 5  6  7  8
 *  2  3  4 |  0  1  2  3  4 | 0  1  2  3  4  | 0  1  2  3
 * ~~~~~~~~~~~~~~~~~
 *
 * For example,
 *
 * ~~~~~~~~~~~~~~~~~ {.cpp}
 * modulo(-8, 5) == 2;
 * modulo( 7, 5) == 2;
 * ~~~~~~~~~~~~~~~~~
 */
template<typename T> constexpr T modulo(const T& a, const T& n) noexcept
{
   static_assert(std::is_integral<T>::value);
   Expects(n > 0);
   if(a >= 0) return a % n;
   return n - (-a % n);
}

namespace detail
{
   template<typename T>
   inline void Expects_is_close_(T a,
                                 T b,
                                 T relative_tolerance,
                                 const char* filename,
                                 const int lineno) noexcept
   {
      if(branch_is_likely(is_close(a, b, relative_tolerance))) return;
      sync_write([&]() {
         cout << ANSI_COLOUR_RED_BG << ANSI_COLOUR_WHITE << "FATAL "
              << ANSI_COLOUR_RESET << ANSI_COLOUR_GREY << filename << ":"
              << lineno << ANSI_COLOUR_RESET << " "
              << format(
                     "is-close({}, {}, {}) = false", a, b, relative_tolerance)
              << endl
              << indent(is_close_report(a, b, relative_tolerance), 3) << endl;
      });
      assert(false);
      FATAL("Aborting");
   }
} // namespace detail

#define Expects_is_close(a, b, r) \
   detail::Expects_is_close_((a), (b), (r), __FILE__, __LINE__)

// ----------------------------------------------------------- quadratic-formula
// Roots of (a x^2 + b x + c)
template<typename T>
inline std::pair<T, T> quadratic_formula(const T a, const T b, const T c)
{
   static_assert(std::is_floating_point<T>::value);

   const T b_4ac = b * b - T(4.0) * a * c;
   if(b_4ac >= T(0.0)) {
      const T two_a_inv  = T(0.5) / a;
      const T sqrt_b_4ac = std::sqrt(b_4ac);
      return {(-b + sqrt_b_4ac) * two_a_inv, (-b - sqrt_b_4ac) * two_a_inv};
   }

   return {std::numeric_limits<T>::quiet_NaN(),
           std::numeric_limits<T>::quiet_NaN()};
}

} // namespace perceive
