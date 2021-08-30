
#pragma once

#include <type_traits>

#include "static_math/static_math.h"

namespace perceive::detail::fast_exp
{
// -----------------------------------------------------------------------------

template<typename R, size_t degree, size_t i> struct Recursion
{
   static_assert(std::is_floating_point<R>::value);

   static R evaluate(R x)
   {
      constexpr R c = R(1.0) / static_cast<R>(1u << degree);
      x             = Recursion<R, degree, i + 1>::evaluate(x);
      return x * x;
   }
};

template<typename R, size_t degree> struct Recursion<R, degree, degree>
{
   static_assert(std::is_floating_point<R>::value);
   static R evaluate(R x)
   {
      constexpr R c = R(1.0) / static_cast<R>(1u << degree);
      x             = R(1.0) + c * x;
      return x;
   }
};

// -----------------------------------------------------------------------------

template<typename R, size_t degree> struct Product
{
   static_assert(std::is_floating_point<R>::value);

   static R evaluate(R x) { return Recursion<R, degree, 0>::evaluate(x); }
};

// -----------------------------------------------------------------------------

template<typename Real, size_t degree, size_t i = 0> struct PolynomialFit;
template<typename Real> struct Info;

template<typename Real, size_t degree> struct IEEE
{
   static Real evaluate(Real x)
   {
      using unsigned_t           = typename Info<Real>::unsigned_t;
      constexpr unsigned_t shift = static_cast<unsigned_t>(1)
                                   << Info<Real>::shift;

      x *= Info<Real>::log2e;
      Real xi = Real(std::floor(x));
      Real xf = x - xi;

      Real k       = PolynomialFit<Real, degree, 0>::evaluate(xf) + Real(1.0);
      unsigned_t e = reinterpret_cast<const unsigned_t&>(k);
      e += shift * static_cast<unsigned_t>(xi);
      return reinterpret_cast<Real&>(e);
   }
};

////////////////////////////////////////////////////////////////////////////////
// Polynomial coefficients for error function fit.
////////////////////////////////////////////////////////////////////////////////

template<> struct Info<float>
{
   using unsigned_t                = uint32_t;
   static constexpr uint32_t shift = 23;
   static constexpr float log2e    = 1.442695040f;
};

template<> struct Info<double>
{
   using unsigned_t                = uint64_t;
   static constexpr uint64_t shift = 52;
   static constexpr double log2e   = 1.442695040;
};

template<typename Real, size_t degree> struct Data;

template<typename Real> struct Data<Real, 1>
{
   static constexpr Real coefficients[2] = {-0.05288671, 0.99232129};
};
template<typename Real> constexpr Real Data<Real, 1>::coefficients[2];

template<typename Real> struct Data<Real, 2>
{
   static constexpr Real coefficients[3]
       = {Real(0.00365539), Real(0.64960693), Real(0.34271434)};
};
template<typename Real> constexpr Real Data<Real, 2>::coefficients[3];

template<typename Real> struct Data<Real, 3>
{
   static constexpr Real coefficients[4]
       = {-1.77187919e-04, 6.96787180e-01, 2.24169036e-01, 7.90302044e-02};
};
template<typename Real> constexpr Real Data<Real, 3>::coefficients[4];

template<typename Real> struct Data<Real, 4>
{
   static constexpr Real coefficients[5] = {6.58721338e-06,
                                            6.92937406e-01,
                                            2.41696769e-01,
                                            5.16742848e-02,
                                            1.36779598e-02};
};
template<typename Real> constexpr Real Data<Real, 4>::coefficients[5];

template<typename Real> struct Data<Real, 5>
{
   static constexpr Real coefficients[6] = {6.58721338e-06,
                                            6.92937406e-01,
                                            2.41696769e-01,
                                            5.16742848e-02,
                                            1.36779598e-02};
};
template<typename Real> constexpr Real Data<Real, 5>::coefficients[6];

template<typename Real> struct Data<Real, 6>
{
   static constexpr Real coefficients[7] = {-1.97880719e-07,
                                            6.93156327e-01,
                                            2.40133447e-01,
                                            5.58740717e-02,
                                            8.94160147e-03,
                                            1.89454334e-03};
};
template<typename Real> constexpr Real Data<Real, 6>::coefficients[7];

template<typename Real> struct Data<Real, 7>
{
   static constexpr Real coefficients[8] = {4.97074799e-09,
                                            6.93146861e-01,
                                            2.40230956e-01,
                                            5.54792541e-02,
                                            9.68583180e-03,
                                            1.23835751e-03,
                                            2.18728611e-04};
};
template<typename Real> constexpr Real Data<Real, 7>::coefficients[8];

template<typename Real> struct Data<Real, 8>
{
   static constexpr Real coefficients[9] = {-1.06974751e-10,
                                            6.93147190e-01,
                                            2.40226337e-01,
                                            5.55053726e-02,
                                            9.61338873e-03,
                                            1.34310382e-03,
                                            1.42959529e-04,
                                            2.16483090e-05};
};
template<typename Real> constexpr Real Data<Real, 8>::coefficients[9];

template<typename Real> struct Data<Real, 9>
{
   static constexpr Real coefficients[10] = {2.00811867e-12,
                                             6.93147180e-01,
                                             2.40226512e-01,
                                             5.55040573e-02,
                                             9.61838113e-03,
                                             1.33265219e-03,
                                             1.55193275e-04,
                                             1.41484217e-05,
                                             1.87497191e-06};
};
template<typename Real> Real constexpr Data<Real, 9>::coefficients[10];

template<typename Real, size_t degree, size_t i> struct PolynomialFit
{
   inline static Real evaluate(Real x)
   {
      Real p = PolynomialFit<Real, degree, i + 1>::evaluate(x) * x;
      p += Data<Real, degree>::coefficients[i];
      return p;
   }
};

template<typename Real, size_t degree>
struct PolynomialFit<Real, degree, degree>
{
   inline static Real evaluate(Real x)
   {
      return Data<Real, degree>::coefficients[degree];
   }
};

template<typename Real> struct PolynomialFit<Real, 0, 0>
{
   inline static Real evaluate(Real x) { return x; }
};

// -----------------------------------------------------------------------------

enum class Approximation { IEEE, PRODUCT };

// -----------------------------------------------------------------------------

/** \brief Fast approximate exponential.
 *
 * This function implements a fast, vectorizable approximation
 * of the exponential function based on the following two articles:
 *
 * - Malossi, A. Cristiano I. & Ineichen, Yves & Bekas, Costas & Curioni,
 *   Alessandro. "Fast Exponential Computation on SIMD Architectures." (2015)
 *   10.13140/2.1.4362.3207.
 * - IEEE, Nicol N. "A fast, compact approximation of the exponential
 *   function." Neural Computation 11.4 (1999): 853-862.
 *
 * The approximation interpolates linearly between points on the curve of
 * the exponential function that can be expressed as 2^i where i is an
 * a signed integer. So yes, that is very approximate ...
 *
 * \tparam Real The floating point type of the arguments.
 * \param x The argument of the exponential function.
 * \return The approximated value of the exponential function.
 */
#pragma omp declare simd notinbranch
// template<typename Real,
//          template<typename, size_t> class Approximation = IEEE,
//          size_t degree                                  = 2>
// inline Real exp(const Real& x)
// {
//    static_assert(std::is_floating_point<Real>::value);
//    return Approximation<Real, degree>::evaluate(x);
// }

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

constexpr double Taylor_min = -20.0;
constexpr double Taylor_max = 20.0;

template<typename R, size_t degree> struct Taylor
{
   static_assert(std::is_floating_point<R>::value);

   struct FactInv
   {
      R fact[degree + 1];

      constexpr FactInv()
          : fact()
      {
         fact[0] = R(1);
         for(size_t i = 1; i <= degree; ++i) fact[i] = fact[i - 1] * R(i);
         for(auto& f : fact) f = R(1) / f;
      }

      constexpr R operator[](int ind) const { return fact[ind]; }
   };

   static constexpr R minval = Taylor_min;
   static constexpr R maxval = Taylor_max;
   static constexpr int N    = smath::round(maxval) - smath::round(minval) + 1;

   static constexpr std::array<R, N> make_f_a()
   {
      std::array<R, N> Xs = {R(smath::exp(double(minval + R(0)))),
                             R(smath::exp(double(minval + R(1)))),
                             R(smath::exp(double(minval + R(2)))),
                             R(smath::exp(double(minval + R(3)))),
                             R(smath::exp(double(minval + R(4)))),
                             R(smath::exp(double(minval + R(5)))),
                             R(smath::exp(double(minval + R(6)))),
                             R(smath::exp(double(minval + R(7)))),
                             R(smath::exp(double(minval + R(8)))),
                             R(smath::exp(double(minval + R(9)))),
                             R(smath::exp(double(minval + R(10)))),
                             R(smath::exp(double(minval + R(11)))),
                             R(smath::exp(double(minval + R(12)))),
                             R(smath::exp(double(minval + R(13)))),
                             R(smath::exp(double(minval + R(14)))),
                             R(smath::exp(double(minval + R(15)))),
                             R(smath::exp(double(minval + R(16)))),
                             R(smath::exp(double(minval + R(17)))),
                             R(smath::exp(double(minval + R(18)))),
                             R(smath::exp(double(minval + R(19)))),
                             R(smath::exp(double(minval + R(20)))),
                             R(smath::exp(double(minval + R(21)))),
                             R(smath::exp(double(minval + R(22)))),
                             R(smath::exp(double(minval + R(23)))),
                             R(smath::exp(double(minval + R(24)))),
                             R(smath::exp(double(minval + R(25)))),
                             R(smath::exp(double(minval + R(26)))),
                             R(smath::exp(double(minval + R(27)))),
                             R(smath::exp(double(minval + R(28)))),
                             R(smath::exp(double(minval + R(29)))),
                             R(smath::exp(double(minval + R(30)))),
                             R(smath::exp(double(minval + R(31)))),
                             R(smath::exp(double(minval + R(32)))),
                             R(smath::exp(double(minval + R(33)))),
                             R(smath::exp(double(minval + R(34)))),
                             R(smath::exp(double(minval + R(35)))),
                             R(smath::exp(double(minval + R(36)))),
                             R(smath::exp(double(minval + R(37)))),
                             R(smath::exp(double(minval + R(38)))),
                             R(smath::exp(double(minval + R(39)))),
                             R(smath::exp(double(minval + R(40))))};
      return Xs;
   };

   static constexpr std::array<R, N> k_f_a  = make_f_a();
   static constexpr FactInv k_factorial_inv = {};

   static R evaluate(R x)
   {
      const auto a   = std::round(x);
      const auto idx = int(std::round(a - minval));
      if(idx >= N) return std::exp(x);
      assert(idx == int(std::round(a - minval)));
      const auto term  = k_f_a[size_t(idx)];
      const auto x_f_a = (x - a);

      R out     = term;
      auto prod = x_f_a;
      for(size_t i = 1; i <= degree; ++i) {
         out += term * prod * k_factorial_inv[int(i)];
         prod *= x_f_a;
      }

      return out;
   }
};

} // namespace perceive::detail::fast_exp

namespace perceive
{
enum class ExpApproximation : int { IEEE = 0, PRODUCT, TAYLOR };

template<typename T,
         ExpApproximation method = ExpApproximation::PRODUCT,
         size_t degree           = 2>
inline T fastexp(T x)
{
   static_assert(std::is_floating_point<T>::value);
   if constexpr(method == ExpApproximation::IEEE)
      return detail::fast_exp::IEEE<T, degree>::evaluate(x);
   else if constexpr(method == ExpApproximation::PRODUCT)
      return detail::fast_exp::Product<T, degree>::evaluate(x);
   else
      return detail::fast_exp::Taylor<T, degree>::evaluate(x);
}

} // namespace perceive
