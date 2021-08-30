
#include <alglib/ap.h>
#include <alglib/interpolation.h>

#include "spline-1d.hpp"

#define This Spline1d

namespace perceive
{
// ----------------------------------------------------------- inclusive-between

// inclusive_between<int>(5, x, 8);
template<typename T>
inline bool
inclusive_between(const T& low_bound, const T& value, const T& high_bound)
{
   return value <= high_bound && value >= low_bound;
}

// ----------------------------------------------------------------------- Pimpl

class This::Pimpl
{
 public:
   Pimpl()
       : is_init(false)
       , n_control_points(0)
       , approx_length(dNAN)
   {}

   bool is_init;
   uint n_control_points;
   double approx_length;
   alglib::spline1dinterpolant spline_x;
   alglib::spline1dfitreport rep_x;

   double min_x{0.0};
   double max_x{1.0};
   double range_inv{1.0}; // 1.0 / (max_x - min_x)

   void update_range_inv() { range_inv = 1.0 / (max_x - min_x); }
};

// ---------------------------------------------------------------- Construction

This::Spline1d()
    : pimpl(make_unique<Pimpl>())
{}

This::Spline1d(const Spline1d& rhs)
    : Spline1d()
{
   *this = rhs;
}

This::Spline1d(Spline1d&& rhs)
    : pimpl{nullptr}
{
   *this = std::move(rhs);
}

This::Spline1d(const std::vector<double>& ps, double rho, int M_factor)
    : Spline1d()
{
   init(ps, rho, M_factor);
}
This::Spline1d(const std::vector<float>& ps, double rho, int M_factor)
    : Spline1d()
{
   init(ps, rho, M_factor);
}
This::Spline1d(const std::vector<Vector2>& ps, double rho, int M_factor)
    : Spline1d()
{
   init(ps, rho, M_factor);
}
This::Spline1d(const std::deque<double>& ps, double rho, int M_factor)
    : Spline1d()
{
   init(ps, rho, M_factor);
}
This::Spline1d(const std::deque<float>& ps, double rho, int M_factor)
    : Spline1d()
{
   init(ps, rho, M_factor);
}
This::Spline1d(const std::deque<Vector2>& ps, double rho, int M_factor)
    : Spline1d()
{
   init(ps, rho, M_factor);
}

This::Spline1d(const std::vector<array<double, 6>>& coefficients)
    : Spline1d()
{
   init(coefficients);
}

This::~Spline1d() = default;

Spline1d& This::operator=(const Spline1d& rhs)
{
   if(this != &rhs) *pimpl = *rhs.pimpl;
   return *this;
}

Spline1d& This::operator=(Spline1d&& rhs) = default;

// ------------------------------------------------------------------- Accessors

bool This::is_init() const { return pimpl->is_init; }
uint This::n_control_points() const { return pimpl->n_control_points; }
double This::approx_length() const { return pimpl->approx_length; }

double This::rms_error() const { return pimpl->rep_x.rmserror; }
double This::avg_error() const { return pimpl->rep_x.avgerror; }
double This::avg_rel_error() const { return pimpl->rep_x.avgrelerror; }
double This::max_error() const { return pimpl->rep_x.maxerror; }

real This::min_x() const { return pimpl->min_x; }
real This::max_x() const { return pimpl->max_x; }
void This::set_min_x(real x)
{
   pimpl->min_x = x;
   pimpl->update_range_inv();
}
void This::set_max_x(real x)
{
   pimpl->max_x = x;
   pimpl->update_range_inv();
}

// -------------------------------------------------------------------- p-spline

/*************************************************************************
Fitting by penalized cubic spline.

Equidistant grid with M nodes on [min(x,xc),max(x,xc)] is  used  to  build
basis functions. Basis functions are cubic splines with  natural  boundary
conditions. Problem is regularized by  adding non-linearity penalty to the
usual least squares penalty function:

    S(x) = arg min { LS + P }, where
    LS   = SUM { w[i]^2*(y[i] - S(x[i]))^2 } - least squares penalty
    P    = C*10^rho*integral{ S''(x)^2*dx } - non-linearity penalty
    rho  - tunable constant given by user
    C    - automatically determined scale parameter,
           makes penalty invariant with respect to scaling of X, Y, W.

INPUT PARAMETERS:
    X   -   points, array[0..N-1].
    Y   -   function values, array[0..N-1].
    N   -   number of points (optional):
            * N>0
            * if given, only first N elements of X/Y are processed
            * if not given, automatically determined from X/Y sizes
    M   -   number of basis functions ( = number_of_nodes), M>=4.
    Rho -   regularization  constant  passed   by   user.   It   penalizes
            nonlinearity in the regression spline. It  is  logarithmically
            scaled,  i.e.  actual  value  of  regularization  constant  is
            calculated as 10^Rho. It is automatically scaled so that:
            * Rho=2.0 corresponds to moderate amount of nonlinearity
            * generally, it should be somewhere in the [-8.0,+8.0]
            If you do not want to penalize nonlineary,
            pass small Rho. Values as low as -15 should work.

OUTPUT PARAMETERS:
    Info-   same format as in LSFitLinearWC() subroutine.
            * Info>0    task is solved
            * Info<=0   an error occured:
                        -4 means inconvergence of internal SVD or
                           Cholesky decomposition; problem may be
                           too ill-conditioned (very rare)
    S   -   spline interpolant.
    Rep -   Following fields are set:
            * RMSError      rms error on the (X,Y).
            * AvgError      average error on the (X,Y).
            * AvgRelError   average relative error on the non-zero Y
            * MaxError      maximum error
                            NON-WEIGHTED ERRORS ARE CALCULATED

IMPORTANT:
    this subroitine doesn't calculate task's condition number for K<>0.

NOTE 1: additional nodes are added to the spline outside  of  the  fitting
interval to force linearity when x<min(x,xc) or x>max(x,xc).  It  is  done
for consistency - we penalize non-linearity  at [min(x,xc),max(x,xc)],  so
it is natural to force linearity outside of this interval.

NOTE 2: function automatically sorts points,  so  caller may pass unsorted
array.

  -- ALGLIB PROJECT --
     Copyright 18.08.2009 by Bochkanov Sergey
*************************************************************************/

template<typename U>
bool init_pspline(Spline1d& curve,
                  const U& container,
                  double rho,
                  unsigned M_factor)
{
   curve.pimpl->n_control_points = 0;
   curve.pimpl->is_init          = false;
   curve.pimpl->approx_length    = dNAN;

   const uint32_t len = uint32_t(container.size());
   if(len < 5) return false;

   alglib::real_1d_array T;
   alglib::real_1d_array X;
   T.setlength(len);
   X.setlength(len);

   alglib::ae_int_t counter = 0;
   double t                 = 0.0;
   double dt                = 1.0 / double(len - 1);
   for(const auto& p : container) {
      T[counter] = t;
      X[counter] = real(p);
      t += dt;
      counter++;
   }

   alglib::ae_int_t info_x;
   alglib::ae_int_t M = std::max<int>(4, int(M_factor));

   alglib::spline1dfitpenalized(
       T, X, M, rho, info_x, curve.pimpl->spline_x, curve.pimpl->rep_x);

   curve.pimpl->n_control_points = unsigned(M);

   if(info_x > 0) {
      curve.pimpl->is_init       = true;
      curve.pimpl->approx_length = 0.0;
      double dt                  = 0.1 / double(M - 1);
      real p                     = curve.evaluate_t(0.0);
      for(double t = dt; t <= 1.0; t += dt) {
         real q = curve.evaluate_t(t);
         curve.pimpl->approx_length += fabs(p - q);
         p = q;
      }
   }

   return curve.pimpl->is_init;
}

template<typename U>
bool init_pspline_xy(Spline1d& curve,
                     const U& container,
                     double rho,
                     unsigned M_factor)
{
   curve.pimpl->n_control_points = 0;
   curve.pimpl->is_init          = false;
   curve.pimpl->approx_length    = dNAN;

   const unsigned len = unsigned(container.size());
   if(len < 5) return false;

   alglib::real_1d_array X;
   alglib::real_1d_array Y;
   X.setlength(len);
   Y.setlength(len);

   double min_x = std::numeric_limits<double>::max();
   double max_x = std::numeric_limits<double>::lowest();

   uint counter = 0;
   for(const auto& p : container) {
      X[counter] = p.x;
      Y[counter] = p.y;
      if(p.x < min_x) min_x = p.x;
      if(p.x > max_x) max_x = p.x;
      counter++;
   }

   double range_inv = 1.0 / (max_x - min_x);

   // Now scale the 'X' values
   for(unsigned i = 0; i < len; ++i) X[i] = (X[i] - min_x) * range_inv;

   alglib::ae_int_t info_x;
   alglib::ae_int_t M = std::max<int>(4, int(M_factor));

   alglib::spline1dfitpenalized(
       X, Y, M, rho, info_x, curve.pimpl->spline_x, curve.pimpl->rep_x);

   curve.pimpl->n_control_points = unsigned(M);

   if(info_x > 0) {
      curve.pimpl->is_init       = true;
      curve.pimpl->approx_length = 0.0;
      curve.set_min_x(min_x);
      curve.set_max_x(max_x);
   }

   return curve.pimpl->is_init;
}

// ------------------------------------------------------------------------ Init

bool This::init(const std::vector<double>& ps, double rho, int M_factor)
{
   return init_pspline(*this, ps, rho, unsigned(M_factor));
}

bool This::init(const std::deque<double>& ps, double rho, int M_factor)
{
   return init_pspline(*this, ps, rho, unsigned(M_factor));
}

bool This::init(const std::vector<float>& ps, double rho, int M_factor)
{
   return init_pspline(*this, ps, rho, unsigned(M_factor));
}

bool This::init(const std::deque<float>& ps, double rho, int M_factor)
{
   return init_pspline(*this, ps, rho, unsigned(M_factor));
}

bool This::init(const std::vector<Vector2>& ps, double rho, int M_factor)
{
   return init_pspline_xy(*this, ps, rho, unsigned(M_factor));
}

bool This::init(const std::deque<Vector2>& ps, double rho, int M_factor)
{
   return init_pspline_xy(*this, ps, rho, unsigned(M_factor));
}

// -------------------------------------------------------------------- Evaluate

real This::evaluate_t(double t) const
{
   // assert(t >= 0.0 && t <= 1.0);
   return is_init() ? real(alglib::spline1dcalc(pimpl->spline_x, t))
                    : real(NAN);
}

real This::evaluate(double x) const
{
   // assert(min-x >= 0.0 && max-x <= 1.0);
   // cout << format("t = {}", (x - pimpl->min_x) * pimpl->range_inv) << endl;
   return evaluate_t((x - pimpl->min_x) * pimpl->range_inv);
}

// ---------------------------------------------------------------------- Unpack

std::vector<array<double, 6>> This::unpack() const
{
   std::vector<array<double, 6>> out;
   unpack(out);
   return out;
}

void This::unpack(std::vector<array<double, 6>>& out) const
{
   alglib::ae_int_t nx{0};
   alglib::real_2d_array tbl_x;
   spline1dunpack(pimpl->spline_x, nx, tbl_x);

   size_t N = size_t(tbl_x.rows());

   out.resize(N + 1);
   for(size_t ind = 0; ind < N; ++ind) {
      auto& coeff = out[size_t(ind)];
      auto ptr    = &coeff[0];
      for(unsigned i = 0; i < 6; ++i) *ptr++ = tbl_x(alglib::ae_int_t(ind), i);
   }

   std::fill(out[N].begin(), out[N].end(), 0.0);
   out[N][0] = pimpl->min_x;
   out[N][1] = pimpl->max_x;
}

void This::init(const std::vector<array<double, 6>>& coefficients)
{
   using alglib::spline1dinterpolant;
   auto make_interpolant = [&](spline1dinterpolant& interpolant) {
      const unsigned n = unsigned(coefficients.size());
      alglib::real_1d_array x, y, d; // (x, f(x), f'(x))
      x.setlength(n);
      y.setlength(n);
      d.setlength(n);
      unsigned pos = 0; // write position

      auto write = [&](double t,
                       double X0,
                       double X1,
                       double C0,
                       double C1,
                       double C2,
                       double C3) {
         x(pos) = X0 + t * (X1 - X0);
         y(pos) = C0 + C1 * t + C2 * t * t + C3 * t * t * t;
         d(pos) = C1 + 2.0 * C2 * t + 3.0 * C3 * t * t;
         pos++;
      };

      for(unsigned ind = 0; ind < coefficients.size() - 1; ++ind) {
         const auto& CC = coefficients[ind];
         auto X0        = CC[0];
         auto X1        = CC[1];
         auto C0        = CC[2];
         auto C1        = CC[3];
         auto C2        = CC[4];
         auto C3        = CC[5];
         write(0.0, X0, X1, C0, C1, C2, C3);

         // Write that final coefficient
         if(ind == coefficients.size() - 2) write(1.0, X0, X1, C0, C1, C2, C3);
      }

      if(pos != n) { FATAL(format("pos = {} != {} = n", pos, n)); }

      alglib::spline1dbuildhermite(x, y, d, interpolant);
   };

   make_interpolant(pimpl->spline_x);

   pimpl->is_init           = true;
   pimpl->n_control_points  = unsigned(coefficients.size());
   pimpl->rep_x.rmserror    = 0.0;
   pimpl->rep_x.avgerror    = 0.0;
   pimpl->rep_x.maxerror    = 0.0;
   pimpl->rep_x.avgrelerror = 0.0;
   pimpl->min_x             = coefficients.back()[0];
   set_max_x(coefficients.back()[1]);
}

// ----------------------------------------------------------------- Interpolate

void This::interpolate_t(unsigned n,
                         std::vector<real>& out,
                         double min_t,
                         double max_t)
{
   assert(min_t >= 0.0 && min_t <= 1.0);
   assert(max_t >= 0.0 && max_t <= 1.0);
   assert(min_t <= max_t);

   out.reserve(n);

   if(n == 0)
      return;
   else if(n == 1)
      out.push_back(evaluate_t(max_t));
   else {
      double t  = min_t;
      double dt = (max_t - min_t) / double(n - 1);
      for(unsigned i = 0; i < n - 1; ++i) {
         out.push_back(evaluate_t(clamp(t, min_t, max_t)));
         t += dt;
      }
      out.push_back(evaluate_t(max_t));
   }

   assert(out.size() == n);
}

// ---------------------------------------------------------------------- Smooth

bool This::smooth(std::vector<real>& path, double rho, int M_factor)
{
   if(path.size() < 5) return false;
   Spline1d spline;
   if(!spline.init(path, rho, M_factor)) return false;
   std::vector<real> t;
   spline.interpolate_t(unsigned(path.size()), t);
   if(!(t.size() == path.size()))
      FATAL(format("Container size mismatch: expected {}, but got {}",
                   path.size(),
                   t.size()));
   auto src = t.begin();
   auto dst = path.begin();
   while(src != t.end()) *src++ = *dst++;
   return true;
}

} // namespace perceive
