
#include <alglib/ap.h>
#include <alglib/interpolation.h>

#include "spline-2d.hpp"

#define This Spline2d

namespace perceive
{
using std::cout;
using std::endl;

// ----------------------------------------------------------------------- Pimpl

class This::Pimpl
{
 public:
   Pimpl()
       : is_init(false)
       , n_control_points(0)
       , approx_length(dNAN)
   {}

   Vector2 range_hint{0.0, 1.0};
   bool is_init;
   uint n_control_points;
   double approx_length;
   alglib::spline1dinterpolant spline_x;
   alglib::spline1dinterpolant spline_y;
   alglib::spline1dfitreport rep_x;
   alglib::spline1dfitreport rep_y;
};

// ---------------------------------------------------------------- Construction

This::Spline2d()
    : pimpl(make_unique<Pimpl>())
{}

This::Spline2d(const Spline2d& rhs)
    : Spline2d()
{
   *this = rhs;
}
This::Spline2d(Spline2d&& rhs)
    : pimpl{nullptr}
{
   *this = std::move(rhs);
}

This::Spline2d(const std::vector<Vector2>& ps, double rho, int M_factor)
    : Spline2d()
{
   init(ps, rho, M_factor);
}
This::Spline2d(const std::vector<Vector2r>& ps, double rho, int M_factor)
    : Spline2d()
{
   init(ps, rho, M_factor);
}
This::Spline2d(const std::deque<Vector2>& ps, double rho, int M_factor)
    : Spline2d()
{
   init(ps, rho, M_factor);
}
This::Spline2d(const std::deque<Vector2r>& ps, double rho, int M_factor)
    : Spline2d()
{
   init(ps, rho, M_factor);
}

This::~Spline2d() = default;

Spline2d& This::operator=(const Spline2d& rhs)
{
   if(this != &rhs) *pimpl = *rhs.pimpl;
   return *this;
}

Spline2d& This::operator=(Spline2d&& rhs) = default;

// ------------------------------------------------------------------- Accessors

bool This::is_init() const { return pimpl->is_init; }
uint This::n_control_points() const { return pimpl->n_control_points; }
double This::approx_length() const { return pimpl->approx_length; }

double This::rms_error_x() const { return pimpl->rep_x.rmserror; }
double This::avg_error_x() const { return pimpl->rep_x.avgerror; }
double This::avg_rel_error_x() const { return pimpl->rep_x.avgrelerror; }
double This::max_error_x() const { return pimpl->rep_x.maxerror; }

double This::rms_error_y() const { return pimpl->rep_y.rmserror; }
double This::avg_error_y() const { return pimpl->rep_y.avgerror; }
double This::avg_rel_error_y() const { return pimpl->rep_y.avgrelerror; }
double This::max_error_y() const { return pimpl->rep_y.maxerror; }

Vector2& This::range_hint() { return pimpl->range_hint; }
const Vector2& This::range_hint() const { return pimpl->range_hint; }

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
bool init_pspline(Spline2d& curve,
                  const U& container,
                  double rho,
                  unsigned M_factor)
{
   curve.pimpl->n_control_points = 0;
   curve.pimpl->is_init          = false;
   curve.pimpl->approx_length    = dNAN;
   curve.pimpl->range_hint       = Vector2(0.0, 1.0);

   const uint32_t len = uint32_t(container.size());
   if(len < 5) return false;

   alglib::real_1d_array T;
   alglib::real_1d_array X;
   alglib::real_1d_array Y;
   auto extra = 0u;
   T.setlength(len + extra);
   X.setlength(len + extra);
   Y.setlength(len + extra);

   uint counter = 0;
   double t     = 0.0;
   double dt    = 1.0 / double(len - 1);
   for(const auto& p : container) {
      T[counter] = t;
      X[counter] = p(0);
      Y[counter] = p(1);
      t += dt;
      counter++;
   }

   // Add the last point in five more times... to get boundary
   for(auto i = 0u; i < extra; ++i) {
      T[len + i] = 1.0;
      X[len + i] = container.back()(0);
      Y[len + i] = container.back()(1);
   }

   alglib::ae_int_t info_x, info_y;
   alglib::ae_int_t M = std::max<int>(4, int(M_factor));

   alglib::spline1dfitpenalized(
       T, X, M, rho, info_x, curve.pimpl->spline_x, curve.pimpl->rep_x);
   alglib::spline1dfitpenalized(
       T, Y, M, rho, info_y, curve.pimpl->spline_y, curve.pimpl->rep_y);

   curve.pimpl->n_control_points = unsigned(M);

   if(info_x > 0 || info_y > 0) {
      curve.pimpl->is_init       = true;
      curve.pimpl->approx_length = 0.0;
      double dt                  = 0.1 / double(M - 1);
      Vector2 p                  = curve.evaluate(0.0);
      for(double t = dt; t <= 1.0; t += dt) {
         Vector2 q = curve.evaluate(t);
         curve.pimpl->approx_length += (p - q).norm();
         p = q;
      }
   }

   return curve.pimpl->is_init;
}

// ------------------------------------------------------------------------ Init

bool This::init(const std::vector<Vector2>& ps, double rho, int M_factor)
{
   return init_pspline(*this, ps, rho, unsigned(M_factor));
}

bool This::init(const std::deque<Vector2>& ps, double rho, int M_factor)
{
   return init_pspline(*this, ps, rho, unsigned(M_factor));
}

bool This::init(const std::vector<Vector2r>& ps, double rho, int M_factor)
{
   return init_pspline(*this, ps, rho, unsigned(M_factor));
}

bool This::init(const std::deque<Vector2r>& ps, double rho, int M_factor)
{
   return init_pspline(*this, ps, rho, unsigned(M_factor));
}

// -------------------------------------------------------------------- Evaluate

Vector2 This::evaluate(double t) const
{
   // assert(t >= 0.0 && t <= 1.0);

   return is_init() ? Vector2(alglib::spline1dcalc(pimpl->spline_x, t),
                              alglib::spline1dcalc(pimpl->spline_y, t))
                    : Vector2::nan();
}

This::GradientResult This::gradient(double t) const
{
   GradientResult x;

   if(is_init()) {
      double s, ds, ds2;
      double r, dr, dr2;
      alglib::spline1ddiff(pimpl->spline_x, t, s, ds, ds2);
      alglib::spline1ddiff(pimpl->spline_y, t, r, dr, dr2);
      x.x(0) = s;
      x.x(1) = r;
      x.g(0) = -dr;
      x.g(1) = ds;
      x.g.normalise();
   } else {
      x.x = x.g = Vector2::nan();
   }

   return x;
}

// ---------------------------------------------------------------------- Unpack

std::vector<array<double, 12>> This::unpack() const
{
   alglib::ae_int_t nx{0};
   alglib::ae_int_t ny{0};
   alglib::real_2d_array tbl_x, tbl_y;
   spline1dunpack(pimpl->spline_x, nx, tbl_x);
   spline1dunpack(pimpl->spline_y, ny, tbl_y);

   if(nx != ny)
      FATAL(format("How did this happen? nx = {} != {} = ny", nx, ny));

   auto N = tbl_x.rows();

   // INFO(format("nx = {}, len = ({}, {})", nx, tbl_x.rows(),tbl_x.cols()));

   std::vector<array<double, 12>> out;
   out.resize(size_t(N));
   for(alglib::ae_int_t ind = 0; ind < N; ++ind) {
      auto& coeff = out[size_t(ind)];
      auto ptr    = &coeff[0];
      for(unsigned i = 0; i < 6; ++i) *ptr++ = tbl_x(ind, i);
      for(unsigned i = 0; i < 6; ++i) *ptr++ = tbl_y(ind, i);
   }

   return out;
}

void This::init(const std::vector<array<double, 12>>& coefficients) const
{
   using alglib::spline1dinterpolant;

   auto make_interpolant = [&](bool is_x, spline1dinterpolant& interpolant) {
      unsigned offset = (is_x) ? 0 : 6;

      const unsigned dilate   = 1; // number of point per cooef set
      const double dilate_inv = 1.0 / double(dilate);
      const unsigned n        = unsigned(coefficients.size() * dilate + 1);
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

      for(unsigned ind = 0; ind < coefficients.size(); ++ind) {
         const auto& CC = coefficients[ind];
         auto X0        = CC[0 + offset];
         auto X1        = CC[1 + offset];
         auto C0        = CC[2 + offset];
         auto C1        = CC[3 + offset];
         auto C2        = CC[4 + offset];
         auto C3        = CC[5 + offset];
         for(unsigned i = 0; i < dilate; ++i)
            write(dilate_inv * double(i), X0, X1, C0, C1, C2, C3);

         // Write that final coefficient
         if(ind == coefficients.size() - 1) write(1.0, X0, X1, C0, C1, C2, C3);
      }

      assert(pos == n);

      alglib::spline1dbuildhermite(x, y, d, interpolant);
   };

   spline1dinterpolant spline_x, spline_y;
   make_interpolant(true, spline_x);
   make_interpolant(false, spline_y);

   pimpl->spline_x         = spline_x;
   pimpl->spline_y         = spline_y;
   pimpl->is_init          = true;
   pimpl->n_control_points = unsigned(coefficients.size());
   pimpl->rep_x.rmserror = pimpl->rep_y.rmserror = 0.0;
   pimpl->rep_x.avgerror = pimpl->rep_y.avgerror = 0.0;
   pimpl->rep_x.maxerror = pimpl->rep_y.maxerror = 0.0;
   pimpl->rep_x.avgrelerror = pimpl->rep_y.avgrelerror = 0.0;
}

// ----------------------------------------------------------------- Interpolate

void This::interpolate(unsigned n,
                       std::vector<Vector2>& out,
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
      out.push_back(evaluate(max_t));
   else {
      double t  = min_t;
      double dt = (max_t - min_t) / double(n - 1);
      for(unsigned i = 0; i < n - 1; ++i) {
         out.push_back(evaluate(clamp(t, min_t, max_t)));
         t += dt;
      }
      out.push_back(evaluate(max_t));
   }

   assert(out.size() == n);
}

// ---------------------------------------------------------------------- Smooth

bool This::smooth(std::vector<Vector2>& path, double rho, int M_factor)
{
   if(path.size() < 5) return false;
   Spline2d spline;
   if(!spline.init(path, rho, M_factor)) return false;
   std::vector<Vector2> t;
   spline.interpolate(unsigned(path.size()), t);
   if(!(t.size() == path.size()))
      FATAL(format("Container size mismatch: expected {}, but got {}",
                   path.size(),
                   t.size()));
   auto src = t.begin();
   auto dst = path.begin();
   while(src != t.end()) *src++ = *dst++;
   return true;
}

// ---------------------------------------------------------------------- Find-t

double find_t(const Spline2d& spline,
              double y_value,
              double start_t,
              bool forward,
              double epsilon)
{
   if(!spline.is_init()) return double(NAN);

   uint n_cps = spline.n_control_points();
   assert(n_cps > 0);
   assert(start_t >= 0 && start_t <= 1.0);
   double step = (forward ? 1.0 : -1.0) * 0.5 / double(n_cps);

   double last_t  = start_t;
   Vector2 last_v = spline.evaluate(start_t);
   double next_t  = last_t + step;
   if(next_t <= 0.0) next_t = 0.0;
   if(next_t >= 1.0) next_t = 1.0;
   Vector2 next_v = spline.evaluate(next_t);

   auto is_between = [&]() {
      return (last_v.y < next_v.y)
                 ? inclusive_between<double>(last_v.y, y_value, next_v.y)
                 : inclusive_between<double>(next_v.y, y_value, last_v.y);
   };

   while(inclusive_between(0.0, next_t, 1.0) && !is_between()) {
      last_v = next_v;
      last_t = next_t;
      next_t += step;
      if(inclusive_between(0.0, next_t, 1.0)) next_v = spline.evaluate(next_t);
   }

   if(!is_between()) return dNAN;

   // Okay now we have to do a binary search to find 't'
   struct S
   {
      S()
          : y(dNAN)
          , t(dNAN)
      {}
      S(double y_, double t_)
          : y(y_)
          , t(t_)
      {}
      double y;
      double t;
   };
   S upper(last_v.y, last_t);
   S lower(next_v.y, next_t);
   if(upper.y < lower.y) { // swap
      S tmp(upper);
      upper = lower;
      lower = tmp;
   }

   double error = 1.0;
   uint counter = 0;
   S mid;
   while(error > epsilon) {
      if(counter++ > 100) FATAL(format("Failed to find 't' value!"));

      mid.t = 0.5 * (upper.t + lower.t);
      mid.y = spline.evaluate(mid.t).y;
      error = fabs(mid.y - y_value);

      bool go_low = inclusive_between(lower.y, y_value, mid.y)
                    || inclusive_between(mid.y, y_value, lower.y);
      bool go_high = inclusive_between(upper.y, y_value, mid.y)
                     || inclusive_between(mid.y, y_value, upper.y);
      if((go_low && go_high) || (!go_low && !go_high))
         LOG_ERR(format("t escaped"));

      if(go_low) upper = mid;
      if(go_high) lower = mid;
   }
   // INFO(format("counter = {}") % counter);
   return mid.t;
}

// --------------------------------------------------------------- Intersections

Vector2 spline_spline_intersection(const Spline2d& a,
                                   const Spline2d& b,
                                   const real threshold) // pixels
{
   // Have to find two 't' values (ta, tb) such that
   // a.evaluate(ta) == b.evaluate(tb)

   array<double, 3> tas, tbs;
   array<Vector2, 3> vas, vbs;

   // This _could_ be more efficient: two*golden-section at worst
   // However, 'intersections' is not the bottle-neck for this
   // cost-function. (Refitting the splines is.)
   auto update = [&](double ta, double tb, double range) {
      tas[0] = clamp(ta - 0.5 * range, 0.0, 1.0);
      tas[1] = ta;
      tas[2] = clamp(ta + 0.5 * range, 0.0, 1.0);
      tbs[0] = clamp(tb - 0.5 * range, 0.0, 1.0);
      tbs[1] = tb;
      tbs[2] = clamp(tb + 0.5 * range, 0.0, 1.0);
      for(unsigned i = 0; i < tas.size(); ++i) {
         vas[i] = a.evaluate(tas[i]);
         vbs[i] = b.evaluate(tbs[i]);
      }

      // which are the closest?
      unsigned best_i{0}, best_j{0};
      real best_dist = std::numeric_limits<real>::max();
      for(unsigned i = 0; i < tas.size(); ++i) {
         for(unsigned j = 0; j < tbs.size(); ++j) {
            auto dist = vas[i].quadrance(vbs[j]);
            if(dist < best_dist) {
               best_dist = dist;
               best_i    = i;
               best_j    = j;
            }
         }
      }

      auto ret = Vector3(tas[best_i], tbs[best_j], best_dist);

      if(false) {
         INFO("REPORT");
         cout << format("tas = [{}]", implode(tas.begin(), tas.end(), ", "))
              << endl;
         cout << format("tbs = [{}]", implode(tbs.begin(), tbs.end(), ", "))
              << endl;
         cout << format("vas = [{}]", implode(vas.begin(), vas.end(), ", "))
              << endl;
         cout << format("vbs = [{}]", implode(vbs.begin(), vbs.end(), ", "))
              << endl;
         cout << format("ret = {}", str(ret)) << endl;

         fgetc(stdin);
      }

      return ret;
   };

   double range = 1.0;
   double ta{0.0}, tb{0.0};
   Vector3 val(0.5, 0.5, std::numeric_limits<real>::max());
   unsigned counter = 0;
   while(val.z > (threshold * threshold)) {
      if(counter++ > 100) { break; }
      val = update(val.x, val.y, range);
      range *= 0.5;
   }

   return 0.5 * (a.evaluate(val.x) + b.evaluate(val.y));
}

} // namespace perceive
