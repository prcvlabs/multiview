
#pragma once

#include <deque>
#include <vector>

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
class Spline1d
{
 public:
   typedef real eval_type;

   CUSTOM_NEW_DELETE(Spline1d)

   // @{ // Construction
   Spline1d();
   Spline1d(const Spline1d& rhs);
   Spline1d(Spline1d&& rhs);
   Spline1d(const std::vector<double>& ps, double rho, int M_factor);
   Spline1d(const std::vector<float>& ps, double rho, int M_factor);
   Spline1d(const std::vector<Vector2>& ps, double rho, int M_factor);
   Spline1d(const std::deque<double>& ps, double rho, int M_factor);
   Spline1d(const std::deque<float>& ps, double rho, int M_factor);
   Spline1d(const std::deque<Vector2>& ps, double rho, int M_factor);
   Spline1d(const std::vector<array<double, 6>>& coefficients);
   ~Spline1d();
   Spline1d& operator=(const Spline1d& rhs);
   Spline1d& operator=(Spline1d&& rhs);
   // @}

   // @{ // init
   // Returns FALSE if there are less than 5 points, or regression fails
   bool init(const std::vector<double>& ps, double rho, int M_factor);
   bool init(const std::vector<float>& ps, double rho, int M_factor);
   bool init(const std::deque<double>& ps, double rho, int M_factor);
   bool init(const std::deque<float>& ps, double rho, int M_factor);

   // This will set the spline min-x, max-x, to produce the
   // smoothed function f(x) from (x, y) pairs.
   bool init(const std::vector<Vector2>& ps, double rho, int M_factor);
   bool init(const std::deque<Vector2>& ps, double rho, int M_factor);

   bool is_init() const;
   // @}

   // @{
   uint n_control_points() const;
   double approx_length() const;
   // @}

   // @{ // accessors
   double rms_error() const;
   double avg_error() const;
   double avg_rel_error() const;
   double max_error() const;

   // This allows you to scale the spline
   real min_x() const;
   real max_x() const;
   void set_min_x(real x);
   void set_max_x(real x);
   // @}

   // Calculates the spline value at t in [0..1]
   real evaluate_t(double t) const;

   // Calculates the spline value at 'x' in [min-x..max-x]
   real evaluate(double x) const;

   // @{ // unpack/pack, for saving and restoring
   std::vector<array<double, 6>> unpack() const;
   void unpack(std::vector<array<double, 6>>& out) const;
   void init(const std::vector<array<double, 6>>& coefficients);
   // @}

   // @{ // interpolate
   // append 'n' points to 'out', in the range [min_t...max_t]
   void interpolate_t(uint n,
                      std::vector<real>& out,
                      double min_t = 0.0,
                      double max_t = 1.0);

   // Fits a spline using path, and then modifies path elements to
   // represent the spline
   static bool smooth(std::vector<real>& path, double rho, int M_factor);
   // @}

   class Pimpl;
   unique_ptr<Pimpl> pimpl;
};

} // namespace perceive
