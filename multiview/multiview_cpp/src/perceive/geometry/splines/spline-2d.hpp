
#pragma once

#include <deque>
#include <vector>

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
class Spline2d
{
 public:
   typedef Vector2 eval_type;

   CUSTOM_NEW_DELETE(Spline2d)

   // @{ // Construction
   Spline2d();
   Spline2d(const Spline2d& rhs);
   Spline2d(Spline2d&& rhs);
   Spline2d(const std::vector<Vector2>& ps, double rho, int M_factor);
   Spline2d(const std::vector<Vector2r>& ps, double rho, int M_factor);
   Spline2d(const std::deque<Vector2>& ps, double rho, int M_factor);
   Spline2d(const std::deque<Vector2r>& ps, double rho, int M_factor);
   ~Spline2d();
   Spline2d& operator=(const Spline2d& rhs);
   Spline2d& operator=(Spline2d&& rhs);
   // @}

   // @{ // init
   // Returns FALSE if there are less than 5 points, or regression fails
   bool init(const std::vector<Vector2>& ps, double rho, int M_factor);
   bool init(const std::vector<Vector2r>& ps, double rho, int M_factor);
   bool init(const std::deque<Vector2>& ps, double rho, int M_factor);
   bool init(const std::deque<Vector2r>& ps, double rho, int M_factor);
   bool is_init() const;
   // @}

   // @{
   uint n_control_points() const;
   double approx_length() const;
   // @}

   // @{ // accessors
   double rms_error_x() const;
   double avg_error_x() const;
   double avg_rel_error_x() const;
   double max_error_x() const;

   double rms_error_y() const;
   double avg_error_y() const;
   double avg_rel_error_y() const;
   double max_error_y() const;

   double rms_error() const { return rms_error_x() + rms_error_y(); }

   Vector2& range_hint();
   const Vector2& range_hint() const;
   // @}

   // Calculates the spline value at t in [0..1]
   Vector2 evaluate(double t) const;

   struct GradientResult
   {
      Vector2 x; // value at 't'
      Vector2 g; // gradient at 't'
   };
   GradientResult gradient(double t) const;

   // @{ // unpack/pack, for saving and restoring
   std::vector<array<double, 12>> unpack() const;
   void init(const std::vector<array<double, 12>>& coefficients) const;
   // @}

   // @{ // interpolate
   // append 'n' points to 'out', in the range [min_t...max_t]
   void interpolate(uint n,
                    std::vector<Vector2>& out,
                    double min_t = 0.0,
                    double max_t = 1.0);

   // Fits a spline using path, and then modifies path elements to
   // represent the spline
   static bool smooth(std::vector<Vector2>& path, double rho, int M_factor);
   // @}

   class Pimpl;
   unique_ptr<Pimpl> pimpl;
};

// First point with the specified y-value, searching from t, with direction
double find_t(const Spline2d& spline,
              double y_value,
              double start_t,
              bool forward,
              double epsilon = 1e-6);

// the intersection of two splines (if it exists and is unique)
Vector2 spline_spline_intersection(const Spline2d& a,
                                   const Spline2d& b,
                                   const real threshold = 1e-4);

} // namespace perceive
