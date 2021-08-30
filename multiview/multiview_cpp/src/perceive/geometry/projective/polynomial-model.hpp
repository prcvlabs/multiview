
#pragma once

#include "perceive/calibration/calibration-utils.hpp"
#include "perceive/geometry/polygon.hpp"
#include "perceive/geometry/surface/rbf-field.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/io/json-io.hpp"
#include "stdinc.hpp"

#include "json/json.h"

namespace perceive
{
// -----------------------------------------------------------------------------
// --                          PolynomialModel<K>                             --
// -----------------------------------------------------------------------------
// Rational distortion model of degree 'K'
template<unsigned K_ = 2> class PolynomialModel
{
 public:
   static constexpr unsigned K  = K_;
   static constexpr unsigned KK = (K + 1) * (K + 2) / 2;

 private:
   MatrixXr A_{MatrixXr::Zero(2, KK)};

   // Matrix 'A' is calculated on conditioned data,
   // So we need to 'condition' distorted points
   Vector2 C{0.0, 0.0};   // radial center
   real s{0.0};           // scale
   Point2 format_{0, 0};  // width/height -- of calibration
   Vector2 ws_{1.0, 1.0}; // working scale
   string sensor_id_{""};
   AABB calib_region_;          // distorted image region that's calibrated
   vector<Vector2> calib_hull_; // convex hull for calibrated region

   // UndistortInverse undist_inv;
   RBFField distort_field_; // NOT thread safe
   shared_ptr<const EmpiricalFieldF> empirical_distort_field_;
   bool distort_field_is_init_{false};
   Vector2 distort_field_evaluate_(const Vector2& U) const;

 public:
   CUSTOM_NEW_DELETE(PolynomialModel)

   using VectorKKr = Eigen::Matrix<real, KK, 1, EIGEN_ALIGN>;
   static string model_name()
   {
      return ::perceive::format("polynomial<{}>", K);
   }

   // ---------------------------------------------------------------- Equality

   bool operator==(const PolynomialModel<K>& o) const noexcept;
   bool operator!=(const PolynomialModel<K>& o) const noexcept
   {
      return !(*this == o);
   }

   // --------------------------------------------------------- Getters/Setters

   const Vector2& radial_epipole() const noexcept { return C; }
   real scale() const noexcept { return s; }
   const Point2& calib_format() const noexcept { return format_; }
   const string& sensor_id() const noexcept { return sensor_id_; }
   const MatrixXr& A() const noexcept { return A_; }
   const AABB& calib_region() const noexcept { return calib_region_; }
   const auto& calib_hull() const noexcept { return calib_hull_; }

   Vector2 working_format() const noexcept
   {
      return Vector2(real(format_.x) / ws_.x, real(format_.y) / ws_.y);
   }

   void init(const string& sensor_id,
             const unsigned w,
             const unsigned h,
             const Vector2& radial_epipole,
             const real average_radial_distance,
             const vector<Vector2> hull) noexcept
   {
      sensor_id_    = sensor_id;
      format_       = Point2(int(w), int(h));
      C             = radial_epipole;
      s             = std::sqrt(2.0) / average_radial_distance;
      calib_hull_   = hull;
      calib_region_ = AABB::minmax();
      for(const auto& x : hull) calib_region_.union_point(x);
   }

   bool point_in_calib_region(const Vector2& x) const noexcept
   {
      return point_in_polygon(
          working_to_calib_format(x), cbegin(calib_hull_), cend(calib_hull_));
   }

   void set_working_format(const unsigned w, const unsigned h)
   {
      ws_ = Vector2(real(format_.x) / real(w), real(format_.y) / real(h));
   }

   // void set_working_format(const int w, const int h)
   // {
   //    Expects(w >= 0);
   //    Expects(h >= 0);
   //    set_working_format(unsigned(w), unsigned(h));
   // }

   template<typename T> T working_to_calib_format(const T& X) const noexcept
   {
      auto Y = X;
      Y(0) *= ws_(0);
      Y(1) *= ws_(1);
      return Y;
   }

   template<typename T> T calib_to_working_format(const T& X) const noexcept
   {
      auto Y = X;
      Y(0) /= ws_(0);
      Y(1) /= ws_(1);
      return Y;
   }

   // Call this after setting parameters, including 'set-working-format'
   void finalize(const bool feedback = false);

   // ---------------------------------------------------- Parameter Estimation

   real estimate_LLS(const vector<Vector2>& Ds,    // Distorted (unconditioned)
                     const vector<Vector2>& Us,    // Undistorted points
                     const Vector2 radial_epipole, // to condition...
                     const real average_radial_dist); // to condition...

   struct LLSLineData
   {
      vector<Vector3r> Xs; // Must NOT be conditioned
      Vector3r line_eq;
   };
   real estimate_LLS_line_and_pts(const vector<LLSLineData>& line_data,
                                  const vector<Vector3r>& Ds,
                                  const vector<Vector3r>& Us);

   real projection_error(const vector<LLSLineData>& line_data,
                         const vector<Vector3r>& Ds,
                         const vector<Vector3r>& Us);

   // Given the distortion model, estimates a
   // homography from the well-calibrated region.
   // Returns <H, H-error>
   std::pair<Matrix3r, real> estimate_homography() const noexcept;

   // ------------------------------------------------------- Distort/Undistort

   // produces image point (i.e., ray to image point)
   Vector2 distort(const Vector2& U) const noexcept
   {
      return distort_field_evaluate_(U);
   }

   // produces ideal point (ray) (i.e., image point to ray)
   Vector2 undistort(const Vector2& D) const noexcept
   {
      const VectorKKr X = calibration::lift<K>((D.x * ws_.x - C.x) * s,
                                               (D.y * ws_.y - C.y) * s);
      const Vector2r x  = A_ * X;
      return Vector2(x(0), x(1));
   }

   // --------------------------------------------- Distort/Undistort Functions

   auto distort_f() const noexcept
   {
      return [this](const Vector2& U) { return this->distort(U); };
   }
   auto undistort_f() const noexcept
   {
      return [this](const Vector2& D) { return this->undistort(D); };
   }

   // --------------------------------------------------------------- To-String

   string to_string() const;

   // ---------------------------------------------------------------------- IO

   Json::Value to_json() const;
   void from_json(const Json::Value&) noexcept(false);
   string to_json_string() const { return to_string(); }

   // ------------------------------------------------------------- Convenience

   Vector3 distort(const Vector3& U) const noexcept
   {
      assert(U.is_finite());
      const auto u = distort(homgen_P2_to_R2(U));
      assert(u.is_finite());
      return Vector3(u.x, u.y, 1.0);
   }

   Vector3r distort(const Vector3r& U) const noexcept
   {
      auto u = distort(Vector2(U(0) / U(2), U(1) / U(2)));
      return Vector3r(u.x, u.y, 1.0);
   }

   Vector3 undistort(const Vector3& D) const noexcept
   {
      auto d = undistort(homgen_P2_to_R2(D));
      return Vector3(d.x, d.y, 1.0);
   }

   Vector3r undistort(const Vector3r& D) const noexcept
   {
      auto d = undistort(Vector2(D(0) / D(2), D(1) / D(2)));
      return Vector3r(d.x, d.y, 1.0);
   }

   Vector3 to_ray(const Vector2& D) const noexcept
   {
      auto u = undistort(D);
      return Vector3(u.x, u.y, 1.0).normalised();
   }
};

template<unsigned K> inline string str(const PolynomialModel<K>& m)
{
   return m.to_string();
}

// ------------------------------------------------------------ Extern Templates
// The 'extern' keyword prevents translation units from
// compiling these template definitions.
// @see http://en.cppreference.com/w/cpp/language/class_template

#ifdef POLYNOMIAL_MODEL_IMPLEMENTATION
#define EXTERN_TEMPLATE_DEF(T)
#else
#define EXTERN_TEMPLATE_DEF(T) extern template class T;
#endif

// EXTERN_TEMPLATE_DEF(PolynomialModel<2>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<3>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<4>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<5>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<6>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<7>)
EXTERN_TEMPLATE_DEF(PolynomialModel<8>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<9>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<10>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<11>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<12>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<13>)
// EXTERN_TEMPLATE_DEF(PolynomialModel<14>)

#undef EXTERN_TEMPLATE_DEF

using DistortionModel = PolynomialModel<8>;

void load(DistortionModel& data, const string& fname) noexcept(false);
void save(const DistortionModel& data, const string& fname) noexcept(false);
void read(DistortionModel& data, const std::string& in) noexcept(false);
void write(const DistortionModel& data, std::string& out) noexcept(false);
} // namespace perceive
