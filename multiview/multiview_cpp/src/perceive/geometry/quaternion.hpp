
// Adapted from:
// * http://www.stanford.edu/~acoates/quaternion.h
// * http://www.euclideanspace.com/maths/geometry/rotations/conversions

#pragma once

#include "vector-2.hpp"
#include "vector-3.hpp"
#include "vector-4.hpp"
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

namespace perceive
{
#pragma pack(push, 1)
template<typename T> class QuaternionT
{
 public:
   using value_type = T;

   T x, y, z, w;

   QuaternionT() noexcept
       : x(0.0)
       , y(0.0)
       , z(0.0)
       , w(1.0)
   {}

   QuaternionT(const Vector3T<T>& v, T w_) noexcept
       : x(v.x)
       , y(v.y)
       , z(v.z)
       , w(w_)
   {}

   QuaternionT(const Vector4T<T>& v) noexcept { from_axis_angle(v); }

   QuaternionT(const T array[4]) noexcept
   {
      x = array[0];
      y = array[1];
      z = array[2];
      w = array[3];
   }

   QuaternionT(T x_, T y_, T z_, T w_) noexcept
       : x(x_)
       , y(y_)
       , z(z_)
       , w(w_)
   {}

   QuaternionT(const Vector3T<T>& a, const Vector3T<T>& b) noexcept
   {
      *this = between_vectors(a, b);
   }

   bool operator==(const QuaternionT& rhs) const noexcept
   {
      const auto o       = this->normalised();
      const auto p       = rhs.normalised();
      const auto cos_phi = p.x * o.x + p.y * o.y + p.z * o.z + p.w * o.w;
      return fabs(cos_phi) > 1.0 - 1e-9;
   }

   bool operator!=(const QuaternionT& rhs) const noexcept
   {
      return !(*this == rhs);
   }

   static inline QuaternionT identity() noexcept
   {
      return QuaternionT(0.0, 0.0, 0.0, 1.0);
   }

   static QuaternionT nan() noexcept
   {
      constexpr T val = T(NAN);
      return QuaternionT(val, val, val, val);
   }

   // Create from between vectors
   static inline QuaternionT between_vectors(const Vector3T<T>& a,
                                             const Vector3T<T>& b) noexcept
   {
      auto u     = a.normalised();
      auto v     = b.normalised();
      auto n     = cross(u, v);
      auto cos_t = dot(u, v);
      if(cos_t < (1e-6 - 1.0)) {
         // 180 degree rotation... about arbitrary axis
         Vector3T<T> axis;
         axis = cross(Vector3T<T>(1, 0, 0), u);
         if(axis.quadrance() < 1e-3) axis = cross(Vector3T<T>(0, 1, 0), u);
         axis.normalise();
         return QuaternionT(axis, 0);
      } else if(cos_t >= 1.0 - 1e-6) {
         return QuaternionT(0, 0, 0, 1);
      }
      QuaternionT q(n, 1.0 + cos_t);
      q.normalise();
      return q;
   }

   // Create from between rotations: r * a = b
   static inline QuaternionT between_rotations(const QuaternionT<T>& a,
                                               const QuaternionT<T>& b) noexcept
   {
      return b * a.conjugate();
   }

   Vector3T<T>& complex() noexcept
   {
      return *(reinterpret_cast<Vector3T<T>*>(this));
   }
   const Vector3T<T>& complex() const noexcept
   {
      return *(reinterpret_cast<const Vector3T<T>*>(this));
   }
   T& real() noexcept { return w; }
   const T& real() const noexcept { return w; }
   QuaternionT conjugate() const noexcept { return QuaternionT(-x, -y, -z, w); }

   QuaternionT inverse() const noexcept { return conjugate().normalised(); }
   QuaternionT& invert() const noexcept
   {
      x = -x;
      y = -y;
      z = -z;
      return normalise();
   }

   T quadrance() const noexcept { return x * x + y * y + z * z + w * w; }
   T norm() const noexcept { return std::sqrt(quadrance()); }

   QuaternionT normalised(T epsilon = T(1e-9)) const noexcept
   {
      QuaternionT ret(*this);
      ret.normalise(epsilon);
      return ret;
   }

   QuaternionT& normalise(T epsilon = T(1e-9)) noexcept
   {
      auto mag2 = quadrance();
      if(std::fabs(mag2 - T(1.0)) > epsilon) {
         auto inv = T(1.0) / T(std::sqrt(mag2));
         x *= inv;
         y *= inv;
         z *= inv;
         w *= inv;
      }
      return *this;
   }

   unsigned size() const noexcept { return 4; }

   T* copy_to(T a[4]) const noexcept
   {
      a[0] = x;
      a[1] = y;
      a[2] = z;
      a[3] = w;
      return a;
   }

   T* ptr() noexcept { return &x; }
   const T* ptr() const noexcept { return &x; }
   T& operator[](int idx) noexcept
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 4);
#endif
      return ptr()[idx];
   }

   const T& operator[](int idx) const noexcept
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 4);
#endif
      return ptr()[idx];
   }

   T& operator()(int idx) noexcept
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 4);
#endif
      return ptr()[idx];
   }

   const T& operator()(int idx) const noexcept
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 4);
#endif
      return ptr()[idx];
   }

   static void product(const QuaternionT<T>& a,
                       const QuaternionT<T>& b,
                       QuaternionT<T>& c) noexcept
   {
      c.x = a.y * b.z - a.z * b.y + a.x * b.w + a.w * b.x;
      c.y = a.z * b.x - a.x * b.z + a.y * b.w + a.w * b.y;
      c.z = a.x * b.y - a.y * b.x + a.z * b.w + a.w * b.z;
      c.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
   }

   QuaternionT<T> operator*(const QuaternionT<T>& rhs) const noexcept
   {
      QuaternionT<T> q;
      product(*this, rhs, q);
      return q;
   }
   QuaternionT<T>& operator*=(const QuaternionT<T>& rhs) noexcept
   {
      *this = *this * rhs;
      return *this;
   }

   QuaternionT<T> operator*(T s) const noexcept
   {
      auto q = *this;
      q *= s;
      return q;
   }
   QuaternionT<T>& operator*=(T s) noexcept
   {
      x *= s;
      y *= s;
      z *= s;
      w *= s;
      return *this;
   }
   QuaternionT<T> operator/(T s) const noexcept { return *this * T(1.0) /= s; }
   QuaternionT<T>& operator/=(T s) noexcept { return *this *= T(1.0) / s; }

   QuaternionT<T> operator+(const QuaternionT<T>& o) const noexcept
   {
      auto q = *this;
      q += o;
      return q;
   }
   QuaternionT<T>& operator+=(const QuaternionT<T>& o) noexcept
   {
      x += o.x;
      y += o.y;
      z += o.z;
      w += o.w;
      return *this;
   }
   QuaternionT<T> operator-(const QuaternionT<T>& o) const noexcept
   {
      auto q = *this;
      q -= o;
      return q;
   }
   QuaternionT<T>& operator-=(const QuaternionT<T>& o) noexcept
   {
      x -= o.x;
      y -= o.y;
      z -= o.z;
      w -= o.w;
      return *this;
   }

   // Same as rotation-maxtrix3x3 * v
   Vector3T<T> rotate(const Vector3T<T>& v) const noexcept
   {
      return (((*this) * QuaternionT(v, 0)) * inverse()).complex();
   }

   Vector3T<T> inverse_rotate(const Vector3T<T>& v) const noexcept
   {
      return conjugate().rotate(v);
   }

   Vector3T<T> apply(const Vector3T<T>& v) const noexcept { return rotate(v); }
   Vector3T<T> inverse_apply(const Vector3T<T>& v) const noexcept
   {
      return inverse_rotate(v);
   }

   // -- spherical-axis-angle
   Vector3T<T> to_spherical_axis_angle() const noexcept
   {
      Vector3T<T> a;
      auto aa = to_axis_angle();
      a.x     = std::acos(aa.z);        // inclination
      a.y     = std::atan2(aa.y, aa.x); // azimuth
      a.z     = aa.w;                   // theta
      return a;
   }

   QuaternionT<T>& from_spherical_axis_angle(const Vector3T<T>& a) noexcept
   {
      // a(inclination, azimuth, amount-of-rotation)
      const T half_theta     = T(0.5) * a.z;
      const T sin_half_theta = std::sin(half_theta);
      x                      = std::sin(a.x) * std::cos(a.y) * sin_half_theta;
      y                      = std::sin(a.x) * std::sin(a.y) * sin_half_theta;
      z                      = std::cos(a.x) * sin_half_theta;
      w                      = std::cos(half_theta);
      return *this;
   }

   QuaternionT& from_axis_angle(const Vector3T<T>& a,
                                const double theta) noexcept
   {
      const T sin_t = std::sin(0.5 * theta);
      const T cos_t = std::cos(0.5 * theta);
      auto b        = a.normalised();
      x             = b.x * sin_t;
      y             = b.y * sin_t;
      z             = b.z * sin_t;
      w             = cos_t;
      normalise();
      return *this;
   }

   QuaternionT& from_axis_angle(const Vector4T<T>& a) noexcept
   {
      return from_axis_angle(a.xyz(), a.d());
   }

   Vector4T<T> to_axis_angle() const noexcept
   {
      if(!is_finite()) return Vector4T<T>::nan();
      auto mag = quadrance();
      if(std::fabs(mag) < T(1e-9)) return Vector4T<T>(1, 0, 0, 0);
      T n_inv = T(1.0) / std::sqrt(mag);

      Vector4T<T> a;
      if(std::fabs(w - T(1.0)) < T(1e-9)) {
         // There is no rotation, so set the axis
         // to an arbitrary value, and theta to 0
         a = Vector4T<T>(1, 0, 0, 0);
      } else {
         auto ww = w * n_inv;
         T theta = T(2.0) * std::acos(ww); // angle of rotation
         T s_inv = T(1.0) / std::sqrt(T(1.0) - ww * ww);
         a.xyz() = complex() * s_inv * n_inv;
         a.w     = T(2.0) * std::acos(ww);
      }
      return a;
   }

   T theta() const noexcept { return to_axis_angle().w; }
   Vector3T<T> axis() const noexcept { return to_axis_angle().xyz(); }

   void set_theta(T f) noexcept { from_axis_angle(axis(), f); }
   void set_axis(const Vector3T<T>& a) noexcept { from_axis_angle(a, theta()); }

   bool is_nan() const noexcept
   {
      return std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w);
   }
   bool is_finite() const noexcept
   {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
             && std::isfinite(w);
   }
   bool is_unit_quaternion(T epsilon = 1e-9) const noexcept
   {
      return is_finite() && fabs(quadrance() - 1.0) < epsilon;
   }

   // -- String shim --
   std::string to_string(const char* fmt = "[{}, {}, {}, {}]") const
   {
      return format(fmt, x, y, z, w);
   }
   std::string to_str() const { return to_string("[{}, {}, {}, {}]"); }

   void print(const char* msg = NULL, bool newline = true) const
   {
      printf("{}{}{}{}",
             (msg == NULL ? "" : msg),
             (msg == NULL ? "" : " "),
             to_string().c_str(),
             (newline ? "\n" : ""));
      fflush(stdout);
   }

   std::string to_readable_str() const
   {
      auto aa = to_axis_angle();
      return format("aa = [{:7.5f}, {:7.5f}, {:7.5f}], theta = {:7.3f} degrees",
                    aa(0),
                    aa(1),
                    aa(2),
                    to_degrees(aa(3)));
   }
};
#pragma pack(pop)

// Scalar multiplication
template<typename T> QuaternionT<T> operator*(float a, const QuaternionT<T>& v)
{
   return v * a;
}
template<typename T> QuaternionT<T> operator/(float a, const QuaternionT<T>& v)
{
   return v / a;
}
template<typename T> QuaternionT<T> operator*(double a, const QuaternionT<T>& v)
{
   return v * a;
}
template<typename T> QuaternionT<T> operator/(double a, const QuaternionT<T>& v)
{
   return v / a;
}

// ------------------- String shim
template<typename T> std::string str(const QuaternionT<T>& v)
{
   return v.to_string();
}
template<typename T>
std::ostream& operator<<(std::ostream& out, const QuaternionT<T>& v)
{
   out << v.to_string();
   return out;
}

} // namespace perceive
