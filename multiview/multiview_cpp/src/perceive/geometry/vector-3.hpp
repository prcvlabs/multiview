
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/utils/sdbm-hash.hpp"
#include "vector-2.hpp"
#include <Eigen/Core>
#include <cmath>

namespace perceive
{
// --------------------------------------------------------------------- Vector3

#pragma pack(push, 1)
template<typename T> class Vector3T
{
 public:
   using value_type = T;

   T x, y, z;

   constexpr Vector3T()
       : x(T(0.0))
       , y(T(0.0))
       , z(T(0.0))
   {}
   constexpr Vector3T(T x_, T y_, T z_)
       : x(x_)
       , y(y_)
       , z(z_)
   {}
   constexpr Vector3T(const float p[3])
   {
      x = p[0];
      y = p[1];
      z = p[2];
   }
   constexpr Vector3T(const double p[3])
   {
      x = p[0];
      y = p[1];
      z = p[2];
   }
   constexpr Vector3T(const Vector2T<T>& p, double z_ = 0.0)
       : x(p.x)
       , y(p.y)
       , z(z_)
   {}
   // explicit Vector3T(const Eigen::Vector3f& v) : x(v(0)), y(v(1)), z(v(2)) {}
   // explicit Vector3T(const Eigen::Vector3d& v) : x(v(0)), y(v(1)), z(v(2)) {}

   static Vector3T nan() { return Vector3T(T(NAN), T(NAN), T(NAN)); }

   Vector3T& operator=(const Vector3T& v) = default;

   Vector3T& operator=(const Eigen::Vector3d& v)
   {
      for(int i = 0; i < 3; ++i) this->operator[](i) = v(i);
      return *this;
   }

   unsigned size() const { return 3; }

   Vector3T& normalise(T epsilon = T(1e-9))
   {
      // Don't normalize if we don't have to
      T mag2 = quadrance();
      if(std::fabs(mag2 - T(1.0)) > epsilon) {
         T mag_inv = T(1.0) / std::sqrt(mag2);
         x *= mag_inv;
         y *= mag_inv;
         z *= mag_inv;
      }
      return *this;
   }

   Vector3T& normalise_line(T epsilon = T(1e-9))
   {
      T mag2 = x * x + y * y;
      if(std::fabs(mag2 - T(1.0)) > epsilon and (mag2 > T(1e-20)))
         *this *= T(1.0) / std::sqrt(mag2);
      return *this;
   }

   Vector3T& normalise_point(T epsilon = T(1e-9))
   {
      if(std::fabs(z - T(1.0)) > epsilon) *this *= T(1.0) / z;
      return *this;
   }

   Vector3T normalised(T epsilon = T(1e-9)) const
   {
      Vector3T res = *this;
      res.normalise(epsilon);
      return res;
   }
   Vector3T normalised_line(T epsilon = T(1e-9)) const
   {
      Vector3T res = *this;
      res.normalise_line(epsilon);
      return res;
   }
   Vector3T normalised_point(T epsilon = T(1e-9)) const
   {
      Vector3T res = *this;
      res.normalise_point(epsilon);
      return res;
   }

   Vector3T& normalize(T ep = T(1e-9)) { return normalise(ep); }
   Vector3T& normalize_line(T ep = T(1e-9)) { return normalise_line(ep); }
   Vector3T& normalize_point(T ep = T(1e-9)) { return normalise_point(ep); }

   Vector3T normalized(T epsilon = T(1e-9)) const
   {
      return normalised(epsilon);
   }
   Vector3T normalized_line(T ep = T(1e-9)) const
   {
      return normalised_line(ep);
   }
   Vector3T normalized_point(T ep = T(1e-9)) const
   {
      return normalised_point(ep);
   }

   T quadrance() const { return x * x + y * y + z * z; }
   inline T norm() const;
   T dot(const Vector3T& o) const { return x * o.x + y * o.y + z * o.z; }
   T distance(const Vector3T& rhs) const { return (*this - rhs).norm(); }
   Vector3T cross(const Vector3T& v) const
   {
      Vector3T o;
      o.x = y * v.z - z * v.y;
      o.y = z * v.x - x * v.z;
      o.z = x * v.y - y * v.x;
      return o;
   }
   Vector3T left_cross(const Vector3T& rhs) const { return rhs.cross(*this); }

   Vector3T& set_to(const T& a, const T& b, const T& c)
   {
      x = a;
      y = b;
      z = c;
      return *this;
   }
   Vector3T& set_to(T a[3])
   {
      set_to(a[0], a[1], a[2]);
      return *this;
   }

   T* copy_to(T a[3]) const
   {
      a[0] = x;
      a[1] = y;
      a[2] = z;
      return a;
   }

   T* ptr()
   {
      assert(&x == reinterpret_cast<const T*>(this) + 0);
      assert(&y == reinterpret_cast<const T*>(this) + 1);
      assert(&z == reinterpret_cast<const T*>(this) + 2);
      return &x;
   }
   const T* ptr() const { return const_cast<Vector3T<T>*>(this)->ptr(); }
   T& operator[](int idx)
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 3);
#endif
      return ptr()[idx];
   }

   const T& operator[](int idx) const
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 3);
#endif
      return ptr()[idx];
   }

   T& operator()(int idx)
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 3);
#endif
      return ptr()[idx];
   }

   const T& operator()(int idx) const
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 3);
#endif
      return ptr()[idx];
   }

   Vector3T round() const
   {
      return Vector3T(
          floor(x + T(0.499)), floor(y + T(0.499)), floor(z + T(0.499)));
   }

   Vector3T& operator*=(T scalar)
   {
      x *= scalar;
      y *= scalar;
      z *= scalar;
      return *this;
   }
   Vector3T& operator/=(T scalar)
   {
      x /= scalar;
      y /= scalar;
      z /= scalar;
      return *this;
   }
   Vector3T operator*(T scalar) const
   {
      Vector3T res(*this);
      res *= scalar;
      return res;
   }
   Vector3T operator/(T scalar) const
   {
      Vector3T res(*this);
      res /= scalar;
      return res;
   }

   Vector3T& operator+=(const Vector3T& rhs)
   {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
      return *this;
   }
   Vector3T& operator-=(const Vector3T& rhs)
   {
      x -= rhs.x;
      y -= rhs.y;
      z -= rhs.z;
      return *this;
   }
   Vector3T operator+(const Vector3T& rhs) const
   {
      Vector3T res(*this);
      res += rhs;
      return res;
   }
   Vector3T operator-(const Vector3T& rhs) const
   {
      Vector3T res(*this);
      res -= rhs;
      return res;
   }
   Vector3T operator-() const { return Vector3T(-x, -y, -z); }

   bool operator==(const Vector3T& rhs) const
   {
      return x == rhs.x && y == rhs.y && z == rhs.z;
   }
   bool operator!=(const Vector3T& rhs) const { return !(*this == rhs); }

   bool operator<(const Vector3T& o) const noexcept
   {
      return (x != o.x) ? (x < o.x) : (y != o.y) ? (y < o.y) : (z < o.z);
   }

   bool operator>(const Vector3T& o) const noexcept
   {
      return !(*this == o) and !(*this < o);
   }

   bool operator<=(const Vector3T& o) const noexcept
   {
      return (*this == o) or (*this < o);
   }

   bool operator>=(const Vector3T& o) const noexcept { return !(*this < o); }

   bool has_nan() const { return x != x || y != y || z != z; }
   bool is_nan() const
   {
      return std::isnan(x) || std::isnan(y) || std::isnan(z);
   }
   bool is_finite() const
   {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
   }
   bool is_unit_vector(T epsilon = T(1e-9)) const
   {
      return is_finite() && fabs(quadrance() - 1.0) < epsilon;
   }

   inline friend bool isfinite(const Vector3T& o) noexcept
   {
      return o.is_finite();
   }

   std::string to_string(const char* fmt = nullptr) const
   {
      if(fmt == nullptr) {
         if constexpr(std::is_floating_point<T>::value) {
            fmt = "[{:7.5f}, {:7.5f}, {:7.5f}]";
         } else {
            fmt = "[{}, {}, {}]";
         }
      }
      return format(fmt, x, y, z);
   }
   std::string to_str() const { return format("[{}, {}, {}]", x, y, z); }

   void print(const char* msg = NULL, bool newline = true) const
   {
      printf("%s%s%s%s",
             (msg == NULL ? "" : msg),
             (msg == NULL ? "" : " "),
             to_string().c_str(),
             (newline ? "\n" : ""));
      fflush(stdout);
   }

   // Treat as spherical
   T& inclination() { return x; }
   T& azimuth() { return y; }
   T& r() { return z; }
   const T& inclination() const { return x; }
   const T& azimuth() const { return y; }
   const T& r() const { return z; }

   // Homogenous point
   bool pt_at_infinity(T epsilon = T(1e-9)) const { return fabs(z) < epsilon; }

   size_t hash() const { return sdbm_hash(ptr(), sizeof(T) * size()); }

   friend std::string str(const Vector3T<T>& o) noexcept
   {
      return o.to_string();
   }
};
#pragma pack(pop)

template<typename T> Vector3T<T> operator*(float a, const Vector3T<T>& v)
{
   return v * a;
}
template<typename T> Vector3T<T> operator/(float a, const Vector3T<T>& v)
{
   return v / a;
}
template<typename T> Vector3T<T> operator*(double a, const Vector3T<T>& v)
{
   return v * a;
}
template<typename T> Vector3T<T> operator/(double a, const Vector3T<T>& v)
{
   return v / a;
}

// String shim
template<typename T> std::string str(const Vector3T<T>& v)
{
   return v.to_string();
}
template<typename T>
std::ostream& operator<<(std::ostream& out, const Vector3T<T>& v)
{
   out << v.to_string();
   return out;
}

template<> inline float Vector3T<float>::norm() const
{
   return sqrtf(quadrance());
}
template<> inline double Vector3T<double>::norm() const
{
   return sqrt(quadrance());
}
} // namespace perceive
