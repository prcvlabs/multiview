
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/utils/sdbm-hash.hpp"
#include <Eigen/Core>
#include <cmath>

namespace perceive
{
// --------------------------------------------------------------------- Vector2
#pragma pack(push, 1)
template<typename T> class Vector2T
{
 public:
   using value_type = T;

   T x, y;

   Vector2T()
       : x(T(0.0))
       , y(T(0.0))
   {}
   Vector2T(T x_, T y_)
       : x(x_)
       , y(y_)
   {}
   Vector2T(const float p[2])
   {
      x = p[0];
      y = p[1];
   }
   Vector2T(const double p[2])
   {
      x = p[0];
      y = p[1];
   }

   Vector2T& operator=(const Vector2T& v) = default;

   Vector2T& operator=(const Eigen::Vector2f& v)
   {
      for(int i = 0; i < 2; ++i) this->operator[](i) = v(i);
      return *this;
   }

   static Vector2T nan() { return Vector2T(T(NAN), T(NAN)); }
   static Vector2T range()
   {
      auto mmax = std::numeric_limits<T>::max();
      auto mmin = std::numeric_limits<T>::lowest();
      return Vector2T(mmax, mmin);
   }

   void union_value(T v)
   {
      if(v < this->x) this->x = v;
      if(v > this->y) this->y = v;
   }

   unsigned size() const { return 2; }

   Vector2T& normalise(T epsilon = 1e-9)
   {
      // Don't normalize if we don't have to
      T mag2 = x * x + y * y;
      if(std::fabs(mag2 - T(1.0)) > epsilon) {
         T mag_inv = T(1.0) / std::sqrt(mag2);
         x *= mag_inv;
         y *= mag_inv;
      }
      return *this;
   }
   Vector2T normalised(T epsilon = T(1e-9)) const
   {
      Vector2T res = *this;
      res.normalise(epsilon);
      return res;
   }
   Vector2T& normalize(T epsilon = T(1e-9)) { return normalise(epsilon); }
   Vector2T normalized(T epsilon = T(1e-9)) const
   {
      return normalised(epsilon);
   }
   T quadrance() const { return x * x + y * y; }
   T norm() const { return T(std::sqrt(quadrance())); }
   T dot(const Vector2T& rhs) const { return x * rhs.x + y * rhs.y; }
   T perp_dot(const Vector2T& rhs) const { return x * rhs.y - y * rhs.x; }
   T quadrance(const Vector2T& rhs) const
   {
      return (x - rhs.x) * (x - rhs.x) + (y - rhs.y) * (y - rhs.y);
   }
   T distance(const Vector2T& rhs) const { return (*this - rhs).norm(); }

   // As polar co-ordinates
   T& mag() { return x; }
   const T& mag() const { return x; }
   T& theta() { return y; }
   const T& theta() const { return y; }

   // As format
   T& width() { return x; }
   const T& width() const { return x; }
   T& height() { return y; }
   const T& height() const { return y; }

   Vector2T& set_to(const T& a, const T& b)
   {
      x = a;
      y = b;
      return *this;
   }
   Vector2T& set_to(T a[2])
   {
      set_to(a[0], a[1]);
      return *this;
   }

   T* copy_to(T a[2]) const
   {
      a[0] = x;
      a[1] = y;
      return a;
   }

   T* ptr()
   {
#ifdef DEBUG_BUILD
      assert(&x == reinterpret_cast<const T*>(this) + 0);
      assert(&y == reinterpret_cast<const T*>(this) + 1);
#endif
      return &x;
   }
   const T* ptr() const { return const_cast<Vector2T<T>*>(this)->ptr(); }

   T& operator[](int idx)
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 2);
#endif
      return ptr()[idx];
   }
   const T& operator[](int idx) const
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 2);
#endif
      return ptr()[idx];
   }

   T& operator()(int idx)
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 2);
#endif
      return ptr()[idx];
   }

   const T& operator()(int idx) const
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 2);
#endif
      return ptr()[idx];
   }

   Vector2T flip_x() const { return Vector2T(-x, y); }
   Vector2T flip_y() const { return Vector2T(x, -y); }
   Vector2T clockwise_90() const { return Vector2T(y, -x); }
   Vector2T counter_clockwise_90() const { return Vector2T(-y, x); }
   Vector2T rotate(T theta) const
   {
      auto sin_t = std::sin(theta);
      auto cos_t = std::cos(theta);
      return Vector2T<T>(x * cos_t - y * sin_t, x * sin_t + y * cos_t);
   }

   Vector2T round() const { return Vector2T(std::round(x), std::round(y)); }

   Vector2T& operator*=(T scalar)
   {
      x *= scalar;
      y *= scalar;
      return *this;
   }

   Vector2T& operator/=(T scalar)
   {
      x /= scalar;
      y /= scalar;
      return *this;
   }
   Vector2T operator*(T scalar) const
   {
      Vector2T res(*this);
      res *= scalar;
      return res;
   }

   Vector2T operator/(T scalar) const
   {
      Vector2T res(*this);
      res /= scalar;
      return res;
   }

   Vector2T& operator+=(const Vector2T& rhs)
   {
      x += rhs.x;
      y += rhs.y;
      return *this;
   }

   Vector2T& operator-=(const Vector2T& rhs)
   {
      x -= rhs.x;
      y -= rhs.y;
      return *this;
   }

   Vector2T operator+(const Vector2T& rhs) const
   {
      Vector2T res(*this);
      res += rhs;
      return res;
   }
   Vector2T operator-(const Vector2T& rhs) const
   {
      Vector2T res(*this);
      res -= rhs;
      return res;
   }

   Vector2T operator-() const { return Vector2T<T>(-x, -y); }

   bool operator==(const Vector2T& rhs) const
   {
      return x == rhs.x && y == rhs.y;
   }
   bool operator!=(const Vector2T& rhs) const { return !(*this == rhs); }

   bool operator<(const Vector2T& rhs) const
   {
      return x == rhs.x ? y < rhs.y : x < rhs.x;
   }
   bool operator<=(const Vector2T& rhs) const
   {
      return x == rhs.x ? y <= rhs.y : x <= rhs.x;
   }
   bool operator>(const Vector2T& rhs) const { return !(*this <= rhs); }
   bool operator>=(const Vector2T& rhs) const { return !(*this < rhs); }

   bool has_nan() const { return x != x || y != y; }
   bool is_nan() const { return std::isnan(x) || std::isnan(y); }
   bool is_finite() const { return std::isfinite(x) && std::isfinite(y); }
   bool is_unit_vector(T epsilon = 1e-9) const
   {
      return is_finite() && fabs(quadrance() - 1.0) < epsilon;
   }

   inline friend bool isfinite(const Vector2T& o) noexcept
   {
      return o.is_finite();
   }

   std::string to_string(const char* fmt = nullptr) const
   {
      if(fmt == nullptr) {
         if constexpr(std::is_floating_point<T>::value) {
            fmt = "[{:7.5f}, {:7.5f}]";
         } else {
            fmt = "[{}, {}]";
         }
      }
      return format(fmt, x, y);
   }
   std::string to_str() const { return format("[{}, {}]", x, y); }

   void print(const char* msg = NULL, bool newline = true) const
   {
      printf("%s%s%s%s",
             msg,
             (msg == NULL ? "" : " "),
             to_string().c_str(),
             (newline ? "\n" : ""));
      fflush(stdout);
   }

   size_t hash() const { return sdbm_hash(ptr(), sizeof(T) * size()); }

   friend std::string str(const Vector2T<T>& o) noexcept
   {
      return o.to_string();
   }
};
#pragma pack(pop)

// Scalar multiplication
template<typename T> Vector2T<T> operator*(float a, const Vector2T<T>& v)
{
   return v * T(a);
}
template<typename T> Vector2T<T> operator/(float a, const Vector2T<T>& v)
{
   return v / T(a);
}
template<typename T> Vector2T<T> operator*(double a, const Vector2T<T>& v)
{
   return v * T(a);
}
template<typename T> Vector2T<T> operator/(double a, const Vector2T<T>& v)
{
   return v / T(a);
}

// String shim
template<typename T> std::string str(const Vector2T<T>& v)
{
   return v.to_string();
}
template<typename T>
std::ostream& operator<<(std::ostream& out, const Vector2T<T>& v)
{
   out << v.to_string();
   return out;
}

template<typename T> inline T perp_dot(T x1, T y1, T x2, T y2)
{
   return x1 * y2 - x2 * y1;
}

} // namespace perceive
