
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/utils/sdbm-hash.hpp"
#include "vector-3.hpp"
#include <Eigen/Core>

namespace perceive
{
// --------------------------------------------------------------------- Vector3

#pragma pack(push, 1)
template<typename T> class Vector4T
{
 public:
   using value_type = T;

   T x, y, z, w;

   Vector4T() noexcept
       : x(T(0.0))
       , y(T(0.0))
       , z(T(0.0))
       , w(T(0.0))
   {}
   Vector4T(T x_, T y_, T z_, T w_) noexcept
       : x(x_)
       , y(y_)
       , z(z_)
       , w(w_)
   {}
   Vector4T(const float p[4]) noexcept
   {
      x = p[0];
      y = p[1];
      z = p[2];
      w = p[3];
   }
   Vector4T(const double p[4]) noexcept
   {
      x = p[0];
      y = p[1];
      z = p[2];
      w = p[3];
   }
   Vector4T(const Vector3T<T>& p, T w_ = T(0.0)) noexcept
       : x(p.x)
       , y(p.y)
       , z(p.z)
       , w(w_)
   {}
   Vector4T(const Vector3T<T>& a,
            const Vector3T<T>& b,
            const Vector3T<T>& c) noexcept
   {
      *this = plane_from_3_points(a, b, c);
   }

   Vector4T& operator=(const Vector4T& v) = default;

   Vector4T& operator=(const Eigen::Vector4d& v) noexcept
   {
      for(int i = 0; i < 4; ++i) this->operator[](i) = v(i);
      return *this;
   }
   Vector4T& operator=(const Eigen::Vector4f& v) noexcept
   {
      for(int i = 0; i < 4; ++i) this->operator[](i) = v(i);
      return *this;
   }

   static Vector4T nan() noexcept
   {
      return Vector4T(T(NAN), T(NAN), T(NAN), T(NAN));
   }

   unsigned size() const noexcept { return 4; }

   Vector4T& normalise(T epsilon = 1e-9) noexcept
   {
      // Don't normalize if we don't have to
      T mag2 = quadrance();
      if(fabs(mag2 - T(1.0)) > epsilon) {
         T mag_inv = T(1.0) / sqrt(mag2);
         x *= mag_inv;
         y *= mag_inv;
         z *= mag_inv;
         w *= mag_inv;
      }
      return *this;
   }

   Vector4T& normalise_plane(T epsilon = 1e-9) noexcept
   {
      T mag2 = x * x + y * y + z * z;
      if(fabs(mag2 - T(1.0)) > epsilon) *this *= T(1.0) / sqrt(mag2);
      return *this;
   }

   Vector4T& normalise_point(T epsilon = 1e-9) noexcept
   {
      if(fabs(w - T(1.0)) > epsilon) *this *= T(1.0) / w;
      return *this;
   }

   Vector4T normalised(T epsilon = 1e-9) const noexcept
   {
      Vector4T res = *this;
      res.normalise(epsilon);
      return res;
   }
   Vector4T normalised_plane(T epsilon = 1e-9) const noexcept
   {
      Vector4T res = *this;
      res.normalise_plane(epsilon);
      return res;
   }
   Vector4T normalised_point(T epsilon = 1e-9) const noexcept
   {
      Vector4T res = *this;
      res.normalise_point(epsilon);
      return res;
   }

   Vector4T& normalize(T ep = 1e-9) noexcept { return normalise(ep); }
   Vector4T& normalize_plane(T ep = 1e-9) noexcept
   {
      return normalise_plane(ep);
   }
   Vector4T& normalize_point(T ep = 1e-9) noexcept
   {
      return normalise_point(ep);
   }

   Vector4T normalized(T epsilon = 1e-9) const noexcept
   {
      return normalised(epsilon);
   }
   Vector4T normalized_plane(T ep = 1e-9) const noexcept
   {
      return normalised_plane(ep);
   }
   Vector4T normalized_point(T ep = 1e-9) const noexcept
   {
      return normalised_point(ep);
   }

   T quadrance() const noexcept { return x * x + y * y + z * z + w * w; }
   T norm() const noexcept { return sqrt(quadrance()); }
   T dot(const Vector4T& rhs) const noexcept
   {
      return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
   }
   T distance(const Vector4T& rhs) const noexcept
   {
      return (*this - rhs).norm();
   }

   Vector3T<T>& xyz() noexcept
   {
#ifdef DEBUG_BUILD
      assert(&x == reinterpret_cast<const T*>(this) + 0);
      assert(&y == reinterpret_cast<const T*>(this) + 1);
      assert(&z == reinterpret_cast<const T*>(this) + 2);
#endif
      return *(reinterpret_cast<Vector3T<T>*>(this));
   }
   const Vector3T<T>& xyz() const noexcept
   {
#ifdef DEBUG_BUILD
      assert(&x == reinterpret_cast<const T*>(this) + 0);
      assert(&y == reinterpret_cast<const T*>(this) + 1);
      assert(&z == reinterpret_cast<const T*>(this) + 2);
#endif
      return *(reinterpret_cast<const Vector3T<T>*>(this));
   }
   T& d() noexcept { return w; } // for plane
   const T& d() const noexcept { return w; }

   Vector4T& set_to(const T& a, const T& b, const T& c, const T& d) noexcept
   {
      x = a;
      y = b;
      z = c;
      w = d;
      return *this;
   }
   Vector4T& set_to(T a[4]) noexcept
   {
      set_to(a[0], a[1], a[2], a[3]);
      return *this;
   }

   T* copy_to(T a[4]) const noexcept
   {
      a[0] = x;
      a[1] = y;
      a[2] = z;
      a[3] = w;
      return a;
   }

   T* ptr() noexcept
   {
#ifdef DEBUG_BUILD
      assert(&x == reinterpret_cast<const T*>(this) + 0);
      assert(&y == reinterpret_cast<const T*>(this) + 1);
      assert(&z == reinterpret_cast<const T*>(this) + 2);
      assert(&w == reinterpret_cast<const T*>(this) + 3);
#endif
      return &x;
   }

   const T* ptr() const noexcept
   {
      return const_cast<Vector4T<T>*>(this)->ptr();
   }

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

   Vector4T round() const noexcept
   {
      return Vector4T(floor(x + T(0.499)),
                      floor(y + T(0.499)),
                      floor(z + T(0.499)),
                      floor(w + T(0.499)));
   }

   Vector4T& operator*=(T scalar) noexcept
   {
      x *= scalar;
      y *= scalar;
      z *= scalar;
      w *= scalar;
      return *this;
   }
   Vector4T& operator/=(T scalar) noexcept
   {
      x /= scalar;
      y /= scalar;
      z /= scalar;
      w /= scalar;
      return *this;
   }
   Vector4T operator*(T scalar) const noexcept
   {
      Vector4T res(*this);
      res *= scalar;
      return res;
   }
   Vector4T operator/(T scalar) const noexcept
   {
      Vector4T res(*this);
      res /= scalar;
      return res;
   }

   Vector4T& operator+=(const Vector4T& rhs) noexcept
   {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
      w += rhs.w;
      return *this;
   }
   Vector4T& operator-=(const Vector4T& rhs) noexcept
   {
      x -= rhs.x;
      y -= rhs.y;
      z -= rhs.z;
      w -= rhs.w;
      return *this;
   }
   Vector4T operator+(const Vector4T& rhs) const noexcept
   {
      Vector4T res(*this);
      res += rhs;
      return res;
   }
   Vector4T operator-(const Vector4T& rhs) const noexcept
   {
      Vector4T res(*this);
      res -= rhs;
      return res;
   }
   Vector4T operator-() const noexcept { return Vector4T(-x, -y, -z, -w); }

   bool operator==(const Vector4T& rhs) const noexcept
   {
      return x == rhs.x and y == rhs.y and z == rhs.z and w == rhs.w;
   }
   bool operator!=(const Vector4T& rhs) const noexcept
   {
      return !(*this == rhs);
   }

   std::string to_string(const char* fmt = "{{{}, {}, {}, {}}}") const noexcept
   {
      return format(fmt, x, y, z, w);
   }

   std::string to_str() const { return format("[{}, {}, {}, {}]", x, y, z, w); }

   bool is_nan() const noexcept
   {
      return std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w);
   }

   bool is_finite() const noexcept
   {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
             && std::isfinite(w);
   }

   inline friend bool isfinite(const Vector4T& o) noexcept
   {
      return o.is_finite();
   }

   bool is_unit_vector(T epsilon = 1e-9) const noexcept
   {
      return is_finite() && fabs(quadrance() - 1.0) < epsilon;
   }

   void print(const char* msg = NULL, bool newline = true) const noexcept
   {
      printf("%s%s%s%s",
             (msg == NULL ? "" : msg),
             (msg == NULL ? "" : " "),
             to_string().c_str(),
             (newline ? "\n" : ""));
      fflush(stdout);
   }

   // Homogenous point
   bool pt_at_infinity(T epsilon = 1e-9) const noexcept
   {
      return fabs(w) < epsilon;
   }

   size_t hash() const noexcept { return sdbm_hash(ptr(), sizeof(T) * size()); }

   // Plane functions
   T side(const Vector3T<T>& o) const noexcept
   {
      return o.x * x + o.y * y + o.z * z + d();
   }

   // WARNING, must be normalised
   double point_plane_distance(const Vector3T<T>& point) const noexcept
   {
      assert(fabs(xyz().quadrance() - 1.0) < 1e-9);
      return fabs(side(point));
   }

   Vector3T<T> image(const Vector3T<T>& p) const noexcept
   {
      assert(fabs(xyz().quadrance() - 1.0) < 1e-9);
      return p - xyz() * side(p);
   }
   Vector3T<T> reflect(const Vector3T<T>& p) const noexcept
   {
      assert(fabs(xyz().quadrance() - 1.0) < 1e-9);
      return p - 2.0 * xyz() * side(p);
   }

   // Reflect a plane
   Vector4T<T> reflect(const Vector4T<T>& p) const noexcept
   {
      auto norm = (p.xyz() - 2.0 * p.xyz().dot(xyz()) * xyz()).normalised();
      auto C    = reflect(p.image(Vector3T<T>(0.0, 0.0, 0.0)));
      return Vector4T<T>(norm, -C.dot(norm));
   }

   friend std::string str(const Vector4T<T>& o) noexcept
   {
      return o.to_string();
   }
};
#pragma pack(pop)

template<typename T>
Vector4T<T> operator*(float a, const Vector4T<T>& v) noexcept
{
   return v * a;
}

template<typename T>
Vector4T<T> operator/(float a, const Vector4T<T>& v) noexcept
{
   return v / a;
}

template<typename T>
Vector4T<T> operator*(double a, const Vector4T<T>& v) noexcept
{
   return v * a;
}

template<typename T>
Vector4T<T> operator/(double a, const Vector4T<T>& v) noexcept
{
   return v / a;
}

template<typename T>
Vector4T<T> plane_from_3_points(const Vector3T<T>& a,
                                const Vector3T<T>& b,
                                const Vector3T<T>& c) noexcept
{
   Vector4T<T> ret;

   auto ab   = a - b;
   auto ac   = a - c;
   ret.xyz() = ab.cross(ac);
   ret.xyz().normalise();
   ret.d() = -1.0 * dot(a, ret.xyz());

   return ret;
}

template<typename T>
inline T ray_position_relative_to_plane_t(const Vector4T<T>& p3,
                                          const Vector3T<T>& a,
                                          const Vector3T<T>& b,
                                          const T side) noexcept
{
   return (side - p3.d() - dot(p3.xyz(), a)) / dot(p3.xyz(), b - a);
}

template<typename T>
inline Vector3T<T> ray_position_relative_to_plane(const Vector4T<T>& p3,
                                                  const Vector3T<T>& a,
                                                  const Vector3T<T>& b,
                                                  const T side) noexcept
{
   const auto t = ray_position_relative_to_plane_t(p3, a, b, side);
   return a + t * (b - a);
}

template<typename T>
inline T plane_ray_intersection_t(const Vector4T<T>& p,
                                  const Vector3T<T>& a,
                                  const Vector3T<T>& b) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   return ray_position_relative_to_plane_t(p, a, b, T(0.0));
}

template<typename T>
inline Vector3T<T> plane_ray_intersection(const Vector4T<T>& p,
                                          const Vector3T<T>& a,
                                          const Vector3T<T>& b) noexcept
{
   static_assert(std::is_floating_point<T>::value);
   return a + plane_ray_intersection_t(p, a, b) * (b - a);
}

// String shim
template<typename T> std::string str(const Vector4T<T>& v) noexcept
{
   return v.to_string();
}
template<typename T>
std::ostream& operator<<(std::ostream& out, const Vector4T<T>& v) noexcept
{
   out << v.to_string();
   return out;
}

} // namespace perceive
