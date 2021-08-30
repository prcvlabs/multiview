
#pragma once

#include "quaternion.hpp"
#include "vector-3.hpp"
#include "vector-4.hpp"

#include "vector.hpp"

namespace perceive
{
template<typename T> struct EuclideanTransformT
{
   CUSTOM_NEW_DELETE(EuclideanTransformT)

   EuclideanTransformT() = default;
   EuclideanTransformT(const Vector3T<T>& t,
                       const QuaternionT<T>& q,
                       T s = T(1.0))
       : translation(t)
       , rotation(q)
       , scale(s)
   {}
   EuclideanTransformT(const Vector3T<T>& t,
                       const Vector4T<T>& axis_angle,
                       T s = T(1.0))
       : translation(t)
       , scale(s)
   {
      rotation.from_axis_angle(axis_angle);
   }
   EuclideanTransformT(const EuclideanTransformT&) = default;
   EuclideanTransformT(EuclideanTransformT&&)      = default;
   ~EuclideanTransformT()                          = default;

   // Create a NAN EuclideanTransform
   static EuclideanTransformT nan() noexcept
   {
      return EuclideanTransformT{
          Vector3T<T>::nan(), QuaternionT<T>::nan(), T(NAN)};
   }

   // Assignment
   EuclideanTransformT& operator=(const EuclideanTransformT&) = default;
   EuclideanTransformT& operator=(EuclideanTransformT&&) = default;

   // Equality
   bool operator==(const EuclideanTransformT& o) const noexcept
   {
      return (translation == o.translation) and (rotation == o.rotation);
   }
   bool operator!=(const EuclideanTransformT& o) const noexcept
   {
      return !(*this == o);
   }

   bool is_finite() const noexcept
   {
      return translation.is_finite() && rotation.is_finite()
             && std::isfinite(scale);
   }

   // Pack/unpack
   void pack(T Y[7]) const noexcept
   {
      auto X   = &Y[0];
      auto saa = quaternion_to_saa(rotation);
      for(auto i = 0; i < 3; ++i) *X++ = saa[i];
      for(auto i = 0; i < 3; ++i) *X++ = translation[i];
      *X++ = scale;
   }

   void unpack(const T Y[7]) noexcept
   {
      auto X   = &Y[0];
      auto saa = Vector3T<T>();
      for(auto i = 0; i < 3; ++i) saa[i] = *X++;
      for(auto i = 0; i < 3; ++i) translation[i] = *X++;
      scale    = *X++;
      rotation = saa_to_quaternion(saa);
   }

   // Operations

   EuclideanTransformT inverse() const noexcept
   {
      return EuclideanTransformT{} / *this;
   }

   // ---- Compose translations
   EuclideanTransformT& operator*=(const EuclideanTransformT& o) noexcept
   {
      translation = o.apply(translation);
      rotation    = (o.rotation * rotation).normalised();
      scale       = scale * o.scale; // composed scale
      return *this;
   }

   EuclideanTransformT& operator/=(const EuclideanTransformT& o) noexcept
   {
      translation = o.inverse_apply(translation);
      rotation    = (o.rotation.conjugate() * rotation).normalised();
      scale       = scale / o.scale; // composed scale
      return *this;
   }

   EuclideanTransformT operator*(const EuclideanTransformT& o) const noexcept
   {
      EuclideanTransformT v(*this);
      v *= o;
      return v;
   }

   EuclideanTransformT operator/(const EuclideanTransformT& o) const noexcept
   {
      EuclideanTransformT v(*this);
      v /= o;
      return v;
   }

   Vector3T<T> apply(const Vector3T<T>& x) const noexcept
   {
      return rotation.rotate(x * scale) + translation;
   }

   Vector3T<T> inverse_apply(const Vector3T<T>& x) const noexcept
   {
      return rotation.conjugate().rotate(x - translation) / scale;
   }

   Vector4T<T> apply_to_plane(const Vector4T<T>& p3) const noexcept
   {
      using Vector4T      = Vector4T<T>;
      using Matrix4T      = Eigen::Matrix<T, 4, 4>;
      using EigenVector4T = Eigen::Matrix<T, 4, 1>;

      auto et_inv = this->inverse();

      Matrix4T S  = Matrix4T::Identity() * et_inv.scale;
      S(3, 3)     = T(1.0);
      Matrix4T R  = quaternion_to_rot4x4(et_inv.rotation);
      Matrix4T Tr = Matrix4T::Identity(); // translation

      const auto t = et_inv.translation;
      Tr(0, 3)     = t.x;
      Tr(1, 3)     = t.y;
      Tr(2, 3)     = t.z;

      Matrix4T H = (Tr * R * S).transpose(); // The transform homography

      EigenVector4T p0;
      p0(0) = p3.x; // For some reason, this is required to
      p0(1) = p3.y; // prevent clang for misaligning memory
      p0(2) = p3.z;
      p0(3) = p3.w;

      EigenVector4T p = H * p0;
      Vector4T out(p(0), p(1), p(2), p(3));
      out.normalise_plane();
      return out;
   }

   // ---- Members
   Vector3T<T> translation{T(0.0), T(0.0), T(0.0)};
   QuaternionT<T> rotation{T(0.0), T(0.0), T(0.0), T(1.0)};
   T scale{T(1.0)};

   // ---- to-string
   string to_string() const noexcept
   {
      return format("Euclidean: translation={}, {}, scale={}",
                    str(translation),
                    rotation.to_readable_str(),
                    scale);
   }

   string to_json_str() const noexcept
   {
      return format(R"V0G0N({{
   "translation": [{}, {}, {}],
   "rotation": [{}, {}, {}, {}],
   "scale": {}
}}
)V0G0N",
                    translation.x,
                    translation.y,
                    translation.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                    scale);
   }

   string to_1line_json_str() const noexcept
   {
      return format("{{\"translation\":[{}, {}, {}], "
                    "\"rotation\":[{}, {}, {}, {}], "
                    "\"scale\":{}}}",
                    translation.x,
                    translation.y,
                    translation.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                    scale);
   }
};

template<typename T>
inline string str(const EuclideanTransformT<T>& et) noexcept
{
   return et.to_string();
}

template<typename T>
inline EuclideanTransformT<T> compose(const EuclideanTransformT<T>& a,
                                      const EuclideanTransformT<T>& b) noexcept
{
   return a * b;
}

Eigen::Matrix<real, 4, 4>
make_transform_matrix(const EuclideanTransformT<real>& et) noexcept;

// Generates the EuclideanTransform between two sets of points.
// Vectors 'A' and 'B' must be the same size, and corresponding points
// in the same order.
// @see: https://en.wikipedia.org/wiki/Kabsch_algorithm
EuclideanTransformT<real>
transform_between(const vector<Vector3T<real>>& A,
                  const vector<Vector3T<real>>& B) noexcept;

// Pack/Unpack the euclidean-transform as 5df. (So no scale)
void pack_et_6df(const EuclideanTransformT<double>& et, double* X) noexcept;
void pack_et_6df(const EuclideanTransformT<float>& et, float* X) noexcept;
EuclideanTransformT<double> unpack_et_6df(const double* X) noexcept;
EuclideanTransformT<float> unpack_et_6df(const float* X) noexcept;
} // namespace perceive
