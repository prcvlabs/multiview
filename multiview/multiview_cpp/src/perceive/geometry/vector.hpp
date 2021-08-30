
#pragma once

#include <Eigen/Dense>

#include "aabb.hpp"
#include "euclidean-transform.hpp"
#include "ordered-pair.hpp"
#include "perceive/utils/math.hpp"
#include "quaternion.hpp"
#include "vector-2.hpp"
#include "vector-3.hpp"
#include "vector-4.hpp"

namespace perceive
{
// --------------------------------------------------------------------- Lp-norm

enum class Lp_norm_t : int { L_INF = 0, L1 = 1, L2 = 2, L3 = 3, L4 = 4 };

inline const char* str(Lp_norm_t method)
{
   switch(method) {
   case Lp_norm_t::L_INF: return "L-inf";
   case Lp_norm_t::L1: return "L1";
   case Lp_norm_t::L2: return "L2";
   case Lp_norm_t::L3: return "L3";
   case Lp_norm_t::L4: return "L4";
   }
   return "<error>";
}

// --------------------------------------------------------------------- Vectors

typedef Vector2T<float> Vector2f;
typedef Vector3T<float> Vector3f;
typedef Vector4T<float> Vector4f;
typedef Vector4T<float> Plane4f;
typedef QuaternionT<float> QuaternionF;

typedef Vector2T<real> Vector2;
typedef Vector3T<real> Vector3;
typedef Vector4T<real> Vector4;
typedef Vector4T<real> Plane;
typedef QuaternionT<real> Quaternion;
typedef AABBT<real> AABB;

typedef Vector2T<int> Point2;
typedef Vector3T<int> Point3;
typedef Vector4T<int> Point4;
typedef AABBT<int> AABBi;
typedef OrderedPairT<int> OrderedPair;

typedef EuclideanTransformT<float> EuclideanTransformF;
typedef EuclideanTransformT<real> EuclideanTransform;

using RealRange = std::pair<real, real>;

// #define EIGEN_ALIGN Eigen::DontAlign
#define EIGEN_ALIGN Eigen::AutoAlign

template<typename T> using EigenVector4T = Eigen::Matrix<T, 4, 1, EIGEN_ALIGN>;

using Vector2r = Eigen::Vector2d;
using Vector3r = Eigen::Vector3d;
using Vector4r = Eigen::Vector4d;
using Vector6r = Eigen::Matrix<real, 6, 1, EIGEN_ALIGN>;
using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1, EIGEN_ALIGN>;

template<typename T> using Matrix4T = Eigen::Matrix<T, 4, 4, EIGEN_ALIGN>;

using Matrix3r  = Eigen::Matrix<real, 3, 3, EIGEN_ALIGN>;
using Matrix4r  = Eigen::Matrix<real, 4, 4, EIGEN_ALIGN>;
using Matrix34r = Eigen::Matrix<real, 3, 4, EIGEN_ALIGN>;
using MatrixXr  = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;

using Matrix3f  = Eigen::Matrix<float, 3, 3, EIGEN_ALIGN>;
using Matrix4f  = Eigen::Matrix<float, 4, 4, EIGEN_ALIGN>;
using Matrix34f = Eigen::Matrix<float, 3, 4, EIGEN_ALIGN>;
using MatrixXf  = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

using MatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;

// ----------------------------------------------------------------------- Round

// template<> inline Vector2 round<Vector2>(const Vector2& v) noexcept
// {
//    return v.round();
// }
// template<> inline Vector3 round<Vector3>(const Vector3& v) noexcept
// {
//    return v.round();
// }
// template<> inline Vector4 round<Vector4>(const Vector4& v) noexcept
// {
//    return v.round();
// }

// ------------------------------------------------------------------ Conversion

inline Vector2 to_vec2(const Point2& v) noexcept { return Vector2(v(0), v(1)); }
inline Vector2 to_vec2(const Vector2& v) noexcept
{
   return Vector2(v(0), v(1));
}
inline Vector2 to_vec2(const Vector2r& v) noexcept
{
   return Vector2(v(0), v(1));
}
inline Vector2 to_vec2(const Vector2f& v) noexcept
{
   return Vector2(real(v(0)), real(v(1)));
}
inline Vector2r to_vec2r(const Point2& v) noexcept
{
   return Vector2r(v(0), v(1));
}
inline Vector2r to_vec2r(const Vector2& v) noexcept
{
   return Vector2r(v(0), v(1));
}
inline Vector2r to_vec2r(const Vector2r& v) noexcept
{
   return Vector2r(v(0), v(1));
}
inline Vector2r to_vec2r(const Vector2f& v) noexcept
{
   return Vector2r(v(0), v(1));
}
inline Vector2f to_vec2f(const Point2& v) noexcept
{
   return Vector2f(float(v(0)), float(v(1)));
}
inline Vector2f to_vec2f(const Vector2& v) noexcept
{
   return Vector2f(float(v(0)), float(v(1)));
}
inline Vector2f to_vec2f(const Vector2r& v) noexcept
{
   return Vector2f(float(v(0)), float(v(1)));
}
inline Vector2f to_vec2f(const Vector2f& v) noexcept
{
   return Vector2f(v(0), v(1));
}

inline Vector3 to_vec3(const Point3& v) noexcept
{
   return Vector3(v(0), v(1), v(2));
}
inline Vector3 to_vec3(const Vector3& v) noexcept
{
   return Vector3(v(0), v(1), v(2));
}
inline Vector3 to_vec3(const Vector3r& v) noexcept
{
   return Vector3(v(0), v(1), v(2));
}
inline Vector3 to_vec3(const Vector3f& v) noexcept
{
   return Vector3(real(v(0)), real(v(1)), real(v(2)));
}
inline Vector3r to_vec3r(const Point3& v) noexcept
{
   return Vector3r(v(0), v(1), v(2));
}
inline Vector3r to_vec3r(const Vector3& v) noexcept
{
   return Vector3r(v(0), v(1), v(2));
}
inline Vector3r to_vec3r(const Vector3r& v) noexcept
{
   return Vector3r(v(0), v(1), v(2));
}
inline Vector3r to_vec3r(const Vector3f& v) noexcept
{
   return Vector3r(real(v(0)), real(v(1)), real(v(2)));
}
inline Vector3f to_vec3f(const Point3& v) noexcept
{
   return Vector3f(float(v(0)), float(v(1)), float(v(2)));
}
inline Vector3f to_vec3f(const Vector3& v) noexcept
{
   return Vector3f(float(v(0)), float(v(1)), float(v(2)));
}
inline Vector3f to_vec3f(const Vector3r& v) noexcept
{
   return Vector3f(float(v(0)), float(v(1)), float(v(2)));
}
inline Vector3f to_vec3f(const Vector3f& v) noexcept
{
   return Vector3f(v(0), v(1), v(2));
}

inline Vector4 to_vec4(const Point4& v) noexcept
{
   return Vector4(v(0), v(1), v(2), v(3));
}
inline Vector4 to_vec4(const Vector4& v) noexcept
{
   return Vector4(v(0), v(1), v(2), v(3));
}
inline Vector4 to_vec4(const Vector4r& v) noexcept
{
   return Vector4(v(0), v(1), v(2), v(3));
}
inline Vector4 to_vec4(const Vector4f& v) noexcept
{
   return Vector4(real(v(0)), real(v(1)), real(v(2)), real(v(3)));
}
inline Vector4r to_vec4r(const Point4& v) noexcept
{
   return Vector4r(v(0), v(1), v(2), v(3));
}
inline Vector4r to_vec4r(const Vector4& v) noexcept
{
   return Vector4r(v(0), v(1), v(2), v(3));
}
inline Vector4r to_vec4r(const Vector4r& v) noexcept
{
   return Vector4r(v(0), v(1), v(2), v(3));
}
inline Vector4r to_vec4r(const Vector4f& v) noexcept
{
   return Vector4r(real(v(0)), real(v(1)), real(v(2)), real(v(3)));
}
inline Vector4f to_vec4f(const Point4& v) noexcept
{
   return Vector4f(float(v(0)), float(v(1)), float(v(2)), float(v(3)));
}
inline Vector4f to_vec4f(const Vector4& v) noexcept
{
   return Vector4f(float(v(0)), float(v(1)), float(v(2)), float(v(3)));
}
inline Vector4f to_vec4f(const Vector4r& v) noexcept
{
   return Vector4f(float(v(0)), float(v(1)), float(v(2)), float(v(3)));
}
inline Vector4f to_vec4f(const Vector4f& v) noexcept
{
   return Vector4f(float(v(0)), float(v(1)), float(v(2)), float(v(3)));
}

inline Point2 to_pt2(const std::pair<int, int>& v) noexcept
{
   return Point2(v.first, v.second);
}
inline Point2 to_pt2(const Vector2& v) noexcept
{
   return Point2(int(v.x), int(v.y));
}
inline Point2 to_pt2(const Vector2f& v) noexcept
{
   return Point2(int(v.x), int(v.y));
}
inline Point3 to_pt3(const Vector3& v) noexcept
{
   return Point3(int(v.x), int(v.y), int(v.z));
}
inline Point3 to_pt3(const Vector3f& v) noexcept
{
   return Point3(int(v.x), int(v.y), int(v.z));
}
inline Point4 to_pt4(const Vector4& v) noexcept
{
   return Point4(int(v.x), int(v.y), int(v.z), int(v.w));
}
inline Point4 to_pt4(const Vector4f& v) noexcept
{
   return Point4(int(v.x), int(v.y), int(v.z), int(v.w));
}

inline Quaternion to_quaternion(const QuaternionF& q) noexcept
{
   return Quaternion(real(q.x), real(q.y), real(q.z), real(q.w));
}

inline QuaternionF to_quaternionf(const Quaternion& q) noexcept
{
   return QuaternionF(float(q.x), float(q.y), float(q.z), float(q.w));
}

template<typename T> inline Vector2T<T> clip_to_xy(const Vector3T<T>& X)
{
   return Vector2T<T>(X.x, X.y);
}

// ------------------------------------------------------------------- is-finite

inline bool is_finite(const Vector2& x) noexcept { return x.is_finite(); }
inline bool is_finite(const Vector3& x) noexcept { return x.is_finite(); }
inline bool is_finite(const Vector4& x) noexcept { return x.is_finite(); }
inline bool is_finite(const Vector2f& x) noexcept { return x.is_finite(); }
inline bool is_finite(const Vector3f& x) noexcept { return x.is_finite(); }
inline bool is_finite(const Vector4f& x) noexcept { return x.is_finite(); }
inline bool is_finite(float x) noexcept { return std::isfinite(x); }
inline bool is_finite(double x) noexcept { return std::isfinite(x); }

// --------------------------------------------------------- matrix-mult-helpers
// P3
inline Vector4r mult_homgen(const Matrix4r& M, const Vector4r& X) noexcept
{
   Vector4r Y = M * Vector4r(X(0), X(1), X(2), X(3));
   if(fabs(Y(3)) < 1e-9) Y /= Y(3);
   return Y;
}

inline Vector4 mult_homgen(const Matrix4r& M, const Vector4& X) noexcept
{
   return to_vec4(mult_homgen(M, to_vec4r(X)));
}

inline Vector3r mult_homgen(const Matrix4r& M, const Vector3r& X) noexcept
{
   auto Y = mult_homgen(M, Vector4r(X(0), X(1), X(2), 1.0));
   return Vector3r(Y(0), Y(1), Y(2));
}

inline Vector3 mult_homgen(const Matrix4r& M, const Vector3& X) noexcept
{
   return to_vec3(mult_homgen(M, to_vec3r(X)));
}

// P2
inline Vector3r mult_homgen(const Matrix3r& M, const Vector3r& X) noexcept
{
   Vector3r Y = M * X;
   if(fabs(Y(2)) < 1e-9) Y /= Y(2);
   return Y;
}

inline Vector3 mult_homgen(const Matrix3r& M, const Vector3& X) noexcept
{
   return to_vec3(mult_homgen(M, to_vec3r(X)));
}

inline Vector2r mult_homgen(const Matrix3r& M, const Vector2r& X) noexcept
{
   auto Y = mult_homgen(M, Vector3r(X(0), X(1), 1.0));
   return Vector2r(Y(0), Y(1));
}

inline Vector2 mult_homgen(const Matrix3r& M, const Vector2& X) noexcept
{
   return to_vec2(mult_homgen(M, to_vec2r(X)));
}

// --------------------------------------------------------------- AABB => AABBi

inline AABBi aabb_to_aabbi(const AABB& aabb)
{
   return AABBi(int(floor(aabb.left)),
                int(floor(aabb.top)),
                int(ceil(aabb.right)),
                int(ceil(aabb.bottom)));
}

// -------------------------------------------------------- Matrix3r => Matrix3d

template<typename U, typename V> void matrixU_to_V(const U& S, V& D) noexcept
{
   using T               = typename V::Scalar;
   const unsigned n_rows = unsigned(S.rows());
   const unsigned n_cols = unsigned(S.cols());
   if(D.rows() != S.rows() || D.cols() != S.cols())
      D = V::Zero(S.rows(), S.cols());
   assert(D.rows() == n_rows && D.cols() == n_cols);
   for(unsigned row = 0; row < n_rows; ++row)
      for(unsigned col = 0; col < n_cols; ++col) D(row, col) = T(S(row, col));
}

// static void matrix3r_to_3d(const Matrix3r& S, Matrix3d& D) {matrixU_to_V(S,
// D);} static void matrix3d_to_3r(const Matrix3d& S, Matrix3r& D)
// {matrixU_to_V(S, D);} static void matrixXr_to_Xd(const MatrixXr& S, MatrixXd&
// D) {matrixU_to_V(S, D);} static void matrixXd_to_Xr(const MatrixXd& S,
// MatrixXr& D) {matrixU_to_V(S, D);}

// --------------------------------------------------------------------- Generic

template<typename T> typename T::value_type quadrance(const T& v) noexcept
{
   return v.quadrance();
}
template<typename T> typename T::value_type norm(const T& v) noexcept
{
   return v.norm();
}
template<typename T> typename T::value_type dot(const T& u, const T& v) noexcept
{
   return u.dot(v);
}
template<typename T>
typename T::value_type distance(const T& u, const T& v) noexcept
{
   return u.distance(v);
}

template<typename T>
inline Vector3T<T> cross(const Vector3T<T>& u, const Vector3T<T>& v)
{
   return u.cross(v);
}

inline Matrix3r make_skew_symmetric(const Vector3& t) noexcept
{
   Matrix3r Tx = Matrix3r::Zero();
   Tx(0, 1)    = -t(2);
   Tx(0, 2)    = t(1);
   Tx(1, 2)    = -t(0);
   Tx(1, 0)    = t(2);
   Tx(2, 0)    = -t(1);
   Tx(2, 1)    = t(0);
   return Tx;
}

// --------------------------------------------------------------------- Vector2

inline Vector2 cartesian_to_polar(const Vector2& x) noexcept
{
   Vector2 ret;
   if(fabs(x.quadrance() - real(1.0)) < real(1e-9)) {
      ret.mag()   = real(1.0);
      ret.theta() = acos(x.y >= real(0.0) ? x.x : -x.x);
   } else {
      ret.mag()   = x.norm();
      ret.theta() = acos((x.y >= real(0.0) ? x.x : -x.x) / ret.mag());
   }
   return ret;
}

inline Vector2 angle_to_cartesian(const real theta) noexcept
{
   return Vector2(cos(theta), sin(theta));
}

inline Vector2 polar_to_cartesian(const Vector2& x) noexcept
{
   return angle_to_cartesian(x.theta()) * x.mag();
}

// ------------------------------------------------------------------- Spherical

inline Vector3f
spherical_to_cartesian(float inc, float azi, float dist) noexcept
{
   const float sinx = std::sin(inc);
   const float cosx = std::cos(inc);
   const float siny = std::sin(azi);
   const float cosy = std::cos(azi);
   //
   return Vector3f(sinx * cosy, sinx * siny, cosx) * dist;
}

inline Vector3 spherical_to_cartesian(real inc, real azi, real dist) noexcept
{
   const Vector3::value_type sinx = sin(inc);
   const Vector3::value_type cosx = cos(inc);
   const Vector3::value_type siny = sin(azi);
   const Vector3::value_type cosy = cos(azi);
   //
   return Vector3(sinx * cosy, sinx * siny, cosx) * dist;
}

inline Vector3 spherical_to_cartesian(const Vector3& s) noexcept
{
   return spherical_to_cartesian(s.x, s.y, s.z);
}

inline Vector3f spherical_to_cartesian(const Vector3f& s) noexcept
{
   return spherical_to_cartesian(s.x, s.y, s.z);
}

inline Vector3 cartesian_to_spherical(real x, real y, real z) noexcept
{
   auto r           = sqrt(x * x + y * y + z * z);
   auto inclination = acos(z / r);
   auto azimuth     = atan2(y, x);
   return Vector3(inclination, azimuth, r);
}

inline Vector3 cartesian_to_spherical(const Vector3& n) noexcept
{
   return cartesian_to_spherical(n.x, n.y, n.z);
}

inline Vector3f cartesian_to_spherical(const Vector3f& n) noexcept
{
   return to_vec3f(cartesian_to_spherical(real(n.x), real(n.y), real(n.z)));
}

// ----------------------------------------------------------------- Homogeneous

inline Vector3r normalized_P2(const Vector3r& x) noexcept
{
   Vector3r v = x;
   v /= v(2);
   if((fabs(v(0)) > 1e20) || (fabs(v(1)) > 1e20))
      return x; // tending towards an ideal point
   return v;
}

template<typename T>
inline Vector2T<T> homgen_P2_to_R2(const Vector3T<T>& x) noexcept
{
   if(std::fabs(x.z) < T(1e-20)) {
      T sign = (x.z < T(0.0)) ? -T(1.0) : T(1.0);
      return Vector2T<T>(sign * x.x / T(1e-20), sign * x.y / T(1e-20));
   }
   auto inv = T(1.0) / x.z;
   return Vector2T<T>(x.x * inv, x.y * inv);
}

template<typename T>
inline Vector3T<T> homgen_R2_to_P2(const Vector2T<T>& x) noexcept
{
   return Vector3T<T>(x.x, x.y, T(1.0));
}

template<typename T>
inline Vector2T<T> line_line_isect(const Vector3T<T>& u,
                                   const Vector3T<T>& v) noexcept
{
   return homgen_P2_to_R2(cross(u, v));
}

template<typename T>
inline Vector3T<T> to_homgen_line(const Vector2T<T>& u,
                                  const Vector2T<T>& v) noexcept
{
   Vector3T<T> ret(u.y - v.y, v.x - u.x, u.x * v.y - u.y * v.x);
   ret.normalise_line();
   return ret;
}

template<typename T>
inline Vector3T<T> to_homgen_line(const Vector3T<T>& u,
                                  const Vector3T<T>& v) noexcept
{
   Vector3T<T> ret = cross(u, v);
   ret.normalise_line();
   return ret;
}

inline Vector3r to_homgen_line(const Vector3r& u, const Vector3r& v) noexcept
{
   return to_vec3r(to_homgen_line(to_vec3(u), to_vec3(v)));
}

template<typename T>
inline T dot_line_point(const Vector3T<T>& line,
                        const Vector3T<T>& point) noexcept
{
   return dot(line, point);
}

template<typename T>
inline T dot_line_point(const Vector3T<T>& line,
                        const Vector2T<T>& point) noexcept
{
   return line.x * point.x + line.y * point.y + line.z;
}

// Project 'point' onto 'line'
template<typename T>
inline Vector2T<T> project_point_line(const Vector3T<T>& line,
                                      const Vector2T<T>& point) noexcept
{
   return point - dot_line_point(line, point) * Vector2T<T>(line.x, line.y);
}

inline bool is_orthogonal(const Vector3& a, const Vector3& b) noexcept
{
   return fabs(dot(a, b)) < 1e-9;
}

template<typename T>
inline typename T::value_type uv_cos_theta(const T& u, const T& v)
{
   using V = typename T::value_type;
   return std::clamp<V>(u.normalised().dot(v.normalised()), V(-1.0), V(1.0));
}

// ---------------------------------------------------------- Lifted Coordinates

inline Vector6r lift_xy(real x, real y) noexcept
{
   Vector6r X;
   X(0) = square(x);
   X(1) = x * y;
   X(2) = square(y);
   X(3) = x;
   X(4) = y;
   X(5) = 1.0;
   return X;
}

inline Vector6r lift_xy(const Vector2& x) noexcept { return lift_xy(x.x, x.y); }
inline Vector6r lift_xy(const Vector2r& x) noexcept
{
   return lift_xy(x(0), x(1));
}
inline Vector6r lift_xy(const Vector3& x) noexcept
{
   return lift_xy(homgen_P2_to_R2(x));
}
inline Vector6r lift_xy(const Vector3r& x) noexcept
{
   return lift_xy(homgen_P2_to_R2(to_vec3(x)));
}

// ------------------------------------------------------------------------ Rays

inline Vector3 to_ray(const MatrixXr& A, real x, real y) noexcept
{
   return to_vec3(A * lift_xy(x, y)).normalized();
}

inline Vector3 to_ray(const MatrixXr& A, const Vector2& x) noexcept
{
   return to_ray(A, x.x, x.y);
}
inline Vector3 to_ray(const MatrixXr& A, const Vector2r& x) noexcept
{
   return to_ray(A, x(0), x(1));
}
inline Vector3 to_ray(const MatrixXr& A, const Vector3r& x) noexcept
{
   return to_ray(A, homgen_P2_to_R2(to_vec3(x)));
}

// ---------------------------------------------------------------- 2D Iteration

static constexpr array<std::pair<int, int>, 4> four_connected
    = {{{-1, 0}, {0, -1}, {0, 1}, {1, 0}}};

static constexpr array<std::pair<int, int>, 8> eight_connected
    = {{{-1, 0}, {0, -1}, {0, 1}, {1, 0}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}};

static constexpr array<std::pair<int, int>, 20> twenty_connected
    = {{{-2, -1}, {-2, 0}, {-2, 1}, {-1, -2}, {-1, -1}, {-1, 0}, {-1, 1},
        {-1, 2},  {0, -2}, {0, -1}, {0, 1},   {0, 2},   {1, -2}, {1, -1},
        {1, 0},   {1, 1},  {1, 2},  {2, -1},  {2, 0},   {2, 1}}};

static constexpr array<std::pair<int, int>, 21> twentyone_connected
    = {{{-2, -1}, {-2, 0}, {-2, 1}, {-1, -2}, {-1, -1}, {-1, 0}, {-1, 1},
        {-1, 2},  {0, -2}, {0, -1}, {0, 1},   {0, 0},   {0, 2},  {1, -2},
        {1, -1},  {1, 0},  {1, 1},  {1, 2},   {2, -1},  {2, 0},  {2, 1}}};

static constexpr array<std::pair<int, int>, 24> twentyfour_connected
    = {{{-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}, {-1, -2},
        {-1, -1}, {-1, 0},  {-1, 1}, {-1, 2}, {0, -2}, {0, -1},
        {0, 1},   {0, 2},   {1, -2}, {1, -1}, {1, 0},  {1, 1},
        {1, 2},   {2, -2},  {2, -1}, {2, 0},  {2, 1},  {2, 2}}};

// -------------------------------------------------------------------- feedback

template<typename T>
inline string vec_feedback(const Vector2T<T>& u, const Vector2T<T>& v) noexcept
{
   return format("|{} - {}| = {}", str(u), str(v), (u - v).norm());
}

template<typename T>
inline string vec_feedback(const Vector3T<T>& u, const Vector3T<T>& v) noexcept
{
   return format("|{} - {}| = {}", str(u), str(v), (u - v).norm());
}

template<typename T>
inline string vec_feedback(const Vector4T<T>& u, const Vector4T<T>& v) noexcept
{
   return format("|{} - {}| = {}", str(u), str(v), (u - v).norm());
}

// ----------------------------------------------------------- Quantize Funciton
std::array<std::pair<Point2, float>, 4> quantize(const Vector2& X) noexcept;

} // namespace perceive

namespace std
{
// OrderedPair
template<> struct hash<perceive::OrderedPair>
{
   size_t operator()(const perceive::OrderedPair& v) const { return v.hash(); }
};

// ---------------------------------------------------------------------- Vector

// Vector2
template<> struct hash<perceive::Vector2>
{
   size_t operator()(const perceive::Vector2& v) const { return v.hash(); }
};

// Vector3
template<> struct hash<perceive::Vector3>
{
   size_t operator()(const perceive::Vector3& v) const { return v.hash(); }
};

// Vector4
template<> struct hash<perceive::Vector4>
{
   size_t operator()(const perceive::Vector4& v) const { return v.hash(); }
};

// ----------------------------------------------------------------------- Point

// Point2
template<> struct hash<perceive::Point2>
{
   size_t operator()(const perceive::Point2& v) const { return v.hash(); }
};

// Point3
template<> struct hash<perceive::Point3>
{
   size_t operator()(const perceive::Point3& v) const { return v.hash(); }
};

// Point4
template<> struct hash<perceive::Point4>
{
   size_t operator()(const perceive::Point4& v) const { return v.hash(); }
};

} // namespace std
