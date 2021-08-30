
#include "perceive/utils/string-utils.hpp"
#include "rotation.hpp"
#include "vector.hpp"

namespace perceive
{
// ------------------------------------------------------------------------- det

template<typename T> T det3x3(T M[9]) noexcept
{
#define R(r, c) M[r * 3 + c]
   return R(0, 0) * R(1, 1) * R(2, 2) + R(0, 1) * R(1, 2) * R(2, 0)
          + R(0, 2) * R(1, 0) * R(2, 1) - R(0, 0) * R(1, 2) * R(2, 1)
          - R(0, 1) * R(1, 0) * R(2, 2) - R(0, 2) * R(1, 1) * R(2, 0);
#undef R
}

// ---------------------------------------------------------- is-rotation-matrix

template<typename T>
bool is_rotation_matrixT(T M[9], const bool row_maj = true) noexcept
{
   // Row-maj doesn't matter, because the transpose of a rotation matrix
   // is always a rotation matrix, and vice-versa

   double ep = 1e-6; // margin to allow for rounding errors
#define R(r, c) M[r * 3 + c]

   int counter = 0;
   auto testit = [&](double val) -> bool {
      auto res = fabs(val) > ep;
      if(res) INFO(format("Fail at counter = {}, val = {}", counter, val));
      ++counter;
      return res;
   };

   auto row = [&](unsigned i) -> Vector3 {
      return Vector3(R(i, 0), R(i, 1), R(i, 2));
   };

   auto col = [&](unsigned i) -> Vector3 {
      return Vector3(R(0, i), R(1, i), R(2, i));
   };

   if(testit(dot(row(0), row(1))) || testit(dot(row(0), row(2)))
      || testit(dot(row(1), row(2))) || testit(dot(col(0), col(1)))
      || testit(dot(col(0), col(2))) || testit(dot(col(1), col(2))))
      return false;
   return det3x3(M) - 1.0 < ep;
#undef R
}

bool is_rotation_matrix(real R[9]) noexcept
{
   return is_rotation_matrixT(R, true);
}

// --------------------------------------------------------- between vector pair
// calc the rotation that transforms P1->W1 and
//                                   P2->W2
Quaternion calc_rotation(const Vector3& P1,
                         const Vector3& W1,
                         const Vector3& P2,
                         const Vector3& W2) noexcept
{
   // Get the orthonormal basis for 'P'
   Vector3 PX = P1.normalised();
   Vector3 PZ = cross(P1, P2).normalised();
   Vector3 PY = cross(PZ, PX).normalised();

   Matrix3r P          = Matrix3r::Zero();
   P.block(0, 0, 3, 1) = to_vec3r(PX);
   P.block(0, 1, 3, 1) = to_vec3r(PY);
   P.block(0, 2, 3, 1) = to_vec3r(PZ);

   const auto qp = rot3x3_to_quaternion(P);

   // Get the orthonormal basis for 'W'
   Vector3 WX = W1.normalised();
   Vector3 WZ = cross(W1, W2).normalised();
   Vector3 WY = cross(WZ, WX).normalised();

   Matrix3r W          = Matrix3r::Zero();
   W.block(0, 0, 3, 1) = to_vec3r(WX);
   W.block(0, 1, 3, 1) = to_vec3r(WY);
   W.block(0, 2, 3, 1) = to_vec3r(WZ);

   const auto qw = rot3x3_to_quaternion(W);

   // 'q' should be the rotation difference
   const auto q = qw * qp.conjugate();

   return q;
}

// Find the rotation from XYZ axis t
std::pair<Quaternion, Quaternion>
calc_rotation(const Vector3& C,     // a camera center
              const Vector3& ray_x, // ideal point for 'x'
              const Vector3& ray_y, // ideal point for 'y'
              const Vector3& X,     // 3D position of 'X'
              const Vector3& N      // Y = X + scalar * n
              ) noexcept
{
   const Vector3 P1 = ray_x.normalised();
   const Vector3 P2 = ray_y.normalised();

   // Find the rotation for W1
   const real cos_t = std::clamp(dot(P1, P2), -1.0, 1.0);
   const real theta = acos(cos_t);
   const auto p3    = Plane(C, X, X + N);
   const auto q     = axis_angle_to_quaternion(Vector4(p3.xyz(), theta));

   const Vector3 W1  = (X - C).normalised();
   const Vector3 W2a = q.rotate(W1);
   const Vector3 W2b = q.inverse_rotate(W1);

   return std::make_pair(calc_rotation(P1, W1, P2, W2a),
                         calc_rotation(P1, W1, P2, W2b));
}

// --------------------------------------------------------------------- look-at

Quaternion
look_at(const Vector3& C, const Vector3& at, const Vector3& up) noexcept
{
   const Vector3 Z = (at - C).normalised();
   const Vector3 X = up.cross(Z).normalised();
   const Vector3 Y = Z.cross(X);

   Matrix3r R;
   for(auto i = 0; i < 3; ++i) {
      R(i, 0) = X(i);
      R(i, 1) = Y(i);
      R(i, 2) = Z(i);
   }

   // cout << format("|R| = {}", R.determinant()) << endl;
   Expects(std::fabs(R.determinant() - 1.0) < 1e-9);
   return rot3x3_to_quaternion(R);
}

// --------------------------------------------------------------- natural-basis

template<typename T>
inline std::pair<Vector3T<T>, Vector3T<T>>
natural_basisT(const Vector3T<T>& U) noexcept
{
   auto Q = QuaternionT<T>::between_vectors(Vector3T<T>(1, 0, 0), U);
   return std::pair<Vector3T<T>, Vector3T<T>>(
       Q.rotate(Vector3T<T>(0, 1, 0)).normalised(),
       Q.rotate(Vector3T<T>(0, 0, 1)).normalised());
}

std::pair<Vector3, Vector3> natural_basis(const Vector3& U) noexcept
{
   return natural_basisT(U);
}

// ------------------------------------------------------------ average-rotation

Quaternion average_rotations(const Quaternion& a, const Quaternion& b) noexcept
{
   const auto Y = Vector3{0.0, 1.0, 0.0};
   const auto Z = Vector3{0.0, 0.0, 1.0};
   const auto U = 0.5 * (a.rotate(Y) + b.rotate(Y));
   const auto V = 0.5 * (a.rotate(Z) + b.rotate(Z));
   return calc_rotation(Y, U, Z, V);
}

Quaternion average_rotations(const vector<Quaternion>& v) noexcept
{
   if(v.size() == 0) return Quaternion{0.0, 0.0, 0.0, 1.0};
   if(v.size() == 1) return v.front();
   if(v.size() == 2) return average_rotations(v[0], v[1]);

   const auto Y = Vector3{0.0, 1.0, 0.0};
   const auto Z = Vector3{0.0, 0.0, 1.0};

   Vector3 U{0.0, 0.0, 0.0}, V{0.0, 0.0, 0.0};
   for(auto r : v) {
      U += r.rotate(Y);
      V += r.rotate(Z);
   }

   U.normalise();
   V.normalise();

   return calc_rotation(Y, U, Z, V);
}

// ---------------------------------------------------- Transforming Quaternions

Vector4 quaternion_to_axis_angle(const Quaternion& q) noexcept
{
   return q.to_axis_angle();
}
Vector3 quaternion_to_saa(const Quaternion& q) noexcept
{
   return q.to_spherical_axis_angle();
}
Matrix3r quaternion_to_rot3x3(const Quaternion& q) noexcept
{
   Matrix3r M;
   quaternion_to_Matrix3r(q, M);
   return M;
}
Vector3 quaternion_to_rodrigues(const Quaternion& q) noexcept
{
   return axis_angle_to_rodrigues(quaternion_to_axis_angle(q));
}

// ----------------------------------------------------- Transforming Axis-Angle

Quaternion axis_angle_to_quaternion(const Vector4& aa) noexcept
{
   Quaternion q;
   q.from_axis_angle(aa);
   return q;
}
Vector3 axis_angle_to_saa(const Vector4& aa) noexcept
{
   return quaternion_to_saa(axis_angle_to_quaternion(aa));
}
Matrix3r axis_angle_to_rot3x3(const Vector4& aa) noexcept
{
   return quaternion_to_rot3x3(axis_angle_to_quaternion(aa));
}
Vector3 axis_angle_to_rodrigues(const Vector4& aa) noexcept
{
   Vector3 ret = aa.xyz().normalised();
   auto theta  = aa.w;
   if(theta < 0) {
      theta = -theta;
      ret   = -ret;
   }
   ret *= theta;
   return ret;
}

// ------------------------------------------------------------ Transforming SAA

Quaternion saa_to_quaternion(const Vector3& saa) noexcept
{
   Quaternion q;
   q.from_spherical_axis_angle(saa);
   return q;
}
Vector4 saa_to_axis_angle(const Vector3& saa) noexcept
{
   return quaternion_to_axis_angle(saa_to_quaternion(saa));
}
Matrix3r saa_to_rot3x3(const Vector3& saa) noexcept
{
   return quaternion_to_rot3x3(saa_to_quaternion(saa));
}
Vector3 saa_to_rodrigues(const Vector3& saa) noexcept
{
   return quaternion_to_rodrigues(saa_to_quaternion(saa));
}

// ------------------------------------------------------ Transforming Rodrigues

Quaternion rodrigues_to_quaternion(const Vector3& r) noexcept
{
   return axis_angle_to_quaternion(rodrigues_to_axis_angle(r));
}
Vector4 rodrigues_to_axis_angle(const Vector3& r) noexcept
{
   auto theta = r.norm();
   auto axis  = r.normalised();
   if(theta < 1e-20) return Vector4(1, 0, 0, theta);
   return Vector4(axis, theta);
}
Vector3 rodrigues_to_saa(const Vector3& r) noexcept
{
   return axis_angle_to_saa(rodrigues_to_axis_angle(r));
}
Matrix3r rodrigues_to_rot3x3(const Vector3& r) noexcept
{
   return axis_angle_to_rot3x3(rodrigues_to_axis_angle(r));
}

// --------------------------------------------------------- Transforming Rot3x3

Quaternion rot3x3_to_quaternion(const Matrix3r& M) noexcept
{
   real R[9];
   R[0] = M(0, 0);
   R[1] = M(0, 1);
   R[2] = M(0, 2);
   R[3] = M(1, 0);
   R[4] = M(1, 1);
   R[5] = M(1, 2);
   R[6] = M(2, 0);
   R[7] = M(2, 1);
   R[8] = M(2, 2);
   return rotation3x3_to_quaternion(R);
}
Vector4 rot3x3_to_axis_angle(const Matrix3r& M) noexcept
{
   return quaternion_to_axis_angle(rot3x3_to_quaternion(M));
}
Vector3 rot3x3_to_saa(const Matrix3r& M) noexcept
{
   return quaternion_to_saa(rot3x3_to_quaternion(M));
}
Vector3 rot3x3_to_rodrigues(const Matrix3r& M) noexcept
{
   return quaternion_to_rodrigues(rot3x3_to_quaternion(M));
}

// -------------------------------------------------------------------------- //
// -------------------------------------------------------------------------- //
// --                        Additional Rotations                          -- //
// -------------------------------------------------------------------------- //
// -------------------------------------------------------------------------- //

// ---------------------------------------------- Quaternion To/from rotation3x3

template<typename T>
inline void
quaternion_to_rotation3x3T(const QuaternionT<T>& q, T R[9], bool row_maj = true)
{
   if(!q.is_finite()) {
      for(unsigned i = 0; i < 9; ++i) R[i] = T(NAN);
      return;
   }

   T n = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
   T s = (n == 0) ? 0.0 : 2.0 / sqrt(n);

   T wx = s * q.w * q.x, wy = s * q.w * q.y, wz = s * q.w * q.z;
   T xx = s * q.x * q.x, xy = s * q.x * q.y, xz = s * q.x * q.z;
   T yy = s * q.y * q.y, yz = s * q.y * q.z, zz = s * q.z * q.z;

   if(row_maj) {
      R[0] = 1 - (yy + zz);
      R[1] = xy - wz;
      R[2] = xz + wy;
      R[3] = xy + wz;
      R[4] = 1 - (xx + zz);
      R[5] = yz - wx;
      R[6] = xz - wy;
      R[7] = yz + wx;
      R[8] = 1 - (xx + yy);
   } else {
      R[0] = 1 - (yy + zz);
      R[3] = xy - wz;
      R[6] = xz + wy;
      R[1] = xy + wz;
      R[4] = 1 - (xx + zz);
      R[7] = yz - wx;
      R[2] = xz - wy;
      R[5] = yz + wx;
      R[8] = 1 - (xx + yy);
   }
}

template<typename T>
inline QuaternionT<T> rotation3x3_to_quaternionT(const T M[9],
                                                 bool row_maj = true)
{
   QuaternionT<T> q;
#define R(r, c) M[r * 3 + c]
#define C(r, c) M[r + c * 3]
   if(row_maj) {
      T trace = R(0, 0) + R(1, 1) + R(2, 2);
      if(trace > 0.0) {
         T s = 0.5 / sqrt(trace + 1.0);
         q.w = 0.25 / s;
         q.x = (R(2, 1) - R(1, 2)) * s;
         q.y = (R(0, 2) - R(2, 0)) * s;
         q.z = (R(1, 0) - R(0, 1)) * s;
      } else if(R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
         T s = 2.0 * sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
         q.w = (R(2, 1) - R(1, 2)) / s;
         q.x = 0.25 * s;
         q.y = (R(0, 1) + R(1, 0)) / s;
         q.z = (R(0, 2) + R(2, 0)) / s;
      } else if(R(1, 1) > R(2, 2)) {
         T s = 2.0 * sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
         q.w = (R(0, 2) - R(2, 0)) / s;
         q.x = (R(0, 1) + R(1, 0)) / s;
         q.y = 0.25 * s;
         q.z = (R(1, 2) + R(2, 1)) / s;
      } else {
         T s = 2.0 * sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
         q.w = (R(1, 0) - R(0, 1)) / s;
         q.x = (R(0, 2) + R(2, 0)) / s;
         q.y = (R(1, 2) + R(2, 1)) / s;
         q.z = 0.25 * s;
      }
   } else {
      T trace = C(0, 0) + C(1, 1) + C(2, 2);
      if(trace > 0) {
         T s = 0.5 / sqrt(trace + 1.0);
         q.w = 0.25 / s;
         q.x = (C(2, 1) - C(1, 2)) * s;
         q.y = (C(0, 2) - C(2, 0)) * s;
         q.z = (C(1, 0) - C(0, 1)) * s;
      } else if(C(0, 0) > C(1, 1) && C(0, 0) > C(2, 2)) {
         T s = 2.0 * sqrt(1.0 + C(0, 0) - C(1, 1) - C(2, 2));
         q.w = (C(2, 1) - C(1, 2)) / s;
         q.x = 0.25 * s;
         q.y = (C(0, 1) + C(1, 0)) / s;
         q.z = (C(0, 2) + C(2, 0)) / s;
      } else if(C(1, 1) > C(2, 2)) {
         T s = 2.0 * sqrt(1.0 + C(1, 1) - C(0, 0) - C(2, 2));
         q.w = (C(0, 2) - C(2, 0)) / s;
         q.x = (C(0, 1) + C(1, 0)) / s;
         q.y = 0.25 * s;
         q.z = (C(1, 2) + C(2, 1)) / s;
      } else {
         T s = 2.0 * sqrt(1.0 + C(2, 2) - C(0, 0) - C(1, 1));
         q.w = (C(1, 0) - C(0, 1)) / s;
         q.x = (C(0, 2) + C(2, 0)) / s;
         q.y = (C(1, 2) + C(2, 1)) / s;
         q.z = 0.25 * s;
      }
   }
#undef C
#undef R
   q.normalise();
   return q;
}

template<typename T>
inline void quaternion_to_Matrix3rT(const QuaternionT<T>& q, Matrix3r& R)
{
   T R_[9];
   quaternion_to_rotation3x3T(q, R_);
   R(0, 0) = R_[0];
   R(0, 1) = R_[1];
   R(0, 2) = R_[2];
   R(1, 0) = R_[3];
   R(1, 1) = R_[4];
   R(1, 2) = R_[5];
   R(2, 0) = R_[6];
   R(2, 1) = R_[7];
   R(2, 2) = R_[8];
}

// ------------------------------------------------ axis-angle-to-rotation-3x3-T

// template<typename T>
// void axis_angle_to_rotation3x3T(const Vector4T<T>& axis_angle,
//                                 T M[9],
//                                 const bool row_maj = true)
// {
//     using std::swap;

//     QuaternionT<T> q;
//     q.from_axis_angle(axis_angle);
//     quaternion_to_rotation3x3T(q, M, row_maj);;
//     return;

//     auto theta = axis_angle.w;
//     auto a = axis_angle.xyz().normalised();

//     double c = cos(theta);
//     double s = sin(theta);
//     double t = 1.0 - c;

// #define R(r, c) M[r*3 + c]
//     R(0,0) = c + a.x*a.x*t;
//     R(1,1) = c + a.y*a.y*t;
//     R(2,2) = c + a.z*a.z*t;

//     double tmp1 = a.x*a.y*t;
//     double tmp2 = a.z*s;
//     R(1,0) = tmp1 + tmp2;
//     R(0,1) = tmp1 - tmp2;
//     tmp1 = a.x*a.z*t;
//     tmp2 = a.y*s;
//     R(2,0) = tmp1 - tmp2;
//     R(0,2) = tmp1 + tmp2;
//     tmp1 = a.y*a.z*t;
//     tmp2 = a.x*s;
//     R(2,1) = tmp1 + tmp2;
//     R(1,2) = tmp1 - tmp2;

//     if(false) {
//         auto s_inv = 1.0 / s;
//         auto x = (R(2,1) - R(1,2)) * 0.5 * s_inv;
//         INFO(format("R21={}, R12={}, c = {}, s = {}, theta = {}, "
//                     "x = {}, a.x = {}",
//                     R(2,1), R(1,2), c, s, theta, x, a.x));

//         INFO("Printing rot3x3");
//         std::cout << str(M, 3, 3) << std::endl;
//     }

// #undef R

//     if(!is_rotation_matrix(M))
//         FATAL(format("rounding errors (?)"));

//     if(!row_maj) {
//         INFO(format("Flip"));
//         swap(M[1], M[3]);
//         swap(M[2], M[6]);
//         swap(M[5], M[7]);
//     }
// }

// ------------------------------------------------ rotation-3x3-to-axis-angle-T

// template<typename T>
// inline Vector4T<T> rotation3x3_to_axis_angleT(const T M[9],
//                                               const bool row_maj = true)
// {
//     QuaternionT<T> q = rotation3x3_to_quaternionT(M, row_maj);
//     return q.to_axis_angle();

//     if(!is_rotation_matrixT(M, row_maj))
//         throw std::runtime_error("Matrix is not a rotation matrix!");

// #define R(r, c) M[r*3 + c]

//     // c = cos(theta)
//     // s = sin(theta)
//     auto c = clamp<double>((R(0,0) + R(1,1) + R(2,2) - 1.0) * 0.5,
//     -1.0, 1.0); auto theta = acos(c); // [0..pi]

//     auto s = sin(theta);
//     auto s_inv = 1.0 / s; // singularity for theta == 0, or theta == 180
//     degrees

//     Vector3T<T> axis;
//     axis.x = (R(2,1) - R(1,2)) * 0.5 * s_inv;
//     axis.y = (R(0,2) - R(2,0)) * 0.5 * s_inv;
//     axis.z = (R(1,0) - R(0,1)) * 0.5 * s_inv;

//     if(false) {
//         INFO(format("R21={}, R12={}, c = {}, theta = {}, s = {},
//         axis.x =
//         {}",
//                     R(2,1), R(1,2),
//                     c, theta*180.0/M_PI, s, axis.x));
//     }

//     if(!axis.is_unit_vector()) {
//         // INFO(format("Axis not a unit vector, axis = {}",
//         axis.to_str()));
//         // we hit a singularity, so theta == 0, or theta == 180, or close
//         if(c > 0.0) {
//             // We'll just say it isn't a rotation at all
//             axis = Vector3T<T>(1.0, 0.0, 0.0);
//         } else {
//             const double epsilon = 1e-3;

//             // A 180 degree rotation, so c is close to -1
//             const double cc_inv = 1.0 - c;

//             const double xx = (R(0,0) - c) * cc_inv;
//             const double yy = (R(1,1) - c) * 0.5;
//             const double zz = (R(2,2) - c) * 0.5;

//             const double xy = (R(0,1) + R(1,0)) * 0.5 * cc_inv;
//             const double xz = (R(0,2) + R(2,0)) * 0.5 * cc_inv;
//             const double yz = (R(1,2) + R(2,1)) * 0.5 * cc_inv;

//             if ((xx > yy) && (xx > zz)) { // R(0,0) is the largest diagonal
//             term
//                 if (xx < epsilon) {
//                     axis.x = 0.0;
//                     axis.y = 0.7071;
//                     axis.z = 0.7071;
//                 } else {
//                     axis.x = sqrt(xx);
//                     axis.y = xy / axis.x;
//                     axis.z = xz / axis.x;
//                 }
//             } else if (yy > zz) { // R(1,1) is the largest diagonal term
//                 if (yy < epsilon) {
//                     axis.x = 0.7071;
//                     axis.y = 0.0;
//                     axis.z = 0.7071;
//                 } else {
//                     axis.y = sqrt(yy);
//                     axis.x = xy / axis.y;
//                     axis.z = yz / axis.y;
//                 }
//             } else { // R(2,2) largest diagonal term so base result on this
//                 if (zz < epsilon) {
//                     axis.x = 0.7071;
//                     axis.y = 0.7071;
//                     axis.z = 0.0;
//                 } else {
//                     axis.z = sqrt(zz);
//                     axis.x = xz / axis.z;
//                     axis.y = yz / axis.z;
//                 }
//             }
//         }
//     }

//     return Vector4T<T>(axis, theta);
// #undef R
// }

// ---- to/from axis-angle
Vector4 rotation3x3_to_axis_angle(const real R[9]) noexcept
{
   return quaternion_to_axis_angle(rotation3x3_to_quaternion(R));
}
void axis_angle_to_rotation3x3(const Vector4& aa, real R[9]) noexcept
{
   quaternion_to_rotation3x3(axis_angle_to_quaternion(aa), R);
}

// ---- to/from quaterion
Quaternion rotation3x3_to_quaternion(const real R[9]) noexcept
{
   return rotation3x3_to_quaternionT(R);
}
void quaternion_to_rotation3x3(const Quaternion& q, real R[9]) noexcept
{
   return quaternion_to_rotation3x3T(q, R);
}

// ---- Matrix3r, Matrix4r
void quaternion_to_Matrix3r(const Quaternion& q, Matrix3r& R) noexcept
{
   quaternion_to_Matrix3rT(q, R);
}

Matrix4r quaternion_to_rot4x4(const Quaternion& q) noexcept
{
   Matrix4r M          = Matrix4r::Identity();
   M.block(0, 0, 3, 3) = quaternion_to_rot3x3(q);
   return M;
}

// ------------------------------ To/from rotation4x4 (homogeneous co-ordinates)

template<typename T>
inline void quaternion_to_rotation4x4T(const QuaternionT<T>& q,
                                       T R[16],
                                       bool row_maj = true)
{
   using std::swap;

   quaternion_to_rotation3x3T(q, &R[0], true);

   // Reshape
   R[10] = R[8];
   R[9]  = R[7];
   R[8]  = R[6];
   R[6]  = R[5];
   R[5]  = R[4];
   R[4]  = R[3];

   // Zero-fill remaining
   R[3] = R[7] = R[11] = R[12] = R[13] = R[14] = T(0.0);

   // Homogeneous '1.0'
   R[15] = 1.0;

   if(!row_maj) {
      swap(R[1], R[4]);
      swap(R[2], R[8]);
      swap(R[3], R[12]);
      swap(R[6], R[9]);
      swap(R[7], R[13]);
      swap(R[11], R[14]);
   }
}

void quaternion_to_rotation4x4(const Quaternion& q,
                               real R[16],
                               bool row_maj) noexcept
{
   quaternion_to_rotation4x4T(q, R, row_maj);
}

} // namespace perceive
