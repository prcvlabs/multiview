
#include "euclidean-transform.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/utils/eigen-helpers.hpp"

namespace perceive
{
// ---- Get the equivalent 4x4 matrix
Matrix4r make_transform_matrix(const EuclideanTransformT<real>& et) noexcept
{
   Matrix4r S = Matrix4r::Identity() * et.scale;
   S(3, 3)    = 1.0;
   Matrix4r R = quaternion_to_rot4x4(et.rotation);
   Matrix4r T = Matrix4r::Identity(); // translation

   const auto t = et.translation;
   T(0, 3)      = t.x;
   T(1, 3)      = t.y;
   T(2, 3)      = t.z;

   return T * R * S;
}

EuclideanTransformT<real>
transform_between(const vector<Vector3T<real>>& A,
                  const vector<Vector3T<real>>& B) noexcept
{
   Expects(A.size() == B.size());

   using Vector3            = Vector3T<real>;
   using EuclideanTransform = EuclideanTransformT<real>;

   const auto O = Vector3(0, 0, 0);
   const auto N = A.size();

   // Centering
   const auto C0 = std::accumulate(cbegin(A), cend(A), O) / real(N);
   const auto C1 = std::accumulate(cbegin(B), cend(B), O) / real(N);

   // Scale...
   auto scale0 = 0.0;
   auto scale1 = 0.0;
   for(size_t i = 0u; i < N; ++i) {
      scale0 += (A[i] - C0).norm();
      scale1 += (B[i] - C1).norm();
   }
   scale0 /= real(N);
   scale1 /= real(N);

   // Set up MatrixXr representation of points
   MatrixXr P = MatrixXr::Zero(long(N), 3);
   MatrixXr Q = MatrixXr::Zero(long(N), 3);

   for(auto i = 0u; i < N; ++i) {
      P.block(i, 0, 1, 3) = to_vec3r((A[i] - C0) / scale0).transpose();
      Q.block(i, 0, 1, 3) = to_vec3r((B[i] - C1) / scale1).transpose();
   }

   Matrix3r H = P.transpose() * Q; // Rotating P ==> Q

   Matrix3r U, D, V;
   svd_UDV(H, U, D, V);

   Matrix3r F = Matrix3r::Identity();
   F(2, 2)    = (V * U.transpose()).determinant();
   Matrix3r R = V * F * U.transpose();

   EuclideanTransform et0(-C0, Quaternion::identity());
   EuclideanTransform et1(Vector3(0, 0, 0), Quaternion::identity());
   EuclideanTransform et2(Vector3(0, 0, 0), rot3x3_to_quaternion(R));
   EuclideanTransform et3(Vector3(0, 0, 0), Quaternion::identity());
   EuclideanTransform et4(C1, Quaternion::identity());

   et1.scale = 1.0 / scale0;
   et3.scale = scale1;

   if(false) {
      INFO("transform-between");
      cout << "P = " << endl << P << endl << endl;
      cout << "H = " << endl << H << endl << endl;
      cout << "F = " << endl << F << endl << endl;
      cout << "U = " << endl << U << endl << endl;
      cout << "D = " << endl << D << endl << endl;
      cout << "V = " << endl << V << endl << endl;
      cout << "R = " << endl << R << endl << endl;

      for(auto i = 0u; i < N; ++i) {
         const Vector3r x = to_vec3r(A[i]), y = to_vec3r(B[i]);
         const Vector3r z = R * (x - to_vec3r(C0)) + to_vec3r(C1);
         cout << format("x = {}, y = {}, z = {}, \u0394 = {}",
                        str(to_vec3(x)),
                        str(to_vec3(y)),
                        str(to_vec3(z)),
                        (to_vec3(y) - to_vec3(z)).norm())
              << endl;

         const auto a = A[i];
         const auto b = B[i];

         const auto c = (et0 * et1 * et2 * et3 * et4).apply(a);
         cout << format("a = {}, b = {}, c = {}, \u0394 = {}",
                        str(a),
                        str(b),
                        str(c),
                        (c - b).norm())
              << endl;

         cout << endl;
      }
   }

   return et0 * et1 * et2 * et3 * et4;
}

// ----------------------------------------------------------------- pack/unpack
// Pack/Unpack the euclidean-transform as 5df. (So no scale)
template<typename T>
void pack_et_6df_(const EuclideanTransformT<T>& et, T* X) noexcept
{
   const auto saa = et.rotation.to_spherical_axis_angle();
   X[0]           = et.translation.x;
   X[1]           = et.translation.y;
   X[2]           = et.translation.z;
   X[3]           = saa.x;
   X[4]           = saa.y;
   X[5]           = saa.z;
}

template<typename T> EuclideanTransformT<T> unpack_et_6df_(const T* X) noexcept
{
   EuclideanTransformT<T> et;
   et.translation = Vector3T<T>{X[0], X[1], X[2]};
   et.rotation.from_spherical_axis_angle(Vector3T<T>{X[3], X[4], X[5]});
   et.scale = T(1);
   return et;
}

void pack_et_6df(const EuclideanTransformT<double>& et, double* X) noexcept
{
   pack_et_6df_(et, X);
}

void pack_et_6df(const EuclideanTransformT<float>& et, float* X) noexcept
{
   pack_et_6df_(et, X);
}

EuclideanTransformT<double> unpack_et_6df(const double* X) noexcept
{
   return unpack_et_6df_(X);
}

EuclideanTransformT<float> unpack_et_6df(const float* X) noexcept
{
   return unpack_et_6df_(X);
}

} // namespace perceive
