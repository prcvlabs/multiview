
#include "perceive/utils/eigen-helpers.hpp"
#include "vector-4.hpp"

namespace perceive
{
namespace detail
{
   static bool debug_flag{false};
   void set_fit_plane_debug_flag(bool value) { debug_flag = value; }
} // namespace detail

template<typename InputItr>
Vector4 fit_plane_T(InputItr start, InputItr end) noexcept
{
   const bool feedback = detail::debug_flag;

   auto sz = std::distance(start, end);
   if(sz < 3) return Vector4::nan();
   if(sz == 3) {
      auto xx = to_vec3(*start++);
      auto yy = to_vec3(*start++);
      auto zz = to_vec3(*start++);
      return plane_from_3_points(xx, yy, zz);
   }

   MatrixXr A   = MatrixXr::Zero(sz, 4);
   unsigned row = 0;
   while(start != end) {
      auto X    = *start++;
      A(row, 0) = X(0);
      A(row, 1) = X(1);
      A(row, 2) = X(2);
      A(row, 3) = 1.0;
      row++;
   }

   VectorXr p3;

   if(feedback) {
      MatrixXr U, V;
      VectorXr D;
      svd_UDV(A, U, D, V);
      INFO(format("fit-plane-t"));
      cout << format("  eigenvalues: {}, {}, {}, {}", D(0), D(1), D(2), D(3))
           << endl;
      cout << format("  condition-number {} (last two: {})",
                     D(0) / D(3),
                     D(2) / D(3))
           << endl;
   }

   {
      // static std::mutex padlock;
      // lock_guard lock(padlock);
      // TRACE("PLANE");
      // cout << format("A = ") << endl << A << endl << endl;
      svd_thin(A, p3);
      // TRACE("<<DONE");
   }

   Vector4 ret(p3(0), p3(1), p3(2), p3(3));
   ret /= ret.xyz().norm();
   return ret;
}

Vector4 fit_plane(const std::vector<Vector3>& Xs) noexcept
{
   return fit_plane_T(Xs.begin(), Xs.end());
}

Vector4 fit_plane(const std::vector<Vector3r>& Xs) noexcept
{
   return fit_plane_T(Xs.begin(), Xs.end());
}

Vector4 fit_plane(const Vector3* Xs, const unsigned n) noexcept
{
   return fit_plane_T(Xs, Xs + n);
}

Vector4 fit_plane(const Vector3r* Xs, const unsigned n) noexcept
{
   return fit_plane_T(Xs, Xs + n);
}

Vector4 fit_plane(std::initializer_list<Vector3> Xs) noexcept
{
   return fit_plane(std::vector<Vector3>(Xs));
}

Vector3 intersection_of_3_planes(const Plane& A,
                                 const Plane& B,
                                 const Plane& C) noexcept
{
   Expects(A.is_finite());
   Expects(B.is_finite());
   Expects(C.is_finite());

   MatrixXr M(3, 4); // one row per plane equation
   for(int j = 0; j < 4; ++j) M(0, j) = A[j];
   for(int j = 0; j < 4; ++j) M(1, j) = B[j];
   for(int j = 0; j < 4; ++j) M(2, j) = C[j];

   MatrixXr Xr = matrix_kernel(M);
   Vector3 O;
   for(auto i = 0; i < 3; ++i) O(i) = Xr(i, 0) / Xr(3, 0);

   { // Sanity check
      // assert(std::abs(A.side(O)) < 1e-6);
      Expects(std::abs(A.side(O)) < 1e-6);
      Expects(std::abs(B.side(O)) < 1e-6);
      Expects(std::abs(C.side(O)) < 1e-6);
   }

   return O;
}

} // namespace perceive
