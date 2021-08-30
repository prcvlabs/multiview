
#include "line-2d.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "stdinc.hpp"

namespace perceive
{
template<typename T>
Vector3r fit_2d_line_T(const T* Xs, unsigned N, unsigned stride)
{
   Vector3r line;
   MatrixXr A(N, 3);
   for(unsigned i = 0; i < N; ++i) {
      for(unsigned j = 0; j < 3; ++j) A(i, j) = (*Xs)(int(j));
      Xs += stride;
   }
   MatrixXr At  = A.transpose();
   Matrix3r AtA = At * A;
   svd_thin(AtA, line);
   line /= sqrt(line(0) * line(0) + line(1) * line(1));
   return line;
}

// Vector3d fit_2d_line(const Vector3d * Xs, unsigned N, unsigned stride)
// {
//     auto X = fit_2d_line_T<Vector3d>(Xs, N, stride);
//     Vector3d Z;
//     Z(0) = X(0);
//     Z(1) = X(1);
//     Z(2) = X(2);
//     return Z;
// }

Vector3r fit_2d_line(const Vector3r* Xs, unsigned N, unsigned stride)
{
   return fit_2d_line_T<Vector3r>(Xs, N, stride);
}

Vector3 fit_2d_line(const Vector3* Xs, unsigned N, unsigned stride)
{
   auto L = fit_2d_line_T<Vector3>(Xs, N, stride);
   return to_vec3(L);
}

Vector3 fit_line(const vector<Vector2>& Xs, const vector<unsigned>& inds)
{
   struct TLParams
   {
      MatrixXr M1;
      MatrixXr M2;
      MatrixXr MtM;
      VectorXr thin;
   };
   static thread_local TLParams tl;

   // Choose the matrix
   int sz        = int(inds.size());
   MatrixXr& M   = (tl.M1.rows() == sz || tl.M1.rows() == 0) ? tl.M1 : tl.M2;
   MatrixXr& MtM = tl.MtM;

   // Fix dimensions
   if(M.rows() != int(inds.size()) || M.cols() != 3)
      M = MatrixXr::Zero(long(inds.size()), 3);
   if(MtM.rows() != 3) tl.MtM = MatrixXr::Zero(3, 3);
   if(tl.thin.rows() != 3) tl.thin = VectorXr::Zero(3);

   // Get the line centre
   Vector2 C(0.0, 0.0);
   for(const auto ind : inds) C += Xs[ind];
   C /= real(inds.size());

   // Fill 'M'
   for(unsigned row = 0; row < inds.size(); ++row) {
      const auto& X = Xs[inds[row]] - C;
      M(row, 0)     = X.x;
      M(row, 1)     = X.y;
      M(row, 2)     = 1.0;
   }

   // Calculate 'MtM'
   auto calc_dot = [](const MatrixXr& M, unsigned i, unsigned j) {
      double val  = 0.0;
      unsigned sz = unsigned(M.cols());
      for(unsigned k = 0; k < sz; ++k) val += M(i, k) * M(k, j);
      return val;
   };
   for(unsigned i = 0; i < 3; ++i)
      for(unsigned j = 0; j < 3; ++j) MtM(i, j) = calc_dot(M, i, j);

   // Get the line equation
   auto err   = svd_thin(MtM, tl.thin);
   auto line  = Vector3(tl.thin(0), tl.thin(1), tl.thin(2));
   auto scale = sqrt(square(line.x) + square(line.y));
   line /= scale;
   line.z = -(C.x * line.x + C.y * line.y);

   return line;
}

} // namespace perceive
