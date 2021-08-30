
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"
#include <Eigen/Dense>

namespace perceive
{
// ------------------------------------------------------------------- is-finite

bool is_finite(const Vector2r& M);
bool is_finite(const Vector3r& M);
bool is_finite(const Vector4r& M);
bool is_finite(const Matrix3r& M);
bool is_finite(const Matrix34r& M);
bool is_finite(const MatrixXr& M);

// ------------------------------------------------------------------- normalize

inline Vector3r& normalize(Vector3r& X)
{
   auto norm_inv = 1.0 / X.norm();
   X *= norm_inv;
   return X;
}

inline Vector3r normalized(const Vector3r& X)
{
   auto Y(X);
   normalize(Y);
   return Y;
}

// ------------------------------------------------------------------------- str

std::string str(const Vector2r& M);
std::string str(const Vector3r& M);
std::string str(const Vector4r& M);
std::string str(const Matrix3r& M);
std::string str(const Matrix34r& M);
std::string str(const MatrixXr& M);

std::string str(std::string name, const Matrix3r& M);
std::string str(std::string name, const Matrix34r& M);
std::string str(std::string name, const MatrixXr& M);

// ------------------------------------------------------------------------- SVD

real svd_thin(const MatrixXr& M, VectorXr& out);
real svd_thin(const Matrix3r& M, Vector3r& out);
// real svd_thin(const Matrix3d& M, Vector3d& out);
// real svd_thin(const MatrixXd& M, VectorXd& out);
real svd_thin(const MatrixXr& M, Vector6r& out);
void svd_UV(const MatrixXr& M, MatrixXr& U, MatrixXr& V);
void svd_UV(const Matrix3r& M, Matrix3r& U, Matrix3r& V);
// void svd_UV(const MatrixXd& M, MatrixXd& U, MatrixXd& V);
void svd_UDV(const MatrixXr& M, MatrixXr& U, VectorXr& D, MatrixXr& V);
void svd_UDV(const Matrix3r& M, Matrix3r& U, Vector3r& D, Matrix3r& V);
void svd_UDV(const Matrix3r& M, Matrix3r& U, Matrix3r& D, Matrix3r& V);
// void svd_UDV(const MatrixXd& M, MatrixXd& U, VectorXd& D, MatrixXd& V);

void svd_UDV(const Eigen::Matrix<std::complex<long double>, 3, 3>& M,
             Eigen::Matrix<std::complex<long double>, 3, 3>& U,
             Eigen::Matrix<std::complex<long double>, 3, 1>& D,
             Eigen::Matrix<std::complex<long double>, 3, 3>& V);
void svd_UDV(const Eigen::Matrix<std::complex<double>, 3, 3>& M,
             Eigen::Matrix<std::complex<double>, 3, 3>& U,
             Eigen::Matrix<std::complex<double>, 3, 1>& D,
             Eigen::Matrix<std::complex<double>, 3, 3>& V);
void svd_UDV(const Eigen::Matrix<std::complex<float>, 3, 3>& M,
             Eigen::Matrix<std::complex<float>, 3, 3>& U,
             Eigen::Matrix<std::complex<float>, 3, 1>& D,
             Eigen::Matrix<std::complex<float>, 3, 3>& V);

void svd_UDV(
    const Eigen::
        Matrix<std::complex<long double>, Eigen::Dynamic, Eigen::Dynamic>& M,
    Eigen::Matrix<std::complex<long double>, Eigen::Dynamic, Eigen::Dynamic>& U,
    Eigen::Matrix<std::complex<long double>, Eigen::Dynamic, 1>& D,
    Eigen::Matrix<std::complex<long double>, Eigen::Dynamic, Eigen::Dynamic>&
        V);

void svd_UDV(
    const Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>&
        M,
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>& U,
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, 1>& D,
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>& V);

void svd_UDV(
    const Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic>& M,
    Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic>& U,
    Eigen::Matrix<std::complex<float>, Eigen::Dynamic, 1>& D,
    Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic>& V);

// double bdcsvd_thin(const MatrixXd& M, VectorXd& out);
real bdcsvd_thin(const MatrixXr& M, VectorXr& out);

template<typename T> struct CentreEigenVectorResult
{
   T C;                           // centre
   vector<std::pair<T, real>> Es; // {eigenvectors, eigenvalues}
};

template<typename InputIt>
auto calc_centre_and_eigenvectors(InputIt begin, InputIt end)
    -> CentreEigenVectorResult<
        typename std::iterator_traits<InputIt>::value_type>;

// For best results, centre and scale the data before performing svd-3d-UDV
struct SVD3DRet
{
   Matrix3r U, V;
   Vector3r D;
   Matrix3r Dm() const noexcept; // `D` as a Diagonal Matrix
   string to_string() const noexcept;
   friend string str(const SVD3DRet& o) noexcept { return o.to_string(); }
   Vector3r eigen_vector(int ind) const noexcept;
   Quaternion rot_vec() const noexcept;
};

template<typename InputIt> SVD3DRet svd_3d_UDV(InputIt start, InputIt finish);

// ------------------------------------------------------------ condition-number

real condition_number(const MatrixXr& M);
real condition_number(const Matrix3r& M);

// ----------------------------------------------------------------- matrix rank

template<typename T> inline unsigned matrix_rank(const T& M)
{
   Eigen::FullPivLU<T> lu(M);
   return lu.rank();
}

// ------------------------------------------------------------------ null space
template<typename T> inline T matrix_kernel(const T& M)
{
   Eigen::FullPivLU<T> lu(M);
   T out = lu.kernel();
   return out;
}

// -------------------------------------------------------- Cross-product Matrix

void to_cross_product_matrix(const Vector3& X, Matrix3r& M);
void to_cross_product_matrix(const Vector3r& X, Matrix3r& M);
Matrix3r make_cross_product_matrix(const Vector3& X);
Matrix3r make_cross_product_matrix(const Vector3r& X);

// --------------------------------------------------- Quadratic with Constraint

// Minimize xGtGx such that Hx = h
//  * G must have more rows than columns.
//  * G.cols() == H.cols()
MatrixXr minimize_with_constraint(const MatrixXr& G, const MatrixXr& H, real h);

//
//
//
//
//
//
//
//
//                                Implementations
//
//
//
//
//
//
//

template<typename InputIt>
auto calc_centre_and_eigenvectors(InputIt begin, InputIt end)
    -> CentreEigenVectorResult<
        typename std::iterator_traits<InputIt>::value_type>
{
   using T   = typename std::iterator_traits<InputIt>::value_type;
   using Ret = CentreEigenVectorResult<T>;

   Ret out;
   const int sz = std::distance(begin, end);
   const int N  = (sz == 0) ? -1 : begin->size();
   if(sz < N) return out;

   // Calculate the centre
   T C;
   for(auto i = 0; i < sz; ++i) C(i) = 0.0;
   for(auto ii = begin; ii != end; ++ii) { C += *ii; }
   C /= sz;

   // Create matrices
   MatrixXr A(sz, N);
   int pos = 0;
   for(auto ii = begin; ii != end; ++ii, ++pos) {
      for(auto j = 0; j < N; ++j) A(pos, j) = ii->operator()(j) - C(j);
   }

   MatrixXr At  = A.transpose();
   MatrixXr AtA = At * A;

   MatrixXr U, V;
   VectorXr D;
   svd_UDV(A, U, D, V);

   Expects(V.rows() == N);
   Expects(V.cols() == N);

   out.C = C;
   out.Es.resize(N);
   for(auto i = 0; i < N; ++i) {
      auto& E  = out.Es[i];
      E.second = D(i);
      for(auto j = 0; j < N; ++j) { E.first(j) = V(j, i); }
   }

   return out;
}

// -------------------------------------------------------------------svd-3d-UDV
// For best results, center and scale the data first.
template<typename InputIt> SVD3DRet svd_3d_UDV(InputIt start, InputIt finish)
{
   using T = typename std::iterator_traits<InputIt>::value_type;

   const size_t n_rows     = size_t(std::distance(start, finish));
   constexpr size_t n_cols = 3;
   Expects(n_rows >= n_cols);

   auto M  = MatrixXr(n_rows, n_cols);
   int row = 0;
   for(auto ii = start; ii != finish; ++ii) {
      const T& o = *ii;
      Expects(o.size() == n_cols);
      for(int col = 0; col < int(n_cols); ++col) M(row, col) = real(o(col));
      ++row;
   }

   MatrixXr Mt  = M.transpose();
   MatrixXr MtM = Mt * M;

   SVD3DRet ret;
   svd_UDV(MtM, ret.U, ret.D, ret.V);

   return {ret};
}

} // namespace perceive
