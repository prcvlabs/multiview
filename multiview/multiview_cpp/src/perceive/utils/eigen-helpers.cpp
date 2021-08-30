
#include "eigen-helpers.hpp"

#include "perceive/geometry/rotation.hpp"

#include <Eigen/SVD>

using std::stringstream;

namespace perceive
{
// ------------------------------------------------------------------- Is finite
// Checks that an Eigen Matrix is finite
template<typename T> bool is_finiteT(const T& M)
{
   int n_rows = int(M.rows());
   int n_cols = int(M.cols());
   for(int i = 0; i < n_rows; ++i)
      for(int j = 0; j < n_cols; ++j)
         if(!std::isfinite(M(i, j))) return false;
   return true;
}

bool is_finite(const Vector2r& M) { return is_finiteT(M); }
bool is_finite(const Vector3r& M) { return is_finiteT(M); }
bool is_finite(const Vector4r& M) { return is_finiteT(M); }
bool is_finite(const Matrix3r& M) { return is_finiteT(M); }
bool is_finite(const Matrix34r& M) { return is_finiteT(M); }
bool is_finite(const MatrixXr& M) { return is_finiteT(M); }

// -------------------------------------------------------------------- str(...)

std::string str(const Vector2r& M) { return format("[{}, {}]", M(0), M(1)); }
std::string str(const Vector3r& M)
{
   return format("[{}, {}, {}]", M(0), M(1), M(2));
}
std::string str(const Vector4r& M)
{
   return format("[{}, {}, {}, {}]", M(0), M(1), M(2), M(3));
}
std::string str(const Matrix3r& M)
{
   stringstream ss("");
   ss << M;
   return ss.str();
}
std::string str(const Matrix34r& M)
{
   stringstream ss("");
   ss << M;
   return ss.str();
}
std::string str(const MatrixXr& M)
{
   stringstream ss("");
   ss << M;
   return ss.str();
}

std::string str(std::string name, const Matrix3r& M)
{
   stringstream ss("");
   ss << name << " = \n" << M << "\n";
   return ss.str();
}
std::string str(std::string name, const Matrix34r& M)
{
   stringstream ss("");
   ss << name << " = \n" << M << "\n";
   return ss.str();
}
std::string str(std::string name, const MatrixXr& M)
{
   stringstream ss("");
   ss << name << " = \n" << M << "\n";
   return ss.str();
}

// ------------------------------------------------------------------------- SVD

// -- Thin

real svd_thin(const MatrixXr& M, VectorXr& out)
{
   Eigen::JacobiSVD<MatrixXr> svd(M, Eigen::ComputeThinV);
   out = svd.matrixV().col(M.cols() - 1);
   return svd.singularValues()(M.cols() - 1);
}

real svd_thin(const Matrix3r& M, Vector3r& out)
{
   Eigen::JacobiSVD<MatrixXr> svd(M, Eigen::ComputeThinV);
   out = svd.matrixV().col(M.cols() - 1);
   return svd.singularValues()(M.cols() - 1);
}

// real svd_thin(const Matrix3d& M, Vector3d& out)
// {
//     Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeThinV);
//     out = svd.matrixV().col(M.cols() - 1);
//     return svd.singularValues()(M.cols() - 1);
// }

// real svd_thin(const MatrixXd& M, VectorXd& out)
// {
//     Eigen::JacobiSVD<MatrixXd> svd(M, Eigen::ComputeThinV);
//     out = svd.matrixV().col(M.cols() - 1);
//     return svd.singularValues()(M.cols() - 1);
// }

real svd_thin(const MatrixXr& M, Vector6r& out)
{
   Eigen::JacobiSVD<MatrixXr> svd(M, Eigen::ComputeThinV);
   out = svd.matrixV().col(M.cols() - 1);
   return svd.singularValues()(M.cols() - 1);
}

// -- UD

void svd_UV(const MatrixXr& M, MatrixXr& U, MatrixXr& V)
{
   Eigen::JacobiSVD<MatrixXr> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
   U = svd.matrixU();
   V = svd.matrixV();
}

void svd_UV(const Matrix3r& M, Matrix3r& U, Matrix3r& V)
{
   Eigen::JacobiSVD<MatrixXr> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
   U = svd.matrixU();
   V = svd.matrixV();
}

// void svd_UV(const MatrixXd& M, MatrixXd& U, MatrixXd& V)
// {
//     Eigen::JacobiSVD<MatrixXd> svd(M,
//     Eigen::ComputeThinU|Eigen::ComputeThinV); U = svd.matrixU(); V =
//     svd.matrixV();
// }

// -- UDV

void svd_UDV(const MatrixXr& M, MatrixXr& U, VectorXr& D, MatrixXr& V)
{
   Eigen::JacobiSVD<MatrixXr> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
   U      = svd.matrixU();
   V      = svd.matrixV();
   auto s = svd.singularValues();
   auto n = s.rows();
   D.resize(n);
   for(uint i = 0; i < n; ++i) D(i) = s(i);
}

void svd_UDV(const Matrix3r& M, Matrix3r& U, Vector3r& D, Matrix3r& V)
{
   Eigen::JacobiSVD<MatrixXr> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
   U      = svd.matrixU();
   V      = svd.matrixV();
   auto s = svd.singularValues();
   for(uint i = 0; i < 3; ++i) D(i) = s(i);
}

void svd_UDV(const Matrix3r& M, Matrix3r& U, Matrix3r& D, Matrix3r& V)
{
   Vector3r d;
   svd_UDV(M, U, d, V);
   D = Matrix3r::Zero();
   for(auto i = 0; i < 3; ++i) D(i, i) = d(i);
}

// void svd_UDV(const MatrixXd& M, MatrixXd& U, VectorXd& D, MatrixXd& V)
// {
//     Eigen::JacobiSVD<MatrixXd> svd(M,
//     Eigen::ComputeThinU|Eigen::ComputeThinV); U = svd.matrixU(); V =
//     svd.matrixV(); auto s = svd.singularValues(); auto n = s.rows();
//     D.resize(n);
//     for(uint i = 0; i < n; ++i)
//         D(i) = s(i);
// }

Matrix3r SVD3DRet::Dm() const noexcept
{
   Matrix3r X = Matrix3r::Identity();
   for(auto i = 0; i < 3; ++i) X(i, i) = D(i);
   return X;
}

string SVD3DRet::to_string() const noexcept
{
   std::stringstream ss{""};
   ss << format("U = \n") << U << "\n\n"
      << format("D = \n") << Dm() << "\n\n"
      << format("V = \n") << V << "\n\n";
   return ss.str();
}

Vector3r SVD3DRet::eigen_vector(int ind) const noexcept
{
   Expects(ind >= 0 && ind < 3);
   return Vector3r(V(ind, 0), V(ind, 1), V(ind, 2));
}

Quaternion SVD3DRet::rot_vec() const noexcept
{
   return rot3x3_to_quaternion(V.transpose());
}

template<typename J>
using MatrixXc_J
    = Eigen::Matrix<std::complex<J>, Eigen::Dynamic, Eigen::Dynamic>;
template<typename J>
using VectorXc_J = Eigen::Matrix<std::complex<J>, Eigen::Dynamic, 1>;

template<typename T>
void svd_UDV_T(const MatrixXc_J<T>& M,
               MatrixXc_J<T>& U,
               VectorXc_J<T>& D,
               MatrixXc_J<T>& V)
{
   using MatrixXc = MatrixXc_J<T>;
   // using VectorXc = VectorXc_J<T>;
   Eigen::JacobiSVD<MatrixXc> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
   U      = svd.matrixU();
   V      = svd.matrixV();
   auto s = svd.singularValues();
   auto n = s.rows();
   D.resize(n, 1);
   for(uint i = 0; i < n; ++i) D(i) = s(i);
}

void svd_UDV(const MatrixXc_J<long double>& M,
             MatrixXc_J<long double>& U,
             VectorXc_J<long double>& D,
             MatrixXc_J<long double>& V)
{
   svd_UDV_T<long double>(M, U, D, V);
}

void svd_UDV(const MatrixXc_J<double>& M,
             MatrixXc_J<double>& U,
             VectorXc_J<double>& D,
             MatrixXc_J<double>& V)
{
   svd_UDV_T<double>(M, U, D, V);
}

void svd_UDV(const MatrixXc_J<float>& M,
             MatrixXc_J<float>& U,
             VectorXc_J<float>& D,
             MatrixXc_J<float>& V)
{
   svd_UDV_T<float>(M, U, D, V);
}

template<typename T>
void svd_UDV_T(const Eigen::Matrix<std::complex<T>, 3, 3>& M,
               Eigen::Matrix<std::complex<T>, 3, 3>& U,
               Eigen::Matrix<std::complex<T>, 3, 1>& D,
               Eigen::Matrix<std::complex<T>, 3, 3>& V)
{
   using MatrixXc = MatrixXc_J<T>;
   // using VectorXc = VectorXc_J<T>;

   Eigen::JacobiSVD<MatrixXc> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
   U      = svd.matrixU();
   V      = svd.matrixV();
   auto s = svd.singularValues();
   auto n = s.rows();
   D.resize(n, 1);
   for(uint i = 0; i < n; ++i) D(i) = s(i);
}

void svd_UDV(const Eigen::Matrix<std::complex<long double>, 3, 3>& M,
             Eigen::Matrix<std::complex<long double>, 3, 3>& U,
             Eigen::Matrix<std::complex<long double>, 3, 1>& D,
             Eigen::Matrix<std::complex<long double>, 3, 3>& V)
{
   svd_UDV_T(M, U, D, V);
}

void svd_UDV(const Eigen::Matrix<std::complex<double>, 3, 3>& M,
             Eigen::Matrix<std::complex<double>, 3, 3>& U,
             Eigen::Matrix<std::complex<double>, 3, 1>& D,
             Eigen::Matrix<std::complex<double>, 3, 3>& V)
{
   svd_UDV_T(M, U, D, V);
}

void svd_UDV(const Eigen::Matrix<std::complex<float>, 3, 3>& M,
             Eigen::Matrix<std::complex<float>, 3, 3>& U,
             Eigen::Matrix<std::complex<float>, 3, 1>& D,
             Eigen::Matrix<std::complex<float>, 3, 3>& V)
{
   svd_UDV_T(M, U, D, V);
}

template<typename T, typename S> double bdcsvd_thin_TS(const T& M, S& out)
{
   Eigen::BDCSVD<T> svd(M, Eigen::ComputeThinV);
   out = svd.matrixV().col(M.cols() - 1);
   return svd.singularValues()(M.cols() - 1);
}

// double bdcsvd_thin(const MatrixXd& M, VectorXd& out)
// { return bdcsvd_thin_TS(M, out); }
real bdcsvd_thin(const MatrixXr& M, VectorXr& out)
{
   return bdcsvd_thin_TS(M, out);
}

// ------------------------------------------------------------ condition-number

template<typename T> real condition_number_T(const T& M)
{
   Eigen::JacobiSVD<T> svd(M);
   auto vals = svd.singularValues();
   return vals(0) / vals(vals.rows() - 1);
}

real condition_number(const MatrixXr& M) { return condition_number_T(M); }

real condition_number(const Matrix3r& M) { return condition_number_T(M); }

// -------------------------------------------------------- Cross-product Matrix

template<typename T> void to_cross_product_matrix_T(const T& X, Matrix3r& M)
{
   M(0, 0) = M(1, 1) = M(2, 2) = 0.0;
   M(0, 1)                     = -X(2);
   M(1, 0)                     = X(2);
   M(0, 2)                     = X(1);
   M(2, 0)                     = -X(1);
   M(1, 2)                     = -X(0);
   M(2, 1)                     = X(0);
}

void to_cross_product_matrix(const Vector3r& X, Matrix3r& M)
{
   to_cross_product_matrix_T(X, M);
}

void to_cross_product_matrix(const Vector3& X, Matrix3r& M)
{
   to_cross_product_matrix_T(X, M);
}

Matrix3r make_cross_product_matrix(const Vector3& X)
{
   Matrix3r M;
   to_cross_product_matrix_T(X, M);
   return M;
}

Matrix3r make_cross_product_matrix(const Vector3r& X)
{
   Matrix3r M;
   to_cross_product_matrix_T(X, M);
   return M;
}

// --------------------------------------------------- Quadratic with Constraint

// Minimize xGtGx such that Hx = h
//  * G must have more rows than columns.
//  * G.cols() == H.cols()
MatrixXr minimize_with_constraint(const MatrixXr& G, const MatrixXr& H, real h)
{
   const unsigned n_vars      = unsigned(G.cols());
   const unsigned n_equations = unsigned(G.rows());
   MatrixXr sol               = MatrixXr::Zero(n_vars, 1);

   if(G.rows() < G.cols()) {
      LOG_ERR(format("Must have more equations than unknowns!"));
      return sol;
   }

   if(H.rows() != n_vars) {
      LOG_ERR(format("G and H must have the same number of columns!"));
      return sol;
   }

   MatrixXr A = MatrixXr::Zero(n_vars + 1, n_vars + 1);
   MatrixXr b = MatrixXr::Zero(n_vars + 1, 1); // Ax - b = 0

   b(n_vars) = h;
   for(unsigned row = 0; row < n_vars; ++row) {
      A(row, n_vars) = H(row);
      A(n_vars, row) = H(row);
   }
   A.block(0, 0, n_vars, n_vars) = G.transpose() * G;

   if(false) {
      MatrixXr U, V;
      VectorXr D;
      svd_UDV(A, U, D, V);

      cout << str("A\n") << A << endl << endl;
      cout << str("U\n") << U << endl << endl;
      cout << str("D\n") << D << endl << endl;
      cout << str("V\n") << V << endl << endl;
   }

   MatrixXr Ainv       = A.inverse();
   MatrixXr sol_lambda = Ainv * b;
   sol                 = sol_lambda.block(0, 0, n_vars, 1);

   return sol;
}

} // namespace perceive
