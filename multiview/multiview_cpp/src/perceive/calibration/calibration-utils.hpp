
#pragma once

#include "stdinc.hpp"

#include <opencv2/core.hpp>

#include "find-homography.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive::calibration
{
// --------------------------------------------------------------------- Lift --

template<unsigned K> auto lift(real x, real y)
{
   using VectorKKr = Eigen::Matrix<real, (K + 1) * (K + 2) / 2, 1, EIGEN_ALIGN>;
   VectorKKr X;

   unsigned counter = unsigned(X.rows());

   X(--counter) = 1.0;

   if(K >= 1) {
      X(--counter) = y;
      X(--counter) = x;
      if(K >= 2) --counter;
   }

   for(unsigned n = 2; n <= K; ++n) {
      for(unsigned i = 0; i < n; ++i, --counter)
         X(counter) = y * X(counter + n);
      X(counter) = x * X(counter + n + 1);
      if(n < K) --counter;
   }

   Ensures(counter == 0);

   return X;
}

// -------------------------------------------------------------- Flip cols/rows

void flip_rows(const unsigned nx, const unsigned ny, vector<Vector3r>& C);
void flip_cols(const unsigned nx, const unsigned ny, vector<Vector3r>& C);

void flip_rows(const unsigned nx, const unsigned ny, vector<Vector2>& C);
void flip_cols(const unsigned nx, const unsigned ny, vector<Vector2>& C);

// ----------------------------------------------------------- Calc line inds --

void calc_line_inds(unsigned nx,
                    unsigned ny,
                    unsigned line_id, // [0..nx+ny]
                    vector<unsigned>& line_inds);

// --------------------------------------------------------- is a border line --

inline bool is_border_line(int nx, int ny, int line_ind)
{
   return (line_ind == 0 || line_ind == ny - 1 || line_ind == ny
           || line_ind == nx + ny - 1);
}

// ----------------------------------------------- Estimate distortion center --

Vector2 estimate_distortion_center(const vector<vector<Vector3r>>& Xs,
                                   const vector<Vector3r>& W,
                                   bool feedback);

// ----------------------------------------------------------------------- AX --

template<unsigned K> Vector2 AX(const MatrixXr& A, Vector2 x)
{
   constexpr unsigned KK = (K + 1) * (K + 2) / 2;
   Expects(A.cols() == KK);
   Expects(A.rows() == 2 || A.rows() == 3);

   using VectorKKr = decltype(lift<K>(x.x, x.y));
   VectorKKr X     = lift<K>(x.x, x.y);
   if(A.rows() == 2) {
      Vector2r y = A * X;
      return Vector2(y(0), y(1));
   }

   Vector3r y = A * X;
   y /= y(2);
   return Vector2(y(0), y(1));
}

// ------------------------------------------------------------- Reproj Error --
// |w - f(x)|
// Expects: |Xs| >= |Ws|
real projection_err(std::function<Vector2(const Vector2& x)> f,
                    const vector<vector<Vector3r>>& Ws,
                    const vector<vector<Vector3r>>& Xs);

real average_R2_err(const vector<vector<Vector3r>>& Ws,
                    const vector<vector<Vector3r>>& Us);

// ------------------------------------------------------------ r1r2ts <=> Hs --

void Hs_to_r1r2ts(const vector<Matrix3r>& Hs, vector<Vector6r>& r1r2ts);
void r1r2ts_to_Hs(const vector<Vector6r>& r1r2ts, vector<Matrix3r>& Hs);

// ------------------------------------------------------------------- r1r2ts --

real estimate_r1r2t(const vector<Vector3r>& W, // Calibration grid
                    const vector<Vector3r>& U, // Undistorted
                    Vector6r& X,
                    const bool feedback);

inline real estimate_H(const vector<Vector3r>& W, // Calibration grid
                       const vector<Vector3r>& U, // Undistorted
                       Matrix3r& H,
                       const bool feedback)
{
   Vector6r r1r2t;
   auto err = estimate_r1r2t(W, U, r1r2t, feedback);
   r1r2t_to_H(r1r2t, H);
   return err;
}

real estimate_r1r2ts(const vector<Vector3r>& W,          // Calibration grid
                     const vector<vector<Vector3r>>& Us, // Undistorted
                     vector<Vector6r>& r1r2ts,
                     const bool use_threads,
                     const bool feedback);

vector<Vector6r> calc_r1r2ts(const vector<Vector3r>& W,
                             const vector<vector<Vector3r>>& Ds,
                             std::function<Vector2(const Vector2&)> undistort);

// ----------------------------------------------------------- make-w-pattern --

vector<Vector3r> make_W_pattern(unsigned nx, unsigned ny);

} // namespace perceive::calibration
