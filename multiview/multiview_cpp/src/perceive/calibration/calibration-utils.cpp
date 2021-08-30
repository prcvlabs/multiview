
#include "calibration-utils.hpp"
#include "find-F.hpp"
#include "find-homography.hpp"
#include "fit-curve-to-checkerboard.hpp"
#include "stdinc.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "perceive/graphics/colour-set.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/threads.hpp"

namespace perceive::calibration
{
// -------------------------------------------------------------- Flip cols/rows

template<typename P>
void flip_rowsP(const unsigned nx, const unsigned ny, vector<P>& C)
{
   // reverses the order of the rows
   auto T = C;
   for(auto y = 0u; y < ny; ++y)
      for(auto x = 0u; x < nx; ++x) T[y * nx + x] = C[(ny - y - 1) * nx + x];
   C = T;
}

template<typename P>
void flip_colsP(const unsigned nx, const unsigned ny, vector<P>& C)
{
   // reverses the order of the cols
   auto T = C;
   for(auto y = 0u; y < ny; ++y)
      for(auto x = 0u; x < nx; ++x) T[y * nx + x] = C[y * nx + nx - x - 1];
   C = T;
}

void flip_rows(const unsigned nx, const unsigned ny, vector<Vector3r>& C)
{
   flip_rowsP(nx, ny, C);
}
void flip_cols(const unsigned nx, const unsigned ny, vector<Vector3r>& C)
{
   flip_colsP(nx, ny, C);
}
void flip_rows(const unsigned nx, const unsigned ny, vector<Vector2>& C)
{
   flip_rowsP(nx, ny, C);
}
void flip_cols(const unsigned nx, const unsigned ny, vector<Vector2>& C)
{
   flip_colsP(nx, ny, C);
}

// ---------------------------------------------------------------- Line indices

void calc_line_inds(unsigned nx,
                    unsigned ny,
                    unsigned line_id, // [0..nx+ny]
                    vector<unsigned>& line_inds)
{
   unsigned L = nx + ny;
   assert(line_id < L);
   const bool is_row = line_id < ny;
   if(is_row) {
      line_inds.resize(nx);
      for(unsigned i = 0; i < nx; ++i) line_inds[i] = line_id * nx + i;
   } else {
      line_inds.resize(ny);
      for(unsigned i = 0; i < ny; ++i) line_inds[i] = (line_id - ny) + i * nx;
   }
}

// -------------------------------------------------- Estimate Distortion Center

Vector2 estimate_distortion_center(const vector<vector<Vector3r>>& img_pts,
                                   const vector<Vector3r>& W,
                                   bool feedback)
{
   Vector3 e;

   const unsigned M   = unsigned(img_pts.size()); // number of images
   const unsigned N   = unsigned(W.size());
   const double M_inv = 1.0 / M;

   std::vector<Matrix3r> Fs(M);
   std::vector<Vector3r> Es(M);    // Thin vectors of Fs
   std::vector<real> thin_size(M); // Eigenvalue of 'Es'

   Vector3r mean_e(0.0, 0.0, 0.0);

   if(M == 0) {
      LOG_ERR(format("No images!"));
      return Vector2::nan();
   }

   if(feedback) {
      INFO(format("Feedback on find distortion-center over {} images", M));
   }

   // We get a different F matrix for every image
   for(unsigned ind = 0; ind < M; ++ind) {
      const auto& Xs = img_pts[ind];
      if(Xs.size() != N) {
         LOG_ERR(format("Image #{} had {} points, but world-points has {}",
                        ind,
                        Xs.size(),
                        N));
         return Vector2::nan();
      }

      // Swapped from Harley and Kang,
      // So the distortion centre is now the RIGHT epipole
      Matrix3r F = estimate_F(Xs, W);
      // Scale F so that the last row is identical everywhere
      Fs[ind]        = F;
      thin_size[ind] = svd_thin(F, Es[ind]);
      mean_e += Es[ind] * (M_inv); // / Es[ind](2));

      if(false && feedback) {
         Vector3r e = Es[ind] / Es[ind](2);
         auto err   = xFx(F, Xs, W);
         auto err2  = xFl_lFx(F, Xs, W);
         cout << format("   #{:4d}   e = { {}, {} }     err = {}, {}, {}",
                        ind,
                        e(0),
                        e(1),
                        thin_size[ind],
                        err,
                        err2)
              << endl;
      }

      for(auto F : Fs) F /= mean_e(2);
   }

   auto cost_fun = [&](const double* X) -> double {
      Vector3r e(X[0], X[1], X[2]);
      auto err = 0.0;
      for(const auto& F : Fs) err += (F * e).norm(); // right epipole
      return err / double(Fs.size());
   };

   // mean_e = Vector3r(0.0, 0.0, 1.0);
   double start[3] = {mean_e[0], mean_e[1], mean_e[2]};
   double xmin[3]  = {0.0, 0.0, 0.0};
   double reqmin   = 1e-12;
   double diffstep = 1e-1;
   int kcount      = 100000; // max interations
   int icount = 0, ifault = 0;

   levenberg_marquardt(
       cost_fun, 3, start, xmin, reqmin, diffstep, 10, kcount, icount, ifault);

   Vector2 ret(xmin[0] / xmin[2], xmin[1] / xmin[2]);

   if(feedback) {
      double ynewlo = cost_fun(xmin);
      cout << format("Levenberg-marquardt, {} iterations", icount) << endl;
      cout << format("  exit-condition: {} = {}",
                     ifault,
                     levenberg_marquardt_fault_str(ifault))
           << endl;
      cout << format("  start         : ({}, {})",
                     start[0] / start[2],
                     start[1] / start[2])
           << endl;
      cout << format("  ynewlo        : {}", cost_fun(xmin)) << endl;
      cout << format("  xmin          : {}", str(ret)) << endl;
   }

   if(false) {
      // Brute force search to test the stability of the above result
      auto best_score = std::numeric_limits<double>::max();
      for(int row = 0; row < 2000; ++row) {
         for(int col = 0; col < 2000; ++col) {
            double XX[3]
                = {double(col) * xmin[2], double(row) * xmin[2], xmin[2]};
            auto score = cost_fun(XX);
            if(score < best_score) {
               best_score = score;
               cout << format(
                   "^ Best-score = {}   ({}, {})", best_score, col, row)
                    << endl;
               memcpy(xmin, XX, sizeof(double) * 3);
            }
         }
      }

      ret = Vector2(xmin[0] / xmin[2], xmin[1] / xmin[2]);

      if(feedback) {
         cout << format("  brute-force   : {}", cost_fun(xmin)) << endl;
         cout << format("  xmin          : {}", str(ret)) << endl;
      }
   }

   return ret;
}

// --------------------------------------------------------- Projection Error --
// w = f(x)
real projection_err(std::function<Vector2(const Vector2& x)> f,
                    const vector<vector<Vector3r>>& Ws,
                    const vector<vector<Vector3r>>& Xs)
{
   Expects(Xs.size() >= Ws.size());

   real err = 0.0;

   const unsigned N  = unsigned(Ws.size());
   const unsigned M  = unsigned((N > 0) ? Ws[0].size() : 0);
   const real NM_inv = 1.0 / real(N * M);

   for(unsigned n = 0; n < N; ++n) {
      const auto& Xsn = Xs[n];
      const auto& Wsn = Ws[n];
      for(unsigned m = 0; m < M; ++m) {
         const auto& w = Wsn[m];
         const auto& x = Xsn[m];
         const auto y  = f(Vector2(x(0), x(1)));
         err += sqrt(square(y(0) - w(0)) + square(y(1) - w(1))) * NM_inv;
      }
   }

   return err;
}

real average_R2_err(const vector<vector<Vector3r>>& Ws,
                    const vector<vector<Vector3r>>& Us)
{
   Expects(Us.size() >= Ws.size());

   real err = 0.0;

   const unsigned N  = unsigned(Ws.size());
   const unsigned M  = unsigned((N > 0) ? Ws[0].size() : 0);
   const real NM_inv = 1.0 / real(N * M);

   for(unsigned n = 0; n < N; ++n) {
      const auto& Usn = Us[n];
      const auto& Wsn = Ws[n];
      for(unsigned m = 0; m < M; ++m) {
         const auto& w = Wsn[m];
         const auto& u = Usn[m];
         err += sqrt(square(u(0) - w(0)) + square(u(1) - w(1))) * NM_inv;
      }
   }

   return err;
}

// ------------------------------------------------------------ r1r2ts <=> Hs --

void Hs_to_r1r2ts(const vector<Matrix3r>& Hs, vector<Vector6r>& r1r2ts)
{
   const unsigned N = unsigned(Hs.size());
   if(r1r2ts.size() != N) r1r2ts.resize(N);
   for(unsigned n = 0; n < N; ++n) H_to_r1r2t(Hs[n], r1r2ts[n]);
}

void r1r2ts_to_Hs(const vector<Vector6r>& r1r2ts, vector<Matrix3r>& Hs)
{
   const unsigned N = unsigned(r1r2ts.size());
   if(Hs.size() != N) Hs.resize(N);
   for(unsigned n = 0; n < N; ++n) r1r2t_to_H(r1r2ts[n], Hs[n]);
}

// ------------------------------------------------------------------- r1r2ts --

static real refine_r1r2t(const vector<Vector3r>& W,  // Grid
                         const vector<Vector3r>& Us, // Undistorted points
                         Vector3& ssa_inout,
                         Vector3r& t_inout,
                         bool use_nelder_mead,
                         bool feedback)
{
   const unsigned n_params = 6;
   const unsigned M        = unsigned(W.size());
   const real M_inv        = 1.0 / M;
   Matrix3r H;
   vector<Vector3r> Ws(W.size());
   Vector3 ssa = ssa_inout;
   Vector3r t  = t_inout;

   auto pack = [&](real* X) {
      unsigned pos = 0;
      X[pos++]     = ssa(0);
      X[pos++]     = ssa(1);
      X[pos++]     = ssa(2);
      X[pos++]     = t(0);
      X[pos++]     = t(1);
      X[pos++]     = t(2);
   };

   auto unpack = [&](const real* X) {
      unsigned pos = 0;
      ssa(0)       = X[pos++];
      ssa(1)       = X[pos++];
      ssa(2)       = X[pos++];
      t(0)         = X[pos++];
      t(1)         = X[pos++];
      t(2)         = X[pos++];
      r1r2t_to_H(ssa, t, H);
      for(unsigned m = 0; m < M; ++m) {
         Ws[m] = H * W[m];
         Ws[m] /= Ws[m](2);
      }
   };

   real best_score = std::numeric_limits<real>::max();
   unsigned counter{0};
   auto fn = [&](const real* X) -> real {
      unpack(X);
      real err_sq = 0.0;
      for(unsigned m = 0; m < M; ++m) {
         const auto w = Ws[m];
         const auto u = Us[m];
         err_sq += square(w(0) - u(0)) + square(w(1) - u(1));
      }
      return sqrt(err_sq) * M_inv;
   };

   real start[n_params];
   real xmin[n_params];
   real step[n_params];
   real ynewlo   = dNAN;
   real reqmin   = 1e-12;
   real diffstep = 0.1;
   int kcount    = 1000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method = nullptr;

   pack(start);
   real ystartlo = fn(&start[0]);

   if(!use_nelder_mead) {
      method = "levenberg-marquardt";
      levenberg_marquardt(fn,
                          n_params,
                          start,
                          xmin,
                          reqmin,
                          diffstep,
                          5,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);
   } else {
      method = "nelder-mead";
      nelder_mead(fn,
                  n_params,
                  start,
                  xmin,
                  ynewlo,
                  reqmin,
                  step,
                  10,
                  kcount,
                  icount,
                  numres,
                  ifault);
   }

   unpack(xmin);

   { // Do we need to flip 'H' to be in front of the camera?
      Vector3r w(0, 0, 1);
      Vector3r x = H * w;
      if(x(2) < 0.0) H *= -1.0;
      pack(xmin);
      unpack(xmin);
   }

   if(feedback) {
      INFO(format("Feedback for estimating r1, r2, and t"));
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;
      cout << "H = " << endl << H << endl << endl;
   }

   if(ynewlo < ystartlo) {
      t_inout   = t;
      ssa_inout = ssa;
   }

   return std::min(ynewlo, ystartlo); // we cannot get worse
}

static real estimate_r1r2t(const vector<Vector3r>& W,
                           const vector<Vector3r>& U, // undistorted
                           Vector3& ssa,
                           Vector3r& t,
                           bool feedback = false)
{
   Matrix3r H;
   auto errH = estimate_homography_LLS(W, U, H);
   if(feedback) INFO(format("Estimate r1r2t, H-err = {}", errH));
   Vector3 ssa1 = ssa;
   Vector3r t1  = t;
   H_to_r1r2t(H, ssa, t);
   real err = 0.0;
   err      = refine_r1r2t(W, U, ssa, t, true, feedback);
   err      = refine_r1r2t(W, U, ssa, t, false, feedback);
   return errH;
}

real estimate_r1r2t(const vector<Vector3r>& W,
                    const vector<Vector3r>& U,
                    Vector6r& X,
                    const bool feedback = false)
{
   auto ret = 0.0;
   Vector3 ssa;
   Vector3r t;
   for(unsigned i = 0; i < 3; ++i) ssa(int(i)) = X(i);
   for(unsigned i = 0; i < 3; ++i) t(i) = X(i + 3);
   ret = estimate_r1r2t(W, U, ssa, t, feedback);
   for(unsigned i = 0; i < 3; ++i) X(i) = ssa(int(i));
   for(unsigned i = 0; i < 3; ++i) X(i + 3) = t(i);
   return ret;
}

real estimate_r1r2ts(const vector<Vector3r>& W,
                     const vector<vector<Vector3r>>& Us,
                     vector<Vector6r>& r1r2ts,
                     const bool use_threads,
                     const bool feedback)
{
   const unsigned N         = unsigned(Us.size());
   const unsigned n_threads = hardware_concurrency() * 2;
   const real N_inv         = 1.0 / N;
   if(r1r2ts.size() != N) r1r2ts.resize(N);

   real rp_err = 0.0;
   vector<real> rp_errs;
   rp_errs.resize(n_threads);
   std::fill(&rp_errs[0], &rp_errs[0] + n_threads, 0.0);

   auto est_r1r2t = [&](unsigned n) {
      auto& r1r2t = r1r2ts[n];

      // Estimate twice with nelder mead (which has random restarts
      // to escape local minima)
      auto err = estimate_r1r2t(W, Us[n], r1r2t, feedback);
      if(feedback && false) cout << format("#{:4d} :: {}", n, err) << endl;
      return err;
   };

   std::atomic<unsigned> job_counter{0};
   auto run_r1r2t = [&](unsigned offset) {
      rp_errs[offset] = 0.0;
      for(unsigned n = offset; n < N; n += n_threads)
         rp_errs[offset] += N_inv * est_r1r2t(n);
      job_counter++;
   };

   if(use_threads) {
      ParallelJobSet jobs;
      for(unsigned offset = 0; offset < n_threads; ++offset)
         jobs.schedule([offset, &run_r1r2t]() { run_r1r2t(offset); });
      jobs.execute();

      rp_err = 0.0;
      for(unsigned i = 0; i < n_threads; ++i) rp_err += rp_errs[i];
   } else {
      rp_err = 0.0;
      for(unsigned n = 0; n < N; ++n) rp_err += N_inv * est_r1r2t(n);
   }

   return rp_err;
}

// ----------------------------------------------------------------- calc r1r2ts

static vector<Vector6r>
calc_r1r2ts(const vector<Vector3r>& W,
            const vector<vector<Vector2>>& Ds,
            std::function<Vector2(const Vector2&)> undistort)
{
   const auto N = Ds.size();
   vector<Vector6r> r1r2ts;
   vector<vector<Vector3r>> Us(N);
   for(unsigned n = 0u; n < N; ++n) {
      std::transform(Ds[n].begin(),
                     Ds[n].end(),
                     std::back_inserter(Us[n]),
                     [&](const Vector2& D) -> Vector3r {
                        auto u = undistort(D);
                        return Vector3r(u(0), u(1), 1.0);
                     });
   }

   estimate_r1r2ts(W, Us, r1r2ts, true, false);
   return r1r2ts;
}

vector<Vector6r> calc_r1r2ts(const vector<Vector3r>& W,
                             const vector<vector<Vector3r>>& Ds,
                             std::function<Vector2(const Vector2&)> undistort)
{
   vector<vector<Vector2>> D2s(Ds.size());
   for(auto n = 0u; n < Ds.size(); ++n) {
      std::transform(
          Ds[n].begin(),
          Ds[n].end(),
          std::back_inserter(D2s[n]),
          [&](Vector3r X) { return Vector2(X(0) / X(2), X(1) / X(2)); });
   }
   return calc_r1r2ts(W, D2s, undistort);
}

// ----------------------------------------------------------- make-W-pattern --

vector<Vector3r> make_W_pattern(unsigned nx, unsigned ny)
{
   vector<Vector3r> W(nx * ny);
   for(auto y = 0u; y < ny; ++y)
      for(auto x = 0u; x < nx; ++x) W[x + y * nx] = Vector3r(y, x, 1.0);
   return W;
}

} // namespace perceive::calibration
