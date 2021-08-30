
#include "perceive/foundation.hpp"
#include "perceive/geometry/projective/camera.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/eigen-helpers.hpp"

#include "find-F.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define This FindFCostFunctor

using std::cout;
using std::endl;
using std::string;
using std::vector;

namespace perceive::calibration
{
// ------------------------------------------------------------------- To String

string This::to_string() const
{
   std::stringstream ss("");

   auto process = [&](const char* label, const decltype(pts1)& X) -> void {
      ss << label << format(".size() == {}", X.size()) << endl;
      ss << "   first 3 points: ";
      for(unsigned i = 0; i < X.size() && i < 3; ++i) {
         const auto& x = X[i];
         if(i > 0) ss << ", ";
         ss << format("[{}, {}]", x(0), x(1));
      }
      ss << endl;
      ss << "    last 3 points: ";
      unsigned i0 = unsigned(X.size() >= 3 ? X.size() - 3 : 0);
      for(unsigned i = i0; i < X.size(); ++i) {
         const auto& x = X[i];
         if(i != i0) ss << ", ";
         ss << format("[{}, {}]", x(0), x(1));
      }
      ss << endl;
   };

   ss << "Stereo Parameter Search Functor:" << endl << endl;
   ss << "K1 = " << endl << K1_ << endl << endl;
   ss << "K2 = " << endl << K2_ << endl << endl;
   process("Pts1", pts1);
   process("Pts2", pts2);
   ss << endl;

   return ss.str();
}

// -------------------------------------------------------------------- Set K1/2

void This::set_K1(const Matrix3r& K)
{
   K1_     = K;
   K1_inv_ = K1_.inverse();
}

void This::set_K2(const Matrix3r& K)
{
   K2_          = K;
   Matrix3r K2t = K2_.transpose();
   K2t_inv_     = K2t.inverse();
}

// -------------------------------------------------------------------- Evaluate

static Matrix3r make_E(const Matrix3r& R, const Vector3& t)
{
   Matrix3r Tx = Matrix3r::Zero();
   Tx(0, 1)    = -t(2);
   Tx(0, 2)    = t(1);
   Tx(1, 2)    = -t(0);
   Tx(1, 0)    = t(2);
   Tx(2, 0)    = -t(1);
   Tx(2, 1)    = t(0);
   Matrix3r E  = Tx * R;
   return E;
}

// Evaluate the cost function for the passed set of 5 parameters
// The first three are a rotation in spherical-axis-angle format
// The last two are the inclincation and azimuth of "t"
real This::evaluate(const real* X) const
{
   if(pts1.size() != pts2.size()) {
      LOG_ERR(format("Refusing to evaluate cost function, "
                     "because |pts1| != |pts2| (i.e., {} != {})",
                     pts1.size(),
                     pts2.size()));
      return dNAN;
   }

   const unsigned N = unsigned(pts1.size());
   if(N == 0) return 0.0;

   Matrix3r R;
   Vector3 t;
   unpack(X, R, t);
   Matrix3r E  = make_E(R, t);
   Matrix3r F  = K2t_inv_ * E * K1_inv_;
   Matrix3r Ft = F.transpose();

   double err = 0.0;
   {
      const double N_inv = 1.0 / double(N);

      for(unsigned i = 0; i < N; ++i) {
         // Points of interest
         const auto& x1 = pts1[i];
         const auto& x2 = pts2[i];

         // Two epipolar lines
         Vector3r l1 = Ft * x2;
         Vector3r l2 = F * x1;

         // Normalizing the lines means that error is in pixels
         l1 /= sqrt(square(l1(0)) + square(l1(1)));
         l2 /= sqrt(square(l2(0)) + square(l2(1)));

         auto err1 = fabs(l1.dot(x1));
         auto err2 = fabs(l2.dot(x2));

         // Add the average pixel distance from (x1=>l1) and (x2=>l2)
         err += N_inv * 0.5 * (err1 + err2);
      }
   }

   return err;
}

// ---------------------------------------------------------------------- Unpack

// Unpack "X" into a rotation matrix and "t"
void This::unpack(const real* X, Matrix3r& R, Vector3& t) const
{
   Quaternion q;
   q.from_spherical_axis_angle(Vector3(X[0], X[1], X[2]));
   quaternion_to_Matrix3r(q, R);
   t = spherical_to_cartesian(Vector3(X[3], X[4], 1.0));
}

// ----------------------------------------------------------- Estimate Baseline

real This::estimate_baseline(unsigned nx,
                             unsigned ny,
                             real square_size,
                             const Matrix3r& R,
                             const Vector3& t) const
{
   if(pts1.size() != pts2.size()) {
      LOG_ERR(format("Refusing to estimate basline, "
                     "because |pts1| != |pts2| (i.e., {} != {})",
                     pts1.size(),
                     pts2.size()));
      return 0.0;
   }

   const unsigned sz = nx * ny;
   if(sz <= 0) return 0.0;

   const unsigned N = unsigned(pts1.size() / sz);
   if(N == 0) return 0.0;

   if(pts1.size() % sz != 0) {
      LOG_ERR(format("Refusing to estimate baseline on {} points, because we "
                     "expect {}x{}={} points per image, but {} %% {} = {}.",
                     pts1.size(),
                     nx,
                     ny,
                     sz,
                     pts1.size(),
                     sz,
                     (pts1.size() % sz)));
      return 0.0;
   }

   // Set up the two cameras, noting that format doesn't matter
   Camera P1, P2;
   P1.set_intrinsic(
       P1.w(), P1.h(), K1_(0, 0), K1_(1, 1), Vector2(K1_(0, 2), K1_(1, 2)));
   P2.set_intrinsic(
       P1.w(), P1.h(), K2_(0, 0), K2_(1, 1), Vector2(K2_(0, 2), K2_(1, 2)));
   P2.set_extrinsic(R, t);

   // Returns the ray-intersected 3D point given the passed baseline
   auto intersect = [&](const Vector3& C2b, unsigned pts_ind) -> Vector3 {
      auto ray1 = to_vec3(P1.back_project(pts1[pts_ind]));
      auto ray2 = to_vec3(P1.back_project(pts2[pts_ind]));
      return intersect_rays(Vector3(0, 0, 0), ray1, C2b, C2b + ray2);
   };

   // Gives the average measured length of all the boards
   auto measured_diag_length = [&](real baseline) -> real {
      Vector3 C2b  = P2.C().normalized() * baseline;
      double len   = 0.0;
      double N_inv = 1.0 / double(N);
      for(unsigned image_ind = 0; image_ind < N; ++image_ind) {
         auto X00 = intersect(C2b, image_ind * sz + 0);
         auto Xx0 = intersect(C2b, image_ind * sz + (nx - 1));
         auto X0y = intersect(C2b, image_ind * sz + (ny - 1) * nx);
         auto Xxy = intersect(C2b, image_ind * sz + (sz - 1));
         auto d1  = (X00 - Xxy).norm();
         auto d2  = (Xx0 - X0y).norm();
         // cout << format("d1 = {}", d1) << endl;
         // cout << format("d2 = {}", d2) << endl;
         len += N_inv * 0.5 * d1;
         len += N_inv * 0.5 * d2;
      }
      return real(len);
   };

   // So what is that diagonal length?
   Vector2 D(nx - 1, ny - 1);
   D *= square_size;
   const real pattern_diagonal_length = D.norm();

   // Okay, what do we have?
   const real diag0 = measured_diag_length(1.0);

   const real baseline1 = pattern_diagonal_length / diag0;
   const real diag1     = measured_diag_length(baseline1);

   // cout << format("diag = {}", pattern_diagonal_length) << endl;

   auto err = fabs(pattern_diagonal_length - diag1);
   if(err > 1e-3) {
      // Mmm... probably not good
      WARN(format("diag-length was {}, but measured length as {}, with "
                  "|{} - {}| = {}",
                  pattern_diagonal_length,
                  diag1,
                  pattern_diagonal_length,
                  diag1,
                  err));
   }

   return baseline1;
}

// ----------------------------------------------------------------- Condition F

Matrix3r condition_F(const Matrix3r& F)
{
   Matrix3r U, D, V;
   Vector3r ss;
   svd_UDV(F, U, ss, V);
   D       = Matrix3r::Zero();
   D(0, 0) = 1.0;
   D(1, 1) = ss(1) / ss(0);
   return U * D * V.transpose();
}

Matrix3r condition_E(const Matrix3r& E)
{
   Matrix3r U, D, V;
   Vector3r ss;
   svd_UDV(E, U, ss, V);
   D       = Matrix3r::Zero();
   D(0, 0) = D(1, 1) = 1.0;
   return U * D * V.transpose();
}

// ------------------------------------------------------------- Longuet Higgins

Matrix3r longuet_higgins(const std::vector<Vector3r>& pts1,
                         const std::vector<Vector3r>& pts2)
{
   if(pts1.size() != pts2.size()) {
      LOG_ERR(format("Refusing to run Longuet-Higgins, "
                     "because |pts1| != |pts2| (i.e., {} != {})",
                     pts1.size(),
                     pts2.size()));
      return Matrix3r::Zero();
   }

   const unsigned N = unsigned(pts1.size());
   if(N == 0) return Matrix3r::Zero();

   MatrixXr A(N, 9);
   for(uint row = 0; row < N; ++row) {
      const Vector3r& xx = pts1[row];
      const Vector3r& yy = pts2[row];     // yy^t F xx
      A(row, 0)          = yy(0) * xx(0); // x'x
      A(row, 1)          = yy(0) * xx(1); // x'y
      A(row, 2)          = yy(0) * 1.0;   // x'1
      A(row, 3)          = yy(1) * xx(0); // y'x
      A(row, 4)          = yy(1) * xx(1); // y'y
      A(row, 5)          = yy(1) * 1.0;   // y'1
      A(row, 6)          = 1.0 * xx(0);   // 1 x
      A(row, 7)          = 1.0 * xx(1);   // 1 y
      A(row, 8)          = 1.0 * 1.0;     // 1 1
   }

   // We have Af = 0, use linear-least-squares
   MatrixXr At  = A.transpose();
   MatrixXr AtA = At * A;
   VectorXr V;
   svd_thin(AtA, V);

   Matrix3r F;
   for(uint i = 0; i < 9; ++i) F(i / 3, i % 3) = V(i);

   F = condition_F(F);

   return F;
}

// ------------------------------------------------------------------ Estimate F

Matrix3r estimate_F(const std::vector<Vector3r>& pts1,
                    const std::vector<Vector3r>& pts2)
{
   std::vector<Vector3r> normalized_pts1, normalized_pts2;

   auto get_normalization_matrix = [](const std::vector<Vector3r>& Xs,
                                      std::vector<Vector3r>& Ns) -> Matrix3r {
      Vector2 C(0.0, 0.0);
      for(const auto& X : Xs) C += Vector2(X(0), X(1));
      C /= real(Xs.size()); // C is the centroid

      double mean_dist = 0.0;
      for(const auto& X : Xs) mean_dist += (Vector2(X(0), X(1)) - C).norm();
      mean_dist /= double(Xs.size());

      double k = sqrt(2.0) / mean_dist;

      Matrix3r N;
      N << k, 0, -C.x * k, 0., k, -C.y * k, 0., 0, 1;

      Ns.resize(Xs.size());
      for(unsigned i = 0; i < Xs.size(); ++i) Ns[i] = N * Xs[i];

      return N;
   };

   Matrix3r norm_mat1 = get_normalization_matrix(pts1, normalized_pts1);
   Matrix3r norm_mat2 = get_normalization_matrix(pts2, normalized_pts2);

   Matrix3r F;
   Matrix3r F0 = longuet_higgins(normalized_pts1, normalized_pts2);

   {
      Matrix3r T;

      auto fn = [&](const real* X) -> real {
         T << X[0], X[1], X[2], X[3], X[4], X[5], X[6], X[7], X[8];
         T = condition_F(T);
         return xFx(T, normalized_pts1, normalized_pts2);
      };

      real start[9]   = {F0(0, 0),
                       F0(0, 1),
                       F0(0, 2),
                       F0(1, 0),
                       F0(1, 1),
                       F0(1, 2),
                       F0(2, 0),
                       F0(2, 1),
                       F0(2, 2)};
      real xmin[9]    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      double reqmin   = 1e-6;
      double diffstep = 1e-6;
      int kcount      = 100000; // max interations
      int icount = 0, ifault = 0;

      levenberg_marquardt(
          fn, 9, start, xmin, reqmin, diffstep, kcount, icount, ifault);

      T << xmin[0], xmin[1], xmin[2], xmin[3], xmin[4], xmin[5], xmin[6],
          xmin[7], xmin[8];

      F = condition_F(T);
   }

   return norm_mat2.transpose() * F * norm_mat1;
}

// ------------------------------------------------------------------ Estimate E

Matrix3r estimate_E(const std::vector<Vector3r>& pts1,
                    const std::vector<Vector3r>& pts2)
{
   Matrix3r E0 = estimate_F(pts1, pts2);
   Matrix3r T;

   auto fn = [&](const real* X) -> real {
      T << X[0], X[1], X[2], X[3], X[4], X[5], X[6], X[7], X[8];
      T = condition_E(T);
      return xFl_lFx(T, pts1, pts2);
   };

   real start[9]   = {E0(0, 0),
                    E0(0, 1),
                    E0(0, 2),
                    E0(1, 0),
                    E0(1, 1),
                    E0(1, 2),
                    E0(2, 0),
                    E0(2, 1),
                    E0(2, 2)};
   real xmin[9]    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   double reqmin   = 1e-6;
   double diffstep = 1e-6;
   int kcount      = 100000; // max interations
   int icount = 0, ifault = 0;

   levenberg_marquardt(
       fn, 9, start, xmin, reqmin, diffstep, kcount, icount, ifault);

   T << xmin[0], xmin[1], xmin[2], xmin[3], xmin[4], xmin[5], xmin[6], xmin[7],
       xmin[8];

   return condition_E(T);
}

// ----------------------------------------------------------------- Estimate Rt

void estimate_Rt_from_E(const Matrix3r& E0,
                        const std::vector<Vector3r>& pts0, // normalized points
                        const std::vector<Vector3r>& pts1,
                        Quaternion& q, // Out
                        Vector3& t,
                        const bool optimize,
                        const bool feedback) // out
{
   // Get Rt from E
   Matrix3r W;
   W << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

   Matrix3r U, _, V;
   Vector3r ss;
   svd_UDV(E0, U, ss, V);

   // The resulting rotation matrix must have determinant 1. (Not -1)
   // This step is necessary because of the sign ambiguity in the SVD.
   //
   if((U.determinant() > 0.0) != (V.determinant() > 0.0)) {
      U.block(0, 2, 3, 1) *= -1.0;
   }

   Matrix3r R0 = U * W * V.transpose();
   Matrix3r R1 = U * W.transpose() * V.transpose();

   assert(fabs(R0.determinant() - 1) < 1e-6);
   assert(fabs(R1.determinant() - 1) < 1e-6);

   Vector3r t0 = U.block(0, 2, 3, 1);

   // There are 4 solutions, but only 1 has recovered points
   // in front of both camers
   array<Matrix3r, 4> Rs{{R0, R0, R1, R1}};
   array<Vector3r, 4> ts{{t0, -t0, t0, -t0}};
   array<unsigned, 4> counts{{0, 0, 0, 0}};
   array<Camera, 4> P0s;
   array<Camera, 4> P1s;
   for(unsigned i = 0; i < 4; ++i) {
      P0s[i].set_intrinsic(10.0, 10.0, 1.0, 1.0, Vector2(0.0, 0.0));
      P0s[i].set_extrinsic(Matrix3r::Identity(), Vector3(0.0, 0.0, 0.0));
      P1s[i].set_intrinsic(10.0, 10.0, 1.0, 1.0, Vector2(0.0, 0.0));
      P1s[i].set_extrinsic(Rs[i], to_vec3(ts[i]));
   }

   auto count_in_front = [&](const Camera& P0, const Camera& P1) {
      int counter = 0;
      for(unsigned n = 0; n < pts1.size(); ++n) {
         const auto& x0 = pts0[0];
         const auto& x1 = pts1[1];
         auto r0        = to_vec3(P0.back_project(x0));
         auto r1        = to_vec3(P1.back_project(x1));
         auto X = intersect_rays(P0.C(), P0.C() + r0, P1.C(), P1.C() + r1);
         if(P0.in_front(X) && P1.in_front(X)) counter++;
      }
      return counter;
   };

   for(size_t i = 0; i < 4; ++i) {
      counts[i] = unsigned(count_in_front(P0s[i], P1s[i]));
   }

   if(feedback) {
      cout << "Printing counts for 4 solutions. Ideally 3 values are zero."
           << endl;
      for(unsigned i = 0; i < 4; ++i) {
         Vector4 aa = rot3x3_to_axis_angle(Rs[i]);
         Vector3 C  = -to_vec3(Rs[i] * ts[i]).normalized();
         cout << format("#{}, rot-axis = {}, rot-theta = {}, "
                        "C = {} :: count = {}",
                        i,
                        aa.xyz().to_string(),
                        to_degrees(aa.w),
                        C.to_string(),
                        counts[i])
              << endl;
      }
      cout << endl;
   }

   // What was the best counter?
   unsigned best_i = 0;
   for(unsigned i = 1; i < 4; ++i)
      if(counts[i] > counts[best_i]) best_i = i;

   q = rot3x3_to_quaternion(Rs[best_i]);
   t = to_vec3(ts[best_i]);

   auto cost_fn = [&](const Quaternion& q, const Vector3& t) {
      Matrix3r E = make_essential_matrix(q, t);
      return 400.0 * xFl_lFx(E, pts0, pts1);
   };

   if(optimize) {
      const auto q0     = q;
      const auto t0     = t;
      const Matrix3r E0 = make_essential_matrix(q0, t0);
      Camera P0, P1;
      P0.set_intrinsic(10.0, 10.0, 1.0, 1.0, Vector2(0.0, 0.0));
      P0.set_extrinsic(Matrix3r::Identity(), Vector3(0.0, 0.0, 0.0));
      P1.set_intrinsic(10.0, 10.0, 1.0, 1.0, Vector2(0.0, 0.0));

      auto pack = [&](real x[5]) {
         Vector3 saa = quaternion_to_saa(q);
         Vector3 sph = cartesian_to_spherical(t);
         int pos     = 0;
         for(auto i = 0; i < 3; ++i) x[pos++] = saa[i];
         for(auto i = 0; i < 2; ++i) x[pos++] = t[i];
      };

      auto unpack = [&](const real x[5]) {
         Vector3 saa, sph;
         int pos = 0;
         for(auto i = 0; i < 3; ++i) saa[i] = x[pos++];
         for(auto i = 0; i < 2; ++i) sph[i] = x[pos++];
         sph.z      = 1.0;
         t          = spherical_to_cartesian(sph);
         q          = saa_to_quaternion(saa);
         Matrix3r R = quaternion_to_rot3x3(q);
         P1.set_extrinsic(R, t);
      };

      auto counter    = 0;
      auto best_score = std::numeric_limits<real>::max();
      auto best_q     = q0;
      auto best_t     = t0;
      auto fn         = [&](const real* x) -> real {
         unpack(x);
         auto score = cost_fn(q, t);

         // count what's in front of the camera
         const auto n_in_front = count_in_front(P0, P1);
         if(n_in_front != int(pts0.size()))
            score = std::numeric_limits<real>::max();

         if(score < best_score) {
            best_score = score;
            best_q     = q;
            best_t     = t;
            if(feedback)
               cout << format("#{:6d},  score = {}", counter, score) << endl;
         }
         ++counter;
         return score;
      };

      constexpr auto use_nelder_mead = true;
      constexpr auto n_params        = 5;
      array<real, n_params> start, xmin, delta;
      pack(&start[0]);
      std::fill(begin(xmin), end(xmin), 0.0);
      std::fill(begin(delta), end(delta), to_radians(1.0));

      const auto reqmin       = 1e-2;
      const auto konvge       = 10;   // convergence check
      const auto kcount       = 1000; // max iterations
      const auto max_restarts = -1;
      int icount = 0, numres = 0, ifault = 0;
      real ynewlo = 0.0;

      const auto diffstep       = 1e-1;
      const auto diffstep_steps = 4;

      if(use_nelder_mead) {
         nelder_mead(fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &delta[0],
                     konvge,
                     kcount,
                     max_restarts,
                     icount,
                     numres,
                     ifault);
      } else {
         levenberg_marquardt(fn,
                             n_params,
                             &start[0],
                             &xmin[0],
                             reqmin,
                             diffstep,
                             diffstep_steps,
                             kcount,
                             icount,
                             ifault);
      }

      if(feedback) {
         const auto ystartlo = cost_fn(q0, t0);
         const auto ynewlo   = cost_fn(best_q, best_t);
         const auto method
             = use_nelder_mead ? "nelder-mead" : "levenberg-marquardt";
         cout << format(R"V0G0N(
Optimizing Rt, using {}
 + ystartlo = {}
 + ynewlo = {}
 + q = {}
 + t = {}
)V0G0N",
                        method,
                        ystartlo,
                        ynewlo,
                        str(best_q),
                        str(best_t));
      }

      q = best_q;
      t = best_t;
   }

   if(feedback) {
      const auto err = cost_fn(q, t);
      // Matrix3r tx = make_cross_product_matrix(t);
      // Matrix3r E  = tx * Rs[best_i];
      // auto err    = 400.0 * xFl_lFx(E, pts0, pts1);
      // cout << str("E", E) << endl;
      cout << format("err (normalized) = {}", err) << endl << endl;
   }
}

// ------------------------------------------------------- make essential matrix

Matrix3r make_essential_matrix(const Quaternion& q, const Vector3& t)
{
   Matrix3r R  = quaternion_to_rot3x3(q);
   Matrix3r tx = make_cross_product_matrix(t);
   Matrix3r E  = tx * R;
   return E;
}

// ----------------------------------------------------------------- Estimate Rt

void estimate_Rt(const std::vector<Vector3r>& pts0, // normalized points
                 const std::vector<Vector3r>& pts1,
                 Quaternion& q, // Out
                 Vector3& t,
                 bool feedback) // out
{
   if(feedback) {
      INFO("Estimate Rt");
      cout << endl;
   }

   Matrix3r E0 = estimate_E(pts0, pts1);
   estimate_Rt_from_E(E0, pts0, pts1, q, t, feedback);
}

// ------------------------------------------------------------------------- xFx

real xFx(const Matrix3r& F,
         const std::vector<Vector3r>& pts1,
         const std::vector<Vector3r>& pts2)
{
   double err         = 0.0;
   const unsigned N   = unsigned(pts1.size());
   const double N_inv = 1.0 / double(N);

   for(unsigned i = 0; i < N; ++i)
      err += N_inv * fabs(pts2[i].transpose() * F * pts1[i]);

   return err;
}

real xFl_lFx(const Matrix3r& F, const Vector3r& x1, const Vector3r& x2)
{
   // Two epipolar lines
   Vector3r l1 = F.transpose() * x2;
   Vector3r l2 = F * x1;

   // Normalizing the lines means that error is in pixels
   l1 /= sqrt(square(l1(0)) + square(l1(1)));
   l2 /= sqrt(square(l2(0)) + square(l2(1)));

   auto err1 = fabs(l1.dot(x1));
   auto err2 = fabs(l2.dot(x2));

   // Add the average pixel distance from (x1=>l1) and (x2=>l2)
   return 0.5 * (err1 + err2);
}

real xFl_lFx(const Matrix3r& F, const Vector2& p1, const Vector2& p2)
{
   Matrix3r Ft = F.transpose();

   // Points of interest
   const auto x1 = Vector3r(p1.x, p1.y, 1.0);
   const auto x2 = Vector3r(p2.x, p2.y, 1.0);

   // Two epipolar lines
   Vector3r l1 = Ft * x2;
   Vector3r l2 = F * x1;

   // Normalizing the lines means that error is in pixels
   l1 /= sqrt(square(l1(0)) + square(l1(1)));
   l2 /= sqrt(square(l2(0)) + square(l2(1)));

   auto err1 = fabs(l1.dot(x1));
   auto err2 = fabs(l2.dot(x2));

   // Add the average pixel distance from (x1=>l1) and (x2=>l2)
   return 0.5 * (err1 + err2);
}

real xFl_lFx(const Matrix3r& F,
             const std::vector<Vector3r>& pts1,
             const std::vector<Vector3r>& pts2)
{
   const unsigned N = unsigned(pts1.size());
   double N_inv     = 1.0 / N;
   Matrix3r Ft      = F.transpose();
   double err       = 0.0;
   {
      const double N_inv = 1.0 / double(N);

      for(unsigned i = 0; i < N; ++i) {
         // Points of interest
         const auto& x1 = pts1[i];
         const auto& x2 = pts2[i];

         // Two epipolar lines
         Vector3r l1 = Ft * x2;
         Vector3r l2 = F * x1;

         // Normalizing the lines means that error is in pixels
         l1 /= sqrt(square(l1(0)) + square(l1(1)));
         l2 /= sqrt(square(l2(0)) + square(l2(1)));

         auto err1 = fabs(l1.dot(x1));
         auto err2 = fabs(l2.dot(x2));

         // Add the average pixel distance from (x1=>l1) and (x2=>l2)
         err += N_inv * 0.5 * (err1 + err2);
      }
   }
   return err;
}

real xFl(const Matrix3r& F,
         const std::vector<Vector3r>& pts1,
         const std::vector<Vector3r>& pts2)
{
   const unsigned N = unsigned(pts1.size());
   double N_inv     = 1.0 / N;
   double err       = 0.0;
   {
      const double N_inv = 1.0 / double(N);

      for(unsigned i = 0; i < N; ++i) {
         // Points of interest
         const auto& x1 = pts1[i];
         const auto& x2 = pts2[i];

         // Two epipolar lines
         Vector3r l2 = F * x1;
         l2 /= sqrt(square(l2(0)) + square(l2(1)));

         // Normalizing the lines means that error is in pixels
         auto err2 = fabs(l2.dot(x2));

         // Add the average pixel distance from (x1=>l1) and (x2=>l2)
         err += N_inv * err2;
      }
   }
   return err;
}

// ---------------------------------------------------------- Estimate Rt RANSAC

void estimate_Rt_RANSAC(const vector<Vector3r>& pts0, // should be normalized
                        const vector<Vector3r>& pts1,
                        vector<bool>& inliers, // returns inliers
                        double inlier_err,     // in normalized units
                        double prob,
                        Quaternion& q, // Out
                        Vector3& t,
                        bool feedback) // out

{
   uint32_t n_rows = uint32_t(pts0.size());
   cv::Mat lpts(int(n_rows), 2, CV_64F);
   cv::Mat rpts(int(n_rows), 2, CV_64F);
   for(unsigned r = 0; r < n_rows; ++r) {
      lpts.at<double>(int(r), 0) = pts0[r](0);
      lpts.at<double>(int(r), 1) = pts0[r](1);
      rpts.at<double>(int(r), 0) = pts1[r](2);
      rpts.at<double>(int(r), 1) = pts1[r](3);
   }

   cv::Mat status(lpts.rows, 1, CV_8U);
   cv::Mat cv_F       = findFundamentalMat(lpts,
                                     rpts,
                                     cv::FM_RANSAC, // cv::LMEDS
                                     inlier_err,
                                     prob,
                                     status);
   unsigned n_inliers = 0;
   inliers.resize(n_rows);
   for(size_t i = 0; i < size_t(status.rows); ++i) {
      inliers[i] = status.at<unsigned char>(int(i), 0) == 1;
      if(inliers[i]) n_inliers++;
   }

   std::vector<Vector3r> pp0, pp1;

   auto run_estimate_Rt = [&]() {
      pp0.clear();
      pp1.clear();
      pp0.reserve(n_inliers);
      pp1.reserve(n_inliers);
      for(unsigned i = 0; i < inliers.size(); ++i) {
         if(inliers[i]) {
            pp0.push_back(pts0[i]);
            pp1.push_back(pts1[i]);
         }
      }

      estimate_Rt(pp0, pp1, q, t, feedback);
   };

   run_estimate_Rt();

   // Now filter the points again
   Matrix3r tx = make_cross_product_matrix(t);
   Matrix3r R  = quaternion_to_rot3x3(q);
   Matrix3r E  = tx * R;

   n_inliers = 0;
   for(unsigned i = 0; i < inliers.size(); ++i) {
      auto err   = xFl_lFx(E, pts1[i], pts0[i]);
      inliers[i] = err < (inlier_err / 2);
      if(inliers[i]) n_inliers++;
   }

   run_estimate_Rt();
}

} // namespace perceive::calibration
