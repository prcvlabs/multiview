
#include "perceive/foundation.hpp"

#include "calibration-utils.hpp"
#include "find-K.hpp"
#include "find-homography.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/utils/eigen-helpers.hpp"

#define This FindKCostFunctor

using std::cout;
using std::endl;

namespace perceive::calibration
{
static const auto g_info         = "\x1b[37m\u261b\x1b[0m"s;
static const auto g_skull        = "\x1b[91m\u2620\x1b[0m"s;
static const auto g_radioactive  = "\x1b[91m\u2622\x1b[0m"s;
static const auto g_dotted_sq    = "\x1b[96m\u2b1a\x1b[0m"s;
static const auto g_bullet       = "\x1b[0m\u2738\x1b[0m"s;
static const auto g_cross_arrows = "\x1b[96m\u2928\x1b[0m"s;
static const auto g_arrow        = "\x1b[96m\u279D\x1b[0m"s;
static const auto g_waves        = "\x1b[96m\u29da\x1b[0m"s;
static const auto g_wave         = "\x1b[96m\u223c\x1b[0m"s;
static const auto g_lines        = "\x1b[96m\u2632\x1b[0m"s;
static const auto g_wedge        = "\x1b[0m\u2023\x1b[0m"s;
static const auto g_cross        = "\x1b[96m\u2613\x1b[0m"s;
static const auto g_victory      = "\x1b[40m\x1b[97m\u270c\x1b[0m"s;
static const auto g_coffee       = "\x1b[40m\x1b[97m\u26fe\x1b[0m"s;
static const auto g_tick         = "\x1b[40m\x1b[92m\u2714\x1b[0m"s;

static const auto g_default    = "\x1b[0m"s;
static const auto g_red        = "\x1b[31m"s;
static const auto g_error      = "\x1b[4m\x1b[91m"s;
static const auto g_light_gray = "\x1b[37m"s;
static const auto g_light_red  = "\x1b[91m"s;
static const auto g_white      = "\x1b[97m"s;

static const auto g_obnoxious = "\x1b[1m\x1b[4m\x1b[5m\x1b[106m\x1b[91m"s;

// ------------------------------------------------------------------- to-string

std::string This::to_string() const
{
   std::stringstream ss("");

   auto f = [](const Vector2r& v) -> std::string {
      return format("[{}, {}]", v(0), v(1));
   };

   ss << format(R"V0G0N(
Intrinsic Parameter Search Functor
   Norm method: {}
   single-f: {}
   Weights: [{}]

)V0G0N",
                str(method),
                str(single_f),
                implode(weights.begin(), weights.end(), ", "));

   // World points
   ss << "   W: [" << implode(W.begin(), W.end(), ", ", f) << "]" << endl
      << endl;

   // Homographies
   for(unsigned i = 0; i < Hs.size(); ++i) {
      std::vector<real> H(9);
      auto ptr = &H[0];
      for(unsigned r = 0; r < 3; ++r)
         for(unsigned c = 0; c < 3; ++c) *ptr++ = Hs[i](r, c);

      ss << format("   H#{:3d} = [", i) << implode(H.begin(), H.end(), ", ")
         << "]" << endl;
   }
   ss << endl;

   // Corners Sets
   for(unsigned i = 0; i < Cs.size(); ++i) {
      ss << format("   C#{:3d} = [", i)
         << implode(Cs[i].begin(), Cs[i].end(), ", ", f) << "]" << endl;
   }

   ss << endl << endl;

   return ss.str();
}

// -------------------------------------------------------------------- Evaluate

real This::evaluate(const real* X) const
{
   const bool feedback = false;

   double err = 0.0;

   // Unpack K
   Matrix3r K = Matrix3r::Identity();
   K(0, 2)    = X[0];
   K(1, 2)    = X[1];
   if(single_f) {
      K(0, 0) = K(1, 1) = X[2];
   } else {
      K(0, 0) = X[2];
      K(1, 1) = X[3];
   }

   // We need K_inv
   Matrix3r K_inv = K.inverse();

   // Now use K_inv on every homography to build updated homographies
   for(unsigned i = 0; i < Hs.size(); ++i) {
      const auto& H = Hs[i];
      Matrix3r M    = K_inv * H;

      // The interesting columns
      Vector3r r1 = M.col(0);
      Vector3r r2 = M.col(1);
      Vector3r t  = M.col(2);

      // Columns must be scaled
      auto norm1 = r1.norm(), norm2 = r2.norm();
      r1 /= r1.norm();
      r2 /= r2.norm();
      t /= (0.5 * (norm1 + norm2));

      // Set up rotation matrix -- conditioning it
      Matrix3r R, U, V;
      R.col(0) = r1;
      R.col(1) = r2;
      R.col(2) = r1.cross(r2);
      svd_UV(R, U, V);
      R = U * V.transpose(); // conditioned

      // Okay, this is a little check, to make sure we haven't
      // generated a reflection matrix because of the sign
      // ambiguity in the SVD.
      if(R.determinant() < 0) {
         LOG_ERR(format("kBAM! I knew it! |R| = {}", R.determinant()));

         // Okay, we can check this, and then multiply the last column of U
         // by -1
         U.col(2) *= -1.0;
         R = U * V.transpose();
         if(R.determinant() < 0) FATAL("Should never happen");
      }

      // Reconstrut the desired homography
      Matrix3r H1;
      H1.col(0) = K * R.col(0);
      H1.col(1) = K * R.col(1);
      H1.col(2) = K * t;

      // And calculate homography error
      auto H_err
          = FindHomographyCostFunctor::homography_err(H1, W, Cs[i], method);
      H_err *= weights[i];

      if(feedback) {
         INFO(format("# ------------------ Feedback for image #{:3d}", i));

         auto in_err
             = FindHomographyCostFunctor::homography_err(H, W, Cs[i], method);

         cout << str("K", K) << endl;
         cout << str("K-inv", K_inv) << endl;
         cout << str("M", M) << endl;
         cout << "------------------- R --" << endl;
         cout << "r1 = " << str(r1) << endl;
         cout << "r2 = " << str(r2) << endl;
         cout << "r3 = " << str(r1.cross(r2)) << endl;
         cout << str("U", U) << endl;
         cout << str("V", V) << endl;
         cout << str("R", R) << endl;
         cout << "------------------- H --" << endl;
         cout << str("H", H) << endl;
         cout << str("H1", H1) << endl;
         cout << "------------------- ... --" << endl;
         cout << format("method = {}", int(method)) << endl;
         cout << format("weight = {}", weights[i]) << endl;
         cout << format("in-err = {}", in_err) << endl;
         cout << format("H1-err = {}", H_err / weights[i]) << endl;
         cout << format("weighted-rr = {}", H_err) << endl;

         cout << endl << endl;
      }

      // And we're done.
      err += H_err;
   }

   return err;
}

Matrix3r estimate_K_Zhang(const vector<Matrix3r>& Hs, bool feedback)
{
   assert(Hs.size() != 0);

   // V matrix (from Zhang's algorithm), but setting gamma = 0
   const unsigned n = 6;
   MatrixXr A       = MatrixXr::Zero(long(2 * Hs.size()), n);

   auto calc_v = [&](const Matrix3r& H, unsigned i, unsigned j) {
      // i and j are columns
      VectorXr v(6);
      v(0) = H(0, i) * H(0, j);                     // B11
      v(1) = H(0, i) * H(1, j) + H(1, i) * H(0, j); // B12
      v(2) = H(1, i) * H(1, j);                     // B22
      v(3) = H(2, i) * H(0, j) + H(0, i) * H(2, j); // B13
      v(4) = H(2, i) * H(1, j) + H(1, i) * H(2, j); // B23
      v(5) = H(2, i) * H(2, j);                     // B33
      return v;
   };

   unsigned row = 0;
   for(const auto& H : Hs) {
      VectorXr v11 = calc_v(H, 0, 0);
      VectorXr v12 = calc_v(H, 0, 1);
      VectorXr v22 = calc_v(H, 1, 1);

      // v12 B = 0. i.e., columns are orthogonal
      A.block(row++, 0, 1, 6) = v12.transpose();

      // (v11 - v22) B = 0. i.e., columns have same length
      A.block(row++, 0, 1, 6) = (v11 - v22).transpose();
   }

   // We now have VB = 0, where B = [B11, B12, B22, B13, B23, B33]
   // Use linear-least-square approximation
   VectorXr B;
   svd_thin(A, B);

   double B11 = B(0);
   double B12 = B(1);
   double B22 = B(2);
   double B13 = B(3);
   double B23 = B(4);
   double B33 = B(5);

   Matrix3r K = Matrix3r::Zero();

   double cy      = (B12 * B13 - B11 * B23) / (B11 * B22 - B12 * B12);
   double lambdda = B33 - (B13 * B13 + cy * (B12 * B13 - B11 * B23)) / B11;
   double fx2     = lambdda / B11;
   double fy2     = lambdda * B11 / (B11 * B22 - B12 * B12);
   double s       = -B12 * fx2 * sqrt(fy2) / lambdda;
   double cx      = s * cy / sqrt(fy2) - B13 * fx2 / lambdda;

   K(0, 0) = sqrt(fx2);
   K(0, 1) = s;
   K(1, 1) = sqrt(fy2);
   K(0, 2) = cx;
   K(1, 2) = cy;
   K(2, 2) = 1.0;

   if(feedback) {
      MatrixXr U, V;
      VectorXr D;
      svd_UDV(A, U, D, V);

      cout << format(" {} Zhang with {} homographies", g_lines, Hs.size())
           << endl
           << endl;
      cout << "      SVD gives Eigenvalues are D = " << endl;
      for(auto i = 0; i < 6; ++i)
         cout << format("      D({}) = {}", i, D(i)) << endl;
      cout << endl;
      cout << format("      cy     = {}", cy) << endl;
      cout << format("      lambda = {}", lambdda) << endl;
      cout << format("      fx2    = {}", fx2) << endl;
      cout << format("      fy2    = {}", fy2) << endl;
      cout << format("      s      = {}", s) << endl;
      cout << format("      cx     = {}", cx) << endl;

      cout << endl << str("      K", K) << endl << endl;
   }

   return K;
}

Matrix3r estimate_K(const vector<Vector3r>& W,
                    const vector<vector<Vector3r>>& Cs,
                    const vector<Matrix3r>& Hs,
                    bool feedback)
{
   if(feedback) INFO("Estimate K");
   Matrix3r K0 = estimate_K_Zhang(Hs, feedback);
   refine_K(K0, W, Cs, Hs, feedback);
   return K0;

   if(false) {
      FindKCostFunctor cf;

      cf.W.resize(W.size());
      for(unsigned i = 0; i < W.size(); ++i)
         cf.W[i] = Vector2r(W[i](0) / W[i](2), W[i](1) / W[i](2));

      cf.Cs.resize(Cs.size());
      for(unsigned j = 0; j < Cs.size(); ++j) {
         if(Cs[j].size() != W.size()) {
            LOG_ERR(format("Every Cs must have size of W"));
            return Matrix3r::Identity();
         }

         auto& dst = cf.Cs[j];
         dst.resize(W.size());
         for(unsigned i = 0; i < W.size(); ++i) {
            const auto& X = Cs[j][i];
            dst[i]        = Vector2r(X(0) / X(2), X(1) / X(2));
         }
      }

      cf.single_f = false;
      cf.weights.resize(Cs.size());
      std::fill(cf.weights.begin(), cf.weights.end(), 1.0 / real(Cs.size()));
      cf.Hs = Hs;

      auto fn = [&](const real* X) -> real { return cf.evaluate(X); };

      // Matrix3r K0 =  estimate_K_Zhang(Hs, feedback);

      real start[4]   = {K0(0, 2), K0(1, 2), K0(0, 0), K0(1, 1)};
      real xmin[4]    = {0.0, 0.0, 0.0, 0.0};
      double reqmin   = 1e-12;
      double diffstep = 1e-1;
      int kcount      = 10000; // max interations
      int icount = 0, ifault = 0;

      levenberg_marquardt(
          fn, 4, start, xmin, reqmin, diffstep, 8, kcount, icount, ifault);

      Matrix3r K = Matrix3r::Identity();
      K(0, 2)    = xmin[0];
      K(1, 2)    = xmin[1];
      K(0, 0)    = xmin[2];
      K(1, 1)    = xmin[3];

      if(feedback) {
         INFO(format("Feedback on finding 'K'"));
         cout << format("   levenberg-marquardt:") << endl;
         cout << endl;
         cout << format("   iterations:           {}", icount) << endl;
         cout << format("   fault-code:           {}", ifault) << endl;
         cout << endl;
         cout << format("   initial-score:        {}", fn(&start[0])) << endl;
         cout << format("   final-score:          {}", fn(&xmin[0])) << endl;
         cout << endl;
         cout << str("K", K) << endl;
      }

      return K;
   }
}

real refine_K(Matrix3r& K0,
              const vector<Vector3r>& W,
              const vector<vector<Vector3r>>& Cs,
              const vector<Matrix3r>& Hs,
              bool feedback)
{
   FindKCostFunctor cf;

   cf.W.resize(W.size());
   for(unsigned i = 0; i < W.size(); ++i)
      cf.W[i] = Vector2r(W[i](0) / W[i](2), W[i](1) / W[i](2));

   cf.Cs.resize(Cs.size());
   for(unsigned j = 0; j < Cs.size(); ++j) {
      if(Cs[j].size() != W.size()) {
         LOG_ERR(format("Every Cs must have size of W"));
         return dNAN;
      }

      auto& dst = cf.Cs[j];
      dst.resize(W.size());
      for(unsigned i = 0; i < W.size(); ++i) {
         const auto& X = Cs[j][i];
         dst[i]        = Vector2r(X(0) / X(2), X(1) / X(2));
      }
   }

   cf.single_f = false;
   cf.weights.resize(Cs.size());
   std::fill(cf.weights.begin(), cf.weights.end(), 1.0 / real(Cs.size()));
   cf.Hs = Hs;

   auto fn = [&](const real* X) -> real { return cf.evaluate(X); };

   real start[4]   = {K0(0, 2), K0(1, 2), K0(0, 0), K0(1, 1)};
   real xmin[4]    = {0.0, 0.0, 0.0, 0.0};
   double reqmin   = 1e-12;
   double diffstep = 1e-1;
   int kcount      = 10000; // max interations
   int icount = 0, ifault = 0;

   levenberg_marquardt(
       fn, 4, start, xmin, reqmin, diffstep, 8, kcount, icount, ifault);

   Matrix3r K = Matrix3r::Identity();
   K(0, 2)    = xmin[0];
   K(1, 2)    = xmin[1];
   K(0, 0)    = xmin[2];
   K(1, 1)    = xmin[3];

   if(feedback) {
      cout << format(" {} Refining 'K' with levenberg-marquardt", g_lines)
           << endl;
      cout << format("      iterations:           {}", icount) << endl;
      cout << format("      fault-code:           {}", ifault) << endl;
      cout << endl;
      cout << format("      initial-score:        {}", fn(&start[0])) << endl;
      cout << format("      final-score:          {}", fn(&xmin[0])) << endl;
      cout << endl;
      cout << str("      K", K) << endl << endl;
   }

   K0 = K;

   return fn(&xmin[0]);
}

Matrix3r estimate_K_Zhang(const unsigned nx,
                          const unsigned ny,
                          const vector<vector<Vector3r>>& Cs,
                          const bool feedback)
{
   const auto W = make_W_pattern(nx, ny);

   // Get initial 'Hs'
   if(feedback)
      cout << format(" {} estimating initial homographies", g_lines) << endl;
   vector<Matrix3r> Hs(Cs.size());
   for(auto i = 0u; i < Cs.size(); ++i) {
      auto Herr = estimate_H(W, Cs[i], Hs[i], false);
      if(feedback)
         cout << format(" {} {:3d}, herr = {}", g_wedge, i, Herr) << endl;
   }

   Matrix3r K = estimate_K_Zhang(Hs, feedback);

   return K;
}

Matrix3r estimate_K(const unsigned nx,
                    const unsigned ny,
                    const vector<vector<Vector3r>>& Cs,
                    const bool feedback)
{
   const auto W = make_W_pattern(nx, ny);

   // Get initial 'Hs'
   if(feedback)
      cout << format(" {} estimating initial homographies", g_lines) << endl;
   vector<Matrix3r> Hs(Cs.size());
   for(auto i = 0u; i < Cs.size(); ++i) {
      auto Herr = estimate_H(W, Cs[i], Hs[i], false);
      if(feedback)
         cout << format(" {} {:3d}, herr = {}", g_wedge, i, Herr) << endl;
   }

   Matrix3r K = estimate_K_Zhang(Hs, feedback);
   auto err   = refine_K(K, W, Cs, Hs, feedback);

   return K;
}

} // namespace perceive::calibration
