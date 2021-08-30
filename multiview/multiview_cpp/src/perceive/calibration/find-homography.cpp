
#include "find-homography.hpp"
#include "perceive/utils/eigen-helpers.hpp"

#define This FindHomographyCostFunctor

namespace perceive::calibration
{
// ------------------------------------------------------------------- to-string

std::string This::to_string() const
{
   std::stringstream ss("");

   ss << format("FindHomographyCostFunctor with {}/{} points, using {} err\n",
                W.size(),
                C.size(),
                str(method));

   for(size_t i = 0; i < std::min(W.size(), C.size()); ++i) {
      ss << format("   [{:6.2f}, {:6.2f}]^T == H [{:6.2f}, {:6.2f}]^T\n",
                   C[i](0),
                   C[i](1),
                   W[i](0),
                   W[i](1));
   }

   for(auto i = std::min(W.size(), C.size()); i < W.size(); ++i) {
      ss << format("   [{:6.2f}, {:6.2f}]^T == H [{:6.2f}, {:6.2f}]^T\n",
                   NAN,
                   NAN,
                   W[i](0),
                   W[i](1));
   }

   for(auto i = std::min(W.size(), C.size()); i < C.size(); ++i) {
      ss << format("   [{:6.2f}, {:6.2f}]^T == H [{:6.2f}, {:6.2f}]^T\n",
                   C[i](0),
                   C[i](1),
                   NAN,
                   NAN);
   }

   return ss.str();
}

// -------------------------------------------------------------------- Evaluate

real This::evaluate(const real X[9]) const
{
   Matrix3r H = unpack(X);
   return homography_err(H, W, C, method);
}

// -------------------------------------------------------------- Homography Err

real This::homography_err(const Matrix3r& H,
                          const std::vector<Vector2r>& W,
                          const std::vector<Vector2r>& C,
                          Lp_norm_t method)
{
   if(W.size() != C.size()) {
      LOG_ERR(format("Got |W| = {}, and |C| = {}, but they must be the same!",
                     W.size(),
                     C.size()));
      return dNAN;
   }

   double err = 0.0;

   for(unsigned i = 0; i < W.size(); ++i) {
      const auto& w = W[i];
      const auto& c = C[i];

      real z_inv = real(1.0) / (w(0) * H(2, 0) + w(1) * H(2, 1) + H(2, 2));

      real x = (w(0) * H(0, 0) + w(1) * H(0, 1) + H(0, 2)) * z_inv;
      real y = (w(0) * H(1, 0) + w(1) * H(1, 1) + H(1, 2)) * z_inv;

      double dist = sqrt(square(c(0) - x) + square(c(1) - y));

      // INFO(format("([{}, {}] - [{}, {}] {i.e., H[{}, {}]}) = {}",
      //             c(0), c(1), x, y, w(0), w(1), dist));

      switch(method) {
      case Lp_norm_t::L_INF: err = std::max(err, dist); break;
      case Lp_norm_t::L1: err += dist; break;
      case Lp_norm_t::L2: err += square(dist); break;
      case Lp_norm_t::L3: err += dist * dist * dist; break;
      case Lp_norm_t::L4: err += square(square(dist)); break;
      default: FATAL("kBAM!");
      }
   }

   if(!std::isfinite(err)) return std::numeric_limits<real>::max();

   switch(method) {
   case Lp_norm_t::L_INF: return err;
   case Lp_norm_t::L1: return err / real(W.size());
   case Lp_norm_t::L2: return std::sqrt(err) / real(W.size());
   case Lp_norm_t::L3: return std::cbrt(err) / real(W.size());
   case Lp_norm_t::L4: return std::sqrt(sqrt(err)) / real(W.size());
   }

   return dNAN;
}

// ---------------------------------------------------------------------- Unpack

Matrix3r This::unpack(const real X[9]) const
{
   Matrix3r H;
   const real* x = &X[0];

   H(0, 0) = *x++;
   H(0, 1) = *x++;
   H(0, 2) = *x++;
   H(1, 0) = *x++;
   H(1, 1) = *x++;
   H(1, 2) = *x++;
   H(2, 0) = *x++;
   H(2, 1) = *x++;
   H(2, 2) = *x++;

   H /= std::cbrt(H.determinant());
   return H;
}

// ----------------------------------------------------- Estimate Homography LLS

void estimate_homography_AtA(const std::vector<Vector3r>& W,
                             const std::vector<Vector3r>& C,
                             MatrixXr& AtA)
{
   if(W.size() != C.size()) {
      LOG_ERR(format("{} = |W| != |C| = {}", W.size(), C.size()));
      return;
   }

   MatrixXr A   = MatrixXr::Zero(long(C.size() * 2), 9);
   unsigned row = 0;
   for(unsigned i = 0; i < C.size(); ++i) {
      const Vector3r w = W[i] / W[i](2);
      const Vector3r c = C[i] / C[i](2);

      A(row, 0) = 0.0;
      A(row, 1) = 0.0;
      A(row, 2) = 0.0;
      A(row, 3) = -w(0);
      A(row, 4) = -w(1);
      A(row, 5) = -1.0;
      A(row, 6) = c(1) * w(0);
      A(row, 7) = c(1) * w(1);
      A(row, 8) = c(1);
      row++;

      A(row, 0) = w(0);
      A(row, 1) = w(1);
      A(row, 2) = 1.0;
      A(row, 3) = 0.0;
      A(row, 4) = 0.0;
      A(row, 5) = 0.0;
      A(row, 6) = -c(0) * w(0);
      A(row, 7) = -c(0) * w(1);
      A(row, 8) = -c(0);
      row++;
   }

   AtA = A.transpose() * A;
}

real estimate_homography_LLS(const std::vector<Vector3r>& W,
                             const std::vector<Vector3r>& C,
                             Matrix3r& H)
{
   MatrixXr AtA;
   estimate_homography_AtA(W, C, AtA);

   VectorXr V;
   auto err     = svd_thin(AtA, V);
   unsigned pos = 0;
   for(unsigned row = 0; row < 3; ++row)
      for(unsigned col = 0; col < 3; ++col) H(row, col) = V(pos++);
   H /= std::cbrt(H.determinant());

   return err;
}

real estimate_homography_LLS(const std::vector<Vector2r>& W,
                             const std::vector<Vector2r>& C,
                             Matrix3r& H)
{
   if(W.size() != C.size()) {
      LOG_ERR(format("{} = |W| != |C| = {}", W.size(), C.size()));
      return dNAN;
   }

   MatrixXr A   = MatrixXr::Zero(long(C.size() * 2), 9);
   unsigned row = 0;
   for(unsigned i = 0; i < C.size(); ++i) {
      const auto& w = W[i];
      const auto& c = C[i];

      A(row, 0) = 0.0;
      A(row, 1) = 0.0;
      A(row, 2) = 0.0;
      A(row, 3) = -w(0);
      A(row, 4) = -w(1);
      A(row, 5) = -1.0;
      A(row, 6) = c(1) * w(0);
      A(row, 7) = c(1) * w(1);
      A(row, 8) = c(1);
      row++;

      A(row, 0) = w(0);
      A(row, 1) = w(1);
      A(row, 2) = 1.0;
      A(row, 3) = 0.0;
      A(row, 4) = 0.0;
      A(row, 5) = 0.0;
      A(row, 6) = -c(0) * w(0);
      A(row, 7) = -c(0) * w(1);
      A(row, 8) = -c(0);
      row++;
   }

   MatrixXr AtA = A.transpose() * A;
   VectorXr V;
   auto err     = svd_thin(AtA, V);
   unsigned pos = 0;
   for(unsigned row = 0; row < 3; ++row)
      for(unsigned col = 0; col < 3; ++col) H(row, col) = V(pos++);
   H /= std::cbrt(H.determinant());

   return err;
}

void H_to_r1r2t(const Matrix3r& H, Vector3& ssa, Vector3r& t, bool feedback)
{
   Vector3r r1 = H.block(0, 0, 3, 1);
   Vector3r r2 = H.block(0, 1, 3, 1);
   auto sz     = 0.5 * (r1.norm() + r2.norm());
   Vector3r r3 = r1.cross(r2);
   r1 /= r1.norm();
   r2 /= r2.norm();
   r3 /= r3.norm();

   const bool do_ortho = true;
   if(do_ortho) {
      // r3 is correct, but r1 and r2 may be smaller than 90 degrees
      Vector3r r12 = normalized(r1 + r2);
      Vector3r r4  = normalized(r12.cross(r3));

      r1 = normalized(r4 + r12);
      r2 = r3.cross(r1);
   }

   Matrix3r R;
   R.block(0, 0, 3, 1) = r1;
   R.block(0, 1, 3, 1) = r2;
   R.block(0, 2, 3, 1) = r3;

   if(do_ortho) {
      // Sanity, |R| should be 1
      if(fabs(1.0 - R.determinant()) > 1e-9)
         FATAL(format("|R| = {}", R.determinant()));
   }

   ssa = rot3x3_to_quaternion(R).to_spherical_axis_angle();
   t   = H.block(0, 2, 3, 1);
   t /= sz;
}

void r1r2t_to_H(const Vector3& ssa, const Vector3r& t, Matrix3r& H)
{
   Quaternion q;
   q.from_spherical_axis_angle(ssa);
   quaternion_to_Matrix3r(q, H);
   H.block(0, 2, 3, 1) = t;
}

void H_to_r1r2t(const Matrix3r& H, Vector6r& X)
{
   Vector3 ssa;
   Vector3r t;
   H_to_r1r2t(H, ssa, t);
   for(unsigned i = 0; i < 3; ++i) X(i) = ssa(int(i));
   for(unsigned i = 0; i < 3; ++i) X(i + 3) = t(i);
}

void r1r2t_to_H(const Vector6r& X, Matrix3r& H)
{
   r1r2t_to_H(Vector3(X(0), X(1), X(2)), Vector3r(X(3), X(4), X(5)), H);
}

static void test_r1r2t_to_H()
{
   Quaternion q(Vector3(.5, .1, .8), 0.2);
   auto ssa = q.to_spherical_axis_angle();
   auto t   = Vector3r(3, 4, 5);
   Matrix3r H1, H2, H3;

   r1r2t_to_H(ssa, t, H1);
   cout << format(
       "[{}, {}, {}] :: [{}, {}, {}]", ssa(0), ssa(1), ssa(2), t(0), t(1), t(2))
        << endl;
   cout << str("H1", H1) << endl;

   cout << "---------------" << endl;
   H_to_r1r2t(H1, ssa, t, true);
   cout << format(
       "[{}, {}, {}] :: [{}, {}, {}]", ssa(0), ssa(1), ssa(2), t(0), t(1), t(2))
        << endl;
   r1r2t_to_H(ssa, t, H2);
   cout << str("H2", H2) << endl;

   cout << "---------------" << endl;
   H_to_r1r2t(H2, ssa, t, true);
   cout << format(
       "[{}, {}, {}] :: [{}, {}, {}]", ssa(0), ssa(1), ssa(2), t(0), t(1), t(2))
        << endl;
   r1r2t_to_H(ssa, t, H3);
   cout << str("H3", H3) << endl;
}

void complete_homography(Matrix3r& H)
{
   auto a = H(0, 0);
   auto b = H(0, 1);
   auto c = H(0, 2);
   auto d = H(1, 0);
   auto e = H(1, 1);
   auto f = H(1, 2);
   auto g = H(2, 0);
   auto h = H(2, 1);
   H(2, 2)
       = (1.0 - g * (b * f - c * e) + h * (a * f - c * d)) / (a * e - b * d);
}

} // namespace perceive::calibration
