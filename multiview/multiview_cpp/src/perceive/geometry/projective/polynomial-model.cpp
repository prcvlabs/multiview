
#include "stdinc.hpp"

#define POLYNOMIAL_MODEL_IMPLEMENTATION
#include "polynomial-model.hpp"
#undef POLYNOMIAL_MODEL_IMPLEMENTATION

#include "perceive/io/fp-io.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/io/xdr.hpp"

#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"

#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

#include "static_math/static_math.h"

#include "perceive/calibration/calibration-utils.hpp"
#include "perceive/calibration/find-homography.hpp"
#include "perceive/calibration/fit-curve-to-checkerboard.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "json/json.h"
#include <rpc/xdr.h>

namespace perceive
{
using namespace ::perceive::calibration;

// ------------------------------------------------- PolynomialModel::operator==

template<typename T> inline real eigen_diff(const T& a, const T& b)
{
   real val{0.0};
   for(unsigned row = 0; row < a.rows(); ++row)
      for(unsigned col = 0; col < a.cols(); ++col)
         val += fabs(a(row, col) - b(row, col));
   return val;
}

template<unsigned K>
bool PolynomialModel<K>::operator==(const PolynomialModel<K>& o) const noexcept
{
#define EIGEN_TEST(v) (eigen_diff(v, o.v) < 1e-9)
#define NORM_TEST(v) (fabs(v - o.v) < 1e-9)
#define TEST(v) (v == o.v)
   return true && EIGEN_TEST(A_) && TEST(sensor_id_)
          && ((C - o.C).norm() < 1e-9) && NORM_TEST(s) && TEST(format_)
          && ((ws_ - o.ws_).norm() < 1e-9) && TEST(sensor_id_)
          && TEST(calib_region_);
#undef TEST
#undef NORM_TEST
#undef EIGEN_TEST
}

// -------------------------------------------------------------------- finalize

template<unsigned K> void PolynomialModel<K>::finalize(const bool feedback)
{
   const auto w = format_.width();
   const auto h = format_.height();
   auto f       = [&](const Vector2& D) { return this->undistort(D); };

   Expects(w > 0);
   Expects(h > 0);

   Matrix3r grid_K = Matrix3r::Identity();
   grid_K(0, 0) = grid_K(1, 1) = w * 0.20;
   grid_K(0, 2)                = w * 0.5;
   grid_K(1, 2)                = h * 0.5;

   auto make_qnn = [&]() {
      vector<Vector2> Us, Ds;
      Us.reserve(size_t(w * h)); // undistorted
      Ds.reserve(size_t(w * h)); // distorted

      const auto side = std::min(w, h);
      const auto step = side / 30;

      AABB distort_domain = AABB::minmax();
      for(auto y = 0; y < h; y += step) {
         for(auto x = 0; x < w; x += step) {
            Vector2 D(x, y);
            Vector2 U = f(D);
            distort_domain.union_point(D);
            Vector3r UU = Vector3r(U.x, U.y, 1.0);
            Vector3r Ur = normalized_P2(grid_K * UU);
            if(Ur(0) >= 0.0 && Ur(0) <= w && Ur(1) >= 0.0 && Ur(1) <= h) {
               Ds.push_back(D);
               Us.push_back(U);
            }
         }
      }

      Expects(Ds.size() > 0);
      distort_field_.init_qnn(Us, Ds); // Maps undistorted to distorted

      // ---- Initialize the empirical field ----
      auto ef = make_shared<EmpiricalFieldF>();
      auto g  = [&](const Vector2f& u) { // distort
         return to_vec2f(distort_field_.evaluate(to_vec2(u)));
      };
      distort_domain.grow(0.2 * distort_domain.width());
      ef->init(g, distort_domain, 400, false, false);
      empirical_distort_field_ = std::move(ef);
   };
   auto seconds           = time_thunk(make_qnn);
   distort_field_is_init_ = true;

   if(feedback) {
      INFO(format("Initializing undistort-inverse (i.e., distort) took {}s",
                  seconds));
   }
}

// ------------------------------------------------------ distort field evaluate

template<unsigned K>
Vector2 PolynomialModel<K>::distort_field_evaluate_(const Vector2& U) const
{
   assert(U.is_finite());
   Expects(U.is_finite());
   if(!distort_field_is_init_)
      FATAL("must call 'finialize' before calling distort");

   static constexpr bool use_nelder_mead = true;

   // Distort-field still isn't accurate enough...
   auto fn = [&](const real* X) {
      Vector2 U2 = this->undistort(Vector2(X[0], X[1]));
      return (U - U2).quadrance();
   };

   Vector2 D00 = to_vec2(empirical_distort_field_->evaluate(to_vec2f(U)));
   Vector2 D0  = Vector2(D00.x / ws_.x, D00.y / ws_.y);
   if(!D0.is_finite()) {
      static std::mutex padlock;
      lock_guard<std::mutex> lock(padlock);
      D0 = distort_field_.evaluate(U);
   }

   const int n_params = 2;
   real start[2]      = {D0.x, D0.y};
   real xmin[2];
   real reqmin   = 1e-20;
   real diffstep = 0.1;
   int kcount    = 1000;
   int icount = 0, numres = 0, ifault = 0;

   if(use_nelder_mead) {
      real ynewlo  = dNAN;
      real step[2] = {2.0, 2.0};
      nelder_mead(fn,
                  n_params,
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  2,
                  10 * kcount,
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
                          2,
                          kcount,
                          icount,
                          ifault);
   }

   return Vector2(xmin[0], xmin[1]); // Accurately distorted
}

// ---------------------------------------------------------------- estimate LLS

template<unsigned K>
real PolynomialModel<K>::estimate_LLS(const vector<Vector2>& Ds,
                                      const vector<Vector2>& Us,
                                      const Vector2 radial_epipole,
                                      const real average_radial_dist)
{
   Expects(Ds.size() == Us.size());
   Expects(real(Ds.size()) > 2.0 * KK);

   const auto N = Ds.size();

   this->C = radial_epipole;
   this->s = sqrt(2.0) / average_radial_dist;

   MatrixXr B = MatrixXr::Zero(KK, long(N));
   MatrixXr C = MatrixXr::Zero(2, long(N));

   unsigned col = 0;
   for(unsigned n = 0; n < N; ++n) {
      real x = (Ds[n](0) - this->C.x) * this->s;
      real y = (Ds[n](1) - this->C.y) * this->s;

      const decltype(lift<K>(x, y)) X = lift<K>(x, y);
      const auto& U                   = Us[n];

      for(unsigned k = 0; k < KK; ++k) B(k, col) = X(k);
      C(0, col) = U(0);
      C(1, col) = U(1);

      ++col;
   }

   // Perform Moore-Penrose pseudo-inverse
   // AB = C,  therefore A = CBt (BBt)^-1
   MatrixXr Bt        = B.transpose();
   MatrixXr BBt       = B * Bt;
   MatrixXr BBt_inv   = BBt.inverse();
   MatrixXr BtBBt_inv = Bt * BBt_inv;
   MatrixXr A12       = C * BtBBt_inv; // Just the first two rows

   A_ = A12;

   // auto undistort_f = [&] (const Vector2& x) { return AX<K>(A, x); };
   // auto cost = projection_err(undistort_f, gt, imgs);
   auto err = 0.0;
   for(auto n = 0u; n < N; ++n) err += (Us[n] - undistort(Ds[n])).norm();
   return err / real(N);
}

// -------------------------------------------------------- do-esimate-lls-lines

template<unsigned K>
real PolynomialModel<K>::estimate_LLS_line_and_pts(
    const vector<LLSLineData>& dat,
    const vector<Vector3r>& Ds,
    const vector<Vector3r>& Us)
{
   // When undistorting: u = AD, where 'D' is the lifted distorted coord
   // If we know that 'u' is on line 'l', then dot(l, AD) = 0.
   // AD can be transformed into homogeneous coordinates, and this gives:
   //
   //       [l_1 D ... l_2 D ... ] [a_1 ... a_2 ...]^t = -l3
   //
   // Where a_1 and a_2 are the rows of A.
   //
   // Additionally if we have the relationship between distorted and
   // undistorted points, we can write:
   //
   //       [D ... 0 ... ] [a_1 ... a_2 ...]^t = u_1
   //       [0 ... D ... ] [a_1 ... a_2 ...]^t = u_2
   //
   // Stack all these equations into:
   //
   //       XA = L
   //
   // And:
   //
   //       A = X^dagger L
   //

   Expects(Ds.size() == Us.size());

   // Home many points are we dealing with?
   const auto n_line_points = std::accumulate(
       dat.begin(), dat.end(), size_t(0), [&](size_t val, const auto& d) {
          return val + d.Xs.size();
       });
   const auto n_points = n_line_points + 2 * Ds.size();

   MatrixXr X = MatrixXr::Zero(long(n_points), 2 * KK);
   MatrixXr L = MatrixXr::Zero(long(n_points), 1);
   auto row   = 0u;

   // Add rows for points that must hit straight lines
   for(const auto& d : dat) {
      const auto l = to_vec3(d.line_eq).normalised_line();
      for(const auto& x : d.Xs) {
         // Condition and lift 'x'
         VectorKKr XI
             = lift<K>((x(0) / x(2) - C.x) * s, (x(1) / x(2) - C.y) * s);

         // Add to 'X'
         for(auto col = 0u; col < KK; ++col) {
            X(row, col + 0)  = l.x * XI(col);
            X(row, col + KK) = l.y * XI(col);
         }
         L(row) = -l.z;
         row++;
      }
   }

   // Add rows for points that must hit points
   for(auto i = 0u; i < Ds.size(); ++i) {
      // Condition and lift 'x'
      VectorKKr XI = lift<K>((Ds[i](0) / Ds[i](2) - C.x) * s,
                             (Ds[i](1) / Ds[i](2) - C.y) * s);
      for(auto col = 0u; col < KK; ++col) {
         X(row + 0, col + 0)  = XI(col);
         X(row + 1, col + KK) = XI(col);
      }
      L(row + 0) = Us[i](0) / Us[i](2);
      L(row + 1) = Us[i](1) / Us[i](2);
      row += 2;
   }

   MatrixXr Xt       = X.transpose();
   MatrixXr XtX      = Xt * X;
   MatrixXr XtX_inv  = XtX.inverse();
   MatrixXr X_dagger = XtX_inv * Xt;
   MatrixXr newA     = X_dagger * L; // newA is a column vector

   for(auto col = 0u; col < KK; ++col) {
      A_(0, col) = newA(col + 0);
      A_(1, col) = newA(col + KK);
   }

   // Return the error
   return projection_error(dat, Ds, Us);
}

// ------------------------------------------------------------ projection-error

template<unsigned K>
real PolynomialModel<K>::projection_error(const vector<LLSLineData>& line_data,
                                          const vector<Vector3r>& Ds,
                                          const vector<Vector3r>& Us)
{
   Expects(Ds.size() == Us.size());

   auto err1    = 0.0;
   auto counter = 0;
   for(const auto& ldat : line_data) {
      auto l = to_vec3(ldat.line_eq);
      for(const auto& X : ldat.Xs) {
         auto u = undistort(Vector2(X(0) / X(2), X(1) / X(2)));
         err1 += fabs(dot_line_point(l, u));
         ++counter;
      }
   }
   if(counter > 0) err1 /= counter;

   auto err2 = 0.0;
   for(auto n = 0u; n < Ds.size(); ++n) {
      auto u0 = homgen_P2_to_R2(to_vec3(Us[n]));
      auto u1 = undistort(homgen_P2_to_R2(to_vec3(normalized_P2(Ds[n]))));
      err2 += (u1 - u0).norm();
   }
   if(Ds.size() > 0) err2 /= real(Ds.size());

   return 0.5 * (err1 + err2);
}

template<unsigned K>
std::pair<Matrix3r, real>
PolynomialModel<K>::estimate_homography() const noexcept
{
   const auto& hull     = calib_hull();
   const auto w         = calib_format().x;
   const auto h         = calib_format().y;
   const int grid_width = 40;
   const real dxy       = real(w) / grid_width;
   const auto ppt       = Vector2(0.5 * w, 0.5 * h);

   // U = H * D // Homography takes distorted point -> undistorted
   vector<Vector3r> Us, Ds;
   Matrix3r H;

   Us.reserve(square(grid_width));
   Ds.reserve(square(grid_width));

   for(auto y = 0; y < grid_width; ++y) {
      for(auto x = 0; x < grid_width; ++x) {
         const auto p = Vector2(x * dxy, y * dxy);
         if(point_in_polygon(p, cbegin(hull), cend(hull))) {
            Ds.push_back(Vector3r(p.x - ppt.x, p.y - ppt.y, 1.0));
            Us.push_back(this->undistort(Vector3r(p.x, p.y, 1.0)));
         }
      }
   }

   //

   const real H_err = calibration::estimate_homography_LLS(Us, Ds, H);

   return std::pair<Matrix3r, real>(H, H_err);
}

// ------------------------------------------------------------------- To-String

template<unsigned K> string PolynomialModel<K>::to_string() const
{
   vector<string> row1, row2;
   for(int c = 0; c < A_.cols(); ++c) {
      row1.push_back(str_precise(A_(0, c)));
      row2.push_back(str_precise(A_(1, c)));
   }

   return ::perceive::format(
       R"V0G0N(
{{
   "model": "{}",
   "sensor-id": "{}",
   "format": [{}, {}],
   "center": [{}, {}],
   "scale":  {},
   "calib-region": [{:g}, {:g}, {:g}, {:g}],
   "A":
[[{}],
 [{}]],
   "calib-hull": [{}]
}}
)V0G0N",
       model_name(),
       sensor_id(),
       calib_format().x,
       calib_format().y,
       str_precise(radial_epipole().x),
       str_precise(radial_epipole().y),
       str_precise(scale()),
       calib_region().left,
       calib_region().top,
       calib_region().right,
       calib_region().bottom,
       implode(row1.begin(), row1.end(), ", "),
       implode(row2.begin(), row2.end(), ", "),
       implode(
           calib_hull_.begin(), calib_hull_.end(), ", ", [](const Vector2& x) {
              return format("[{:g}, {:g}]", x.x, x.y);
           }));
}

// --------------------------------------------------------------------- to-json

template<unsigned K> Json::Value PolynomialModel<K>::to_json() const
{
   Json::Value node(Json::objectValue);
   node["model"]        = model_name();
   node["sendor_id"]    = sensor_id();
   node["format"]       = json_save(Vector2(format_.x, format_.y));
   node["center"]       = json_save(C);
   node["scale"]        = s;
   node["calib-region"] = json_save(calib_region_);
   node["A"]            = json_save(A_);
   auto calib_json      = Json::Value{Json::arrayValue};
   calib_json.resize(unsigned(calib_hull_.size()));
   for(auto i = 0u; i < calib_hull_.size(); ++i)
      calib_json[i] = json_save(calib_hull_[i]);
   node["calib-hull"] = calib_json;
   return node;
}

template<unsigned K>
void PolynomialModel<K>::from_json(const Json::Value& root) noexcept(false)
{
   string name;
   json_load(get_key(root, "model"), name);
   if(name != model_name())
      throw std::runtime_error(::perceive::format("invalid model name; "
                                                  "expected '{}', but read "
                                                  "'{}'",
                                                  model_name(),
                                                  name));

   const auto s0 = time_thunk([&]() {
      json_load(get_key(root, "sensor-id"), sensor_id_);
      json_load(get_key(root, "format"), format_);
      json_load(get_key(root, "center"), C);
      json_load(get_key(root, "scale"), s);
      json_load(get_key(root, "calib-region"), calib_region_);
      json_load(get_key(root, "A"), A_);
   });

   const auto s1 = time_thunk([&]() {
      auto node = get_key(root, "calib-hull");
      if(!node.isArray())
         throw std::runtime_error("expecting JSON array value");
      calib_hull_.resize(node.size());
      for(auto i = 0u; i < calib_hull_.size(); ++i)
         json_load(node[i], calib_hull_[i]);
   });

   const auto s2 = time_thunk([&]() { finalize(); });
}

// --------------- Make sure the tranlation unit instantiates polynomial classes

// template class PolynomialModel<2>;
// template class PolynomialModel<3>;
// template class PolynomialModel<4>;
// template class PolynomialModel<5>;
template class PolynomialModel<6>;
// template class PolynomialModel<7>;
template class PolynomialModel<8>;
// template class PolynomialModel<9>;
// template class PolynomialModel<10>;
// template class PolynomialModel<11>;
// template class PolynomialModel<12>;
// template class PolynomialModel<13>;
// template class PolynomialModel<14>;
// template class PolynomialModel<16>;
// template class PolynomialModel<20>;

// -------------------------------------------------------------------------- IO

void load(DistortionModel& data, const string& fname) noexcept(false)
{
   read(data, file_get_contents(fname));
}

void save(const DistortionModel& data, const string& fname) noexcept(false)
{
   std::string s;
   write(data, s);
   file_put_contents(fname, s);
}

void read(DistortionModel& data, const std::string& in) noexcept(false)
{
   const auto root = parse_json(in);

   std::string err = "";

   try {
      data.from_json(root);
   } catch(std::logic_error& e) {
      err = strlen(e.what()) == 0 ? "logic error" : e.what();
   } catch(std::runtime_error& e) {
      err = strlen(e.what()) == 0 ? "runtime error" : e.what();
   } catch(std::exception& e) {
      // JSON parse error
      err = strlen(e.what()) == 0 ? "exception" : e.what();
   } catch(...) {
      err = format("unknown error");
   }

   if(err != "") throw std::runtime_error(err);

   data.finalize();
}

void write(const DistortionModel& data, std::string& out) noexcept(false)
{
   out = data.to_string();
}

} // namespace perceive
