
#include "stdinc.hpp"

#include "calibration-utils.hpp"
#include "fit-curve-to-checkerboard.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "perceive/geometry/splines/spline-2d.hpp"
#include "perceive/graphics.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/file-system.hpp"

#define This FitCurveToCheckerboardCostFunctor

namespace perceive::calibration
{
using std::cout;
using std::endl;

// ---------------------------------------------------- optimize-spline-on-field

real optimize_spline_on_field(const Field& field,
                              Spline2d& spline0,
                              const bool opt_endpoints,
                              const bool feedback)
{
   const unsigned n_points = 30;
   const auto dt           = 1.0 / real(n_points + 1);

   Spline2d spline                       = spline0;
   std::vector<array<double, 12>> coeffs = spline.unpack();

   auto pack = [&](real* X) {
      if(opt_endpoints) {
         *X++ = spline.range_hint().x;
         *X++ = spline.range_hint().y;
      }
      auto coeffs = spline.unpack();
      for(const auto& coeff : coeffs)
         for(const auto& val : coeff) *X++ = val;
   };

   auto unpack = [&](const real* X) {
      Vector2 r;
      if(opt_endpoints) {
         r.x = *X++;
         r.y = *X++;
      }
      for(auto& coeff : coeffs)
         for(auto& val : coeff) val = *X++;
      if(opt_endpoints) {
         if(r.x > r.y) std::swap(r.x, r.y);
         if(r.x < 0.0) r.x = 0.0;
         if(r.y > 1.0) r.y = 1.0;
         spline.range_hint() = r;
      }
      spline.init(coeffs);
   };

   auto fn = [&](const real* X) -> real {
      unpack(X);

      // (1) Get N points along the spline
      // (2) Calculate the gradient at that point
      // (3) Get the dot-product with scharr field

      auto eval1 = [&](real t) {
         auto gx = spline.gradient(t);
         auto x  = to_pt2(Vector2(gx.x.x, gx.x.y));
         if(field.in_bounds(x)) {
            auto g      = field(x);
            auto g_norm = g.norm();
            if(g_norm < 0.1) return -0.1;
            auto cos_t = dot(gx.g, g / g_norm);
            return 2.0 * (fabs(cos_t) - 0.6);
         }
         return -1e6;
      };

      auto eval2 = [&](real t) {
         auto gx = spline.gradient(t);
         auto x  = to_pt2(Vector2(gx.x.x, gx.x.y));
         if(field.in_bounds(x)) return fabs(dot(gx.g, field(x)));
         return -1e6;
      };

      auto t0       = spline.range_hint().x;
      auto t1       = spline.range_hint().y;
      auto score    = 0.0;
      const auto dt = 1.0 / 100;
      if(opt_endpoints) {
         for(auto t = t0; t < t1; t += dt) score += eval1(t);
         score += eval1(t1);
      } else {
         for(auto t = t0; t < t1; t += dt) score += eval2(t);
         score += eval2(t1);
      }

      return -score;
   };

   const bool use_nelder_mead = true;
   const unsigned n_params
       = 12 * unsigned(coeffs.size()) + (opt_endpoints ? 2 : 0);
   vector<real> start(n_params);
   vector<real> xmin(n_params);
   real ynewlo   = dNAN;
   real reqmin   = 1e-7;
   real diffstep = 0.1;
   int kcount    = 1000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method = nullptr;

   pack(&start[0]);
   real ystartlo = fn(&start[0]);

   if(!use_nelder_mead) {
      method = "levenberg-marquardt";
      levenberg_marquardt(fn,
                          n_params,
                          &start[0],
                          &xmin[0],
                          reqmin,
                          diffstep,
                          5,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);
   } else {
      method = "nelder-mead";

      vector<real> step(n_params);

      // The initial size of the simplex is crucial
      auto simplex_factor = 0.05;
      for(unsigned i = 0; i < n_params; ++i)
         step[i]
             = fabs(start[i]) > 1e-3 ? fabs(simplex_factor * start[i]) : 1e-3;
      if(opt_endpoints) { // i.e., we're opt. end points
         step[0] = 0.01;
         step[1] = 0.01;
      }

      nelder_mead(fn,
                  n_params,
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  10 * kcount,
                  icount,
                  numres,
                  ifault);
   }

   unpack(&xmin[0]);

   if(feedback) {
      INFO(format("Feedback for uber-line spline fitting"));
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;
   }

   if(ynewlo < ystartlo) spline0 = spline;

   return std::min(ystartlo, ynewlo);
}

// -------------------------------------------------------- find-initial-splines

vector<Spline2d>
find_initial_splines(const unsigned nx,
                     const unsigned ny,
                     const Vector6r r1r2t,
                     std::function<Vector2(const Vector2& x)> distort,
                     const real rho,
                     const unsigned m_factor)
{
   vector<Spline2d> splines;

   auto calc_Us = [&](unsigned line_id, vector<Vector2>& Us) {
      Us.clear();
      const bool is_row = line_id < ny;
      Vector2 a, b;
      if(is_row) {
         a = Vector2(line_id, -0.5);
         b = Vector2(line_id, nx);
      } else {
         a = Vector2(-0.5, line_id - ny);
         b = Vector2(ny, line_id - ny);
      }

      const auto n_points = 100;
      Us.reserve(n_points);
      for(auto i = 0; i < n_points; ++i)
         Us.push_back(a + (i / real(n_points - 1)) * (b - a));
   };

   // -- Transforms a grid location (world coord) into an undistorted
   //    point (i.e., a ray, but as a normalized coord)
   Matrix3r H;
   r1r2t_to_H(r1r2t, H);
   auto mult_H = [&](Vector2 G) {
      Vector3r g = normalized_P2(H * Vector3r(G.x, G.y, 1.0));
      return Vector2(g(0), g(1));
   };

   auto vec_to_s = [&](Vector2 u) { return u.to_string(); };

   vector<Vector2> Gs;
   vector<Vector2> Us;
   vector<Vector2> Ds;
   splines.resize(nx + ny);
   for(auto line_id = 0u; line_id < nx + ny; ++line_id) {
      Gs.clear();
      Us.clear();
      Ds.clear();
      calc_Us(line_id, Gs);
      std::transform(Gs.begin(), Gs.end(), std::back_inserter(Us), mult_H);
      std::transform(Us.begin(), Us.end(), std::back_inserter(Ds), distort);

      // cout << format("line {:2d}: {}", line_id,
      //                implode(Gs.begin(), Gs.end(), ", ", vec_to_s))
      //      << endl << endl;
      //     cout << format("line {:2d}: {}", line_id,
      //                implode(Us.begin(), Us.end(), ", ", vec_to_s))
      //      << endl << endl;
      // cout << format("line {:2d}: {}", line_id,
      //                implode(Ds.begin(), Ds.end(), ", ", vec_to_s))
      //      << endl << endl;

      splines[line_id].init(Ds, rho, int(m_factor));
   }

   return splines;
}

// -------------------------------------------------------- find-initial-splines

vector<Spline2d> find_initial_splines(const unsigned nx,
                                      const unsigned ny,
                                      const vector<Vector3r>& Ds,
                                      const real rho,
                                      const unsigned m_factor)
{
   vector<Spline2d> ret(nx + ny);
   vector<unsigned> line_inds;

   for(auto l = 0u; l < nx + ny; ++l) {
      calc_line_inds(nx, ny, l, line_inds);
      vector<Vector2> ps;
      for(auto ind : line_inds)
         ps.emplace_back(Ds[ind](0) / Ds[ind](2), Ds[ind](1) / Ds[ind](2));
      ret[l].init(ps, rho, int(m_factor));
   }

   return ret;
}

// --------------------------------------------------------- accurate-spline-fit

void accurate_spline_fit(const cv::Mat& im, // distorted
                         vector<Spline2d>& splines_in_out,
                         const bool feedback,
                         const unsigned refits,
                         const real max_blur,
                         const unsigned max_blur_sz,
                         const real rho,
                         const unsigned m_factor)
{
   vector<Spline2d>& splines = splines_in_out;

   Expects(max_blur_sz % 2 == 1);
   Expects(max_blur_sz <= 31u);
   Expects(refits > 0);

   // ---- Make the fields
   vector<Field> scharrs(refits);
   real blur_sigma    = max_blur;
   unsigned blur_size = max_blur_sz;
   for(auto i = 0u; i < refits; ++i) {
      scharrs[i] = make_field(im, blur_size, blur_sigma, true);

      // Shrink blur-size and blur-sigma
      blur_sigma /= 2.0;
      blur_size = 2 * blur_size / 3;
      if(blur_size % 2 == 0) blur_size++; // must be odd
      if(blur_size < 3) blur_size = 3;    // must be >= 3
   }

   // ---- Make the initial splines
   for(auto& spline : splines)
      for(auto i = 0u; i < scharrs.size(); ++i)
         optimize_spline_on_field(scharrs[i], spline, false, false);
}

void accurate_spline_fit(const string& image_filename, // distorted
                         vector<Spline2d>& splines_in_out,
                         const bool feedback,
                         const unsigned refits,
                         const real max_blur,
                         const unsigned max_blur_sz,
                         const real rho,
                         const unsigned m_factor)
{
   // ---- Read the image from disk
   if(!is_regular_file(image_filename))
      throw std::runtime_error(format("file not found: '{}'", image_filename));
   cv::Mat im = cv::imread(image_filename, cv::IMREAD_COLOR);

   accurate_spline_fit(im,
                       splines_in_out,
                       feedback,
                       refits,
                       max_blur,
                       max_blur_sz,
                       rho,
                       m_factor);
}

// -------------------------------------------------------------- render splines

ARGBImage
render_splines(const cv::Mat& im, const vector<Spline2d>& splines, int dxy)
{
   ARGBImage argb;
   cv_to_argb(im, argb);

   auto draw_f = [&](int x, int y) {
      for(auto dy = -dxy; dy <= dxy; ++dy) {
         for(auto dx = -dxy; dx <= dxy; ++dx) {
            Point2 p(x + dx, y + dy);
            if(argb.in_bounds(p)) argb(p) = k_crimson;
         }
      }
   };

   const auto dt = 0.02;
   for(const auto& spline : splines)
      for(auto t = dt; t <= 1.0; t += dt)
         bresenham(spline.evaluate(t - dt), spline.evaluate(t), draw_f);

   return argb;
}

// ------------------------------------------------------------------- find-grid

vector<Vector3r> find_nx_ny_grid(const unsigned nx,
                                 const unsigned ny,
                                 const cv::Mat& image,
                                 const bool fast_check)
{
   vector<Vector3r> out;

   const cv::Mat* im = &image;
   cv::Mat grey_data;
   if(image.channels() > 1) {
      Expects(image.channels() == 3);
      cv::cvtColor(image, grey_data, cv::COLOR_BGR2GRAY);
      im = &grey_data;
   }
   const cv::Mat& grey = *im;

   // ---- Find the initial corners grid

   vector<cv::Point2f> cv_corners;
   auto flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE;
   if(fast_check) flags += cv::CALIB_CB_FAST_CHECK;
   if(!cv::findChessboardCorners(
          grey, cv::Size(int(nx), int(ny)), cv_corners, flags))
      throw std::runtime_error("failed to find initial corner-grid");

   auto subpixel_flags = cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS;
   cv::cornerSubPix(grey,
                    cv_corners,
                    cv::Size(7, 7),
                    cv::Size(-1, -1),
                    cv::TermCriteria(subpixel_flags, 30, 0.1));

   out.reserve(size_t(std::distance(cbegin(cv_corners), cend(cv_corners))));
   std::transform(
       cbegin(cv_corners),
       cend(cv_corners),
       std::back_inserter(out),
       [&](cv::Point2f p) { return to_vec3r(Vector3f(p.x, p.y, 1.0f)); });

   flip_rows(nx, ny, out);
   flip_cols(nx, ny, out);

   return out;
}

ARGBImage render_detected_corner_grid(const vector<Vector3r>& corners,
                                      const cv::Mat& image)
{
   // ---- Stop here to generate an output image that shows the results
   ARGBImage argb;
   cv_to_argb(image, argb);
   {
      vector<Vector2> corners2;
      std::transform(corners.begin(),
                     corners.end(),
                     std::back_inserter(corners2),
                     [&](Vector3r p) { return Vector2(p(0), p(1)); });

      for(auto i = 0u; i < corners2.size(); ++i)
         draw_cross(argb, to_pt2(corners2[i]), k_red, 4);
      for(auto i = 0u; i < corners2.size(); ++i)
         render_string(argb,
                       format("{}", i),
                       to_pt2(corners2[i]) + Point2(3, -6),
                       k_yellow,
                       k_black);
   }
   return argb;
}

vector<Vector3r> find_nx_ny_grid(const unsigned n, // the image number
                                 const unsigned nx,
                                 const unsigned ny,
                                 const string& image_filename,
                                 const string& out_dir)
{
   vector<Vector3r> out;

   // ---- Start by finding the initial corners grid
   cv::Mat image = cv::imread(image_filename, cv::IMREAD_COLOR);
   out           = find_nx_ny_grid(nx, ny, image, false);

   // ---- Stop here to generate an output image that shows the results
   if(out_dir != "") {
      ARGBImage argb = render_detected_corner_grid(out, image);
      argb.save(format("{}/image-{:2d}_01_initial-grid.png", out_dir, n));
   }

   return out;
}

// ----------------------------------------------------------- average-grid-size

real average_grid_size(const unsigned nx,
                       const unsigned ny,
                       const vector<Vector3r>& corners)
{
   auto distance = [&](const Vector3r& A, const Vector3r& B) {
      return Vector2(A(0) - B(0), A(1) - B(1)).norm();
   };

   auto counter = 0;
   auto sum     = 0.0;
   for(auto y = 0u; y < ny; ++y) {
      for(auto x = 1u; x < nx; ++x) {
         sum += distance(corners[y * nx + x], corners[y * nx + x - 1]);
         ++counter;
      }
   }

   for(auto y = 1u; y < ny; ++y) {
      for(auto x = 0u; x < nx; ++x) {
         sum += distance(corners[y * nx + x], corners[(y - 1) * nx + x]);
         ++counter;
      }
   }

   return sum / real(counter);
}

} // namespace perceive::calibration
