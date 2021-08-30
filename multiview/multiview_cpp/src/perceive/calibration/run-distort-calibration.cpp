
#include "stdinc.hpp"

#include "calibration-utils.hpp"
#include "fit-curve-to-checkerboard.hpp"
#include "run-distort-calibration.hpp"

#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/file-system.hpp"

#include "perceive/io/lazy-s3.hpp"

#include "perceive/geometry/polygon.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/geometry/splines/spline-2d.hpp"
#include "perceive/geometry/vector.hpp"

#include "perceive/graphics.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

#include "json/json.h"

#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace perceive::calibration
{
using namespace std::string_literals;

// --------------------------------------------------------- for output purposes

inline void print(const string& glyph,
                  const string& color,
                  const string& msg,
                  const bool print_newline = true)
{
   cout << format(" {}  {}{}\x1b[0m", glyph, color, msg);
   if(print_newline)
      cout << endl;
   else
      cout.flush();
}

static const auto g_info         = "\x1b[37m\u261b\x1b[0m"s;
static const auto g_skull        = "\x1b[91m\u2620\x1b[0m"s;
static const auto g_radioactive  = "\x1b[91m\u2622\x1b[0m"s;
static const auto g_dotted_sq    = "\x1b[96m\u2b1a\x1b[0m"s;
static const auto g_bullet       = "\x1b[0m\u2738\x1b[0m"s;
static const auto g_cross_arrows = "\x1b[96m\u2928\x1b[0m"s;
static const auto g_waves        = "\x1b[96m\u29da\x1b[0m"s;
static const auto g_wave         = "\x1b[96m\u223c\x1b[0m"s;
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

// --------------------------------------------------------------- load cv image
// fname could be an s3 URI
static cv::Mat load_cv_image(const string_view fname) noexcept
{
   vector<char> raw_data;
   try {
      if(is_s3_uri(fname)) {
         s3_load(fname, raw_data);
      } else {
         file_get_contents(fname, raw_data);
      }
   } catch(std::exception&) {
      return cv::Mat{};
   }
   return argb_to_cv(decode_image(raw_data));
}

// ------------------------------------------------------------ Theta Difference

static real model_theta_err(const unsigned nx,
                            const unsigned ny,
                            const vector<vector<Vector3r>>& gts,
                            const DistortionModel& model,
                            const vector<vector<Vector3r>>& dts)
{
   Expects(gts.size() == dts.size());

   real err = 0.0;

   auto process = [&](auto ind0, auto ind1) -> void {
      for(auto i = 0u; i < gts.size(); ++i) {
         auto u        = to_vec3(gts[i][ind0]).normalised();
         auto v        = to_vec3(gts[i][ind1]).normalised();
         auto gt_theta = acos(clamp(u.dot(v), -1.0, 1.0));

         auto s  = model.undistort(homgen_P2_to_R2(to_vec3(dts[i][ind0])));
         auto t  = model.undistort(homgen_P2_to_R2(to_vec3(dts[i][ind1])));
         auto sn = homgen_R2_to_P2(s).normalised();
         auto tn = homgen_R2_to_P2(t).normalised();
         auto dt_theta = acos(clamp(sn.dot(tn), -1.0, 1.0));

         err += fabs(short_angle_diff(gt_theta, dt_theta));
      }
   };

   for(auto y = 0u; y < ny; ++y) process((y + 0) * nx - 0, (y + 1) * nx - 1);
   for(auto x = 0u; x < nx; ++x) process(x, (ny - 1) * nx + x);

   return err / real(gts.size() * (nx + ny));
}

// ---------------------------------------------------------------- Print Angles

static void print_gt_angles(string message,
                            const unsigned nx,
                            const unsigned ny,
                            vector<Vector3r>& coords,
                            std::function<Vector3r(Vector3r)> f)
{
   vector<Vector3r> normalized_coords(coords.size());
   std::transform(cbegin(coords), cend(coords), begin(normalized_coords), f);

   auto process = [&](string msg, auto ind0, auto ind1) {
      auto u     = to_vec3(normalized_coords[ind0]).normalised();
      auto v     = to_vec3(normalized_coords[ind1]).normalised();
      auto theta = acos(clamp(u.dot(v), -1.0, 1.0));
      cout << format("theta({}) = {}", msg, to_degrees(theta)) << endl;
   };

   cout << message << endl;
   for(auto y = 0u; y < ny; ++y)
      process(format("row={}", y), (y + 0) * nx - 0, (y + 1) * nx - 1);
   cout << endl;
   for(auto x = 0u; x < nx; ++x)
      process(format("col={}", x), x, (ny - 1) * nx + x);
   cout << endl;
}

// -------------------------------------------------------------- render-splines

static ARGBImage make_gt_calib_image(const vector<Vector3r>& corners,
                                     const vector<Vector3r>& gt_corners,
                                     const string_view image_fname,
                                     const cv::Mat& image,
                                     const Vector2& C)
{
   if(corners.size() != gt_corners.size())
      FATAL(format("{} != {}", corners.size(), gt_corners.size()));
   Expects(corners.size() == gt_corners.size());

   cv::Mat mat = image; // load_cv_image(image_fname);
   auto im     = render_detected_corner_grid(corners, mat);
   render_string(im, image_fname, Point2(20, 20), k_yellow, k_black);

   fill_circle(im, to_pt2(C.round()), k_magenta, 3);

   for(auto i = 0u; i < corners.size(); ++i) {
      auto pt  = to_pt2(Vector2(corners[i](0), corners[i](1) + 16.0));
      auto X   = to_vec3(gt_corners[i]);
      auto msg = format("{}, {}, {}", X.x, X.y, X.z);
      render_string(im, msg, pt, k_cyan, k_black);
   }

   return im;
}

// --------------------------------------------------------- parse-manifest-file

struct ManifestData
{
   bool has_error{false};
   vector<string> image_fnames;
   vector<cv::Mat> images;
   vector<Vector3> measurements; // X, Y, Z
};

static real accept_real(std::string& line, size_t& pos) noexcept(false)
{
   const auto len = line.size();
   if(pos >= len)
      throw std::runtime_error("failed to parse floating point number");

   const char* str = &line[pos];
   char* end{nullptr};
   errno    = 0;
   real val = strtod(str, &end);
   if(errno != 0)
      throw std::runtime_error("failed to parse floating point number");
   pos += size_t(end - str);
   return val;
}

static ManifestData parse_manifest_file(const string& manifest_fname)
{
   ManifestData dat;

   const bool is_s3 = is_s3_uri(manifest_fname);

   auto line_no = 1; // lines in files are '1'-indexed
   try {             // ---- Parse the manifest file
      string raw_data;
      if(is_s3)
         s3_load(manifest_fname, raw_data);
      else
         file_get_contents(manifest_fname, raw_data);
      std::stringstream infile(raw_data);

      for(string line; std::getline(infile, line); ++line_no) {
         trim(line);
         if(line.size() == 0) continue;
         if(line[0] == '#') continue;

         auto pos         = static_cast<size_t>(line.find_first_of(" \t"));
         auto measurement = Vector3::nan();

         if(pos == string::npos) {
            dat.image_fnames.push_back(line);
         } else {
            auto fname = line.substr(0, pos);
            dat.image_fnames.push_back(fname);
            for(auto i = 0; i < 3; ++i) measurement(i) = accept_real(line, pos);
         }

         dat.measurements.push_back(measurement);
      }
   } catch(std::exception& e) {
      print(g_radioactive,
            g_default,
            format("error processing manifest file line #{}: {}",
                   line_no,
                   e.what()));
      dat.has_error = true;
   }

   auto directory = is_s3_uri(manifest_fname) ? s3_dirname(manifest_fname)
                                              : dirname(manifest_fname);
   dat.images.resize(dat.image_fnames.size());
   for(auto i = 0u; i < dat.image_fnames.size(); ++i) {
      auto fname = dat.image_fnames[i];

      if(starts_with(fname, "/"s) or is_s3_uri(fname)) {
         ; // fname is good
      } else {
         fname = format("{}/{}", directory, fname);
      }

      cv::Mat im;
      bool has_error = false;
      try {
         if(is_s3_uri(fname)) {
            im = load_cv_image(fname);
         } else if(!is_regular_file(fname)) {
            print(g_radioactive,
                  g_default,
                  format("error processing manifest file: "
                         "file not found, '{}'",
                         fname));
            has_error = true;
         } else {
            im = load_cv_image(fname);
         }
      } catch(std::exception& e) {
         print(g_radioactive,
               g_default,
               format("error processing manifest file, "
                      "failed to load image '{}': {}",
                      fname,
                      e.what()));
         has_error = true;
      }

      if(has_error) { dat.has_error = true; }

      print(g_tick, g_default, format("loaded {}", fname));

      dat.image_fnames[i] = fname;
      dat.images[i]       = im;
   }

   return dat;
}

// ------------------------------------------------------- lookup-rig-correction

static Vector3 lookup_rig_correction()
{
   // This gets us to the bottom-left corner of the calib grid.
   // @see Camera Calibration Procedures (google-docs)
   // auto rig_correction = Vector3{0.000, -0.037, 0.010};
   auto rig_correction = Vector3{0.000, -0.037, -0.010};
   return rig_correction;
}

// ------------------------------------------------------------- run calibration

// X-axis right (away from window/wall)
// Y-axis upwards (away from table)
// Z-axis forward (from camera)
static vector<Vector3r> calc_grid1_ground_truth(const unsigned nx,
                                                const unsigned ny,
                                                const Vector3& measurement,
                                                const real grid_size)
{
   const int M = int(nx * ny);
   const Vector3 dx(grid_size, 0, 0);
   const Vector3 dy(0, grid_size, 0);

   // The bottom-left corner of the board
   const auto GBL = Vector3(measurement.x, -measurement.y, measurement.z);

   const auto G00 = (GBL - real(nx) * dx - real(ny) * dy);

   // Calculate the grid points
   vector<Vector3r> gt;
   gt.reserve(size_t(M));
   for(auto row = 0u; row < ny; ++row)
      for(auto col = 0u; col < nx; ++col)
         gt.push_back(to_vec3r(G00 + real(row) * dy + real(col) * dx));

   return gt;
}

// ---------------------------------------------------------------- save gt-json

struct CalibrationData
{
   DistortionModel model;
   vector<Vector2> gt_hull;
   real hfov      = 0.0;
   real vfov      = 0.0;
   real est_f     = 400.0;
   real final_err = 0.0;
   bool success   = false;
};

// ---------------------------------------------------------------- save gt-json

static bool save_model(const CalibrationData& dat,
                       const string& out_file,
                       const DataSource out_source)
{
   bool success = false;

   vector<string> row1, row2;
   const auto& A = dat.model.A();
   for(int c = 0; c < A.cols(); ++c) {
      row1.push_back(str_precise(A(0, c)));
      row2.push_back(str_precise(A(1, c)));
   }

   try {
      if(out_source == DataSource::DEFAULT) {
         if(out_file == "")
            throw std::runtime_error("failed to specify an output file!");
         save(dat.model, out_file);
         print(g_tick, g_default, format("calibration saved to {}", out_file));
      } else {
         store(dat.model, dat.model.sensor_id(), out_source);
         auto path = resolve_key(
             AssetType::DISTORTION_MODEL, dat.model.sensor_id(), out_source);
         print(g_tick, g_default, format("calibration saved to {}", path));
      }
      success = true;
   } catch(std::exception& e) {
      print(g_radioactive,
            g_default,
            format("error saving calibration: {}", e.what()));
   }

   return success;
}

// --------------------------------------------------------------- Calculate fov

template<typename T>
static std::pair<real, real> calc_fov(std::vector<Vector2> gt_hull,
                                      const T& model)
{
   // ---- Calculate the field-of-view
   auto C = std::accumulate(gt_hull.begin(), gt_hull.end(), Vector2(0, 0));
   C /= real(gt_hull.size());

   auto aabb = AABB::minmax();
   for(const auto& X : gt_hull) aabb.union_point(X);

   // Get the horizontal field of view
   auto get_fov = [&](const Vector2& a, const Vector2& b) {
      auto ray0 = homgen_R2_to_P2(model.undistort(a)).normalized();
      auto ray1 = homgen_R2_to_P2(model.undistort(b)).normalized();
      return acos(clamp(dot(ray0, ray1), -1.0, 1.0));
   };

   real hfov = get_fov(Vector2(aabb.left, C.y), Vector2(aabb.right, C.y));
   real vfov = get_fov(Vector2(C.x, aabb.top), Vector2(C.x, aabb.bottom));

   return std::make_pair(hfov, vfov);
}

// ------------------------------------------------------------- find model0 dxy

template<unsigned K> struct InitialResult
{
   PolynomialModel<K> model0;
   real est_f   = 400.0;
   real err     = 0.0;
   bool success = false;
};

template<unsigned K>
static void calc_opt_initial_model(const vector<Vector2>& Ds,
                                   const vector<Vector2>& Us,
                                   const real est_f,
                                   const Vector2& radial_epipole,
                                   const real average_radial_dist,
                                   InitialResult<K>& ret)
{
   ret.est_f = est_f;
   ret.err
       = ret.model0.estimate_LLS(Ds, Us, radial_epipole, average_radial_dist);
   ret.success = ret.err * est_f < 6.0;
}

// -------------------------------------------------------- Apply gt-corrections

static void apply_gt_corrections(const vector<vector<Vector3r>>& gtUs,
                                 const vector<Vector2>& corrections,
                                 vector<vector<Vector3r>>& out)
{
   Expects(gtUs.size() == corrections.size());
   out.resize(gtUs.size());

   for(auto i = 0u; i < gtUs.size(); ++i) {
      auto X = Vector3r(corrections[i].x, corrections[i].y, 1.0);
      out[i].resize(gtUs[i].size());
      for(auto j = 0u; j < out[i].size(); ++j) { out[i][j] = gtUs[i][j] + X; }
   }
}

// ---------------------------------------------------------------- Optimize LLS

template<unsigned K>
static void optimize_LLS(const unsigned nx,
                         const unsigned ny,
                         const vector<vector<Vector3r>>& gDs,
                         const vector<vector<Vector3r>>& gUs,
                         const vector<unsigned>& inds, //
                         vector<Vector2>& dxys,
                         const real est_f,
                         const Vector2& radial_epipole,
                         const real average_radial_dist,
                         const bool use_nelder_mead,
                         InitialResult<K>& ret,
                         const bool feedback)
{
   const auto N        = unsigned(inds.size());
   const auto n_params = 2 * N;
   vector<Vector2> Ds, Us;
   const Vector2 dxy0{0.0, 0.0};
   auto f = [&](const Vector3r& x, const Vector2& dxy) {
      return homgen_P2_to_R2(Vector3(x(0) + dxy.x, x(1) + dxy.y, x(2)));
   };

   if(dxys.size() != N) {
      dxys.resize(N);
      std::fill(begin(dxys), end(dxys), Vector2{0.0, 0.0});
   } else {
      // USE whatever data is in dxys (corrections) already
   }

   auto pack = [&](real* X) {
      for(auto n = 0u; n < N; ++n) {
         *X++ = dxys[n](0);
         *X++ = dxys[n](1);
      }
   };

   auto unpack = [&](const real* X) {
      Expects(dxys.size() == N);
      for(auto n = 0u; n < N; ++n) {
         dxys[n](0) = *X++;
         dxys[n](1) = *X++;
      }

      Ds.clear();
      Us.clear();
      for(auto n = 0u; n < N; ++n) {
         for(const auto& D : gDs[n]) Ds.push_back(f(D, dxy0));
         for(const auto& U : gUs[n]) Us.push_back(f(U, dxys[n]));
      }
   };

   auto counter  = 0;
   auto best_err = std::numeric_limits<real>::max();
   auto fn       = [&](const real* X) {
      unpack(X);
      calc_opt_initial_model(
          Ds, Us, est_f, radial_epipole, average_radial_dist, ret);
      // We also want the radial epipole to go to (0, 0)
      auto ppt     = ret.model0.undistort(radial_epipole);
      auto ppt_err = est_f * (ppt - dxy0).norm();
      auto theta_err
          = 1.0 * to_degrees(model_theta_err(nx, ny, gUs, ret.model0, gDs));
      auto err = ret.err * est_f + ppt_err + theta_err;

      if(feedback) {
         if(err < best_err) {
            best_err = err;
            cout << format("#{:4d}, err={:7.5f} + {:7.5f} + {:7.5f} = {:7.5f} "
                           "ep={}",
                           counter,
                           est_f * ret.err,
                           ppt_err,
                           theta_err,
                           err,
                           str(ppt))
                 << endl;
         }
         ++counter;
      }
      return err;
   };

   vector<real> start(n_params);
   vector<real> xmin(n_params);
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-7;
   real diffstep = 0.1;
   int kcount    = 1000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method = nullptr;

   pack(&start[0]);
   ystartlo = fn(&start[0]);

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
      auto simplex_factor = 0.05;
      for(unsigned i = 0; i < n_params; ++i)
         step[i]
             = fabs(start[i]) > 1e-3 ? fabs(simplex_factor * start[i]) : 1e-3;
      nelder_mead(fn,
                  n_params,
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  20 * kcount,
                  icount,
                  numres,
                  ifault);
   }

   unpack(&xmin[0]);

   if(feedback) {
      INFO(format("Feedback for finding dxys on {} images", N));
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;
      for(auto i = 0u; i < inds.size(); ++i) {
         cout << format(
             "    #{:2d} dxy = {:7.4f}, {:7.4f}", inds[i], dxys[i].x, dxys[i].y)
              << endl;
      }
      cout << endl;
   }
}

// ----------------------------------------------------- run gt-only calibration

static CalibrationData run_distort_calibration_(const unsigned nx,
                                                const unsigned ny,
                                                const real grid_size,
                                                const string& sensor_id,
                                                const string& manifest_file,
                                                const bool fast_calibration,
                                                const string& out_file,
                                                const DataSource out_source,
                                                const string& out_dir)
{
   CalibrationData dat;

   bool success     = false;
   const auto est_f = 400.0;

   // ---- Print up a nice banner
   cout << endl
        << format(" \x1b[42m\x1b[97m "
                  "\u265f \u265e \u265d \u265c \u265b \u265a"
                  " -- \x1b[1m"
                  "calibration"
                  "\x1b[0m\x1b[42m\x1b[97m -- "
                  "\u265a \u265b \u265c \u265d \u265e \u265f"
                  " \x1b[0m")
        << endl;
   print(g_info, g_light_gray, format("nx, ny         = [{}x{}]", nx, ny));
   print(g_info, g_light_gray, format("grid size      = {}", grid_size));
   print(g_info, g_light_gray, format("manifest file  = {}", manifest_file));
   print(g_info, g_light_gray, format("output file    = {}", out_file));
   print(g_info, g_light_gray, format("output dir     = {}", out_dir));

   // ---- Get the rig-correction
   const auto correction = lookup_rig_correction();
   print(g_info, g_light_gray, format("rig correction = {}", str(correction)));
   print(g_info, g_light_gray, format("assumed f      = {}", est_f));

   // ---- Parse the manifest file
   auto manifest = parse_manifest_file(manifest_file);
   if(manifest.has_error) return dat;
   const auto manifest_N = manifest.image_fnames.size();
   print(g_info, g_light_gray, format("n images       = {}", manifest_N));
   if(manifest_N == 0) {
      print(g_radioactive, g_error, format("expected at least 1 input image"));
      return dat;
   }

   // ---- Get width and height
   unsigned w{0};
   unsigned h{0};
   {
      const auto& im = manifest.images[0];
      w              = unsigned(im.cols);
      h              = unsigned(im.rows);
      print(g_info, g_light_gray, format("image format   = [{}x{}]", w, h));
   }

   // ---- Find ground-truth corners
   vector<vector<Vector3r>> gt_corners;
   {
      gt_corners.resize(manifest_N);
      for(auto n = 0u; n < manifest_N; ++n) {
         if(manifest.measurements[n].is_finite()) {
            const auto m  = manifest.measurements[n] - correction;
            gt_corners[n] = calc_grid1_ground_truth(nx, ny, m, grid_size);
         }
      }
   }

   // ---- Get the corner grids
   vector<vector<Vector3r>> corners;
   {
      corners.resize(manifest_N);
      for(auto n = 0u; n < manifest_N; ++n) {
         if(gt_corners[n].size() == 0) continue;
         const auto& fname = manifest.image_fnames[n];
         const auto& im    = manifest.images[n];
         print(g_dotted_sq,
               g_default,
               format("finding corners in {}...", basename(fname)),
               false);
         try {
            auto s = time_thunk(
                [&]() { corners[n] = find_nx_ny_grid(nx, ny, im); });

            cout << format(" success, {} seconds", s) << endl;
         } catch(std::runtime_error& e) {
            cout << format(" failure: {}", e.what()) << endl;
         }
      }
   }

   // ---- Get the subset of images with ground-truth and corners
   vector<vector<Vector3r>> gtDs;
   vector<vector<Vector3r>> gtUs;
   vector<string> image_filenames;
   vector<cv::Mat> images;
   vector<unsigned> inds;
   {
      for(auto n = 0u; n < manifest_N; ++n) {
         if(corners[n].size() > 0 && gt_corners[n].size() > 0) {
            Expects(corners[n].size() == nx * ny);
            Expects(gt_corners[n].size() == nx * ny);
            gtDs.push_back(corners[n]);
            gtUs.push_back(gt_corners[n]);
            image_filenames.push_back(manifest.image_fnames[n]);
            images.push_back(manifest.images[n]);
            inds.push_back(unsigned(image_filenames.size()) - 1);
         } else {
            inds.push_back(unsigned(-1));
         }
      }

      if(gtDs.size() < 3) {
         print(g_radioactive,
               g_error,
               format("failed to find at least 3 ground-truth images "
                      "with corners, aborting"));
         return dat;
      }
   }
   // auto crUs = gtUs; // corrected
   const auto gt_N = gtDs.size();
   const auto N    = gt_N;

   // ---- Get the radial epipole and scale for conditioning input data
   Vector2 radial_epipole{0.0, 0.0};
   real average_radial_dist{0.0}; // for conditioning
   vector<Vector3r> W;
   {
      W              = make_W_pattern(nx, ny);
      radial_epipole = estimate_distortion_center(gtDs, W, false);

      // average_radial_dist is for conditioning points
      auto sum = 0.0;
      for(const auto& img : gtDs)
         for(const auto& X : img)
            sum += (homgen_P2_to_R2(to_vec3(X)) - radial_epipole).norm();
      average_radial_dist = sum / real(N * W.size());

      print(g_bullet,
            g_default,
            format("radial-center  = {}", str(radial_epipole)));
      print(g_bullet,
            g_default,
            format("av-radial-dist = {}", average_radial_dist));
   }

   // ---- Output some nice images to show the result so far
   {
      for(auto n = 0u; n < N; ++n) {
         auto argb = make_gt_calib_image(
             gtDs[n], gtUs[n], image_filenames[n], images[n], radial_epipole);
         argb.save(format("{}/image-{:2d}_01_gt.png", out_dir, n));
      }
   }

   // ---- Calculate the convex hull of ground-truth corners
   vector<Vector2> gt_hull;
   {
      vector<Vector2> Ds;
      for(const auto& im : gtDs)
         for(const auto& X : im) Ds.emplace_back(X(0) / X(2), X(1) / X(2));
      andrews_convex_hull(Ds.begin(), Ds.end(), gt_hull);

      ARGBImage argb;
      argb.resize(w, h, w);
      argb.zero();

      for(auto i = 0u; i < gt_hull.size(); ++i)
         plot_line_AA(gt_hull[i],
                      gt_hull[(i + 1) % gt_hull.size()],
                      [&](int x, int y, float a) {
                         if(argb.in_bounds(x, y)) {
                            argb(x, y) = blend(k_white, argb(x, y), a);
                         }
                      });

      for(const auto& im : gtDs)
         for(const auto& X : im)
            fill_circle(
                argb, to_pt2(Vector2(X(0) / X(2), X(1) / X(2))), k_red, 3);

      argb.save(format("{}/field-of_view.png", out_dir));
   }

   vector<Vector2> gt_corrections;
   InitialResult<DistortionModel::K> m0;

   // ---- Get corrections
   {
      vector<unsigned> selected;
      selected.resize(N);
      std::iota(begin(selected), end(selected), 0);
      print(g_bullet,
            g_default,
            format("estimating measurement initial model..."),
            false);

      auto s = time_thunk([&]() {
         optimize_LLS(nx,
                      ny,
                      gtDs,
                      gtUs,
                      selected,
                      gt_corrections,
                      est_f,
                      radial_epipole,
                      average_radial_dist,
                      false,
                      m0,
                      false); // feedback
      });
      cout << format(" {} seconds", s) << endl;

      // INFO("---");
      // print_gt_angles("printing gtUs[0]", nx, ny, gtUs[0],
      //                 [&] (Vector3r x) { return x; });
      // print_gt_angles("printing gtDs[0]", nx, ny, gtDs[0],
      //                 [&] (Vector3r x) { return m0.model0.undistort(x); });

      print(g_bullet,
            g_default,
            format("initial LLS error is {}", m0.err * est_f));

      if(m0.err * est_f > 6.0) {
         print(g_radioactive,
               g_error,
               format("initial LLS error is too high, aborting"));
         return dat;
      }

      // apply_gt_corrections(gtUs, gt_corrections, crUs);
   }

   // ----
   vector<Vector6r> r1r2ts;
   DistortionModel model0;
   auto mid_err = 0.0;
   {
      model0.init(
          sensor_id, w, h, radial_epipole, average_radial_dist, gt_hull);

      vector<Vector2> Ds, Us;
      auto f = [&](const Vector3r& x) {
         return homgen_P2_to_R2(Vector3(x(0), x(1), x(2)));
      };
      for(auto n = 0u; n < N; ++n) {
         for(const auto& D : gtDs[n]) Ds.push_back(f(D));
         for(const auto& U : gtUs[n]) Us.push_back(f(U));
      }
      model0.estimate_LLS(Ds, Us, radial_epipole, average_radial_dist);

      for(auto counter = 0; counter < 20; ++counter) {
         print(g_wedge,
               g_default,
               format("estimation iteration {:2d}...", counter),
               false);

         // Get the latest rotation estimation
         auto undistort = [&](const Vector2& x) { return model0.undistort(x); };
         r1r2ts         = calc_r1r2ts(W, gtDs, undistort);

         // Marshall Drs and Urs arguments
         Matrix3r H;
         auto Hx = [&](const Vector3r& x) { return normalized_P2(H * x); };

         vector<Vector3r> Drs, Urs;
         for(auto n = 0u; n < N; ++n) {
            r1r2t_to_H(r1r2ts[n], H);
            std::transform(W.begin(), W.end(), std::back_inserter(Urs), Hx);
            Drs.insert(Drs.end(), gtDs[n].begin(), gtDs[n].end());
         }
         Expects(Drs.size() == Urs.size());

         // Empty line data
         using LLSLineData = decltype(model0)::LLSLineData;
         vector<LLSLineData> ldata;

         // Finally do the error estimation
         mid_err = model0.estimate_LLS_line_and_pts(ldata, Drs, Urs);

         auto theta_err = model_theta_err(nx, ny, gtUs, model0, gtDs);

         cout << format(" point err = {:12.10f}, \u03b8 = {:7.5f}",
                        mid_err * est_f,
                        to_degrees(theta_err))
              << endl;
      }
   }

   if(fast_calibration) {
      real hfov = 0.0, vfov = 0.0;
      { // ---- Calculate the field-of-view
         std::tie(hfov, vfov) = calc_fov(gt_hull, model0);
         print(g_bullet,
               g_default,
               format("estimation hfov = {} degrees, and vfov = {} degrees.",
                      to_degrees(hfov),
                      to_degrees(vfov)));
      }

      { // Build the result
         dat.model     = model0;
         dat.gt_hull   = gt_hull;
         dat.hfov      = hfov;
         dat.vfov      = vfov;
         dat.est_f     = est_f;
         dat.final_err = mid_err;
         dat.success   = true;
      }

      { // Save the model
         dat.success = save_model(dat, out_file, out_source);
      }

      // INFO("---");
      // print_gt_angles("final gtUs[0]", nx, ny, gtUs[0],
      //                 [&] (Vector3r x) { return x; });
      // print_gt_angles("final gtDs[0]", nx, ny, gtDs[0],
      //                 [&] (Vector3r x) { return model0.undistort(x); });

      return dat;
   }

   // ---- Use 'distort0' to get the inital fitted splines
   vector<vector<Spline2d>> splines;
   {
      model0.finalize();
      auto dist_f = [&](const Vector2& U) { return model0.distort(U); };

      print(g_bullet, g_default, format("finding initial splines..."), false);

      auto s = time_thunk([&]() {
         splines.resize(N);
         for(auto n = 0u; n < N; ++n) {
            splines[n] = find_initial_splines(nx, ny, r1r2ts[n], dist_f);
            auto argb  = render_splines(images[n], splines[n]);
            argb.save(
                format("{}/image-{:2d}_02_initial-splines.png", out_dir, n));
         }
      });

      cout << format(" success, {} seconds", s) << endl;
   }

   // ---- Make those splines super-accurate
   {
      for(auto n = 0u; n < N; ++n) {
         print(g_wave,
               g_default,
               format("refining spline fits {:2d}/{}...", n + 1, N),
               false);
         auto s = time_thunk([&]() {
            accurate_spline_fit(images[n], splines[n], false, 7);
            auto argb = render_splines(images[n], splines[n], 0);
            argb.save(
                format("{}/image-{:2d}_03_accurate-splines.png", out_dir, n));
         });
         cout << format(" success, {} seconds", s) << endl;
      }
   }

   // ---- Calculate spline-spline intersections
   vector<vector<Vector3r>> ss_isects;
   {
      print(g_cross,
            g_default,
            format("calculating spline-spline intersections..."),
            false);

      auto s = time_thunk([&]() {
         ss_isects.resize(N);
         for(auto n = 0u; n < N; ++n) {
            for(auto row = 0u; row < ny; ++row) {
               for(auto col = 0u; col < nx; ++col) {
                  auto& rs = splines[n][row];      // row and column
                  auto& cs = splines[n][ny + col]; // splines
                  auto d   = spline_spline_intersection(rs, cs, 1e-4);
                  ss_isects[n].emplace_back(d.x, d.y, 1.0);
               }
            }
         }
      });
      cout << format(" success, {} seconds", s) << endl;
   }

   // ---- Final estimation sequence
   DistortionModel model;
   auto final_err = 0.0;
   {
      model.init(sensor_id, w, h, radial_epipole, average_radial_dist, gt_hull);

      using LLSLineData         = decltype(model)::LLSLineData;
      const auto n_pts_per_line = 30u;

      for(auto counter = 0; counter < 20; ++counter) {
         print(g_wedge,
               g_default,
               format("estimation iteration {:2d}...", counter),
               false);

         // Get the latest rotation estimation
         if(counter > 0) {
            auto undistort
                = [&](const Vector2& x) { return model.undistort(x); };
            r1r2ts = calc_r1r2ts(W, ss_isects, undistort);
         }

         // Marshall Drs and Urs arguments
         Matrix3r H;
         auto Hx = [&](const Vector3r& x) { return normalized_P2(H * x); };

         vector<Vector3r> Drs, Urs;
         for(auto n = 0u; n < N; ++n) {
            r1r2t_to_H(r1r2ts[n], H);
            std::transform(W.begin(), W.end(), std::back_inserter(Urs), Hx);
            Drs.insert(Drs.end(), ss_isects[n].begin(), ss_isects[n].end());
         }
         Expects(Drs.size() == Urs.size());

         // Marshall line data
         vector<LLSLineData> ldata;
         const auto n_pts_per_line = 30u;
         for(auto n = 0u; n < N; ++n) {
            r1r2t_to_H(r1r2ts[n], H);

            for(auto ind = 0u; ind < nx + ny; ++ind) {
               ldata.emplace_back();
               auto& dat = ldata.back();

               // Get the line equation
               vector<unsigned> linds;
               calc_line_inds(nx, ny, ind, linds);
               dat.line_eq
                   = to_homgen_line(Hx(W[linds.front()]), Hx(W[linds.back()]));

               // Get the spline points
               const auto& spline = splines[n][ind];
               const auto dt      = 1.0 / real(n_pts_per_line - 1);
               for(auto t = 0.0; t <= 1.0; t += dt) {
                  auto x = spline.evaluate(t);
                  dat.Xs.emplace_back(x(0), x(1), 1.0);
               }
            }
         }

         // Finally do the error estimation
         final_err = model.estimate_LLS_line_and_pts(ldata, Drs, Urs);

         cout << format(" err = {:12.10f}", final_err * est_f) << endl;
      }
   }

   real hfov = 0.0, vfov = 0.0;
   { // ---- Calculate the field-of-view
      std::tie(hfov, vfov) = calc_fov(gt_hull, model);
      print(g_bullet,
            g_default,
            format("estimation hfov = {} degrees, and vfov = {} degrees.",
                   to_degrees(hfov),
                   to_degrees(vfov)));
   }

   { // Build the result
      dat.model     = model;
      dat.gt_hull   = gt_hull;
      dat.hfov      = hfov;
      dat.vfov      = vfov;
      dat.est_f     = est_f;
      dat.final_err = final_err;
      dat.success   = true;
   }

   { // Save the model
      dat.success = save_model(dat, out_file, out_source);
   }

   return dat;
}

// ----------------------------------------------------- run gt-only calibration

bool run_distort_calibration(const unsigned nx,
                             const unsigned ny,
                             const real grid_size,
                             const string& sensor_id,
                             const string& manifest_file,
                             const bool fast_calibration,
                             const string& out_file,
                             const string& out_dir)
{
   auto dat = run_distort_calibration_(nx,
                                       ny,
                                       grid_size,
                                       sensor_id,
                                       manifest_file,
                                       fast_calibration,
                                       out_file,
                                       DataSource::DEFAULT,
                                       out_dir);
   return dat.success;
}

bool run_distort_calibration(const unsigned nx,
                             const unsigned ny,
                             const real grid_size,
                             const string& sensor_id,
                             const string& manifest_file,
                             const bool fast_calibration,
                             const DataSource out_source,
                             const string& out_dir)
{
   auto dat = run_distort_calibration_(nx,
                                       ny,
                                       grid_size,
                                       sensor_id,
                                       manifest_file,
                                       fast_calibration,
                                       "",
                                       out_source,
                                       out_dir);
   return dat.success;
}

} // namespace perceive::calibration
