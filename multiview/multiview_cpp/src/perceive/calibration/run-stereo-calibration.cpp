
#include "run-stereo-calibration.hpp"
#include "stdinc.hpp"

#include "calibration-utils.hpp"
#include "find-F.hpp"
#include "find-K.hpp"
#include "fit-curve-to-checkerboard.hpp"
#include "proposals.hpp"

#include "perceive/calibration/fit-curve-to-checkerboard.hpp"
#include "perceive/calibration/sharpness-metric.hpp"
#include "perceive/foundation.hpp"
#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/camera.hpp"
#include "perceive/geometry/projective/polynomial-model.hpp"
#include "perceive/geometry/projective/triangulation.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/cv-helpers.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace perceive::calibration
{
// --------------------------------------------------------------------- structs

struct ImageCorners
{
   string filename;
   unsigned frame_no{0};
   unsigned out_counter{0};
   vector<Vector3r> im[2];       // two images, cam0 and cam1
   vector<Vector3r> accurate[2]; // two images, cam0 and cam1
   real sharpness{0.0};
   Vector6r r1r2t[2];
   vector<Spline2d> splines[2];
};

struct CornersJob
{
   CornersJob(const DistortionModel& M0_,
              const DistortionModel& M1_,
              const cv::Mat& dual_im_,
              const string& fname,
              const int frame_no_,
              const int counter,
              const bool use_splines_,
              const int nx_,
              const int ny_,
              const int w_,
              const int h_,
              const real time_threshold_,
              const string& out_dir_)
       : M0(M0_)
       , M1(M1_)
       , dual_im(dual_im_)
       , filename(fname)
       , frame_no(frame_no_)
       , out_counter(counter)
       , use_splines(use_splines_)
       , nx(unsigned(nx_))
       , ny(unsigned(ny_))
       , w(w_)
       , h(h_)
       , time_threshold(time_threshold_)
       , out_dir(out_dir_)
   {}

   const DistortionModel& M0;
   const DistortionModel& M1;

   cv::Mat dual_im;
   string filename{""};
   int frame_no{0};
   int out_counter{0}; // also functions as a job-id

   bool use_splines{false};
   unsigned nx{0};
   unsigned ny{0};
   int w{0};
   int h{0};
   real time_threshold{5.0};
   string out_dir{"/tmp"};

   bool has_corners{false};
   ImageCorners corners;

   string out_message{""};
   string err_message{""};
};

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
static const auto g_dot_line     = "\x1b[96m\u2504\x1b[0m"s;
static const auto g_lines        = "\x1b[96m\u2632\x1b[0m"s;

static const auto g_default    = "\x1b[0m"s;
static const auto g_red        = "\x1b[31m"s;
static const auto g_error      = "\x1b[4m\x1b[91m"s;
static const auto g_light_gray = "\x1b[37m"s;
static const auto g_light_red  = "\x1b[91m"s;
static const auto g_white      = "\x1b[97m"s;

static const auto g_obnoxious = "\x1b[1m\x1b[4m\x1b[5m\x1b[106m\x1b[91m"s;

// ---------------------------------------------------

static void finish_corners(const cv::Mat& im0,
                           const cv::Mat& im1,
                           const unsigned frame_no,
                           const unsigned nx,
                           const unsigned ny,
                           const DistortionModel& model0,
                           const DistortionModel& model1,
                           const string& out_dir,
                           const bool use_splines,
                           ImageCorners& corners)
{
   array<std::function<Vector2(const Vector2&)>, 2> distort{
       {model0.distort_f(), model1.distort_f()}};

   array<std::function<Vector2(const Vector2&)>, 2> undistort{
       {model0.undistort_f(), model1.undistort_f()}};

   auto find_region = [&](const vector<Vector3r>& corners) {
      // We want to inscribe an axis-aligned bounding box,
      // but that algorithm would take me half a day to write.
      // So we'll ape something approximate...
      auto aabb = AABB::minmax();
      for(const auto& X : corners) aabb.union_point(X(0), X(1));

      // Now take the center 50%
      cv::Rect roi;
      roi.x      = int(aabb.left + 0.25 * aabb.width());
      roi.y      = int(aabb.top + 0.25 * aabb.height());
      roi.width  = int(aabb.width() * 0.5);
      roi.height = int(aabb.height() * 0.5);
      return roi;
   };

   { // Get the sharpness
      cv::Mat crop0     = im0(find_region(corners.im[0]));
      cv::Mat crop1     = im1(find_region(corners.im[1]));
      auto sharp0       = sharpness_metric(crop0);
      auto sharp1       = sharpness_metric(crop1);
      corners.sharpness = std::min(sharp0, sharp1);
   }

   { // Save the composite image showing corner detections
      ARGBImage argb0 = render_detected_corner_grid(corners.im[0], im0);
      ARGBImage argb1 = render_detected_corner_grid(corners.im[1], im1);
      ARGBImage argb  = hcat(argb0, argb1);
      argb.save(
          format("{}/cv-corners_{:4d}.png", out_dir, corners.out_counter));

      if(false) {
         WARN(format("corners.out_counter = {}", corners.out_counter));
         for(auto i = 0u; i < corners.im[0].size(); ++i) {
            cout << format(
                "{}: {}, {}", i, str(corners.im[0][i]), str(corners.im[1][i]))
                 << endl;
         }
      }
   }

   if(use_splines) { // get r1r2ts
      auto W = make_W_pattern(nx, ny);
      for(size_t i = 0; i < 2; ++i) {
         vector<vector<Vector3r>> Ds(1);
         Ds[0]            = corners.im[i];
         auto r1r2ts      = calc_r1r2ts(W, Ds, undistort[i]);
         corners.r1r2t[i] = r1r2ts[0];
      }
   }

   if(use_splines) { // get splines
      for(size_t i = 0; i < 2; ++i) {
         auto av_sz       = average_grid_size(nx, ny, corners.im[i]);
         auto max_blur_sz = unsigned(ceil(av_sz / 3));
         if(max_blur_sz % 2 == 0) max_blur_sz += 1;
         if(max_blur_sz < 3u) max_blur_sz = 3u;
         if(max_blur_sz > 31u) max_blur_sz = 31u;
         auto max_blur = av_sz / 6.0;
         unsigned n_fits
             = unsigned(std::floor(std::log(max_blur) / std::log(2)));
         if(n_fits < 1) n_fits = 1;
         if(n_fits > 7) n_fits = 7;

         auto& im      = (i == 0 ? im0 : im1);
         auto& splines = corners.splines[i];
         splines = find_initial_splines(nx, ny, corners.r1r2t[i], distort[i]);
         accurate_spline_fit(im, splines, false, n_fits, max_blur, max_blur_sz);
         auto argb = render_splines(im, splines, 0);
         argb.save(format(
             "{}/image-%02d_{}_03_accurate-splines.png", out_dir, i, frame_no));
      }
   }

   // ---- Calculate spline-spline intersections
   if(use_splines) {
      for(size_t n = 0; n < 2; ++n) {
         const auto& splines = corners.splines[n];
         for(auto row = 0u; row < ny; ++row) {
            for(auto col = 0u; col < nx; ++col) {
               auto& rs = splines[row];      // row and column
               auto& cs = splines[ny + col]; // splines
               auto d   = spline_spline_intersection(rs, cs, 1e-4);
               corners.accurate[n].emplace_back(d.x, d.y, 1.0);
            }
         }
      }
   } else {
      for(auto i = 0; i < 2; ++i) corners.accurate[i] = corners.im[i];
   }
}

// --------------------------------------------------------------- process-frame

static void process_corners_job(CornersJob& job)
{
   std::stringstream ss{""};

   auto& corners       = job.corners;
   corners.filename    = job.filename;
   corners.frame_no    = unsigned(job.frame_no);
   corners.out_counter = unsigned(job.out_counter);

   const auto& dual_im       = job.dual_im;
   const auto& w             = job.w;
   const auto& h             = job.h;
   const auto nx             = job.nx;
   const auto ny             = job.ny;
   const auto& model0        = job.M0;
   const auto& model1        = job.M1;
   const auto time_threshold = job.time_threshold;
   const auto frame_no       = job.frame_no;
   const auto use_splines    = job.use_splines;
   const auto& out_dir       = job.out_dir;

   job.has_corners = false;
   auto& err_msg   = job.err_message;

   // Make sure we're dealing with greyscale data
   const cv::Mat* im = &dual_im;
   cv::Mat grey_data;
   if(dual_im.channels() > 1) {
      cvtColor(dual_im, grey_data, cv::COLOR_BGR2GRAY);
      im = &grey_data;
   }

   // Split the image
   cv::Mat im0, im1;
   hsplit(*im, im0, im1);
   job.dual_im.release(); // we're done with this memory

   // Save 'w' and 'h' for prosterity
   if(int(w) != im0.cols || int(h) != im0.rows) {
      err_msg = format("expects frame of size [{}x{}], but got [{}x{}]",
                       w,
                       h,
                       im0.cols,
                       im0.rows);
      return;
   }

   if(false) {
      cv::imwrite(format("/tmp/{:4d}-l.png", job.out_counter), im0);
      cv::imwrite(format("/tmp/{:4d}-r.png", job.out_counter), im1);
   }

   // Get the corner grids
   bool has_corners = false;
   try {
      const bool fast_check = false;
      auto s1               = time_thunk([&]() {
         corners.im[0] = find_nx_ny_grid(nx, ny, im0, fast_check);
         corners.im[1] = find_nx_ny_grid(nx, ny, im1, fast_check);
      });
      has_corners           = (corners.im[0].size() == nx * ny)
                    and (corners.im[1].size() == nx * ny);
      if(s1 < time_threshold) {
         has_corners = true;
      } else {
         ss << format(" \x1b[31mtimeout\x1b[0m");
         has_corners = false;
      }
   } catch(std::runtime_error& e) {
      // just means that we failed to detect corners on this image
      ss << format(" \x1b[31mFAIL\x1b[0m {}", e.what());
   }

   // Check that those corners are in the region of interest
   if(has_corners) {
      auto corners_in_calib_region
          = [](const DistortionModel& M, const vector<Vector3r>& Xs) {
               const auto& hull = M.calib_hull();
               return std::all_of(cbegin(Xs), cend(Xs), [&](const auto& X) {
                  auto Y = M.working_to_calib_format(X);
                  return point_in_polygon(Y, cbegin(hull), cend(hull));
               });
            };

      auto in0 = corners_in_calib_region(model0, corners.im[0]);
      auto in1 = corners_in_calib_region(model1, corners.im[1]);
      if(!in0 || !in1) {
         ss << format(" \x1b[31mFAIL\x1b[0m calib-region { {}, {} }",
                      str(in0),
                      str(in1));
         has_corners = false;
      }
   }

   if(has_corners) {
      finish_corners(im0,
                     im1,
                     unsigned(frame_no),
                     nx,
                     ny,
                     model0,
                     model1,
                     out_dir,
                     use_splines,
                     corners);
      ss << format(" \x1b[92mfound\x1b[0m");
   }

   job.out_message = ss.str();
   job.has_corners = has_corners and err_msg.empty();
}

// --------------------------------------------------- load-and-detect (corners)

static void load_and_detect(const unsigned nx,
                            const unsigned ny,
                            const string& fname,
                            DistortionModel& model0,
                            DistortionModel& model1,
                            const string& out_dir,
                            const bool fit_splines,
                            const real time_threshold,
                            unsigned& out_counter,
                            vector<ImageCorners>& out,
                            unsigned& w,
                            unsigned& h)
{
   string err_msg{""};
   auto out_initial_size = out.size();

   print(g_dotted_sq,
         g_default,
         format("detecting corners in '{}'...", basename(fname)),
         false);

   cv::Mat dual_im, im0, im1;
   cv::VideoCapture video;

   Expects(model0.calib_format() == model1.calib_format());

   std::atomic<unsigned> job_counter{0};
   const auto n_parallel_jobs = hardware_concurrency();
   std::mutex padlock;

   auto process_frame = [&](const cv::Mat& dual_im,
                            const int frame_no,
                            const bool use_splines,
                            const bool is_video) {
      const auto counter = out_counter++;

      // We need to set the working format from the first image
      if(counter == 0) {
         if(dual_im.cols % 1 != 0)
            FATAL(format("dual-image has odd number of horizontal "
                         "pixels. (i.e., cannot split image evenly.)"));
         w = unsigned(dual_im.cols / 2);
         h = unsigned(dual_im.rows);

         model0.set_working_format(w, h);
         model1.set_working_format(w, h);
         model0.finalize();
         model1.finalize();
      }

      // Wait until some jobs are processed
      while(job_counter > n_parallel_jobs) std::this_thread::yield();

      // Enqueue a job
      ++job_counter;

      CornersJob j(model0,
                   model1,
                   dual_im,
                   fname,
                   frame_no,
                   int(counter),
                   use_splines,
                   int(nx),
                   int(ny),
                   int(w),
                   int(h),
                   time_threshold,
                   out_dir);

      if(is_video) {
         auto f = [job = std::move(j), &job_counter, &padlock, &out]() mutable {
            try {
               process_corners_job(job);
               {
                  lock_guard<decltype(padlock)> lock(padlock);
                  cout << format(
                      "    {}  frame #%04d", g_dot_line, job.frame_no)
                       << job.out_message << endl;
                  if(job.has_corners) out.emplace_back(std::move(job.corners));
               }
            } catch(std::exception& e) {
               FATAL(format("uncaught exception while processing job: {}",
                            e.what()));
            } catch(...) {
               FATAL(format("uncaught exception while processing job"));
            }

            if(!job.err_message.empty())
               FATAL(
                   format("error processing corners job: {}", job.err_message));

            --job_counter;
         };

         schedule(f);
      } else {
         process_corners_job(j);
         if(j.has_corners) {
            out.emplace_back(std::move(j.corners));
         } else {
            cout << j.out_message;
         }
      }

      return true;
   };

   bool is_video = false;
   auto s        = time_thunk([&]() {
      // Try to read as an image
      dual_im = cv::imread(fname, cv::IMREAD_GRAYSCALE);
      if(!dual_im.empty()) {
         process_frame(dual_im, 0, fit_splines, false);
         is_video = false;
         return;
      }

      // Try to read as video
      if(!video.open(fname)) {
         err_msg = format("failed to read '{}' as either "
                          "an image or video file",
                          basename(fname));
         return;
      }

      is_video        = true;
      auto frame_no   = 0;
      auto frame_okay = true;
      cout << endl;
      while(video.read(dual_im) and frame_okay)
         process_frame(dual_im, frame_no++, fit_splines, true);

      while(job_counter > 0) std::this_thread::yield();
   });

   if(err_msg.empty()) {
      auto sz = out.size() - out_initial_size;
      if(sz > 0 && is_video)
         cout << format(" {} frame{}, {} seconds", sz, (sz > 1 ? "s" : ""), s)
              << endl;
      else
         cout << format(" {} seconds", s) << endl;
   } else {
      cout << endl;
      print(g_radioactive, g_default, err_msg);
   }
}

// ---------------------------------------------------------------------- find-K

static void find_K01(const unsigned nx,
                     const unsigned ny,
                     const DistortionModel& sensor0,
                     const DistortionModel& sensor1,
                     const vector<ImageCorners>& raw_corners,
                     const vector<unsigned>& proposals)
{
   vector<vector<Vector3r>> Cs0, Cs1; // corners

   for(auto i = 0u; i < proposals.size(); ++i) {
      const auto& ic = raw_corners[proposals[i]];
      Cs0.push_back(ic.accurate[0]);
      Cs1.push_back(ic.accurate[1]);
   }

   for(auto& Cs : Cs0)
      for(auto& D : Cs) D = sensor0.undistort(D);
   for(auto& Cs : Cs1)
      for(auto& D : Cs) D = sensor1.undistort(D);

   cout << format(" {} estimating homographies", g_lines) << endl << endl;

   if(false) {
      INFO("REMOVE LINES BELOW");
      cout << format("|Cs0| = {}", Cs0.size()) << endl;
      cout << format("|Cs1| = {}", Cs1.size()) << endl;
      for(auto i = 0u; i < Cs0.size(); ++i) {
         cout << format("|Cs0[{}]| = {}", i, Cs0[i].size()) << endl;
         cout << format("|Cs1[{}]| = {}", i, Cs1[i].size()) << endl;
      }
   }

   const Matrix3r K0 = estimate_K_Zhang(nx, ny, Cs0, false);
   cout << str("      K0", K0) << endl;

   const Matrix3r K1 = estimate_K_Zhang(nx, ny, Cs1, false);
   cout << str("      K1", K1) << endl;
}

// ------------------------------------------------------ run-stereo-calibration

bool run_stereo_calibration_(const unsigned nx,
                             const unsigned ny,
                             const real square_size,
                             const string& camera_id,
                             const string& out_file,
                             const DataSource out_source,
                             const string& out_dir,
                             const string& calib_fname0,
                             const string& calib_fname1,
                             const bool fit_splines,
                             const bool find_K,
                             const bool estimate_error_map,
                             const real time_threshold,
                             const vector<string>& filenames)
{
   Expects(filenames.size() > 0);
   bool success = false;
   DistortionModel sensor0, sensor1;
   unsigned w{0};
   unsigned h{0};

   // ---- Load sensor calibration data
   auto load_calib_file
       = [](DataSource source, DistortionModel& sensor, const string& fname) {
            try {
               if(source == DataSource::DEFAULT)
                  load(sensor, fname);
               else
                  fetch(sensor, fname, source);
               print(g_info,
                     g_default,
                     format("loaded calibration file '{}'", basename(fname)));
               return true;
            } catch(std::exception& e) {
               print(g_radioactive,
                     g_default,
                     format("error reading calibration file '{}': {}",
                            fname,
                            e.what()));
            }
            return false;
         };

   if(!load_calib_file(out_source, sensor0, calib_fname0)) return false;
   if(!load_calib_file(out_source, sensor1, calib_fname1)) return false;

   Expects(sensor0.calib_format() == sensor1.calib_format());

   // ---- Load each filename, and get the corners. Note these are dual images.
   vector<ImageCorners> raw_corners;
   {
      auto out_counter = 0u;
      for(const auto& fname : filenames)
         load_and_detect(nx,
                         ny,
                         fname,
                         sensor0,
                         sensor1,
                         out_dir,
                         fit_splines,
                         time_threshold,
                         out_counter,
                         raw_corners,
                         w,
                         h);
   }

   print(g_info, g_default, format("image format was set to [{}x{}]", w, h));

   array<std::function<Vector2(const Vector2&)>, 2> distort{
       {sensor0.distort_f(), sensor1.distort_f()}};

   array<std::function<Vector2(const Vector2&)>, 2> undistort{
       {sensor0.undistort_f(), sensor1.undistort_f()}};

   // ---- Calculate proposals, which should remove duplicate frames
   vector<unsigned> proposals;
   {
      print(g_bullet, g_default, format("selecting proposals..."), false);

      auto s = time_thunk([&]() {
         // Sort the detected frames by sharpness
         std::sort(raw_corners.begin(),
                   raw_corners.end(),
                   [&](const auto& a, const auto& b) {
                      return a.sharpness > b.sharpness;
                   });

         vector<vector<Vector3r>> pts0;
         std::transform(raw_corners.begin(),
                        raw_corners.end(),
                        std::back_inserter(pts0),
                        [&](const ImageCorners& X) { return X.im[0]; });
         auto prop_fname = format("{}/zzz-proposals.png", out_dir);
         proposals       = calc_proposals(pts0, 2.0, 1, false, prop_fname);
      });

      cout << format(" {} proposals selected, {} seconds", proposals.size(), s)
           << endl;

      if(proposals.size() < 20) {
         print(g_radioactive,
               g_default,
               format("only {} proposal calibration grids; "
                      "require at least 20",
                      proposals.size()));
         return false;
      }
   }

   // ---- Estimate K
   if(find_K) { find_K01(nx, ny, sensor0, sensor1, raw_corners, proposals); }

   // ----
   Quaternion q;
   Vector3 t;
   Matrix3r E;
   auto baseline = 0.160;
   auto err_E    = 0.0;

   auto flatten = [&](int ind) {
      auto transform = [&](const Vector3r& D) {
         auto U = undistort[size_t(ind)](homgen_P2_to_R2(to_vec3(D)));
         return to_vec3r(homgen_R2_to_P2(U));
      };

      vector<Vector3r> out;
      for(auto prop_ind : proposals)
         for(const auto& X : raw_corners[prop_ind].accurate[ind])
            out.push_back(transform(X));
      return out;
   };

   auto pts0 = flatten(0);
   auto pts1 = flatten(1);

   {
      print(g_bullet,
            g_default,
            format("estimating rotation and translation..."),
            false);

      auto s = time_thunk([&]() {
         estimate_Rt(pts0, pts1, q, t, false);
         Matrix3r tx = make_cross_product_matrix(t);
         Matrix3r R  = quaternion_to_rot3x3(q);
         E           = tx * R;
         err_E       = xFx(E, pts0, pts1);

         if(false) {
            for(auto i = 0u; i < pts0.size(); ++i) {
               cout << format("{:3d}: [{}, {}]: {}",
                              i,
                              str(to_vec3(pts0[i])),
                              str(to_vec3(pts1[i])),
                              400.0 * xFl_lFx(E, pts0[i], pts1[i]))
                    << endl;
            }
         }
      });

      cout << format(" err = {}", err_E) << endl;
   }

   { // Get the baseline

      const Matrix3r R     = quaternion_to_rot3x3(q);
      const Matrix3r R_inv = R.inverse();

      auto get_3dXs_C01 =
          [&](const ImageCorners& corners, const Vector3 C0, const Vector3 C1) {
             const auto& xs = corners.accurate;
             vector<Vector3> Xs(xs[0].size());
             for(auto i = 0u; i < Xs.size(); ++i) {
                auto u0   = sensor0.undistort(xs[0][i]);
                auto u1   = sensor1.undistort(xs[1][i]);
                auto ray0 = to_vec3(u0).normalised();
                auto ray1 = to_vec3(R_inv * u1).normalised();
                Xs[i]     = intersect_rays_2(C0, C0 + ray0, C1, C1 + ray1);
             }
             return Xs;
          };

      auto measure_ss = [&](real baseline) {
         const auto C0 = Vector3(0, 0, 0);
         const auto C1 = to_vec3(-baseline * R_inv * to_vec3r(t));

         auto get_3dXs = [&](const ImageCorners& corners) {
            return get_3dXs_C01(corners, C0, C1);
         };

         // Examine_Xs is merely a debug function
         auto examine_Xs = [&](const auto& corners, auto ind) {
            const auto& xs = corners.accurate;
            auto x0        = xs[0][ind];
            auto x1        = xs[1][ind];
            auto u0        = to_vec3(sensor0.undistort(x0));
            auto u1        = to_vec3(sensor1.undistort(x1));
            auto s0        = cartesian_to_spherical(u0);
            auto s1        = cartesian_to_spherical(u1);
            auto ray0      = to_vec3(sensor0.undistort(x0)).normalised();
            auto ray1 = to_vec3(R_inv * sensor1.undistort(x1)).normalised();
            auto sr0  = cartesian_to_spherical(ray0);
            auto sr1  = cartesian_to_spherical(ray1);

            cout << format("----------------------- {}", ind) << endl;
            cout << format("x0 = {}", str(x0)) << endl;
            cout << format("x1 = {}", str(x1)) << endl;
            // cout << format("d0 = {}", str(sensor0.distort(u0))) << endl;
            // cout << format("d1 = {}", str(sensor1.distort(u1))) << endl;
            cout << format("u0 = {}  {{}, {}}\n",
                           str(u0),
                           to_degrees(s0.x),
                           to_degrees(s0.y));
            cout << format("u1 = {}  {{}, {}}\n",
                           str(u1),
                           to_degrees(s1.x),
                           to_degrees(s1.y));
            cout << format("r0 = {}  {{}, {}}\n",
                           str(ray0),
                           to_degrees(sr0.x),
                           to_degrees(sr0.y));
            cout << format("r1 = {}  {{}, {}}\n",
                           str(ray1),
                           to_degrees(sr1.x),
                           to_degrees(sr1.y));

            detail::set_fit_plane_debug_flag(true);
            fit_plane({C0, C1, C0 + ray0, C1 + ray1});
            detail::set_fit_plane_debug_flag(false);

            auto v0  = homgen_P2_to_R2(u0);
            auto v1  = homgen_P2_to_R2(u1);
            auto X0  = triangulate(v0, v1, R, t * baseline);
            auto x00 = homgen_P2_to_R2(X0);
            auto x01 = project(R, t * baseline, X0);

            cout << format("triangulate...") << endl;
            cout << format("X0 = {}, |X0| = {}\n    |{}-{}|={}, "
                           "|{}-{}|={}",
                           str(X0),
                           X0.norm(),
                           str(v0),
                           str(x00),
                           (v0 - x00).norm(),
                           str(v1),
                           str(x01),
                           (v1 - x01).norm())
                 << endl;

            auto X1  = intersect_rays_2(C0, C0 + ray0, C1, C1 + ray1);
            auto x10 = homgen_P2_to_R2(X1);
            auto x11 = project(R, t * baseline, X1);
            cout << format("intersect-rays-2...") << endl;
            cout << format("X1 = {}, |X1| = {}\n    |{}-{}|={}, "
                           "|{}-{}|={}",
                           str(X1),
                           X1.norm(),
                           str(v0),
                           str(x10),
                           (v0 - x10).norm(),
                           str(v1),
                           str(x11),
                           (v1 - x11).norm())
                 << endl;

            cout << endl << endl;
         };

         auto av_corner_dist = [&](const ImageCorners& corners) {
            auto Xs = get_3dXs(corners); // Resolve the 3D locations
            auto X  = [&](int x, int y) { return Xs[size_t(x + int(nx) * y)]; };
            auto dist    = 0.0;
            auto counter = 0;
            for(auto y = 0; y < int(ny); ++y)
               for(auto x = 1; x < int(nx); ++x) {
                  dist += (X(x, y) - X(x - 1, y)).norm();
                  ++counter;
               }
            for(auto y = 1; y < int(ny); ++y)
               for(auto x = 0; x < int(nx); ++x) {
                  dist += (X(x, y) - X(x, y - 1)).norm();
                  ++counter;
               }
            return dist / real(counter);
         };

         auto av_dist = 0.0;
         for(auto prop_ind : proposals)
            av_dist += av_corner_dist(raw_corners[prop_ind]);
         return av_dist / real(proposals.size());
      };

      const auto dist1 = measure_ss(1.0);
      baseline         = square_size / dist1;
   }

   {
      print(g_bullet,
            g_default,
            format("N calibration images:   {}", proposals.size()));
      print(g_bullet,
            g_default,
            format("estimated rotation:     {}", q.to_readable_str()));
      print(g_bullet, g_default, format("estimated translation:  {}", str(t)));
      print(g_bullet,
            g_default,
            format("estimated baseline:     {}", str(baseline)));
      print(g_bullet,
            g_default,
            format("square size:            {}m", square_size));

      const Matrix3r R     = quaternion_to_rot3x3(q);
      const Matrix3r R_inv = R.inverse();
      const auto C1        = to_vec3(-baseline * R_inv * to_vec3r(t));
      print(g_bullet, g_default, format("center C1:             {}", str(C1)));

      if(C1.x < 0.0) {
         print(g_radioactive,
               g_obnoxious,
               format("Right sensor apperas to be left of the left "
                      "sensor! WARNING!!!"));
      } else {
         print(g_bullet, g_default, format("Sensor left-right check passed"));
      }
   }

   // Save the camera to json
   { // Save the model
      try {
         BinocularCameraInfo info;
         info.camera_id = camera_id;
         info.q         = q;
         info.t         = t;
         info.baseline  = baseline;
         info.M[0]      = sensor0;
         info.M[1]      = sensor1;

         store(info, camera_id, out_source);

         if(false) { // this code is surely wrong
            if(out_source == DataSource::DEFAULT) {
               std::ofstream fs;
               fs.open(out_file);
               if(fs.is_open()) {
                  fs << info.to_json_string();
               } else {
                  auto msg = format("failed to open file '{}'", out_file);
                  throw std::runtime_error(msg);
               }
            }
         }

         const auto out_key
             = resolve_key(AssetType::BCAM_INFO, camera_id, out_source);
         print(g_tick, g_default, format("calibration saved to '{}'", out_key));
         success = true;
      } catch(std::exception& e) {
         print(g_radioactive,
               g_default,
               format("error saving calibration: {}", e.what()));
      }
   }

   if(estimate_error_map) { // Output error

      const auto N = pts0.size();
      vector<real> errs;
      vector<Vector2> Xs; // (x, y) => err
      vector<Vector2> Ys;
      errs.reserve(N);
      Xs.reserve(N);
      Ys.reserve(N);

      for(auto i = 0u; i < N; ++i) {
         auto err_e = xFl_lFx(E, pts0[i], pts1[i]);
         // auto err_c = clamp(err_e * 400.0, 0.0, 1.0);
         // cout << format("err-e = {}, err-c = {}", err_e, err_c) << endl;
         // auto k=vector3_to_kolour(hsv_to_rgb(Vector3(0.0,err_c-1.0,1.0)));
         const auto p0 = sensor0.distort(homgen_P2_to_R2(to_vec3(pts0[i])));

         Xs.push_back(p0);
         Ys.emplace_back(err_e * 400.0, 0.0);
         errs.emplace_back(err_e * 400.0);

         // Expects(argb.in_bounds(p0));
         // const auto radius = 5;
         // for(int dy = -radius; dy <= radius; ++dy) {
         //     for(int dx = -radius; dx <= radius; ++dx) {
         //         if(dx*dx + dy*dy > radius*radius) continue;
         //         auto p = p0 + Vector2(dx, dy);
         //         if(!argb.in_bounds(p)) continue;
         //         if(argb(p) == 0u)
         //             argb(p) = k;
         //         else
         //             argb(p) = blend(k, argb(p), 0.5);
         //     }
         // }
      }

      // Calculate statistics
      SampleStatistics stats = calc_sample_statistics(begin(errs), end(errs));
      print(
          g_bullet, g_default, format("Sample statistics of epipolar errors"));
      cout << stats.to_string();

      print(g_bullet,
            g_default,
            format("Calculating RBF field of epipolar errors..."));

      RBFField rbf;
      rbf.init_qnn(Xs, Ys);

      ARGBImage argb;
      argb.resize(w, h);
      argb.zero();

      const auto& hull = sensor0.calib_hull();
      HeatMap heat_map;

      for(auto y = 0u; y < h; ++y) {
         for(auto x = 0u; x < w; ++x) {
            if(point_in_polygon(Vector2(x, y), cbegin(hull), cend(hull))) {
               real val     = rbf.evaluate(Vector2(x, y)).x;
               auto clamped = clamp(val, 0.0, 3.5) / 3.5;
               auto hsv     = Vector3(0.0, clamped, 1.0);
               auto k       = vector3_to_kolour(hsv_to_rgb(hsv));
               argb(x, y)   = k;
            }
         }
      }

      auto fname = format("{}/zzz-stereo-error-image.png", out_dir);
      print(g_bullet,
            g_default,
            format("stereo error image saved to: {}", fname));

      argb.save(fname);
   }

   return success;
}

bool run_stereo_calibration(const unsigned nx,
                            const unsigned ny,
                            const real square_size,
                            const string& camera_id,
                            const DataSource out_source,
                            const string& out_dir,
                            const string& distort_key0,
                            const string& distort_key1,
                            const bool fit_splines,
                            const bool find_K,
                            const bool estimate_error_map,
                            const real time_threshold,
                            const vector<string>& filenames)
{
   return run_stereo_calibration_(nx,
                                  ny,
                                  square_size,
                                  camera_id,
                                  "",
                                  out_source,
                                  out_dir,
                                  distort_key0,
                                  distort_key1,
                                  fit_splines,
                                  find_K,
                                  estimate_error_map,
                                  time_threshold,
                                  filenames);
}

bool run_stereo_calibration(const unsigned nx,
                            const unsigned ny,
                            const real square_size,
                            const string& camera_id,
                            const string& out_file,
                            const string& out_dir,
                            const string& calib_fname0,
                            const string& calib_fname1,
                            const bool fit_splines,
                            const bool find_K,
                            const bool estimate_error_map,
                            const real time_threshold,
                            const vector<string>& filenames)
{
   return run_stereo_calibration_(nx,
                                  ny,
                                  square_size,
                                  camera_id,
                                  out_file,
                                  DataSource::DEFAULT,
                                  out_dir,
                                  calib_fname0,
                                  calib_fname1,
                                  fit_splines,
                                  find_K,
                                  estimate_error_map,
                                  time_threshold,
                                  filenames);
}

} // namespace perceive::calibration
