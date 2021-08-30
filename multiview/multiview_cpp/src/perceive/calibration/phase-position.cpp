
#include "phase-position.hpp"

#include <opencv2/opencv.hpp>

#include "phase-position-helpers.hpp"

#include "perceive/calibration/aruco-cube.hpp"
#include "perceive/calibration/plane-set/cps-operations.hpp"
#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/geometry/projective/back-project-kite.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/movie/debug-movie.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"

namespace perceive::calibration
{
// --------------------------------------------------------- for output purposes

inline void print(const string& glyph,
                  const string& color,
                  const string_view msg,
                  const bool print_newline = true)
{
   cout << format(" {}  {}{}\x1b[0m", glyph, color, msg.data());
   if(print_newline)
      cout << endl;
   else
      cout.flush();
}

inline void print_timing(decltype(tick())& timer)
{
   cout << format(" {}s", tock(timer)) << endl;
   timer = tick();
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
static const auto g_green      = "\x1b[32m"s;

static string pack_et(const EuclideanTransform& et)
{
   array<real, 7> vars;
   et.pack(&vars[0]);
   return format("{{}}", implode(cbegin(vars), cend(vars), ", "));
}

// ----------------------------------------------

string PhasePositionParams::to_string() const noexcept
{
   return format(
       R"V0G0N({{
   "output-dir":        {},
   "aruco-cube-id":     {},
   "animate-cube":      {},
   "render-cube-only":  {},
   "camera-id":         {},
   "baseline-est":      {},
   "estimate-bcam-qt":  {},  
   "dense-et0-est":     {},
   "use-pointwise":     {},
   "use-fast-distort":  {},
   "image-fnames":      [{}]
}}{})V0G0N",
       json_encode_str(outdir),
       json_encode_str(ac.id),
       str(animate_cube),
       str(render_cube_only),
       json_encode_str(camera_id),
       baseline_estimate,
       str(estimate_bcam_qt),
       str(dense_et0),
       str(use_pointwise_result),
       str(use_fast_distort),
       trim_copy(indent(implode(cbegin(input_image_fnames),
                                cend(input_image_fnames),
                                ",\n",
                                json_encode_str),
                        6)),
       "");
}

// string str(const PhasePositionParams& p) noexcept { return p.to_string(); }

// ---------------------------------------------- run phase position calibration

bool run_phase_position_calibration(PhasePositionParams& p) noexcept
{
   PhasePositionOptDetails opt;
   opt.imgs.resize(p.images.size());
   array<array<cv::Mat, 2>, 2> mapxys;
   BinocularCameraInfo bcam_inout;

   const auto N0 = opt.imgs.size();

   // Check image sizes
   if(!p.render_cube_only and N0 == 0) {
      LOG_ERR(format("requires at least 1 image"));
      return false;
   }

   if(!p.render_cube_only and N0 < 1 and p.estimate_bcam_qt) {
      LOG_ERR(format("stereo calibration requires at least 1 image"));
      return false;
   }

   // Input width/height
   const unsigned iw = p.render_cube_only ? 0 : p.images[0][0].width;
   const unsigned ih = p.render_cube_only ? 0 : p.images[0][0].height;

   for(size_t j = 0; j < N0; ++j) {
      for(size_t i = 0; i < 2; ++i) {
         if(p.images[j][i].width != iw || p.images[j][i].height != ih) {
            LOG_ERR(format("error in input image format: expected [{}x{}], but "
                           "got [{}x{}]",
                           iw,
                           ih,
                           p.images[j][i].width,
                           p.images[j][i].height));
            return false;
         }
      }
   }

   const unsigned uw = iw; // Undistorted width/height
   const unsigned uh = unsigned(std::ceil(uw * ih / real(iw)));
   const real vfov   = 70.0; // vertical field of view

   ParallelJobSet pjobs;

   opt.p  = p;
   opt.ac = p.ac;

   // The undistorted camera
   opt.K       = Matrix3r::Identity();
   opt.K(0, 0) = opt.K(1, 1) = 0.5 * real(uh) / tan(0.5 * to_radians(vfov));
   opt.K(0, 2)               = uw * 0.5;
   opt.K(1, 2)               = uh * 0.5;

   auto now = tick();

   if(!p.quite_mode) { // output the configuration
      print(g_info,
            g_default,
            format("running phase-position with configuration:"),
            true);
      cout << endl << indent(p.to_string(), 4) << endl << endl;
   }

   { // save aruco-cube json
      if(!p.quite_mode)
         print(
             g_info,
             g_default,
             format("saving '{}' aruco cube json file to output directory '{}'",
                    p.ac.id,
                    p.outdir),
             false);
      file_put_contents(format("{}/{}.json", p.outdir, opt.ac.id),
                        opt.ac.to_json_str());
      if(!p.quite_mode) print_timing(now);
   }

   { // draw the cube faces, and output
      if(!p.quite_mode)
         print(g_info,
               g_default,
               format("drawing the faces of the '{}' aruco cube", p.ac.id),
               false);

      auto s = 800u;
      for(int i = 0; i < ArucoCube::k_n_faces; ++i) {
         auto f_ind              = ArucoCube::face_e(i);
         opt.face_ims[size_t(i)] = opt.ac.draw_face(f_ind, s, false, false);

         if(f_ind == ArucoCube::BOTTOM) continue;
         const auto fname
             = format("{}/aaa_{}.png", p.outdir, ArucoCube::face_to_str(f_ind));
         cv::imwrite(fname, opt.ac.draw_face(f_ind, s, true, true));
      }
      if(!p.quite_mode) print_timing(now);
   }

   if(p.animate_cube) { // render the cube as an animation
      if(!p.quite_mode)
         print(g_info,
               g_default,
               format("rendering '{}' aruco cube animation", opt.ac.id),
               false);

      array<cv::Mat, 6> face_ims;
      { // get the face images
         const auto s = 800u;
         for(int i = 0; i < ArucoCube::k_n_faces; ++i)
            face_ims[size_t(i)]
                = opt.ac.draw_face(ArucoCube::face_e(i), s, false, true);
      }

      auto now = tick();
      {
         const auto w           = 1024;
         const auto h           = 768;
         const auto hfov        = to_radians(55);
         const int n_frames     = 360;
         const real dist        = 1.5;
         const real inclination = to_radians(40.0);
         const real axes_len    = 2.00;

         const auto x_axis      = Vector3(1, 0, 0);
         const auto y_axis      = Vector3(0, 1, 0);
         const auto z_axis      = Vector3(0, 0, 1);
         const real radius      = dist * cos(inclination);
         const real elevation   = dist * sin(inclination);
         const auto delta_theta = 2.0 * M_PI / (n_frames - 1);
         const auto frame_dir   = format("{}/frames", p.outdir);
         const auto offset      = opt.ac.center();
         mkdir_p(frame_dir);
         if(!is_directory(frame_dir))
            FATAL(format("failed to create directory '{}'", frame_dir));

         auto make_frame_fname = [&](int frame_no) {
            return format("{}/frame-{:3d}.png", frame_dir, frame_no + 1);
         };

         auto proc_frame = [&](int frame_no) {
            const auto theta = delta_theta * frame_no;
            const auto pos   = radius * cos(theta) * x_axis
                             + radius * sin(theta) * y_axis + elevation * z_axis
                             + offset;
            const auto frame = render_aruco_cube_helicopter(
                pos, w, h, hfov, opt.ac, face_ims, axes_len);
            frame.save(make_frame_fname(frame_no));
         };

         { // execute the jobs
            ParallelJobSet pjobs;
            for(auto i = 0; i < n_frames; ++i)
               pjobs.schedule([i, &proc_frame]() { proc_frame(i); });
            pjobs.execute();
         }

         { // create the animation
            vector<string> fnames;
            fnames.reserve(n_frames);
            for(auto i = 0; i < n_frames; ++i)
               fnames.push_back(make_frame_fname(i));
            const string input_fnames
                = implode(cbegin(fnames), cend(fnames), " ");

            const string command0
                = format("convert -loop 0 {} {} 1>/dev/null 2>/dev/null",
                         input_fnames,
                         format("{}/aruco-cube.gif", p.outdir));

            const string command1 = format(
                "ffmpeg -i {} -y -f gif -loop 0 {} 1>/dev/null 2>/dev/null",
                format("{}/frame-{:03d}.png", frame_dir),
                format("{}/aruco-cube.gif", p.outdir));

            auto ret = system(command1.c_str());

            if(ret != 0) {
               LOG_ERR(format("failed to call FFMPEG to make aruco-cube gif"));
            } else {
               // All is good
            }
         }
      }
      if(!p.quite_mode) print_timing(now);
   }

   if(p.render_cube_only) {
      if(!p.quite_mode)
         print(g_tick,
               g_default,
               format("render-cube-only mode... we're done!"),
               true);
      return true;
   }

   for(auto j = 0u; j < N0; ++j) {
      if(!p.quite_mode)
         print(g_info,
               g_default,
               format("loaded image '{}'", basename(p.input_image_fnames[j])));
      opt.imgs[j].image_fname = p.input_image_fnames[j];
   }

   { // Convert images to cv format
      if(!p.quite_mode)
         print(g_info, g_default, "converting images to cv-format", false);
      auto f = [&](size_t j, size_t i) {
         opt.imgs[j].cv_im[i] = argb_to_cv(p.images[j][i]);
         const auto fname
             = format("{}/bbb_distorted_{}-{}.png", p.outdir, j, i);
         cv::imwrite(fname, opt.imgs[j].cv_im[i]);
      };
      for(size_t j = 0; j < N0; ++j)
         for(size_t i = 0; i < 2; ++i) pjobs.schedule([f, i, j]() { f(j, i); });
      pjobs.execute();
      if(!p.quite_mode) print_timing(now);
   }

   { // Load "caching-undistort" objects
      if(!p.quite_mode)
         print(g_info, g_default, "loading caching-undistort data", true);
      for(size_t i = 0; i < 2; ++i) {
         opt.cus[i].init(p.bcam_info.M[i]);
         opt.cus[i].set_working_format(iw, ih);
      }
      if(!p.quite_mode) print_timing(now);
   }

   { // Create undistort maps
      if(!p.quite_mode)
         print(g_info,
               g_default,
               format("creating undistort maps, w={}, h={}, fast={}",
                      uw,
                      uh,
                      str(p.use_fast_distort))
                   .c_str(),
               false);
      Matrix3r H = Matrix3r::Identity();
      for(size_t i = 0; i < 2; ++i) {
         // auto& mapxy        = mapxys[i];
         const string fname = format(
             "{}/mapxys_{}_fast={}.data", p.outdir, i, str(p.use_fast_distort));
         if(is_regular_file(fname)) {
            cv::FileStorage file(fname, cv::FileStorage::READ);
            file[format("cam{}_mapx", i)] >> mapxys[i][0];
            file[format("cam{}_mapy", i)] >> mapxys[i][1];
         } else {
            ParallelJobSet job_set;
            std::function<Vector2(const Vector2&)> f
                = [&](const Vector2& x) { return opt.cus[i].fast_distort(x); };
            std::function<Vector2(const Vector2&)> g
                = [&](const Vector2& x) { return opt.cus[i].distort(x); };
            auto ff = p.use_fast_distort ? f : g;

            create_cv_remap_threaded(
                uw, uh, H, ff, opt.K, mapxys[i][0], mapxys[i][1], job_set);

            if(false) {
               cout << endl;
               INFO("REPORT");
               cout << format("  i   = {}", i) << endl;
               cout << format("  str = {}", opt.cus[i].M().sensor_id()) << endl;
               cout << format("  uwh = [{}x{}]", uw, uh) << endl;
               cout << format("  H   = \n") << H << endl << endl;
               cout << format("  K   = \n") << opt.K << endl << endl;
               cout << endl;
            }

            if(false) {
               cv::FileStorage file(fname, cv::FileStorage::WRITE);
               file << format("cam{}_mapx", i) << mapxys[i][0];
               file << format("cam{}_mapy", i) << mapxys[i][1];
            }
         }
      }
      if(!p.quite_mode) print_timing(now);
   }

   { // undistort images
      if(!p.quite_mode) print(g_info, g_default, "undistorting images", false);
      auto f = [&](size_t j, size_t i) {
         cv::remap(opt.imgs[j].cv_im[i],
                   opt.imgs[j].undistorted[i],
                   mapxys[i][0],
                   mapxys[i][1],
                   cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(255, 255, 255));
         const auto fname
             = format("{}/ccc_undistorted_{}-{}.png", p.outdir, j, i);
         cv::imwrite(fname, opt.imgs[j].undistorted[i]);
      };
      for(size_t j = 0; j < N0; ++j)
         for(size_t i = 0; i < 2; ++i) pjobs.schedule([f, i, j]() { f(j, i); });
      pjobs.execute();
      if(!p.quite_mode) print_timing(now);
   }

   { // Let's detect some markers
      auto make_out_fname = [&](unsigned j, unsigned i) {
         return format("{}/ddd_detect_{}-{}.png", p.outdir, j, i);
      };

      if(!p.quite_mode) print(g_info, g_default, "detecting markers", false);
      auto f = [&](size_t j, size_t i) {
         const string out_fname = make_out_fname(unsigned(j), unsigned(i));
         opt.imgs[j].detects[i] = opt.ac.detect_markers(
             opt.imgs[j].undistorted[i], opt.K, out_fname);
      };
      for(size_t j = 0; j < N0; ++j)
         for(size_t i = 0; i < 2; ++i) pjobs.schedule([f, i, j]() { f(j, i); });
      pjobs.execute();
      if(!p.quite_mode) print_timing(now);

      for(size_t j = 0; j < N0; ++j) {
         for(size_t i = 0; i < 2; ++i) {
            if(opt.imgs[j].detects[i].size() < 2) {
               print(g_radioactive,
                     g_default,
                     format("found {} faces in image '{}' (ie, {}-{}); "
                            "however, "
                            "expect to find "
                            "2 or more faces",
                            opt.imgs[j].detects[i].size(),
                            p.input_image_fnames[j],
                            j,
                            i));
            }
         }
      }
   }

   { // REMOVE bad images
      auto new_end = std::stable_partition(
          begin(opt.imgs), end(opt.imgs), [&](const auto& o) {
             return o.detection_is_good();
          });
      opt.imgs.erase(new_end, end(opt.imgs));

      // p.{input_image_fnames, raw_input_images, images}
      // opt.{imgs}

      if(opt.imgs.size() == 0) {
         print(g_radioactive,
               g_default,
               format("detected faces on only {} input images; aborting...",
                      opt.imgs.size()));
         return false; // a fail
      }
   }
   const unsigned N = unsigned(opt.imgs.size());

   { // Let's find the EuclideanTransform estimate:
      now = tick();
      if(!p.quite_mode)
         print(g_info, g_default, "estimating rotation and translation", false);
      auto f = [&](size_t j, size_t i) {
         ARGBImage argb = cv_to_argb(opt.imgs[j].undistorted[i]);
         opt.imgs[j].et0s[i]
             = estimate_et0(opt.ac, opt.imgs[j].detects[i], opt.K, &argb);
         argb.save(
             format("{}/eee_{}-cam{}_pointwise_INITIAL.png", p.outdir, j, i));
         opt.imgs[j].ets[i] = opt.imgs[j].et0s[i];
      };
      for(size_t j = 0; j < N; ++j)
         for(size_t i = 0; i < 2; ++i) pjobs.schedule([f, i, j]() { f(j, i); });
      pjobs.execute();
      if(!p.quite_mode) print_timing(now);
   }

   { // Refine the initial estimate
      now = tick();
      if(!p.quite_mode)
         print(g_info, g_default, "refining point-wise estimate", false);
      bool has_error = false;
      auto f         = [&](size_t j, size_t i) {
         opt.imgs[j].ets[i] = opt.imgs[j].et0s[i];
         ARGBImage argb     = cv_to_argb(opt.imgs[j].undistorted[i]);
         const auto success = refine_pointwise_et(opt.ac,
                                                  opt.imgs[j].detects[i],
                                                  opt.K,
                                                  int(i),
                                                  opt.imgs[j].ets[i],
                                                  &argb,
                                                  false);
         argb.save(
             format("{}/fff_{}-cam{}_pointwise_FINAL.png", p.outdir, j, i));
         if(!success) {
            has_error = true;
            sync_write([&]() {
               cout << format("point-wise refine failed on image {}", j)
                    << endl;
            });
         }
      };
      for(size_t j = 0; j < N; ++j)
         for(size_t i = 0; i < 2; ++i) pjobs.schedule([f, i, j]() { f(j, i); });
      pjobs.execute();
      if(!p.quite_mode) print_timing(now);
      if(has_error) FATAL(format("aborting, because of optimization failure"));
   }

   if(p.use_pointwise_result) {
      if(!p.quite_mode)
         print(g_info, g_default, "skipping dense adjustment", true);
   } else { // Perform dense-method
      now = tick();
      if(!p.quite_mode)
         print(g_info, g_default, "performing dense adjustment", true);
      auto f = [&](size_t j, size_t i) {
         ARGBImage argb = cv_to_argb(opt.imgs[j].undistorted[i]);
         refine_et_dense(opt.ac,
                         opt.imgs[j].detects[i],
                         opt.face_ims,
                         opt.K,
                         int(i),
                         opt.imgs[j].ets[i],
                         argb,
                         true);
         argb.save(format("{}/ggg_{}-cam{}_dense_FINAL.png", p.outdir, j, i));
      };
      for(size_t j = 0; j < N; ++j)
         for(size_t i = 0; i < 2; ++i) pjobs.schedule([f, i, j]() { f(j, i); });
      pjobs.execute();
      if(!p.quite_mode) {
         cout << "\n    --------------------\n    Total time: ";
         cout << format("{:7.5f}s", tock(now)) << endl;
         cout << endl;
      }
      now = tick();
   }

   if(p.estimate_bcam_qt) { // Estimate 'q' and 't'
      now = tick();
      if(!p.quite_mode)
         print(g_info, g_default, "performing dense adjustment (stereo)", true);
      EuclideanTransform _;
      bcam_inout = p.bcam_info;

      refine_bcam_qt(opt, true, p.K_opt, bcam_inout, _, 0, p.outdir, true);
      cout << "    ---" << endl;
      const string outfile = format("{}/{}.json", p.outdir, p.camera_id);
      cout << format("    Output file: '{}'", outfile) << endl;
      bcam_inout.camera_id = p.camera_id;
      save(bcam_inout, outfile);

      if(!p.quite_mode) {
         cout << "\n    --------------------\n    Total time: ";
         cout << format("{:7.5f}s", tock(now)) << endl;
         cout << endl;
      }
   }

   if(p.estimate_bcam_qt) { // Save the rectified images
      now = tick();
      if(!p.quite_mode)
         print(g_info, g_default, "saving a rectified image", true);

      Expects(opt.imgs.size() > 0);
      const auto& opt_img = opt.imgs[0];
      BinocularCamera bcam;
      const int dw = opt_img.cv_im[0].cols;
      const int dh = opt_img.cv_im[0].rows;
      const int rw = 800;
      const int rh = 600;
      bcam.init(bcam_inout,
                unsigned(dw),
                unsigned(dh),
                Matrix3r::Identity(),
                rw,
                rh,
                true);

      cv::Mat mapx[2];
      cv::Mat mapy[2];
      cv::Mat rect[2];
      bcam.get_mapxy(mapx, mapy);
      for(size_t i = 0; i < 2; ++i) {
         cv::remap(opt_img.cv_im[i],
                   rect[i],
                   mapx[i],
                   mapy[i],
                   cv::INTER_LANCZOS4,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(255, 255, 255));
      }

      ARGBImage rim = hcat(cv_to_argb(rect[0]), cv_to_argb(rect[1]));

      for(auto y = 4u; y < rim.height; y += 16) {
         uint32_t* row = rim.row_ptr(y);
         for(auto x = 0u; x < rim.width; ++x) {
            rim(x, y) = blend(rim(x, y), k_yellow, 0.5);
         }
      }

      rim.save(format("{}/zzz-rectified.png", p.outdir));

      if(!p.quite_mode) {
         cout << "\n    --------------------\n    Total time: ";
         cout << format("{:7.5f}s", tock(now)) << endl;
         cout << endl;
      }
   }

   if(!p.use_pointwise_result and p.dense_et0
      and !p.estimate_bcam_qt) { // Dense refine et0
      now = tick();
      if(!p.quite_mode)
         print(g_info,
               g_default,
               "performing dense adjustment (et0 binocular)",
               true);
      bcam_inout = p.bcam_info;

      for(auto k = 0u; k < N; ++k) {
         refine_bcam_qt(opt,
                        false,
                        false,
                        bcam_inout,
                        opt.imgs[k].ets[0],
                        k,
                        p.outdir,
                        true);
      }

      if(!p.quite_mode) {
         cout << "\n    --------------------\n    Total time: ";
         cout << format("{:7.5f}s", tock(now)) << endl;
      }
   }

   if(!p.estimate_bcam_qt) {
      Json::Value arr{Json::arrayValue};
      arr.resize(N);
      for(auto k = 0u; k < N; ++k)
         arr[k] = json_save(opt.imgs[k].ets[0].inverse());
      std::stringstream ss{""};
      ss << arr << endl;
      const auto fname = format("{}/positions.json", p.outdir);
      file_put_contents(fname, ss.str());
      if(!p.quite_mode)
         print(g_info,
               g_default,
               format("extrinsic position{} written to {}",
                      (N > 1 ? "s" : ""),
                      fname),
               true);
   }

   return true;
}

} // namespace perceive::calibration
