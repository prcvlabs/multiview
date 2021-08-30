
#include <opencv2/opencv.hpp>

#include "calib-plane-set.hpp"
#include "cps-operations.hpp"
#include "run-calibration.hpp"

#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

namespace perceive::calibration
{
static void optimize_et01_ds(PlaneOpsCalculator& pcalc,
                             const string& outdir,
                             const bool feedback)
{
   const int n_cps      = int(pcalc.ds.size());
   const int n_params   = 12 + (n_cps - 1);
   const Vector3 C10    = pcalc.C[1];
   const Quaternion q10 = pcalc.q[1];
   Expects(pcalc.ops_data.cps.p3s.size() == pcalc.ds.size());
   Expects(n_cps >= 1);

   array<cv::Mat, 2> mapx;
   array<cv::Mat, 2> mapy;
   array<cv::Mat, 2> dst;
   array<cv::Mat, 2> cv_im;
   for(size_t i = 0; i < 2; ++i) cv_im[i] = pcalc.ops_data.cv_im[i];

   auto pack_ind = [&](real* X, int ind) {
      Vector3 saa = quaternion_to_saa(pcalc.q[ind]);
      for(auto i = 0; i < 3; ++i) *X++ = pcalc.C[ind][i];
      for(auto i = 0; i < 3; ++i) *X++ = saa[i];
      return X;
   };

   auto pack = [&](real* X) {
      X = pack_ind(X, 0);
      X = pack_ind(X, 1);
      for(auto i = 1; i < n_cps; ++i) *X++ = pcalc.ds[size_t(i)];
   };

   auto unpack_ind = [&](const real* X, int ind) {
      Vector3 saa;
      for(int i = 0; i < 3; ++i) pcalc.C[ind][i] = *X++;
      for(int i = 0; i < 3; ++i) saa[i] = *X++;
      pcalc.q[ind] = saa_to_quaternion(saa);
      return X;
   };

   auto unpack = [&](const real* X) {
      X = unpack_ind(X, 0);
      X = unpack_ind(X, 1);
      for(auto i = 1; i < n_cps; ++i) pcalc.ds[size_t(i)] = *X++;
   };

   auto counter      = 0;
   auto best_err     = std::numeric_limits<real>::max();
   bool final_unpack = false;
   auto fn           = [&](const real* X) {
      unpack(X);

      pcalc.make_mapxy(true, -1, mapx[0], mapy[0]);
      pcalc.make_mapxy(false, -1, mapx[1], mapy[1]);

      cv::remap(cv_im[0], dst[1], mapx[0], mapy[0], cv::INTER_LINEAR);
      cv::remap(cv_im[1], dst[0], mapx[1], mapy[1], cv::INTER_LINEAR);

      const auto fname00 = final_unpack ? format("{}/z00.png", outdir) : ""s;
      const auto fname11 = final_unpack ? format("{}/z11.png", outdir) : ""s;
      auto err0          = image_match_score(cv_im[0], dst[0], fname00);
      auto err1          = image_match_score(cv_im[1], dst[1], fname11);

      auto err = 0.5 * (err0 + err1);
      if(feedback) {
         if(err < best_err) {
            best_err = err;
            cout << format("#{:4d}, err={:7.5f}, [{}]",
                           counter,
                           best_err,
                           implode(&X[0], &X[n_params], ", "))
                 << endl;
         }
         ++counter;
      }
      return err;
   };

   vector<real> start((size_t(n_params)));
   vector<real> xmin((size_t(n_params)));
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-7;
   real diffstep = 0.1;

   int kcount = 1000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method         = nullptr;
   const bool use_nelder_mead = true;

   pack(&start[0]);
   ystartlo = fn(&start[0]);

   if(!use_nelder_mead) {
      method = "levenberg-marquardt";
      levenberg_marquardt(fn,
                          unsigned(n_params),
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

      vector<real> step((size_t(n_params)));
      auto simplex_factor = 0.05;
      for(auto i = 0; i < n_params; ++i) step[size_t(i)] = one_degree();
      nelder_mead(fn,
                  unsigned(n_params),
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

   final_unpack = true;
   fn(&xmin[0]);

   if(feedback) {
      INFO(format("Feedback for finding 'et01' and 'd' ({}) params", n_params));
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;

      cout << format("C0     = {}", str(pcalc.C[0])) << endl;
      cout << format("C1     = {}", str(pcalc.C[1])) << endl;
      cout << format("bline  = {}", (pcalc.C[1] - pcalc.C[0]).norm()) << endl;

      for(auto i = 0; i < n_cps; ++i) {
         auto d  = pcalc.ds[size_t(i)];
         auto p3 = pcalc.ops_data.cps.p3s[size_t(i)].p3;
         cout << format("p3[{}] = {}, {}", i, str(Plane(p3.xyz(), d)), d)
              << endl;
      }
   }
}

bool run_plane_set_calibration(const PlaneSetCalibParams& params)
{
   PlaneOpsData data;
   if(!data.init(params)) return false;

   PlaneOpsCalculator pcalc(data);

   if(false) {
      auto bcam_info = data.bcam_info;
      pcalc.update_bcam_info(bcam_info);

      string s;
      write(data.bcam_info, s);
      INFO("bcam0 = ");
      cout << s;
      write(bcam_info, s);
      INFO("bcam1 = ");
      cout << s;
      FATAL("kBAM!");
   }

   if(false) { // test out remapping...
      array<cv::Mat, 2> mapx;
      array<cv::Mat, 2> mapy;
      cv::Mat dst0, dst1;
      pcalc.make_mapxy(true, 0, mapx[0], mapy[0]); // From
      pcalc.make_mapxy(false, 0, mapx[1], mapy[1]);

      cv::remap(data.cv_im[0], dst1, mapx[0], mapy[0], cv::INTER_LINEAR);
      cv::remap(data.cv_im[1], dst0, mapx[1], mapy[1], cv::INTER_LINEAR);

      cv::imwrite("/tmp/www-im0.png", data.cv_im[0]);
      cv::imwrite("/tmp/www-ds0.png", dst0);
      cv::imwrite("/tmp/www-im1.png", data.cv_im[1]);
      cv::imwrite("/tmp/www-ds1.png", dst1);
   }

   // WARN(format("Must fix line below"));
   optimize_et01_ds(pcalc, data.outdir, true);

   // Okay, output the new rotation and translation
   auto bcam_info = data.bcam_info;
   pcalc.update_bcam_info(bcam_info);
   auto et0 = pcalc.extract_et0();

   // cout << "et0 = " << json_encode(et0) << endl;
   // string s;
   // write(bcam_info, s);
   // cout << "bcam = " << s << endl;

   auto out_fname = format("{}/bcam-info.json", data.outdir);
   save(bcam_info, out_fname);
   INFO(format("saved bcam-info to {}", out_fname));
   cout << format(" + camera-id = {}", bcam_info.camera_id) << endl;
   cout << format(" + q         = {}", str(bcam_info.q)) << endl;
   cout << format(" + t         = {}", str(bcam_info.t)) << endl;
   cout << format(" + baseline  = {}", bcam_info.baseline) << endl;

   cout << format(" + et0       = {}", et0.to_json_str()) << endl << endl;
   cout << format(" + et0^-1    = {}", et0.inverse().to_json_str()) << endl
        << endl;
   INFO(format("Planes"));

   // Output the planes
   vector<Plane> out_planes;
   out_planes.resize(data.cps.p3s.size());
   for(auto i = 0u; i < out_planes.size(); ++i) {
      auto& p3 = out_planes[i];
      switch(data.cps.p3s[i].plane_type) {
      case 0: p3.xyz() = Vector3(1, 0, 0); break;
      case 1: p3.xyz() = Vector3(0, 1, 0); break;
      case 2: p3.xyz() = Vector3(0, 0, 1); break;
      }
      p3.w = pcalc.ds[i];
   }
   cout << format("[{}]",
                  implode(cbegin(out_planes),
                          cend(out_planes),
                          ",\n ",
                          [](auto& p3) {
                             return format(
                                 "[{}, {}, {}, {}]", p3.x, p3.y, p3.z, p3.w);
                          }))
        << endl;

   { // Output disparity maps
      BinocularCamera bcam;
      const auto w0    = data.cv_im[0].cols;
      const auto h0    = data.cv_im[0].rows;
      const auto w     = 800;
      const auto h     = 600;
      const Matrix3r K = Matrix3r::Identity();
      cv::Mat mapx[2];
      cv::Mat mapy[2];
      cv::Mat undistorted[2];

      DistortedCamera cam0, cam1;
      std::tie(cam0, cam1) = make_distorted_camera_pair(
          bcam_info, et0, unsigned(w0), unsigned(h0));

      const auto& f2d0 = data.f2d[0]; // has slic info

      bcam.init(bcam_info, unsigned(w0), unsigned(h0), K, w, h, true);
      bcam.set_working_format(unsigned(w0), unsigned(h0));
      bcam.get_mapxy(mapx, mapy);

      // Save diagnostic undistort image
      for(size_t i = 0; i < 2; ++i) {
         cv::remap(data.cv_im[i],
                   undistorted[i],
                   mapx[i],
                   mapy[i],
                   cv::INTER_LANCZOS4,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(255, 255, 255));
         cv::imwrite(format("{}/u_undist{}.png", data.outdir, i),
                     undistorted[i]);
      }

      // Create disparity map
      cv::Mat disp(h, w, CV_32FC1);

      auto process_row = [&](int y) {
         float* row = disp.ptr<float>(y);

         for(auto x = 0; x < w; ++x) {
            row[x] = std::numeric_limits<float>::quiet_NaN();

            // Rectified to distorted. (checked.)
            const auto D = to_vec2(
                Vector2f(mapx[0].at<float>(y, x), mapy[0].at<float>(y, x)));

            // What plane-index are we talking about?
            if(!data.complete_cp_lookup[0].in_bounds(int(D.x), int(D.y)))
               continue;

            int p3_ind = data.complete_cp_lookup[0](D.x, D.y);
            if(p3_ind < 0) continue;
            Expects(unsigned(p3_ind) < data.complete_cps.p3s.size());

            const Plane& p3 = data.complete_cps.p3s[size_t(p3_ind)].p3;

            // 3D point, world coordinates
            const auto W = plane_ray_intersect(cam0, p3, D);

            // 3D point, CAM0 coordinates
            const auto X = et0.inverse_apply(W);

            auto f = [&](real dx) -> real {
               return (X - bcam.solve3d(x, x - dx, y)).quadrance();
            };

            // Disparity
            // We want x1 such that X == bcam.solve3d(x, x1, y);
            const auto dx = golden_section_search(f, 0.001, 200.0, 1e-6);

            if(false) {
               // How did we go?
               const auto Y = bcam.solve3d(x, x - dx, y);
               INFO(format("|{} - {}| = {}", str(X), str(Y), (X - Y).norm()));

               cout << format("[d]  = {}x{}", w0, h0) << endl;
               cout << format("[r]  = {}x{}", w, h) << endl;
               cout << format("R    = ({}, {})", x, y) << endl;
               cout << format("D    = {}", str(D)) << endl;
               cout << format("p3   = {}", str(p3)) << endl;
               cout << format("W    = {}", str(W)) << endl;
               cout << format("X    = {}", str(X)) << endl;
               cout << format("p3.X = {}", p3.side(W)) << endl;
               cout << format("P(D) = {}", str(bcam.project(CAM0, X))) << endl;
               cout << format("P(R) = {}",
                              str(bcam.project_to_rectified(CAM0, X)))
                    << endl;
               FATAL("kBAM!");
            }

            Expects((D - bcam.project(CAM0, X)).norm() < 1e-3);
            Expects((Vector2(x, y) - bcam.project_to_rectified(CAM0, X)).norm()
                    < 1e-3);

            row[x] = float(dx);
         }
      };

      for(auto y = 0; y < h; ++y)
         data.pjobs.schedule([&process_row, y]() { process_row(y); });
      data.pjobs.execute();

      auto make_disp_image = [&](const cv::Mat& disp) {
         cv::Mat disp_image = cv::Mat(h, w, CV_8UC1);
         double minVal;
         double maxVal;
         minMaxLoc(disp_image, &minVal, &maxVal);
         minVal = 0.0;
         maxVal = 100.0;
         disp.convertTo(disp_image, CV_8UC1, 255.0 / (maxVal - minVal));
         return disp_image;
      };
      cv::imwrite(format("{}/v_disp-map.png", data.outdir),
                  make_disp_image(disp));

      FloatImage fim;
      cv_to_float_im(disp, fim);
      fim.save(format("{}/w_disp-float-image.data", data.outdir));

      string key = format("{}_{}",
                          strip_suffix(data.scene_key, '_'),
                          strip_suffix(bcam_info.camera_id, '_'));
      store(fim, key);

      if(true) { // Sanity check
         FloatImage im2;
         fetch(im2, key);
         cv::Mat disp2 = float_im_to_cv(fim);
         cv::imwrite(format("{}/v_disp-map2.png", data.outdir),
                     make_disp_image(disp2));
      }
   }

   return true;
}

} // namespace perceive::calibration
