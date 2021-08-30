
#include "phase-stereo-calibration.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "perceive/calibration/plane-set/cps-operations.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/optimization/golden-section-search.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
// ------------------------------------------------------------ OptimizationData

struct OptimizationData
{
   Plane p3;
   int w;
   int h;
   array<CachingUndistortInverse, 2> cu;
   bool use_cu{true};
   array<cv::Mat, 2> im;
   string outdir;

   void update_transfered(const Vector3& C,
                          const Quaternion& q,
                          ParallelJobSet& pjobs,
                          cv::Mat mapx[2],             // output
                          cv::Mat mapy[2],             // output
                          cv::Mat transfered[2]) const // output
   {
      using namespace perceive::calibration;
      // -- update transfer maps
      array<Vector3, 2> Cs{Vector3(0, 0, 0), C};
      array<Quaternion, 2> Qs{Quaternion(0, 0, 0, 1), q};
      make_cam_p3_cam_transfer_mapxy(
          true, p3, w, h, cu, use_cu, &Cs[0], &Qs[0], pjobs, mapx[0], mapy[0]);
      make_cam_p3_cam_transfer_mapxy(
          false, p3, w, h, cu, use_cu, &Cs[0], &Qs[0], pjobs, mapx[1], mapy[1]);
      // -- apply transfers
      cv::remap(im[0], transfered[1], mapx[0], mapy[0], cv::INTER_LINEAR);
      cv::remap(im[1], transfered[0], mapx[1], mapy[1], cv::INTER_LINEAR);
   }
};

// ----------------------------------------------------------------- optimize_Cq

static void optimize_Cq(const OptimizationData& opt_data,
                        const bool use_nelder_mead,
                        ParallelJobSet& pjobs,
                        Vector3& C0,
                        Quaternion& q0)
{
   using namespace perceive::calibration;

   const int n_params  = 6;
   const bool feedback = true;
   const string outdir = opt_data.outdir;
   Vector3 C{C0};
   Quaternion q{q0};
   cv::Mat mapx[2], mapy[2];
   cv::Mat dst[2]; // Left-to-right and right-to-left
   array<cv::Mat, 2> ref_im{opt_data.im};

   auto update_transfered
       = [&]() { opt_data.update_transfered(C, q, pjobs, mapx, mapy, dst); };

   auto pack = [&](real* X) {
      Vector3 saa = quaternion_to_saa(q);
      for(auto i = 0; i < 3; ++i) *X++ = C[i];
      for(auto i = 0; i < 3; ++i) *X++ = saa[i];
   };

   auto unpack = [&](const real* X) {
      Vector3 saa;
      for(auto i = 0; i < 3; ++i) C[i] = *X++;
      for(auto i = 0; i < 3; ++i) saa[i] = *X++;
      q = saa_to_quaternion(saa);
   };

   auto counter  = 0;
   auto best_err = std::numeric_limits<real>::max();
   auto fn       = [&](const real* X) {
      unpack(X);
      update_transfered();

      auto err0 = image_match_score(ref_im[0], dst[0], ""s);
      auto err1 = image_match_score(ref_im[1], dst[1], ""s);
      auto err  = 0.5 * (err0 + err1);

      if(feedback) {
         if(err < best_err) {
            best_err = err;
            cout << format("#{:4d}, err={:10.8f}, [{}]",
                           counter,
                           best_err,
                           implode(&X[0], &X[n_params], ", "))
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
   real reqmin   = 1e-6;
   real diffstep = 0.1;

   int kcount = 1000; // max interations
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
                          kcount / 10,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);
   } else {
      method = "nelder-mead";

      vector<real> step(n_params);
      auto simplex_factor = 0.05;
      for(size_t i = 0; i < n_params; ++i) step[i] = one_degree();
      nelder_mead(fn,
                  n_params,
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  kcount,
                  icount,
                  numres,
                  ifault);
   }

   fn(&xmin[0]);

   if(feedback) {
      INFO(format("Feedback for finding 'C' and 'q' ({}) params", n_params));
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;

      cout << format("C0       = {}", str(Vector3(0, 0, 0))) << endl;
      cout << format("C1       = {}", str(C)) << endl;
      cout << format("bline    = {}", C.norm()) << endl;
      cout << format("rotation = {}", q.to_readable_str()) << endl;
   }

   C0 = C;
   q0 = q;
}

// ------------------------------------------------ run phase stereo calibration

bool run_phase_stereo_calibration_old(PhaseCalibrationParams& p) noexcept
{
   const int w = int(p.images[0][0].width);
   const int h = int(p.images[0][0].height);
   if(!p.do_undistort) INFO(format("Not performing undistortion"));

   // -- Initialize optimization data
   OptimizationData opt_data;
   {
      opt_data.p3 = Plane(0.0, 0.0, 1.0, -p.distance);
      opt_data.w  = w;
      opt_data.h  = h;
      for(auto i = 0u; i < 2; ++i) opt_data.im[i] = argb_to_cv(p.images[0][i]);
      INFO(format("[wxh] = [{}x{}]", w, h));
      if(p.do_undistort) {
         for(auto i = 0u; i < 2; ++i) {
            opt_data.cu[i].init(p.sensors[i]);
            opt_data.cu[i].set_working_format(w, h);
         }
      }
      opt_data.use_cu = p.do_undistort;
      for(auto i = 0u; i < 2; ++i) opt_data.im[i] = argb_to_cv(p.images[0][i]);
      opt_data.outdir = p.outdir;

      INFO(format("Optimizing using plane {}", str(opt_data.p3)));
   }

   { // -- Save the initial input images (i.e., feedback)
      for(unsigned i{0}; i < p.images[0].size(); ++i)
         p.images[0][i].save(format("{}/aaa_input-images_{}.png", p.outdir, i));
   }

   // -- Initialize the optimization procedure
   Vector3 C0    = Vector3(0.16, 0.0, 0.0);
   Quaternion q0 = Quaternion(0, 0, 0, 1); // identity quaternion
   ParallelJobSet pjobs;
   cv::Mat mapx[2], mapy[2], dst[2];

   { // Save what the initial calibration looks like
      opt_data.update_transfered(C0, q0, pjobs, mapx, mapy, dst);
      cv::imwrite(format("{}/bbb_initial-warp-0.png", p.outdir), dst[0]);
      cv::imwrite(format("{}/bbb_initial-warp-1.png", p.outdir), dst[1]);
      const auto fname00 = format("{}/ccc_initial-score-0.png", p.outdir);
      const auto fname11 = format("{}/ccc_initial-score-1.png", p.outdir);
      calibration::image_match_score(opt_data.im[0], dst[0], fname00);
      calibration::image_match_score(opt_data.im[1], dst[1], fname11);
   }

   optimize_Cq(opt_data, true, pjobs, C0, q0);
   optimize_Cq(opt_data, false, pjobs, C0, q0);

   { // Save what the final calibration looks like
      opt_data.update_transfered(C0, q0, pjobs, mapx, mapy, dst);
      cv::imwrite(format("{}/ddd_final-warp-0.png", p.outdir), dst[0]);
      cv::imwrite(format("{}/ddd_final-warp-1.png", p.outdir), dst[1]);
      const auto fname00 = format("{}/eee_final-score-0.png", p.outdir);
      const auto fname11 = format("{}/eee_final-score-1.png", p.outdir);
      calibration::image_match_score(opt_data.im[0], dst[0], fname00);
      calibration::image_match_score(opt_data.im[1], dst[1], fname11);
   }

   BinocularCameraInfo bcam_info;
   { // -- Save the bcam object
      bcam_info.camera_id = p.camera_id;
      bcam_info.q         = q0;
      bcam_info.t         = -q0.rotate(C0).normalised();
      bcam_info.baseline  = C0.norm();
      for(size_t i = 0; i < 2; ++i) bcam_info.M[i] = p.sensors[i];
      Expects((bcam_info.C1() - C0).norm() < 1e-9);
      const auto outfile = format("{}/{}.json", p.outdir, p.camera_id);
      file_put_contents(outfile, bcam_info.to_json_string());
   }

   if(false) { // Let's resolve some points
               // Points are measured from a 5184x1944 image. (2592 width)
      static const Matrix3r K = Matrix3r::Identity();
      BinocularCamera bcam;
      bcam.init(bcam_info,
                unsigned(w),
                unsigned(h),
                K,
                unsigned(w),
                unsigned(h),
                true);

      int ow = 2592, oh = 1944;
      array<Vector2, 2> in_lls{{{853., 863.}, {2526., 862.}}};
      array<Vector2, 2> in_rrs{{{36., 945.}, {1745., 916.}}};
      const auto p3  = opt_data.p3;
      const auto et0 = EuclideanTransform{};

      INFO(format("p3 = {}", str(p3)));
      array<Vector2, 2> lls, rrs;
      auto f = [&](auto x) {
         return Vector2{x.x * real(w) / ow, x.y * real(h) / oh};
      };
      std::transform(begin(in_lls), end(in_lls), begin(lls), f);
      std::transform(begin(in_rrs), end(in_rrs), begin(rrs), f);

      const auto X0 = bcam.plane_ray_intersect(CAM0, et0, p3, lls[0]);
      const auto X1 = bcam.plane_ray_intersect(CAM0, et0, p3, lls[1]);
      const auto Y0 = bcam.plane_ray_intersect(CAM1, et0, p3, rrs[0]);
      const auto Y1 = bcam.plane_ray_intersect(CAM1, et0, p3, rrs[1]);

      cout << string(60, '-') << endl;
      cout << format("X0 = {}", str(X0)) << endl;
      cout << format("X1 = {}", str(X1)) << endl;
      cout << format("Y0 = {}", str(Y0)) << endl;
      cout << format("Y1 = {}", str(Y1)) << endl;
      cout << string(60, '-') << endl;
      cout << format("|X-X| = {}", (X0 - X1).norm()) << endl;
      cout << format("|Y-Y| = {}", (Y0 - Y1).norm()) << endl;
      cout << string(60, '-') << endl;
   }

   return true;
}

bool run_phase_stereo_calibration(PhaseCalibrationParams& p) noexcept
{
   FATAL("new phase-stereo");
   return true;
}

} // namespace perceive
