
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "photo-consistency.hpp"

namespace perceive
{
// ---------------------------------------------------------- colour-consistency

real colour_consistency(const Vector2& D0, // Distorted location we're checking
                        const unsigned ref_camera,
                        const vector<unsigned>& cam_inds, // cams of interest
                        const vector<CameraPosOptContext>& opt_ctxs,
                        const unsigned debug_id,
                        const bool feedback)
{
   auto ret = 0.0;

   // Recover the 3D position of X
   const auto& ctx0 = opt_ctxs[ref_camera];
   const auto W     = ctx0.distorted_to_world(D0);
   if(!W.is_finite()) return ret;

   Expects(ctx0.f2d.slic_im_LAB.in_bounds(D0));
   const auto& C0 = ctx0.f2d.slic_im_LAB(D0);

   if(feedback) {
      cout << string(70, '-') << " " << debug_id << endl;
      cout << format(" * {}", ctx0.model.sensor_id()) << endl;
      cout << format(" * W  = {}", str(W)) << endl;
      cout << format(" * D0 = {}", str(D0)) << endl;
      cout << format(" * C0 = {}", str(C0)) << endl;
   }

   auto counter = 0;
   for(auto ind : cam_inds) {
      if(ind == ref_camera) continue;

      const auto& ctx      = opt_ctxs[ind];
      const auto D1        = ctx.world_to_distorted(W);
      const auto& LAB      = ctx.f2d.slic_im_LAB;
      const auto in_bounds = LAB.in_bounds(D1);

      // Is the point in front of the camera?
      // What is the angle/distance between the two views??

      if(feedback) {
         cout << format(
             "   + D1 = {} ({} bounds)", str(D1), (in_bounds ? "in" : "out of"))
              << endl;
      }

      auto val = 0.0;
      if(LAB.in_bounds(D1)) {
         const auto& C1 = LAB(D1);
         val            = real(cie1976_normalized_distance(C0, C1));
         ++counter;

         if(feedback) {
            cout << format("   + C1 = {}", str(C1)) << endl;
            cout << format("   + dist = {}", val) << endl;
         }

      } else {
      }

      ret += val;
   }

   ret /= real(counter);

   if(feedback) {
      cout << format(" - ret={}", ret) << endl;
      cout << endl;
   }

   return ret;
}

// ---------------------------------------------------------- colour-consistency

real colour_consistency(const vector<unsigned>& cam_inds, // cams of interest
                        const vector<CameraPosOptContext>& opt_ctxs,
                        const real threshold,
                        const bool spixel_centers_only,
                        SampleStatistics* sample_stats)
{
   if(opt_ctxs.size() == 0) return dNAN;

   auto ret     = 0.0;
   auto counter = 0;

   const auto dims  = opt_ctxs[0].model.working_format();
   const unsigned w = unsigned(dims.x);
   const unsigned h = unsigned(dims.y);
   vector<real> stats;
   stats.reserve(w * h * cam_inds.size());

   auto apply_xy = [&](const unsigned ind, const Vector2& X) {
      const auto val = colour_consistency(X, ind, cam_inds, opt_ctxs);
      if(sample_stats != nullptr and std::isfinite(val))
         stats.emplace_back(val);
      if(val < threshold) ++counter;
   };

   auto calc_one_ctx = [&](unsigned ind) {
      for(auto y = 0u; y < h; ++y)
         for(auto x = 0u; x < w; ++x)
            if(opt_ctxs[ind].well_calibrated(x, y))
               apply_xy(ind, Vector2(x, y));
   };

   auto calc_one_ctx_spixel_only = [&](unsigned ind) {
      const auto& ctx = opt_ctxs[ind];
      const auto N    = ctx.f2d.slic_numlabels;
      for(auto spixel_ind = 0u; spixel_ind < N; ++spixel_ind)
         apply_xy(ind, ctx.f2d.slic_info[spixel_ind].center);
   };

   if(spixel_centers_only) {
      for(auto ind : cam_inds) calc_one_ctx_spixel_only(ind);
   } else {
      for(auto ind : cam_inds) calc_one_ctx(ind);
   }

   if(sample_stats)
      *sample_stats = calc_sample_statistics(begin(stats), end(stats));

   if(true) {
      auto sum     = 0.0;
      auto counter = 0;
      auto hsum    = 0.0;
      const int N  = int(stats.size());
      std::sort(begin(stats), end(stats));
      for(const auto& val : stats) {
         const auto inv = (N - counter++) / real(N);
         hsum += inv;
         sum += val * inv;
      }
      return sum / hsum;
   }

   return counter;
}

// ------------------------------------------------------ print camera-pair info

static string camera_pair_info(const CameraPosOptContext& A,
                               const CameraPosOptContext& B)
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
   return "opengl not compiled in";
#else
   const auto et0      = A.port.cam_et();
   const auto et1      = B.port.cam_et();
   const auto etA      = et0 / et0;
   const auto etB      = et1 / et0;
   const auto baseline = etB.translation.norm();
   const auto t        = etB.translation.normalized();
   return format("{}-{} ==> baseline={}, t={}, {}",
                 A.sensor_id(),
                 B.sensor_id(),
                 baseline,
                 str(t),
                 etB.rotation.to_readable_str());
#endif
}

// ----------------------------------------------------- run-camera-optimization

real run_camera_optimization(const vector<unsigned>& cam_inds,
                             const BinocularCameraInfo& bcam_info,
                             vector<CameraPosOptContext>& opt_ctxs,
                             const bool use_nelder_mead,
                             const bool feedback)
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
   return 0.0;
#else
   Expects(cam_inds.size() == 2);
   auto ret = 0.0;

   const auto bcam_et = bcam_info.euclid_transform();
   auto& ctx0         = opt_ctxs[cam_inds[0]];
   auto& ctx1         = opt_ctxs[cam_inds[1]];

   cout << str(ctx0.port.cam_et()) << endl;
   cout << str(ctx1.port.cam_et()) << endl;

   auto [et0, et1] = estimate_bino_camera_euclid_transforms(
       bcam_info, ctx0.port.cam_et(), ctx1.port.cam_et());
   auto et_diff = et1 / et0;
   ctx0.port.set_cam_et_from_perceive_cam_et(et0);
   ctx1.port.set_cam_et_from_perceive_cam_et(et1);

   cout << str(ctx0.port.cam_et()) << endl;
   cout << str(ctx1.port.cam_et()) << endl;
   cout << str(bcam_et) << endl;

   SampleStatistics sstats;
   colour_consistency(cam_inds, opt_ctxs, 0.01, true, &sstats);
   const auto threshold = sstats.median;

   if(true) {
      const auto C0 = ctx0.port.cam_et().translation;
      const auto C1 = ctx1.port.cam_et().translation;
      const auto CC = C0 - bcam_et.translation;
      cout << format("C0 = {}", str(C0)) << endl;
      cout << format("C1 = {}", str(C1)) << endl;
      cout << format("CC = {}", str(CC)) << endl;
   }

   const auto params_per_et = 6;
   const auto n_params      = params_per_et;

   auto pack = [&](real* X) {
      const auto& et = ctx0.port.cam_et();
      const auto saa = quaternion_to_saa(et.rotation);
      for(auto i = 0; i < 3; ++i) *X++ = et.translation(i);
      for(auto i = 0; i < 3; ++i) *X++ = saa(i);
   };

   auto unpack = [&](const real* X) {
      EuclideanTransform et;
      Vector3 saa;
      for(auto i = 0; i < 3; ++i) et.translation(i) = *X++;
      for(auto i = 0; i < 3; ++i) saa(i) = *X++;
      et.rotation           = saa_to_quaternion(saa);
      EuclideanTransform ot = et_diff * et;

      ctx0.port.set_cam_et_from_perceive_cam_et(et);
      ctx1.port.set_cam_et_from_perceive_cam_et(ot);
      update_depths(cam_inds, opt_ctxs, false);
   };

   auto counter  = 0;
   auto best_err = std::numeric_limits<real>::max();
   auto fn       = [&](const real* X) {
      unpack(X);
      auto err = -colour_consistency(cam_inds, opt_ctxs, threshold);

      if(feedback) {
         if(err < best_err) {
            best_err           = err;
            auto cam_pair_info = ""s;
            if(cam_inds.size() == 2) {
               const auto& A = opt_ctxs[cam_inds[0]];
               const auto& B = opt_ctxs[cam_inds[1]];
               cam_pair_info = camera_pair_info(A, B);
            }
            cout << format(
                "#{:4d}, err = {} :: {}", counter, -err, cam_pair_info)
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
   int kcount    = 100000; // max interations
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
                          10,
                          kcount,
                          icount,
                          ifault);
      ynewlo = fn(&xmin[0]);
   } else {
      method = "nelder-mead";

      vector<real> step(n_params);

      auto X = &step[0];
      for(auto i = 0; i < 3; ++i) *X++ = 0.01;            // 1cm
      for(auto i = 0; i < 3; ++i) *X++ = to_radians(2.0); // 2 degrees

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
      auto str_f
          = [&](unsigned ind) -> string { return opt_ctxs[ind].sensor_id(); };
      INFO(format("Feedback for positioning sensors: [{}]",
                  implode(cbegin(cam_inds), cend(cam_inds), ", ", str_f)));
      cout << format("   iterations:           {}", icount) << endl;
      cout << format("   fault-code:           {}", ifault) << endl;
      auto msg = (use_nelder_mead) ? nelder_mead_fault_str(ifault)
                                   : levenberg_marquardt_fault_str(ifault);
      cout << format("   fault-string:         {}", msg) << endl;
      cout << endl;
      cout << format("   initial-score:        {}", ystartlo) << endl;
      cout << format("   final-score:          {}", ynewlo) << endl;
      cout << endl;
   }

   return ret;
#endif
}

} // namespace perceive
