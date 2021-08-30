
#include <opencv2/opencv.hpp>

#include "plane-opts-data.hpp"

#include "perceive/graphics.hpp"

#include "perceive/scene/scene-description-info.hpp"

#include "perceive/io/fp-io.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/cli-utils.hpp"
#include "perceive/utils/eigen-helpers.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/tick-tock.hpp"

#define This PlaneOpsData

namespace perceive::calibration
{
// ------------------------------------------------------------------------ init
//
bool This::init(const PlaneSetCalibParams& params)
{
   outdir = params.outdir;
   f2dp   = params.f2dp;

   // ---- Load image ----
   {
      cv::Mat im01 = cv::imread(params.image_fname);
      hsplit(im01, cv_im[0], cv_im[1]);
      for(size_t i = 0; i < 2; ++i) {
         cv::cvtColor(cv_im[i], cv_grey[i], cv::COLOR_BGR2GRAY);
         // im[i] = cv_to_argb(cv_im[i]);
         cv::imwrite(format("{}/a_{}_colour.png", outdir, i), cv_im[i]);
         cv::imwrite(format("{}/a_{}_grey.png", outdir, i), cv_grey[i]);
      }
      w = cv_im[0].cols;
      h = cv_im[0].rows;
      INFO(format(
          "loaded '{}' to produce {}x{} image pair", params.image_fname, w, h));
   }

   // ---- Load camera models ----
   {
      SceneDescriptionInfo scene_info;

      try {
         load(scene_info, params.scene_fname);
      } catch(std::exception& e) {
         LOG_ERR(
             format("failed to load '{}': {}", params.scene_fname, e.what()));
         return false;
      }
      INFO(format("loaded scene '{}'", params.scene_fname));

      if(unsigned(params.bcam_index) >= scene_info.bcam_transforms.size()) {
         LOG_ERR(format("bcam index '{}' out of range. ({} bcams.)",
                        params.bcam_index,
                        scene_info.bcam_transforms.size()));
         return false;
      }

      scene_key = scene_info.scene_key;
      et0       = scene_info.bcam_transforms[size_t(params.bcam_index)];

      try {
         fetch(bcam_info, scene_info.bcam_keys[size_t(params.bcam_index)]);
      } catch(std::exception& e) {
         LOG_ERR(format("failed to fetch bcam '{}': {}",
                        scene_info.bcam_keys[size_t(params.bcam_index)],
                        e.what()));
         return false;
      }

      for(size_t i = 0; i < 2; ++i) {
         bcam_info.M[i].set_working_format(unsigned(w), unsigned(h));
         cu[i].init(bcam_info.M[i]);
         cu[i].set_working_format(unsigned(w), unsigned(h));
      }
      INFO(format("loaded bcam '{}'", bcam_info.camera_id));
   }

   // ---- Load CalibPlaneSet ----
   {
      vector<CalibPlaneSet> all_cps;
      try {
         load(all_cps, params.calib_plane_set_fname);
      } catch(std::exception& e) {
         LOG_ERR(format("failed to load '{}': {}",
                        params.calib_plane_set_fname,
                        e.what()));
         return false;
      }

      if(unsigned(params.bcam_index) >= all_cps.size()) {
         LOG_ERR(format("bcam index '{}' out of range. ({} cps.)",
                        params.bcam_index,
                        all_cps.size()));
         return false;
      }

      cps = all_cps[size_t(params.bcam_index)];

      INFO(format("loaded calib-plane-set '{}'", params.calib_plane_set_fname));
   }

   // ---- Load CalibCompletePlaneSet ----
   {
      vector<CalibPlaneSet> all_cps;
      try {
         load(all_cps, params.complete_cps_fname);
      } catch(std::exception& e) {
         LOG_ERR(format(
             "failed to load '{}': {}", params.complete_cps_fname, e.what()));
         return false;
      }

      if(unsigned(params.bcam_index) >= all_cps.size()) {
         LOG_ERR(format("bcam index '{}' out of range. ({} complete cps.)",
                        params.bcam_index,
                        all_cps.size()));
         return false;
      }

      complete_cps = all_cps[size_t(params.bcam_index)];

      INFO(format("loaded complete-cps '{}'", params.complete_cps_fname));
   }

   // ---- Checking for duplicates in CalibPlaneSet ----
   {
      auto report = cps.duplicates_report();
      if(!report.empty()) {
         WARN(format("duplicate indices in calib-plane-set"));
         cout << report << endl;
      }
   }

   // ---- Checking for duplicates in CalibPlaneSet ----
   {
      auto report = complete_cps.duplicates_report();
      if(!report.empty()) {
         WARN(format("duplicate indices in complete-cps"));
         cout << report << endl;
      }
   }

   // ---- Perform f2d initialization ----
   {
      for(size_t i = 0; i < 2; ++i)
         init_2d_features(f2d[i], cv_im[i], cv_grey[i], nullptr, f2dp, []() {
            return false;
         });
      for(int i = 0; i < 2; ++i) {
         make_slic_plane_image(*this, i).save(
             format("{}/b_{}_slic-planes.png", outdir, i));
      }
   }

   finish_init();

   return true;
}

// ------------------------------------------------------------------------ init
//
// bool This::init(const TweakerResults& dat,
//                 int cam_ind,
//                 const CalibPlaneSet& in_cps,
//                 const CalibPlaneSet& in_complete_cps)
// {
//    outdir     = "/tmp"s;
//    scene_key  = dat.scene_info.scene_key;
//    bcam_info  = dat.bcams[cam_ind];
//    f2dp       = dat.p[0].f2d_params;
//    cv_im[0]   = dat.raw(cam_ind, 0);
//    cv_im[1]   = dat.raw(cam_ind, 1);
//    w          = cv_im[0].cols;
//    h          = cv_im[0].rows;
//    cv_grey[0] = dat.grey(cam_ind, 0);
//    cv_grey[1] = dat.grey(cam_ind, 1);

//    f2d[0]           = dat.f2d(cam_ind, 0);
//    f2d[1]           = dat.f2d(cam_ind, 1);
//    const auto& bcam = dat.bino_cam(cam_ind);
//    cu[0]            = bcam.cu(0);
//    cu[1]            = bcam.cu(1);
//    et0              = dat.scene_info.bcam_transforms[cam_ind];
//    cps              = in_cps;
//    complete_cps     = in_complete_cps;
//    finish_init();
//    return true;
// }

// ----------------------------------------------------------------- finish init
//
void This::finish_init()
{
   // ---- Set Up 'cp' lookup ----
   {
      auto process = [](const ImageFeatures2d& f2,
                        const CalibPlaneSet& cps,
                        auto cam_ind,
                        IntImage& l,
                        const string& prefix,
                        const int w,
                        const int h,
                        const string& outdir) {
         l.resize(unsigned(w), unsigned(h));
         l.fill(-1);
         for(auto cp_ind = 0u; cp_ind < cps.p3s.size(); ++cp_ind) {
            for(auto label : cps.p3s[cp_ind].indices(int(cam_ind))) {
               if(unsigned(label) >= f2.slic_info.size()) {
                  LOG_ERR(format("corrupt label found: {}", label));
                  continue;
               }
               const auto& info = f2.slic_info[size_t(label)];
               for(auto& x : info.inliers) {
                  if(!l.in_bounds(x)) {
                     LOG_ERR(format("image {}, spixel {}, has pixel {}, "
                                    "which is outside bound [{}x{}]",
                                    cam_ind,
                                    label,
                                    str(x),
                                    w,
                                    h));
                     continue;
                  }
                  l(x) = int(cp_ind);
               }
            }
         }

         // Double-check the result
         ARGBImage argb;
         argb.resize(unsigned(w), unsigned(h));
         for(auto y = 0; y < h; ++y)
            for(auto x = 0; x < w; ++x)
               argb(x, y)
                   = (l(x, y) >= 0) ? colour_set_4(unsigned(l(x, y))) : k_black;
         argb.save(format("{}/{}_{}_plane_image.png", outdir, prefix, cam_ind));
      };

      for(size_t i = 0; i < 2; ++i)
         process(f2d[i], cps, i, cp_lookup[i], "c"s, w, h, outdir);

      process(
          f2d[0], complete_cps, 0, complete_cp_lookup[0], "x"s, w, h, outdir);
   }
}

// ------------------------------------------------------- make-slic-plane-image
//
ARGBImage make_slic_plane_image(const PlaneOpsData& data, int ind)
{
   const auto& f2d         = data.f2d[size_t(ind)];
   const auto& src_im      = draw_superpixel_contours(f2d);
   const unsigned ow       = unsigned(data.w);
   const unsigned oh       = unsigned(data.h);
   const unsigned f        = 4; // 4x the size
   const unsigned n_labels = f2d.slic_numlabels;
   const auto& cps         = data.cps;

   ARGBImage im;
   im.resize(ow * f, oh * f);

   { // Copy the contour information
      for(auto y = 0u; y < oh * f; ++y)
         for(auto x = 0u; x < ow * f; ++x) im(x, y) = src_im(x / f, y / f);
   }

   auto draw_borders
       = [&](const unsigned selected_index,
             const uint32_t kolour) { // Invert the selected spixels
            auto set_pixel = [&](Point2 p, uint32_t k) {
               unsigned px = unsigned(p.x) * f, py = unsigned(p.y) * f;
               for(unsigned dy = 0; dy < f; ++dy)
                  for(unsigned dx = 0; dx < f; ++dx) im(px + dx, py + dy) = k;
            };

            auto set_pixel_v
                = [&](Vector2 p, uint32_t k) { set_pixel(to_pt2(p), k); };

            auto decorate_selected = [&](auto label) {
               if(unsigned(label) >= f2d.slic_info.size()) return;
               const auto& info = f2d.slic_info[size_t(label)];
               for(const auto& x : info.borders)
                  set_pixel(info.inliers[x], kolour);
            };

            if(selected_index < cps.p3s.size()) {
               const auto& cp = cps.p3s[selected_index];
               if(ind == 0) {
                  for(auto label : cp.l_spixel_indices)
                     decorate_selected(label);
               } else if(ind == 1) {
                  for(auto label : cp.r_spixel_indices)
                     decorate_selected(label);
               }
            }
         };

   if(cps.p3s.size() > n_colour_set_4()) {
      WARN(format("there are {} planes; however, only {} colours in our "
                  "colour set. Some colours will be repeated.",
                  cps.p3s.size(),
                  n_colour_set_4()));
   }
   for(auto i = 0u; i < cps.p3s.size(); ++i) draw_borders(i, colour_set_4(i));

   { // Now draw the labels in the centre of the super-pixels
      auto& argb = im;
      for(unsigned i = 0; i < f2d.slic_numlabels; ++i) {
         const auto centre = f2d.slic_info[i].center * double(f);
         const string s    = format("{}", i);
         int w = 0, h = 0;
         render_tiny_dimensions(s.c_str(), w, h);
         double dx = centre.x - 0.5 * double(w);
         double dy = centre.y - 0.5 * double(h);

         auto draw = [&](int x, int y, uint32_t k) {
            Point2 p
                = to_pt2(Vector2(real(x) + dx + 0.49, real(y) + dy + 0.49));
            if(argb.in_bounds(p)) argb(p) = k;
         };

         render_tiny(s.c_str(), [&](int x, int y) { draw(x, y, k_yellow); });
         render_tiny_mask(s.c_str(),
                          [&](int x, int y) { draw(x, y, k_black); });
      }
   }

   return im;
}

} // namespace perceive::calibration
