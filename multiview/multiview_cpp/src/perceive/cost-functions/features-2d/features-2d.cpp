
#include "cuda/perceive/graphics/slic.hpp"
#include "features-2d.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

#define This ImageFeatures2d

namespace perceive
{
// ------------------------------------------------------------------- meta data

const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
#define ThisParams ImageFeatures2d::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, UNSIGNED, scharr_blur_sz, true));
      m.push_back(MAKE_META(ThisParams, REAL, scharr_blur_sigma, true));
      m.push_back(MAKE_META(ThisParams, REAL, scharr_threshold_absdevs, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, block_size, true));
      m.push_back(MAKE_META(ThisParams, REAL, harris_k, true));
      m.push_back(MAKE_META(ThisParams, BOOL, is_subpixel, true));
      m.push_back(MAKE_META(ThisParams, BOOL, use_harris, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, max_corners, true));
      m.push_back(MAKE_META(ThisParams, REAL, quality_level, true));
      m.push_back(MAKE_META(ThisParams, REAL, min_dist, true));
      m.push_back(
          MAKE_META(ThisParams, BOOL, use_slic_cuda_if_available, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, superpixel_size, true));
      m.push_back(MAKE_META(ThisParams, REAL, compactness, true));
      m.push_back(MAKE_META(ThisParams, REAL, slic_still_cutoff, true));
      m.push_back(MAKE_META(ThisParams, BOOL, slic_edge_enhance, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, stipple_stride, true));
      m.push_back(MAKE_META(ThisParams, BOOL, test_do_opt, true));
      m.push_back(MAKE_META(ThisParams, BOOL, test_set_label, true));
      m.push_back(MAKE_META(ThisParams, REAL, test_d, true));
      m.push_back(MAKE_META(ThisParams, REAL, test_inc, true));
      m.push_back(MAKE_META(ThisParams, REAL, test_azi, true));
      return m;
   };
#undef ThisParams
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
}

// ------------------------------------------------------------ Init 2d Features

void init_2d_features(ImageFeatures2d& f2d,
                      const ARGBImage& colour,
                      const LABImage& lab,
                      const cv::Mat& grey,
                      const LABImage* still, // pass null to skip computation
                      const ImageFeatures2d::Params& p,
                      std::function<bool(void)> is_canceled,
                      const cuda::slic::SLIC& cuda_slic_context)
{
   ParallelJobSet jobs;

   f2d.w = unsigned(grey.cols);
   f2d.h = unsigned(grey.rows);

   f2d.init_scharr_s = 0.0;
   f2d.init_harris_s = 0.0;
   f2d.init_slic_s   = 0.0;

   f2d.is_empty = grey.empty();

   if(f2d.is_empty) return;

   auto stop_f = is_canceled;

   jobs.schedule([&]() {
      f2d.init_scharr_s
          = time_thunk([&]() { run_scharr(f2d, grey, p, stop_f); });
   });

   jobs.schedule([&]() {
      f2d.init_harris_s = time_thunk([&]() {
         if(!grey.empty()) run_harris(f2d, grey, p, stop_f);
      });
   });

   jobs.schedule([&]() {
      f2d.init_slic_s = time_thunk([&]() {
         Vec3fImage still3f;
         if(still) still3f = LAB_im_to_LAB_vec3f(*still);

         run_slic(f2d,
                  colour,
                  LAB_im_to_LAB_vec3f(lab),
                  (still ? &still3f : nullptr),
                  p,
                  stop_f,
                  cuda_slic_context);
      });
   });

   jobs.execute();
}

void init_2d_features(ImageFeatures2d& f2d,
                      const cv::Mat& colour,
                      const cv::Mat& grey,
                      const LABImage* still, // pass null to skip computation
                      const ImageFeatures2d::Params& p,
                      std::function<bool(void)> is_canceled)
{
   ARGBImage argb{cv_to_argb(colour)};
   LABImage lab{argb_to_LAB_im(argb)};
   init_2d_features(
       f2d, argb, lab, grey, still, p, is_canceled, cuda::slic::SLIC());
}

// ---------------------------------------------------- draw superpixel contours

ARGBImage draw_superpixel_contours(const ImageFeatures2d& f2d, uint32_t color)
{
   const auto& labels = f2d.slic_labels;
   const auto& image  = f2d.slic_input;

   auto w = labels.width;
   auto h = labels.height;

   ARGBImage out{image};

   Expects(w == out.width);
   Expects(h == out.height);

   auto label_at = [&](int x, int y) {
      return labels.in_bounds(x, y) ? labels(x, y) : -1;
   };

   // Draw a border wherever two different labels meet.
   for(unsigned y = 0; y < h; ++y) {
      for(unsigned x = 0; x < w; ++x) {
         int label = label_at(int(x), int(y));
         for(const auto& dxy : eight_connected) {
            if(label != label_at(int(x) + dxy.first, int(y) + dxy.second)) {
               out(x, y) = color;
               break;
            }
         }
      }
   }

   return out;
}

// ----------------------------------------------------- Make Slic Labeled Image

ARGBImage make_slic_labeled_image(const ImageFeatures2d& f2d,
                                  const bool show_still_score,
                                  const unsigned f) // 4x of size
{
   const auto& src_im      = draw_superpixel_contours(f2d);
   const unsigned ow       = src_im.width;
   const unsigned oh       = src_im.height;
   const unsigned n_labels = f2d.slic_numlabels;

   ARGBImage im;
   im.resize(ow * f, oh * f);

   { // Copy the contour information
      for(unsigned y = 0; y < oh * f; ++y)
         for(unsigned x = 0; x < ow * f; ++x) im(x, y) = src_im(x / f, y / f);
   }

   auto set_pixel = [&](Point2 p, uint32_t k) {
      unsigned px = unsigned(p.x) * f, py = unsigned(p.y) * f;
      for(unsigned dy = 0; dy < f; ++dy)
         for(unsigned dx = 0; dx < f; ++dx) im(px + dx, py + dy) = k;
   };

   if(show_still_score) {
      auto& lls = f2d.slic_labels;

      auto kolour_xy = [&](int x, int y) {
         const auto& info = f2d.slic_info[size_t(lls(x, y))];
         const auto s     = info.still_score;
         const auto k     = heat_map(2.0 * s, HeatMap::JET);
         set_pixel(Point2{x, y}, k);
      };

      auto is_border = [&](int x, int y) -> bool {
         const auto ll = lls(x, y);
         for(const auto& dxy : eight_connected) {
            const auto p = Point2{x + dxy.first, y + dxy.second};
            if(unsigned(p.x) < lls.width and unsigned(p.y) < lls.height)
               if(lls(p) != ll) return true;
         }
         return false;
      };

      for(auto y = 0; y < int(lls.height); ++y)
         for(auto x = 0; x < int(lls.width); ++x)
            if(is_border(x, y)) kolour_xy(x, y);
   }

   { // Now draw the labels in the centre of the super-pixels
      auto& argb = im;
      for(auto i = 0u; i < f2d.slic_numlabels; ++i) {
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

} // namespace perceive
