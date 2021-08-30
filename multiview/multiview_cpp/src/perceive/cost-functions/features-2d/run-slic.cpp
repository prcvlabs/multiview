
#include "features-2d.hpp"

#include "cuda/perceive/graphics/slic.hpp"
#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics.hpp"
#include "perceive/graphics/canny.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "perceive/contrib/SLIC/SLIC.h"

namespace perceive
{
template<typename T, typename S> bool contains(const T& container, const S& val)
{
   for(const auto& x : container)
      if(x == val) return true;
   return false;
}

// ----------------------------------------------------------- run CPU slic impl

static void run_cpu_slic_impl(const Vec3fImage& image,
                              const int superpixel_area,
                              const float compactness,
                              IntImage& klabels,
                              int& num_k)
{
   int* klabels_ptr{klabels.data()};
   SLIC slic;
   slic.DoSuperpixelSegmentation_ForGivenSuperpixelSize_LAB(image.data(),
                                                            int(image.width),
                                                            int(image.height),
                                                            klabels_ptr,
                                                            num_k,
                                                            superpixel_area,
                                                            real(compactness));
};

// ---------------------------------------------------------- run CUDA slic impl

static void run_cuda_slic_impl(const Vec3fImage& image,
                               const int segment_area,
                               const float compactness,
                               IntImage& klabels,
                               int& num_k,
                               const cuda::slic::SLIC& cuda_slic_context)
{
#ifndef WITH_CUDA
   FATAL("unreachable code");
#else

   auto result = cuda_slic_context.calc_labels(
       image, unsigned(segment_area), compactness);
   klabels = std::move(result.labels);
   num_k   = int(result.n_labels);

#endif // WITH_CUDA
}

// --------------------------------------------------------------- run slic impl

static void run_slic_impl(const ARGBImage& image,
                          const Vec3fImage& lab_image,
                          const bool in_use_cuda,
                          const int segment_area,
                          const float compactness,
                          IntImage& klabels,
                          int& num_k,
                          const cuda::slic::SLIC& cuda_slic_context)
{
   const bool use_cuda = k_cuda_is_available and in_use_cuda;
   if(in_use_cuda and !k_cuda_is_available) {
      WARN(format("cannot use cuda-slic, because it wasn't compiled in! "
                  "(Falling back to cpu-slic.)"));
   }

   if(use_cuda)
      run_cuda_slic_impl(lab_image,
                         segment_area,
                         compactness,
                         klabels,
                         num_k,
                         cuda_slic_context);
   else
      run_cpu_slic_impl(lab_image, segment_area, compactness, klabels, num_k);
}

// ------------------------------------------------------------------------ SLIC

void run_slic(ImageFeatures2d& ss,
              const ARGBImage& colour,
              const Vec3fImage& lab,
              const Vec3fImage* still, // pass null to skip computation
              const ImageFeatures2d::Params& p,
              std::function<bool(void)> is_canceled,
              const cuda::slic::SLIC& cuda_slic_context)
{
   const bool use_cuda            = p.use_slic_cuda_if_available;
   const unsigned superpixel_size = p.superpixel_size;
   const double compactness       = p.compactness;

   const unsigned w = colour.width, h = colour.height;

   Expects(ss.w != 0);
   Expects(ss.h != 0);

   ss.slic_im_LAB.resize(w, h);
   ss.slic_input.resize(w, h);
   ss.slic_labels.resize(w, h);
   ss.slic_info.clear();
   ss.slic_numlabels = 0;

   const auto& im        = ss.slic_input;
   int num_k             = 0;
   int* klabels          = nullptr;
   const uint32_t* ubuff = nullptr;
   auto s1               = 0.0;

   ARGBImage* input = &ss.slic_input;
   ARGBImage edge_enhanced;

   if(is_canceled()) goto clear;

   ss.slic_input = colour;

   if(is_canceled()) goto clear;

   { // Run canny
      GreyImage g = colour_to_grey(ss.slic_input);
      Expects(g.width == g.row_stride);
      ss.canny.resize(w, h, w);
      canny(g.pixels, w, h, ss.canny.data());
   };

   if(is_canceled()) goto clear;

   if(p.slic_edge_enhance) {
      edge_enhanced = ss.slic_input;

      for(auto y = 0u; y < h; ++y)
         for(auto x = 0u; x < w; ++x)
            if(ss.canny(x, y) > 127) edge_enhanced(x, y) = k_pink;

      input = &edge_enhanced;
   }

   if(is_canceled()) goto clear;

   run_slic_impl(ss.slic_input,
                 lab,
                 use_cuda,
                 int(superpixel_size),
                 float(compactness),
                 ss.slic_labels,
                 num_k,
                 cuda_slic_context);
   if(is_canceled()) goto clear;

   ss.slic_numlabels = unsigned(num_k);

   // ---- LAB image
   ss.slic_im_LAB = LAB_vec3f_im_to_LAB(lab);

   if(is_canceled()) goto clear;

   ss.slic_info.resize(size_t(num_k));

   for(auto i = 0; i < num_k; ++i) ss.slic_info[size_t(i)].label = i;

   { // Figure out the intensities, centres, and neighbours

      auto push_neighbour = [&](unsigned label, int x, int y) {
         if(unsigned(x) >= w || unsigned(y) >= h) return;
         unsigned neighbour = unsigned(ss.slic_labels(x, y));
         if(label != neighbour)
            if(!contains(ss.slic_info[label].neighbours, neighbour))
               ss.slic_info[label].neighbours.push_back(neighbour);
      };

      // Iterate over pixels... setting arrays
      for(unsigned y = 0; y < h; ++y) {
         for(unsigned x = 0; x < w; ++x) {
            auto label     = ss.slic_labels(x, y);
            auto argb      = im(x, y);
            auto intensity = rgb_to_grey(argb);
            auto& info     = ss.slic_info[size_t(label)];
            Expects(info.label == int(label));
            info.inliers.emplace_back(x, y);
            info.intensity += intensity;
            info.mean_LAB += lab(x, y);
            info.center += Vector2(x, y);
            for(const auto& dxy : four_connected)
               push_neighbour(
                   unsigned(label), int(x) + dxy.first, int(y) + dxy.second);
         }
      }

      // Finalize averages
      for(auto& info : ss.slic_info) {
         info.intensity /= double(info.inliers.size());
         info.mean_LAB /= float(info.inliers.size());
         info.center /= double(info.inliers.size());
      }
   }

   if(is_canceled()) goto clear;

   { // Create the border map...
      const auto w = ss.slic_labels.width;
      const auto h = ss.slic_labels.height;
      auto& edges  = ss.slic_border_lookup;
      edges.resize(w, h);

      auto is_boundary = [&](unsigned x, unsigned y) -> GreyImage::value_type {
         if(x == 0 || y == 0) return 0;
         if(x == w - 1 || y == h - 1) return 0;
         const auto label = ss.slic_labels(x, y);
         for(int dx = -1; dx <= 1; ++dx)
            for(int dy = -1; dy <= 1; ++dy)
               if(label != ss.slic_labels(int(x) + dx, int(y) + dy)) return 1;
         return 0;
      };

      for(unsigned y = 0; y < h; ++y)
         for(unsigned x = 0; x < w; ++x) edges(x, y) = is_boundary(x, y);
   }

   if(is_canceled()) goto clear;

   { // Create the lookup for slic-inliers
      const auto w        = ss.slic_labels.width;
      const auto h        = ss.slic_labels.height;
      auto& inlier_lookup = ss.slic_inlier_lookup;
      inlier_lookup.resize(w, h);
      for(const auto& info : ss.slic_info)
         for(auto i = 0u; i < info.inliers.size(); ++i)
            inlier_lookup(info.inliers[i]) = i;
   }

   if(is_canceled()) goto clear;

   // { // Where to all the corners fit in?
   //     const auto& inlier_lookup = ss.slic_inlier_lookup;
   //     ss.slic_corners.resize(num_k);
   //     for(const auto& x: ss.corners) {
   //         Point2 p(x.x, x.y); // Which label is the pixel in?
   //         if(!ss.slic_labels.in_bounds(p))
   //             FATAL("Logic Error");
   //         auto label = ss.slic_labels(p);
   //         ss.slic_corners[label].push_back(inlier_lookup(p));
   //     }
   // }

   if(is_canceled()) goto clear;

   { // Okay, where are the borders?
      // ss.slic_borders.clear();
      // ss.slic_borders.resize(num_k);
      const auto w       = ss.slic_labels.width;
      const auto h       = ss.slic_labels.height;
      const auto& labels = ss.slic_labels;
      const auto& lookup = ss.slic_inlier_lookup;
      const auto& border = ss.slic_border_lookup;
      for(unsigned y = 0; y < h; ++y)
         for(unsigned x = 0; x < w; ++x)
            if(border(x, y))
               ss.slic_info[size_t(labels(x, y))].borders.push_back(
                   lookup(x, y));
   }

   if(is_canceled()) goto clear;

   { // Get the stipple pixels
      const unsigned stipple_stride      = p.stipple_stride;
      const unsigned half_stipple_stride = stipple_stride / 2;
      auto is_stipple_point              = [&](const Point2& p) {
         auto line = unsigned(p.y) / stipple_stride;
         if(unsigned(p.y) - line * stipple_stride != 0)
            return false; // This is % op
         if(line & 0x01)
            return (unsigned(p.x) + half_stipple_stride) % stipple_stride == 0;
         return unsigned(p.x) % stipple_stride == 0;
      };

      for(auto& info : ss.slic_info) {
         // Surface points
         for(unsigned j = 0; j < info.inliers.size(); ++j) {
            const auto& x = info.inliers[j];
            if(is_stipple_point(x)) {
               if(!ss.slic_border_lookup(x)) // Must be surface
                  info.surf_inds.push_back(j);
            }
         }

         // border points
         for(auto pix_ind = 0u; pix_ind < info.borders.size(); ++pix_ind)
            if(pix_ind % stipple_stride == 0)
               info.edge_inds.push_back(info.borders[pix_ind]);
      }
   }

   ss.slic_still_corrected = argb_to_cv(colour);

   if(still && still->size() > 0) { // Get the still-scores
      Expects(ss.slic_still_corrected.cols == int(ss.slic_input.width));
      Expects(ss.slic_still_corrected.rows == int(ss.slic_input.height));

      const auto& ref = *still;
      const auto& lab = ss.slic_im_LAB;
      auto calc_score = [&](const auto& info) {
         auto err = 0.0;
         for(const auto& x : info.inliers)
            err += real(
                cie1976_normalized_distance(vec3f_LAB_to_LAB(ref(x)), lab(x)));
         return err / real(info.inliers.size());
      };

      for(auto& info : ss.slic_info) {
         info.still_score   = calc_score(info);
         info.still_cuttoff = info.still_score >= p.slic_still_cutoff;

         // Attenuate the 'value' (in hsv space) of inliers
         const auto s = std::clamp<real>(info.still_score * 2.0, 0.0, 1.0);
         cv::Vec3b blank(255, 255, 255);
         for(const auto& x : info.inliers) {
            Expects(ss.slic_input.in_bounds(x));
            // if(!info.still_cuttoff) {
            //    ss.slic_still_corrected.at<cv::Vec3b>(x.y, x.x) = blank;
            // }
            auto hsv = rgb_to_hsv(kolour_to_vector3(ss.slic_input(x)));
            hsv.z *= s;
            const auto k = rgb_to_vec3b(vector3_to_kolour(hsv_to_rgb(hsv)));
            ss.slic_still_corrected.at<cv::Vec3b>(x.y, x.x) = k;
         }
      }
   }

   if(!is_canceled()) goto end;

clear:

   ss.slic_info.clear();
   ss.slic_numlabels = 0;

end:

   // for(auto i = 0u; i < ss.slic_info.size(); ++i) {
   //     cout << format("#{:4d} --> {}", i,
   //                    ss.slic_info[i].inliers.size()) << endl;
   // }
   // FATAL("kBAM!");

   return;
}
} // namespace perceive
