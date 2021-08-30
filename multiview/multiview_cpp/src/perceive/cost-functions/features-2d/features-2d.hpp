
#pragma once

#include <opencv2/core.hpp>

#include "cuda/perceive/graphics/slic.hpp"
#include "perceive/foundation.hpp"
#include "perceive/geometry/spatial-index.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
// Container for all 2d features
struct ImageFeatures2d
{
   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      // bool operator==(const Params&) const;
      // bool operator!=(const Params& o) const { return !(*this == o); }
      // std::string to_string() const;

      unsigned scharr_blur_sz{9}; // Scharr
      real scharr_blur_sigma{5.0};
      real scharr_threshold_absdevs{1.5};

      unsigned block_size{7}; // Harris
      real harris_k{0.04};
      bool is_subpixel{true};
      bool use_harris{true};
      unsigned max_corners{1000};
      real quality_level{0.02};
      real min_dist{9};

      bool use_slic_cuda_if_available{true}; // SLIC
      unsigned superpixel_size{430};         //{201};
      real compactness{15.0};
      real slic_still_cutoff{0.12};
      bool slic_edge_enhance{true};
      unsigned stipple_stride{7};
      bool test_do_opt{false};
      bool test_set_label{false};
      real test_d{2.0};
      real test_inc{0.0};
      real test_azi{0.0};
   };

   bool is_empty{false};

   unsigned w{0};
   unsigned h{0};

   double init_scharr_s{0.0}; // Timings
   double init_harris_s{0.0};
   double init_slic_s{0.0};

   // -- Scharr --
   Field scharr;
   FloatImage scharr_invmag;
   SampleStatistics scharr_mag_stats;
   ARGBImage scharr_binary; // thresholded

   // -- Harris --
   vector<Vector2> corners; // x, y, harris-energy
   Spatial2Index harris_index;

   // -- Slic --
   GreyImage canny;
   unsigned slic_numlabels{0};
   LABImage slic_im_LAB;
   ARGBImage slic_input;
   IntImage slic_labels;
   ARGBImage slic_inlier_lookup; // Points to index in slic-inliers
   GreyImage slic_border_lookup; // 1 if border between two spixels, 0 otherwise
   cv::Mat slic_still_corrected;

   // Immutable characteristics of super-pixels
   struct SpixelInfo
   {
      int label{0};
      bool still_cuttoff{false};
      real still_score{0.0}; // How similar spixel is to still image
      real intensity{0.0};
      Vector2 center{0.0, 0.0};         // center pixel of the spixel
      Vector3f mean_LAB{0.0, 0.0, 0.0}; //
      vector<Point2> inliers;           // all inliers to the spixel
      vector<unsigned> neighbours;      // neighbour spixels

      vector<unsigned> borders; // index into 'slic-inliers'

      // We take a sparse set of edge, corner, and pixels (neither)
      // to computer super pixel label scoring.
      // Indicies into 'pixels' for surface, corner and edge pixels
      vector<unsigned> surf_inds, edge_inds;
   };

   vector<SpixelInfo> slic_info;
};

// Initialize everything (threaded)
void init_2d_features(ImageFeatures2d& f2d,
                      const ARGBImage& colour,
                      const LABImage& lab,
                      const cv::Mat& grey,
                      const LABImage* still, // pass null to skip computation
                      const ImageFeatures2d::Params& p,
                      std::function<bool(void)> is_canceled,
                      const cuda::slic::SLIC& cuda_slic_context);

void init_2d_features(ImageFeatures2d& f2d,
                      const cv::Mat& colour,
                      const cv::Mat& grey,
                      const LABImage* still, // pass null to skip computation
                      const ImageFeatures2d::Params& p,
                      std::function<bool(void)> is_canceled);

// Individual functions
void run_scharr(ImageFeatures2d& ss,
                const cv::Mat& grey,
                const ImageFeatures2d::Params& p,
                std::function<bool(void)> is_canceled);

void run_harris(ImageFeatures2d& ss,
                const cv::Mat& grey,
                const ImageFeatures2d::Params& p,
                std::function<bool(void)> is_canceled);

void run_slic(ImageFeatures2d& ss, // Harris and Scharr _MUST_ be valid
              const ARGBImage& colour,
              const Vec3fImage& lab,
              const Vec3fImage* still, // pass null to skip computation
              const ImageFeatures2d::Params& p,
              std::function<bool(void)> is_canceled,
              const cuda::slic::SLIC& cuda_slic_context);

ARGBImage draw_superpixel_contours(const ImageFeatures2d& f2d,
                                   uint32_t color = 0x00ff00ff);

ARGBImage make_slic_labeled_image(const ImageFeatures2d& f2d,
                                  const bool show_still_score,
                                  const unsigned f = 4); // 4x of size

} // namespace perceive
