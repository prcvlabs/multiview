
#pragma once

#include <opencv2/bgsegm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
// Container for all 2d features
struct MotionData
{
   enum Method : int {
      SIMPLE_MOTION = 0,
      CV_MOG,
      CV_MOG2,
      CV_GMG,
      CV_CNT,
      CV_GSOC,
      CV_LSBP
   };

   struct Params
   {
      bool operator==(const Params&) const noexcept;
      bool operator!=(const Params& o) const noexcept { return !(*this == o); }
      std::string to_string() const;

      // Make an output filename
      string make_fname(const string& s) const;

      Method method{SIMPLE_MOTION};

      int frame_no{0}; // only used for output
      string out_dir{"/tmp"};

      // Simple-motion parameters
      bool use_LAB_diff{true};
      real binary_threshold{125.0 / 255.0};
      real min_contour_area{100.0};

      // All CV methods
      real learning_rate{-1.0};

      // MOG parameters
      int history{200};
      int n_mixtures{5};
      real background_ratio{0.7};
      real noise_sigma{0.0};

      // MOG2
      real var_threshold{16.0};
      bool detect_shadows{true};

      // GMG
      int n_init_frames{120};
      real decision_threshold{0.8};

      // CNT
      int min_pix_stability{15};
      int max_pix_stability{15 * 60};
      bool use_history{true};
      bool is_parallel{true};

      // GSOC
      int n_samples{20};
      real replace_rate{0.003};
      real propagation_rate{0.01};
      int hits_threshold{32};
      real alpha{0.01};
      real beta{0.0022};
      real blinking_supression_decay{0.1};
      real blinking_supression_multiplier{0.1};
      real noise_removal_threshold_facBG{0.0004};
      real noise_removal_threshold_facFG{0.0008};

      // LSBP
      int LSBP_radius{16};
      real t_lower{2.0};
      real t_upper{32.0};
      real t_inc{1.0};
      real t_dec{0.05};
      real r_scale{10.0};
      real r_incdec{0.005};
      int LSBP_threshold{8};
      int min_count{2};
   };

   cv::Ptr<cv::BackgroundSubtractor> bs_ptr;

   FloatImage frame_delta;
   cv::Mat frame_delta_grey;
   cv::Mat frame_thresholded;
   cv::Mat frame_dilated;
   cv::Mat frame_contours;

   cv::Mat mog_mask;
};

// Initialize everything (threaded)
void init_motion(MotionData& motion, const cv::Mat& im0, const cv::Mat& im1,
                 const MotionData::Params& p,
                 std::function<bool(void)> is_canceled);

} // namespace perceive
