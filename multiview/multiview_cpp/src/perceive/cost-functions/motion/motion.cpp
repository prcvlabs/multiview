
#include "motion.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define This MotionData

namespace perceive
{
const char* method_to_str(const MotionData::Method m)
{
   switch(m) {
   case MotionData::SIMPLE_MOTION: return "simple-motion";
   case MotionData::CV_MOG: return "cv-mog";
   case MotionData::CV_MOG2: return "cv-mog-2";
   case MotionData::CV_GMG: return "cv-gmg";
   case MotionData::CV_CNT: return "cv-cnt";
   case MotionData::CV_GSOC: return "cv-gsoc";
   case MotionData::CV_LSBP: return "cv-lsbp";

   default: FATAL("unknown method");
   }
   return nullptr;
}

// ------------------------------------------------------------------ operator==

bool This::Params::operator==(const Params& o) const noexcept
{
#define TEST(x) (x == o.x)
#define TEST_REAL(x) (fabs(x - o.x) < 1e-9)
   return true && TEST(method) && TEST(use_LAB_diff) // -- Simple Motion
          && TEST_REAL(binary_threshold) && TEST_REAL(min_contour_area)
          && TEST_REAL(learning_rate) // -- CV (all CV methods)
          && TEST(history)            // -- MOG
          && TEST(n_mixtures) && TEST_REAL(background_ratio)
          && TEST_REAL(noise_sigma) && TEST_REAL(var_threshold)       // -- MOG2
          && TEST(detect_shadows) && TEST(n_init_frames)              // -- GMG
          && TEST_REAL(decision_threshold) && TEST(min_pix_stability) // -- CNT
          && TEST(max_pix_stability) && TEST(use_history) && TEST(is_parallel)
          && TEST(n_samples) // -- GSOC
          && TEST_REAL(replace_rate) && TEST_REAL(propagation_rate)
          && TEST(hits_threshold) && TEST_REAL(alpha) && TEST_REAL(beta)
          && TEST_REAL(blinking_supression_decay)
          && TEST_REAL(blinking_supression_multiplier)
          && TEST_REAL(noise_removal_threshold_facBG)
          && TEST_REAL(noise_removal_threshold_facFG)
          && TEST(LSBP_radius) // -- LSBP
          && TEST_REAL(t_lower) && TEST_REAL(t_upper) && TEST_REAL(t_inc)
          && TEST_REAL(t_dec) && TEST_REAL(r_scale) && TEST_REAL(r_incdec)
          && TEST(LSBP_threshold) && TEST(min_count);
#undef TEST
#undef TEST_REAL
}

// ------------------------------------------------------------------- to-string

std::string This::Params::to_string() const
{
   return format(R"V0G0N(
MotionParams

   method:             {}
   use-lab:            {}
   threshold:          {}
   min-contour-area:   {}
   
   learning-rate:      {}

   history:            {}
   n-mixtures:         {}
   background-ratio:   {}
   noise-sigma:        {}

   var-threshold:      {}
   detect-shadows:     {}   

   n-init-frames:      {}
   decision-threshold: {}

   pix-stability:     [{}..{}]
   use-history:        {}
   is-parallel:        {}

   n-samples:          {}
   replace-rate:       {}
   propagation-rate:   {}
   hits-threshold:     {}
   alpha:              {}
   beta:               {}
   blink-decay:        {}
   blink-multiplier:   {}
   noise-facBG:        {}
   noise-facFG:        {}

   LSBP-radius:        {}
   t-lower:            {}
   t-upper:            {}
   t-inc:              {}
   t-dec:              {}
   r-scale:            {}
   r-incdec:           {}
   LSBP-threshold:     {}
   min-count:          {}

{})V0G0N",
                 method_to_str(method),
                 str(use_LAB_diff),
                 binary_threshold,
                 min_contour_area,
                 learning_rate,
                 history, // MOG
                 n_mixtures,
                 background_ratio,
                 noise_sigma,
                 var_threshold, // MOG2
                 str(detect_shadows),
                 n_init_frames, // GMG
                 decision_threshold,
                 min_pix_stability,
                 max_pix_stability, // CNT
                 str(use_history),
                 str(is_parallel),
                 n_samples, // GSOC
                 replace_rate,
                 propagation_rate,
                 hits_threshold,
                 alpha,
                 beta,
                 blinking_supression_decay,
                 blinking_supression_multiplier,
                 noise_removal_threshold_facBG,
                 noise_removal_threshold_facFG,
                 LSBP_radius, // LSBP
                 t_lower,
                 t_upper,
                 t_inc,
                 t_dec,
                 r_scale,
                 r_incdec,
                 LSBP_threshold,
                 min_count,
                 "");
}

// ------------------------------------------------------------------ make-fname

string This::Params::make_fname(const string& s) const
{
   return format("{}/zzz-{:3d}_{}.png", out_dir, frame_no, s);
}

// --------------------------------------------------------------- SIMPLE MOTION

static void simple_motion(MotionData& motion,
                          const cv::Mat& im0,
                          const cv::Mat& im1,
                          const MotionData::Params& p,
                          std::function<bool(void)> is_canceled)
{
   if(is_canceled()) return;

   const auto w = im0.cols;
   const auto h = im0.rows;

   Expects(im1.cols == w);
   Expects(im1.rows == h);

   motion.frame_delta.resize(w, h);

   if(p.use_LAB_diff) {
      if(im0.type() != CV_8UC3 or im1.type() != CV_8UC3)
         FATAL("attempt to use LAB diff, but images were wrong format");

      motion.frame_delta_grey = cv::Mat(h, w, CV_8UC1);

      for(auto y = 0; y < h; ++y) {
         for(auto x = 0; x < w; ++x) {
            cv::Vec3b pix0 = im0.at<cv::Vec3b>(y, x);
            cv::Vec3b pix1 = im1.at<cv::Vec3b>(y, x);

            // auto ck0 = Vector3(pix0.val[2], pix0.val[1], pix0.val[0]);
            // auto ck1 = Vector3(pix1.val[2], pix1.val[1], pix1.val[0]);
            // auto raw_score = cie2000_compare(ck0, ck1);
            // auto z_score = (raw_score - cie2000_mean) / cie2000_stddev;
            // auto s = clamp(phi_function(z_score) * 2.0, 0.0, 1.0);

            auto s                   = cie2000_score(pix0, pix1);
            motion.frame_delta(x, y) = float(s);
            motion.frame_delta_grey.at<uint8_t>(y, x) = uint8_t(s * 255.0);
         }
      }

      cv::threshold(motion.frame_delta_grey,
                    motion.frame_thresholded,
                    p.binary_threshold * 255.0,
                    255,
                    cv::THRESH_BINARY);

      cv::Mat eroded;
      cv::erode(
          motion.frame_thresholded, eroded, cv::Mat(), cv::Point(-1, -1), 1);
      cv::dilate(
          eroded, motion.frame_dilated, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

      vector<vector<cv::Point>> contours0;
      vector<vector<cv::Point>> contours;
      vector<cv::Vec4i> hierarchy;

      cv::findContours(motion.frame_dilated,
                       contours0,
                       hierarchy,
                       cv::RETR_TREE,
                       cv::CHAIN_APPROX_SIMPLE);

      motion.frame_contours = im1.clone();

      for(const auto& contour : contours0) {
         const auto area = cv::contourArea(contour);
         // cout << format("... area = {}", area) << endl;
         if(area < p.min_contour_area) continue;
         contours.push_back(contour);
      }

      if(contours.size() > 0)
         drawContours(
             motion.frame_contours, contours, -1, cv::Scalar(128, 255, 255), 1);

      // cv::imwrite(p.make_fname("01-delta"),     motion.frame_delta_grey);
      // cv::imwrite(p.make_fname("02-threshold"), motion.frame_thresholded);
      // cv::imwrite(p.make_fname("03-dilated"),   motion.frame_dilated);
      // cv::imwrite(p.make_fname("04-contours"),  motion.frame_contours);

   } else {
      // motion.frame_delta = ;
   }
}

// ---------------------------------------------------------------------- CV MOG

static void motion_cv_MOG(MotionData& motion,
                          const cv::Mat& im0,
                          const cv::Mat& im1,
                          const MotionData::Params& p,
                          std::function<bool(void)> is_canceled)
{
   if(!motion.bs_ptr)
      motion.bs_ptr = cv::bgsegm::createBackgroundSubtractorMOG(
          p.history, p.n_mixtures, p.background_ratio, p.noise_sigma);

   motion.bs_ptr->apply(im1, motion.mog_mask, p.learning_rate);
   cv::imwrite(p.make_fname("05-cv-mask"), motion.mog_mask);
}

// --------------------------------------------------------------------- CV MOG2

static void motion_cv_MOG2(MotionData& motion,
                           const cv::Mat& im0,
                           const cv::Mat& im1,
                           const MotionData::Params& p,
                           std::function<bool(void)> is_canceled)
{
   if(!motion.bs_ptr)
      motion.bs_ptr = cv::createBackgroundSubtractorMOG2(
          p.history, p.var_threshold, p.detect_shadows);

   motion.bs_ptr->apply(im1, motion.mog_mask, p.learning_rate);
   cv::imwrite(p.make_fname("05-cv-mask"), motion.mog_mask);
}

// ---------------------------------------------------------------------- CV GMG

static void motion_cv_GMG(MotionData& motion,
                          const cv::Mat& im0,
                          const cv::Mat& im1,
                          const MotionData::Params& p,
                          std::function<bool(void)> is_canceled)
{
   if(!motion.bs_ptr)
      motion.bs_ptr = cv::bgsegm::createBackgroundSubtractorGMG(
          p.n_init_frames, p.decision_threshold);

   motion.bs_ptr->apply(im1, motion.mog_mask, p.learning_rate);
   cv::imwrite(p.make_fname("05-cv-mask"), motion.mog_mask);
}

// ---------------------------------------------------------------------- CV CNT

static void motion_cv_CNT(MotionData& motion,
                          const cv::Mat& im0,
                          const cv::Mat& im1,
                          const MotionData::Params& p,
                          std::function<bool(void)> is_canceled)
{
   if(!motion.bs_ptr)
      motion.bs_ptr
          = cv::bgsegm::createBackgroundSubtractorCNT(p.min_pix_stability,
                                                      p.use_history,
                                                      p.max_pix_stability,
                                                      p.is_parallel);

   motion.bs_ptr->apply(im1, motion.mog_mask, p.learning_rate);
   cv::imwrite(p.make_fname("05-cv-mask"), motion.mog_mask);
}

// --------------------------------------------------------------------- CV GSOC

static void motion_cv_GSOC(MotionData& motion,
                           const cv::Mat& im0,
                           const cv::Mat& im1,
                           const MotionData::Params& p,
                           std::function<bool(void)> is_canceled)
{
   if(!motion.bs_ptr)
      motion.bs_ptr = cv::bgsegm::createBackgroundSubtractorGSOC(
          cv::bgsegm::LSBP_CAMERA_MOTION_COMPENSATION_NONE,
          p.n_samples,
          float(p.replace_rate),
          float(p.propagation_rate),
          p.hits_threshold,
          float(p.alpha),
          float(p.beta),
          float(p.blinking_supression_decay),
          float(p.blinking_supression_multiplier),
          float(p.noise_removal_threshold_facBG),
          float(p.noise_removal_threshold_facFG));

   motion.bs_ptr->apply(im1, motion.mog_mask, p.learning_rate);
   cv::imwrite(p.make_fname("05-cv-mask"), motion.mog_mask);
}

// --------------------------------------------------------------------- CV LSBP

static void motion_cv_LSBP(MotionData& motion,
                           const cv::Mat& im0,
                           const cv::Mat& im1,
                           const MotionData::Params& p,
                           std::function<bool(void)> is_canceled)
{
   if(!motion.bs_ptr)
      motion.bs_ptr = cv::bgsegm::createBackgroundSubtractorLSBP(
          cv::bgsegm::LSBP_CAMERA_MOTION_COMPENSATION_NONE,
          p.n_samples,
          p.LSBP_radius,
          float(p.t_lower),
          float(p.t_upper),
          float(p.t_inc),
          float(p.t_dec),
          float(p.r_scale),
          float(p.r_incdec),
          float(p.noise_removal_threshold_facBG),
          float(p.noise_removal_threshold_facFG),
          p.LSBP_threshold,
          p.min_count);

   motion.bs_ptr->apply(im1, motion.mog_mask, p.learning_rate);
   cv::imwrite(p.make_fname("05-cv-mask"), motion.mog_mask);
}

// ----------------------------------------------------------------- init motion

void init_motion(MotionData& motion,
                 const cv::Mat& im0,
                 const cv::Mat& im1,
                 const MotionData::Params& p,
                 std::function<bool(void)> is_canceled)
{
   switch(p.method) {
   case This::SIMPLE_MOTION:
      return simple_motion(motion, im0, im1, p, is_canceled);
   case This::CV_MOG: return motion_cv_MOG(motion, im0, im1, p, is_canceled);
   case This::CV_MOG2: return motion_cv_MOG2(motion, im0, im1, p, is_canceled);
   case This::CV_GMG: return motion_cv_GMG(motion, im0, im1, p, is_canceled);
   case This::CV_CNT: return motion_cv_CNT(motion, im0, im1, p, is_canceled);
   case This::CV_GSOC: return motion_cv_GSOC(motion, im0, im1, p, is_canceled);
   case This::CV_LSBP: return motion_cv_LSBP(motion, im0, im1, p, is_canceled);
   default: FATAL(format("unknown motion method: {}", p.method));
   }
}

} // namespace perceive
