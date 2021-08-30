
#include "interpolate.hpp"

#include "p2d-affinity.hpp"

#include "perceive/geometry/skeleton/2d-helpers.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/movie/ffmpeg.hpp"

namespace perceive
{
// --------------------------------------- is-projective-interpolation-candidate
//
bool is_projective_interpolation_candidate(
    const Skeleton2DInterpolation::Params& params,
    const Skeleton2D& p2d0, // pd20.sensor_no() == p2d1.sensor_no()
    const Skeleton2D& p2d1) noexcept
{
   const int frame_0 = p2d0.frame_no();
   const int frame_N = p2d1.frame_no();

   if(frame_N - frame_0 < 1) return false;
   if(unsigned(frame_N - frame_0) > params.frame_t_window) return false;
   if(p2d0.sensor_no() != p2d1.sensor_no()) return false;

   Expects(p2d0.keypoints().size() == p2d1.keypoints().size());
   float average_projective_distance = 0.0f;
   int counter                       = 0;
   const int N                       = int(p2d0.keypoints().size());
   for(auto i = 0; i < N; ++i) {
      const auto kp0 = p2d0.keypoints()[size_t(i)].xy();
      const auto kp1 = p2d1.keypoints()[size_t(i)].xy();
      if(kp0.is_finite() && kp1.is_finite()) {
         ++counter;
         average_projective_distance += (kp1 - kp0).norm();
      }
   }

   if(counter > 1) average_projective_distance /= float(counter);

   // N-frames
   const float n_frames = float(frame_N - frame_0);
   average_projective_distance /= n_frames;

   return (counter > 0)
          && average_projective_distance < params.projective_distance_threshold;
}

// ---------------------------------------------------- projective-interpolation
//
static float calc_average_keypoint_shift(const Skeleton2D& A,
                                         const Skeleton2D& B)
{
   float sum    = 0.0f;
   int counter  = 0;
   const auto N = A.keypoints().size();
   for(size_t i = 0; i < N; ++i) {
      const auto& kA = A.keypoints()[i];
      const auto& kB = B.keypoints()[i];
      if(kA.is_valid() && kB.is_valid()) {
         sum += (kA.xy() - kB.xy()).norm();
         ++counter;
      }
   }
   return sum / (counter == 0 ? 1.0f : float(counter));
}

Skeleton2DInterpolation projective_interpolation(
    const Skeleton2DInterpolation::Params& params,
    const Skeleton2D::Params& p2d_params,
    const Skeleton2D& p2d_0, // pd20.sensor_no() == p2d1.sensor_no()
    const Skeleton2D& p2d_n,
    const vector<LABImage>& patches_0,
    const vector<LABImage>& patches_n,
    const DistortedCamera* dcam_ptr,
    std::function<const LABImage*(int frameno, int sensor_no)> get_lab)
{
   ParallelJobSet pjobs;

   const int frame_0 = p2d_0.frame_no();
   const int frame_N = p2d_n.frame_no();

   Expects(frame_N > frame_0 + 1);
   Expects(p2d_0.sensor_no() == p2d_n.sensor_no());
   Skeleton2DInterpolation out;

   const auto sensor_no = p2d_0.sensor_no();
   const auto N         = frame_N - frame_0 - 1;

   { // Calculate the 2d interpolations...
      auto process_i = [&](int i) {
         Skeleton2DInfo& info = out.p2d_infos[size_t(i)];

         auto p2d_ptr = make_shared<Skeleton2D>(Skeleton2D::interpolate(
             p2d_params, p2d_0, p2d_n, i + 1 + p2d_0.frame_no(), dcam_ptr));
         Expects(p2d_ptr != nullptr);
         const auto& lab_im
             = *get_lab(p2d_ptr->frame_no(), p2d_ptr->sensor_no());
         vector<SparseHistCell> hist;
         info.init(-1,
                   p2d_ptr,
                   lab_im,
                   params.patch_w,
                   params.patch_h,
                   std::move(hist),
                   0.0);
      };

      out.p2d_infos.resize(size_t(N));
      for(auto i = 0; i < N; ++i)
         pjobs.schedule([&process_i, i]() { process_i(i); });
      pjobs.execute();
   }

   if(N > 0) { // Fill out the geometry score
      float sum_keypoint_shift = 0.0f;
      sum_keypoint_shift
          += calc_average_keypoint_shift(p2d_0, *out.p2d_infos.front().p2d_ptr);
      for(auto i = 1u; i < out.p2d_infos.size(); ++i)
         sum_keypoint_shift += calc_average_keypoint_shift(
             *out.p2d_infos[i - 1].p2d_ptr, *out.p2d_infos[i - 0].p2d_ptr);
      sum_keypoint_shift
          += calc_average_keypoint_shift(*out.p2d_infos.back().p2d_ptr, p2d_n);
      out.geometry_score = sum_keypoint_shift / float(frame_N - frame_0);
   }

   if(N > 0) { // Fill out the LAB score
      float lab_sum = 0.0f;
      lab_sum += calc_lab_patch_score(patches_0, out.p2d_infos.front().patches);
      for(auto i = 1u; i < out.p2d_infos.size(); ++i)
         lab_sum += calc_lab_patch_score(out.p2d_infos[i - 1].patches,
                                         out.p2d_infos[i - 0].patches);
      lab_sum += calc_lab_patch_score(out.p2d_infos.back().patches, patches_n);

      out.lab_score = lab_sum / float(frame_N - frame_0);
   }

   return out;
}

// --------------------------------------------------------- create-interp-movie
//
void create_interp_movie(
    const string_view filename,
    const Skeleton2DInfo& info0,
    const Skeleton2DInfo& infoN,
    const Skeleton2DInterpolation& interp,
    std::function<const LABImage*(int frameno, int sensor_no)> get_lab)
{
   static std::mutex padlock;
   lock_guard lock(padlock);

   auto process_frame = [&](auto& encoder, const auto& info) {
      const int frame_no  = info.p2d_ptr->frame_no();
      const int sensor_no = info.p2d_ptr->sensor_no();
      const auto lab_ptr  = get_lab(frame_no, sensor_no);
      Expects(lab_ptr);
      ARGBImage argb = LAB_im_to_argb(*lab_ptr);

      // Place the frame number
      render_string(
          argb, format("{}", frame_no), Point2(9, 9), k_yellow, k_black);

      // Draw the skeleton
      render_pose(argb, *info.p2d_ptr);

      // Push to movie
      encoder.push_frame(argb_to_cv(argb));
   };

   const auto lab0_ptr
       = get_lab(info0.p2d_ptr->frame_no(), info0.p2d_ptr->sensor_no());
   Expects(lab0_ptr != nullptr);

   auto encoder = movie::StreamingFFMpegEncoder::create(
       filename, int(lab0_ptr->width), int(lab0_ptr->height), 15.0);

   process_frame(encoder, info0);
   for(const auto& info : interp.p2d_infos) process_frame(encoder, info);
   process_frame(encoder, infoN);

   encoder.close();
}

} // namespace perceive
