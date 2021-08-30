
#include "render-tracks.hpp"

#include "movie-utils.hpp"

#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/movie/debug-movie.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/threads.hpp"

namespace perceive::movie
{
// ----------------------------------------------------------- make-tracks-image
//
ARGBImage make_tracks_image(const Tracks& tracks,
                            const LocalizationData& loc_data,
                            const int frame_no) noexcept
{
   ARGBImage argb;
   argb = grey16_im_to_argb(loc_data.loc_hist);

   // What is our "track" index?
   const int track_idx = tracks.start_frame / tracks.max_frames_per_track;

   // Now draw the tracks...
   for(const auto& seq : tracks.seqs) {
      const auto track      = node_sequence_to_track(seq);
      const uint32_t kolour = colour_set_4(unsigned(track.id));
      for(const auto& p : track.path) {
         if(p.t == frame_no) {
            const auto x = p.rounded_xy(); // centre of track
            fill_circle(argb, x, kolour, 4);
            const auto f = p.forward_vector();
            if(f.is_finite()) {
               plot_line_AA(to_vec2(x) + f * 3.0,
                            to_vec2(x) + f * 6.0,
                            [&](int x, int y, float a) {
                               if(argb.in_bounds(x, y)) {
                                  argb(x, y) = blend(k_black, argb(x, y), a);
                               }
                            });
            }
         }
      }
   }

   return argb;
}

ARGBImage make_tracks_image(const Tracks& tracks,
                            const ARGBImage& background_image,
                            const int frame_no) noexcept
{
   return make_tracks_image(
       node_sequence_to_tracks(cbegin(tracks.seqs), cend(tracks.seqs)),
       background_image,
       frame_no);
}

ARGBImage make_tracks_image(const vector<Track>& tracks,
                            const ARGBImage& background_image,
                            const int frame_no) noexcept
{
   ARGBImage argb = background_image;

   // Now draw the tracks...
   for(const auto& track : tracks) {
      const uint32_t kolour = colour_set_4(unsigned(track.id));
      for(const auto& p : track.path) {
         if(p.t == frame_no)
            fill_circle(argb, to_pt2(p.xy().round()), kolour, 4);
      }
   }

   return argb;
}

static void circle_points(const Vector3& C,
                          const Vector3& N,
                          const real r,
                          const unsigned n_divisions,
                          std::function<void(const Vector3&)> f) noexcept
{
   auto q    = Quaternion::between_vectors(Vector3(0, 0, 1), N);
   Vector3 X = q.rotate(Vector3(1, 0, 0));
   Vector3 Y = q.rotate(Vector3(0, 1, 0));

   const auto n = n_divisions;
   real step    = 2.0 * M_PI / real(n);

   for(auto i = 0u; i <= n; ++i) {
      const auto dxy = Vector2(cos(i * step), sin(i * step));
      const auto U   = C + r * (dxy.x * X + dxy.y * Y);
      f(U);
   }
}

// ------------------------------------------------- render-tracks on raw-videos
//
void render_tracks_on_raw_video(ARGBImage& im,
                                const SceneDescription& scene_desc,
                                const int sensor_no,
                                const int frame_no,
                                const vector<Track>& tracks,
                                const vector<Track>* gt_tracks,
                                const LocalizationData* loc_ptr,
                                const bool gt_truth_only) noexcept
{
   Expects(loc_ptr);
   Expects(unsigned(sensor_no) < scene_desc.sensor_transforms.size());
   const auto et = scene_desc.sensor_transforms[size_t(sensor_no)];
   const DistortedCamera& dcam = scene_desc.dcam(sensor_no);
   const int hist_w            = int(loc_ptr->loc_hist.width);
   const int hist_h            = int(loc_ptr->loc_hist.height);
   const real hist_sz          = loc_ptr->hist_sz;
   const auto& bounds          = loc_ptr->bounds;
   const auto top_left         = bounds.top_left();

   const auto mutils
       = MovieUtils(unsigned(hist_w), unsigned(hist_h), bounds, hist_sz, false);

   for(const auto& tt : tracks)
      for(const auto& tp : tt.path)
         if(tp.t == frame_no)
            mutils.render_trackpoint(im, dcam, tt.id, tp, true);

   if(gt_tracks != nullptr)
      for(const auto& tt : *gt_tracks)
         for(const auto& tp : tt.path)
            if(tp.t == frame_no)
               mutils.render_gt_trackpoint(im, dcam, tp, gt_truth_only);
}

} // namespace perceive::movie
