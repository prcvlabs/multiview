
#include "tracklet-ops.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/tracklets/merge-aliased-op.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/optimization/hungarian-algorithm.hpp"

namespace perceive
{
// ------------------------------------------------------------------- load/save
//
vector<Tracklet> load_tracklets_data(FILE* fp) noexcept(false)
{
   const Tracklet tracklet0;
   vector<Tracklet> out;
   try {
      string s;
      unsigned N = 0;
      load_str(fp, s);
      if(s != "TRACKLETS-DATA"s)
         throw std::runtime_error(
             format("corrupt header: expected '{}', but got '{}'",
                    "TRACKLETS-DATA"s,
                    s));
      load_uint(fp, N);
      out.resize(N);
      for(auto& tt : out) { tt.read(fp); }
   } catch(std::exception& e) {
      LOG_ERR(format("exception while loading tracks data: {}", e.what()));
      throw e;
   }
   return out;
}

void save_tracklets_data(
    FILE* fp,
    const unsigned N_tracklets,
    std::function<const Tracklet*(unsigned)> get_tracklet) noexcept
{
   try {
      save_str(fp, "TRACKLETS-DATA"s);
      save_uint(fp, N_tracklets);
      const Tracklet* tt0_ptr = nullptr;
      for(auto i = 0u; i < N_tracklets; ++i) {
         if(tt0_ptr == nullptr) tt0_ptr = get_tracklet(0);
         auto tt_ptr = get_tracklet(i * tt0_ptr->p.max_frames_per_tracklet);
         Expects(tt_ptr != nullptr);
         tt_ptr->write(fp);
      }
   } catch(std::exception& e) {
      FATAL(format("exception while saving tracks data: {}", e.what()));
   }
}

// --------------------------------------------------------- make tracklet image
//
ARGBImage make_tracklet_image(const Tracklet& tracklet,
                              const LocalizationData& loc_data,
                              const int frame_no) noexcept
{
   Expects(frame_no >= tracklet.start_frame);
   Expects(frame_no < tracklet.start_frame + tracklet.n_frames);

   ARGBImage argb;
   argb = grey16_im_to_argb(loc_data.loc_hist);
   return make_tracklet_image(tracklet, argb, frame_no);
}

// --------------------------------------------------------- make-tracklet-image
//
ARGBImage make_tracklet_image(const Tracklet& tracklet,
                              const ARGBImage& background_image,
                              const int frame_no) noexcept
{
   const bool frame_no_okay
       = (frame_no >= tracklet.start_frame)
         and (frame_no < tracklet.start_frame + tracklet.n_frames);
   if(!frame_no_okay) {
      FATAL(format("({} <= {} < {}) = {}",
                   tracklet.start_frame,
                   frame_no,
                   tracklet.start_frame + tracklet.n_frames,
                   str(frame_no_okay)));
   }

   ARGBImage argb = background_image;

   // What is our "tracklet" index?
   const int tracklet_idx
       = tracklet.start_frame / int(tracklet.p.max_frames_per_tracklet);

   // The frame-no shifted to the range [0..n_frames)
   const int t_frame_no = frame_no - tracklet.start_frame;

   // Now draw the tracks...
   // for(auto i = 0u; i < tracklet.tracks.tracks.size(); ++i) {
   //    const auto& track     = tracklet.tracks.tracks[i];
   //    const auto& path      = track.path;
   //    const uint32_t kolour = colour_set_4(i);
   //    for(const auto p : path) {
   //       if(p.t == frame_no) {
   //          fill_circle(argb, Point2(p.x, p.y), kolour, 4);
   //          // if(draw_labels) {
   //          //    string label    = format("{}", track.id);
   //          //    const Point2 sz = render_tiny_dimensions(label);
   //          //    const auto dxy  = Point2(-sz.x * 0.5, -sz.y * 0.5);
   //          //    render_string(
   //          //        argb, label, Point2(p.x, p.y) + dxy, k_yellow,
   //          k_black);
   //          // }
   //       }
   //    }
   // }

   return argb;
}

} // namespace perceive
