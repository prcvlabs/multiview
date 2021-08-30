
#include "tracks-exec.hpp"

#include "ops.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/cost-functions/tracks/localization-memo.hpp"
#include "perceive/cost-functions/tracks/track-ops.hpp"

#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/io/fp-io.hpp"

#define This Tracks

namespace perceive
{
// ------------------------------------------------------------------- meta-data
//
const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
   auto make_meta = [&]() {
      using ThisParams = This::Params;
      vector<MemberMetaData> m;
      m.push_back(
          MAKE_META(This::Params, UNSIGNED, fixed_window_prev_tracklet, true));
      m.push_back(MAKE_META(This::Params, UNSIGNED, random_seed, true));
      m.push_back(MAKE_META(
          This::Params, VECTOR4F, false_positive_weight_vector, true));
      m.push_back(
          MAKE_META(This::Params, FLOAT, hungarian_score_threshold, true));
      m.push_back(MAKE_META(This::Params, FLOAT, edge_d0_delta, true));
      m.push_back(MAKE_META(This::Params, FLOAT, edge_d0_scale, true));
      m.push_back(MAKE_META(This::Params, FLOAT, edge_d1_delta, true));
      m.push_back(MAKE_META(This::Params, FLOAT, edge_d1_scale, true));
      m.push_back(MAKE_META(This::Params, FLOAT, edge_dt_delta, true));
      m.push_back(MAKE_META(This::Params, FLOAT, edge_dt_scale, true));
      m.push_back(MAKE_META(This::Params, FLOAT, merge_threshold, true));
      m.push_back(
          MAKE_META(This::Params, STRING, export_training_data_prefix, true));
      m.push_back({meta_type::STRING,
                   "method"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      return std::any(string(str(o.method)));
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o         = *reinterpret_cast<ThisParams*>(ptr);
                      const string& s = std::any_cast<const string>(x);
                      o.method        = to_method(s);
                   }});
      return m;
   };
   static vector<MemberMetaData> meta = make_meta();
   return meta;
}

const char* This::Params::str(const Method m) noexcept
{
   switch(m) {
   case HUN1: return "HUN1";
   case HUN2: return "HUN2";
   case SP1: return "SP1";
   }
}

This::Params::Method This::Params::to_method(const string_view s) noexcept
{
   if(s == "HUN1") return HUN1;
   if(s == "HUN2") return HUN2;
   if(s == "SP1") return SP1;
   FATAL("kBAM!");
   return HUN1;
}

// ---------------------------------------------------------------- memory-usage
//
size_t Tracks::memory_usage() const noexcept
{
   return sizeof(Tracks) + vector_memory_usage(seqs, [](const auto& o) {
             return o.memory_usage();
          });
}

// ------------------------------------------------------------------- to-string
//
string Tracks::to_string() const noexcept
{
   const auto tracks_s = implode(
       cbegin(seqs),
       cend(seqs),
       ",\n",
       [&](const tracks::NodeSequence& x) -> string {
          return format(R"V0G0N({
   id:     {},
   path:  [{}]
}{})V0G0N",
                        x.id(),
                        implode(cbegin(x.nodes()), cend(x.nodes()), ", "),
                        "");
       });

   return format(R"V0G0N(
Tracks
   start-frame:    {},
   n-frames:       {},
   wh:            [{}x{}],
   max-per-track:  {},
   max-id:         {},
   tracks: {
{}   
   }

{})V0G0N",
                 start_frame,
                 n_frames,
                 w,
                 h,
                 max_frames_per_track,
                 max_id,
                 indent(tracks_s, 6),
                 "");
}

// ------------------------------------------------------------------ calc track
//
Tracks calc_tracks(const SceneDescription& scene_desc,
                   const LocalizationData::Params& loc_params,
                   const Tracklet::Params& tracklet_params,
                   const Tracks::Params& params,
                   const real frame_duration,
                   const int max_frames_per_tracklet,
                   const Tracklet* tracklet,
                   const Tracklet* prev_tracklet,
                   Tracks* prev_track,
                   std::function<bool()> in_is_cancelled,
                   const string_view outdir,
                   const bool feedback) noexcept(false)
{
   std::function<bool()> is_cancelled
       = (in_is_cancelled ? in_is_cancelled : []() { return false; });

   if(is_cancelled()) return {};

   auto computation = tracks::calc_tracks_comp_data(scene_desc,
                                                    loc_params,
                                                    tracklet_params,
                                                    params,
                                                    frame_duration,
                                                    max_frames_per_tracklet,
                                                    tracklet,
                                                    prev_tracklet,
                                                    prev_track,
                                                    is_cancelled,
                                                    outdir,
                                                    feedback);

   return computation->execute();
}

} // namespace perceive
