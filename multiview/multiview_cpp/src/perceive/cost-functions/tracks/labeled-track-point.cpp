
#include "labeled-track-point.hpp"

#define This LabeledTrackPoint

namespace perceive
{
// --------------------------------------------------------------------- to-json
//
Json::Value This::to_json() const noexcept
{
   auto x        = Json::Value{Json::objectValue};
   x["track_id"] = json_save(track_id);
   x["tp"]       = tp.to_json();
   return x;
}

bool This::load_json(const Json::Value& x) noexcept(false)
{
   This o;

   const auto op = "reading LabeledTrackPoint";
   if(!json_try_load_key(o.track_id, x, "track_id", op)) return false;
   if(!o.tp.load_json(x["tp"])) {
      WARN(format("failed to load track-point while {}", op));
      return false;
   }

   *this = o;

   return true;
}

// ------------------------------------------------------- tracks-to-trackpoints
//
vector<LabeledTrackPoint>
tracks_to_trackpoints(const vector<Track>& tracks) noexcept
{
   int sz = 0;
   for(const auto& tt : tracks) sz += tt.path.size();

   vector<LabeledTrackPoint> tps;
   tps.reserve(size_t(sz));
   for(const auto& tt : tracks) {
      for(const auto& tp : tt.path) {
         LabeledTrackPoint ltp;
         ltp.track_id = tt.id;
         ltp.tp       = tp;
         tps.push_back(ltp);
      }
   }

   return tps;
}

namespace detail
{
   static std::pair<vector<Track>, string>
   trackpoints_to_tracks_worker(const vector<LabeledTrackPoint>& in) noexcept
   {
      vector<Track> out;
      std::stringstream ss{""};

      if(in.size() == 0) return {out, ss.str()};

      { // set the output size
         int max_id = std::numeric_limits<int>::lowest();
         for(const auto& ltp : in)
            if(ltp.track_id > max_id) max_id = ltp.track_id;

         if(max_id > 10000) {
            ss << format("cowardly refusing to save {} tracks\n", max_id);
         }

         if(max_id < 0) return {out, ss.str()};

         out.resize(size_t(max_id + 1));
      }

      // Set all the ids
      for(size_t i = 0; i < out.size(); ++i) out[i].id = int(i);

      // copy in track points
      for(const auto& ltp : in) {
         const int id = ltp.track_id;
         if(id < 0) {
            ss << format(
                "skipping track-point on frame {}, with track-id '{}'\n",
                ltp.tp.t,
                id);
            continue;
         }
         Track& tt = out[size_t(id)];
         tt.id     = id;
         tt.path.push_back(ltp.tp);
      }

      // sort the track paths
      for(auto& tt : out) {
         std::sort(begin(tt.path), end(tt.path), [&](auto& tp0, auto& tp1) {
            return tp0.t < tp1.t;
         });
      }

      // sanity-checks, and warnings
      auto check_path = [&](const int id, const vector<TrackPoint>& tp) {
         if(tp.size() == 0) return;

         for(auto i = 1u; i < tp.size(); ++i) {
            if(tp[i - 1].t == tp[i - 0].t) {
               ss << format("track {} has duplicate track-point at time {}\n",
                            id,
                            tp[i].t);
            } else if(tp[i - 1].t + 1 == tp[i - 0].t) {
               // Good
            } else if(tp[i - 1].t + 1 < tp[i - 0].t) {
               ss << format(
                   "track {} is missing tracks between t={} and t={}\n",
                   id,
                   tp[i - 1].t,
                   tp[i].t);
            } else {
               FATAL("logic error");
            }
         }
      };

      for(const auto& tt : out) { check_path(tt.id, tt.path); }

      return {out, ss.str()};
   }
} // namespace detail

// ------------------------------------------------------- tracks-to-trackpoints
//
bool check_trackpoints_as_tracks(const vector<LabeledTrackPoint>& in) noexcept
{
   const auto [tts, msg] = detail::trackpoints_to_tracks_worker(in);
   if(msg.size() > 0) {
      WARN(format("errors found in trackpoints:"));
      cout << indent(msg, 3) << endl;
   }
   return msg.size() == 0; // no message => no error
}

vector<Track>
trackpoints_to_tracks(const vector<LabeledTrackPoint>& in) noexcept
{
   auto [tts, msg] = detail::trackpoints_to_tracks_worker(in);
   if(msg.size() > 0) {
      WARN(format("errors found in trackpoints:"));
      cout << indent(msg, 3) << endl;
   }
   return tts;
}

// ------------------------------------------------- extract-labeled-trackpoints
//
vector<LabeledTrackPoint>
extract_labeled_trackpoints(const vector<Track>& tracks,
                            const int frame_no) noexcept
{
   int count = 0;
   for(const auto& tt : tracks)
      for(const auto& tp : tt.path)
         if(frame_no == tp.t) ++count;

   vector<LabeledTrackPoint> o;
   o.reserve(size_t(count));
   for(const auto& tt : tracks)
      for(const auto& tp : tt.path)
         if(frame_no == tp.t) {
            LabeledTrackPoint ltp;
            ltp.track_id = tt.id;
            ltp.tp       = tp;
            o.push_back(ltp);
         }

   return o;
}

} // namespace perceive
