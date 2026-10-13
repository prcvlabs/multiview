
#include "provisional-localizer.hpp"

#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"
#include "perceive/optimization/hungarian-algorithm.hpp"

namespace perceive::pipeline::realtime
{
// ------------------------------------------------------------------- calc-peak
// The cell of the largest value of `info.prob_xy`, and that value
//
static std::pair<Point2, float> calc_peak(const Skeleton2DInfo& info) noexcept
{
   Point2 xy  = {-1, -1};
   float best = -1.0f;
   for(auto y = 0; y < int(info.prob_xy.height); ++y) {
      for(auto x = 0; x < int(info.prob_xy.width); ++x) {
         const float val = info.prob_xy(x, y);
         if(val > best) {
            best = val;
            xy   = Point2{x, y};
         }
      }
   }
   return {xy, best};
}

// ---------------------------------------------------------- cluster-detections
//
vector<vector<int>>
cluster_detections(const vector<const Skeleton2DInfo*>& infos,
                   real hist_sz,
                   real diameter) noexcept
{
   vector<vector<int>> groups;
   try {
      struct Candidate
      {
         int index  = -1;
         Point2 xy  = {-1, -1};
         float peak = 0.0f;
      };

      vector<Candidate> candidates;
      candidates.reserve(infos.size());
      for(size_t i = 0; i < infos.size(); ++i) {
         if(infos[i] == nullptr or infos[i]->prob_xy.empty()) continue;
         const auto [xy, peak] = calc_peak(*infos[i]);
         Candidate o;
         o.index = int(i);
         o.xy    = xy;
         o.peak  = peak;
         candidates.push_back(o);
      }

      std::stable_sort(
          begin(candidates), end(candidates), [](const auto& a, const auto& b) {
             return a.peak > b.peak;
          });

      vector<Point2> seeds; // the peak of each group's first detection
      for(const auto& o : candidates) {
         int best_group = -1;
         real best_dist = std::numeric_limits<real>::max();
         for(size_t j = 0; j < seeds.size(); ++j) {
            const real dist = hist_sz
                              * std::hypot(real(o.xy.x - seeds[j].x),
                                           real(o.xy.y - seeds[j].y));
            if(dist <= diameter and dist < best_dist) {
               best_dist  = dist;
               best_group = int(j);
            }
         }

         if(best_group < 0) {
            seeds.push_back(o.xy);
            groups.push_back({o.index});
         } else {
            groups[size_t(best_group)].push_back(o.index);
         }
      }
   } catch(std::exception& e) {
      LOG_ERR(format("exception clustering detections: {}", e.what()));
      groups.clear();
   }
   return groups;
}

// ------------------------------------------------------- provisional-positions
//
vector<ProvisionalPosition> provisional_positions(const SceneDescription& scene,
                                                  const LocalizationData& loc,
                                                  real diameter) noexcept
{
   vector<ProvisionalPosition> out;
   try {
      vector<const Skeleton2DInfo*> infos;
      for(const auto& sensor_infos : loc.p2ds)
         for(const auto& info : sensor_infos)
            if(info.p2d_ptr != nullptr and !info.prob_xy.empty())
               infos.push_back(&info);

      const auto groups = cluster_detections(infos, loc.hist_sz, diameter);
      out.reserve(groups.size());
      for(const auto& group : groups) {
         vector<const Skeleton2DInfo*> members;
         members.reserve(group.size());
         float sum_prob_fp = 0.0f;
         for(const auto index : group) {
            members.push_back(infos[size_t(index)]);
            sum_prob_fp += infos[size_t(index)]->prob_fp;
         }

         const auto pos = position_p2d_infos(
             &scene, &members[0], members.size(), loc.hist_sz, loc.bounds);

         ProvisionalPosition o;
         o.hist_xy = (pos.xy.x >= 0) ? pos.xy : calc_peak(*members[0]).first;
         o.X       = FloorHistogram::unproject_hist_xy(
             o.hist_xy, loc.bounds.top_left(), loc.hist_sz);
         o.n_detections = int(members.size());
         o.score        = std::isfinite(pos.score) ? pos.score : 0.0f;
         o.prob_fp      = sum_prob_fp / float(members.size());
         out.push_back(o);
      }
   } catch(std::exception& e) {
      LOG_ERR(format("exception placing the detections of frame {}: {}",
                     loc.frame_no,
                     e.what()));
      out.clear();
   }
   return out;
}

// ----------------------------------------------------------------- live-tracks
//
vector<LiveTrack> live_tracks(const Tracks& window) noexcept
{
   vector<LiveTrack> out;
   try {
      auto to_X = [&window](const tracks::Node& node) {
         return Vector2(real(node.xy().x) * window.hist_sz + window.aabb.left,
                        real(node.xy().y) * window.hist_sz + window.aabb.top);
      };

      for(const auto& seq : window.seqs) {
         const auto& nodes = seq.nodes();
         if(nodes.empty()) continue;
         const auto& last = nodes.back();
         LiveTrack o;
         o.id = seq.id();
         o.t  = last.t();
         o.X  = to_X(last);
         if(nodes.size() > 1) {
            const auto& prev = nodes[nodes.size() - 2];
            const int dt     = last.t() - prev.t();
            if(dt > 0) o.velocity = (o.X - to_X(prev)) / real(dt);
         }
         out.push_back(o);
      }
   } catch(std::exception& e) {
      LOG_ERR(format("exception reading live tracks: {}", e.what()));
      out.clear();
   }
   return out;
}

// -----------------------------------------------------------------------------
// --------------------------------------------------------- ProvisionalLabeller
// -----------------------------------------------------------------------------

// ---------------------------------------------------------------- Construction
//
ProvisionalLabeller::ProvisionalLabeller(real match_radius, int max_age)
    : match_radius_(match_radius)
    , max_age_(max_age)
{}

// ---------------------------------------------------------------------- tracks
//
vector<LiveTrack> ProvisionalLabeller::tracks() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return tracks_;
}

// ------------------------------------------------------------------ set-tracks
//
void ProvisionalLabeller::set_tracks(vector<LiveTrack> tracks) noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   tracks_ = std::move(tracks);
}

// ----------------------------------------------------------------------- label
//
void ProvisionalLabeller::label(
    int frame_no,
    vector<ProvisionalPosition>& positions) const noexcept
{
   for(auto& o : positions) o.pid = -1;

   try {
      vector<LiveTrack> candidates; // where each track is expected
      {
         std::lock_guard<std::mutex> lock(padlock_);
         for(const auto& track : tracks_) {
            const int dt = frame_no - track.t;
            if(std::abs(dt) > max_age_) continue;
            LiveTrack o = track;
            o.X         = track.X + track.velocity * real(dt);
            candidates.push_back(o);
         }
      }
      if(positions.empty() or candidates.empty()) return;

      auto dist = [&](unsigned pos_idx, unsigned track_idx) {
         return (positions[pos_idx].X - candidates[track_idx].X).norm();
      };

      const auto matches = hungarian_algorithm(
          unsigned(positions.size()),
          unsigned(candidates.size()),
          [&](unsigned l, unsigned r) { return double(dist(l, r)); });

      for(const auto& [pos_idx, track_idx] : matches)
         if(dist(pos_idx, track_idx) <= match_radius_)
            positions[pos_idx].pid = candidates[track_idx].id;
   } catch(std::exception& e) {
      LOG_ERR(format("exception labelling frame {}: {}", frame_no, e.what()));
   }
}

} // namespace perceive::pipeline::realtime
