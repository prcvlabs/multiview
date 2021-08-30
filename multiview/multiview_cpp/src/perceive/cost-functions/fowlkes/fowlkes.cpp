
#include "fowlkes.hpp"

#include <optional>

#include "perceive/optimization/kz-filter.hpp"

namespace perceive
{
// --------------------------------------------------------- filter short tracks

FowlkesResult filter_short_tracks(const fowlkes::Params& p,
                                  const FowlkesResult& result) noexcept
{
   const auto now = tick();
   TRACE(format("BEGIN filter short tracks"));

   const real frame_duration = result.frame_duration;
   const unsigned min_frames
       = unsigned(std::ceil(p.min_track_seconds / frame_duration));

   FowlkesResult out = result;

   auto ii = std::stable_partition(
       begin(out.tracks), end(out.tracks), [&](const Track& x) -> bool {
          return x.size() >= min_frames;
       });
   out.tracks.erase(ii, end(out.tracks));

   TRACE(format("END filter short tracks, {}s", tock(now)));

   return out;
}

// --------------------------------------------------------------- smooth tracks
//
FowlkesResult smooth_tracks(const fowlkes::Params& p,
                            const FowlkesResult& result) noexcept
{
   if(p.smooth_kz_filter_m <= 0 || p.smooth_kz_filter_k <= 0) return result;

   const auto now = tick();
   TRACE(format("BEGIN smooth tracks"));

   KzFilter filter;
   filter.init(int(p.smooth_kz_filter_m), int(p.smooth_kz_filter_k));

   auto unpack_path = [](const vector<TrackPoint>& path) {
      vector<Vector2> xys(path.size());
      vector<Vector2> thetas(path.size());
      vector<vector<float>> pose_scores(path.size());

      for(auto i = 0u; i < path.size(); ++i) {
         xys[i]         = to_vec2(path[i].xy());
         thetas[i]      = path[i].forward_vector();
         pose_scores[i] = path[i].pose_scores;
         Expects(pose_scores[i].size() == size_t(n_pose_annotations()));
      }
      return make_tuple(xys, thetas, pose_scores);
   };

   auto pack_path = [](vector<TrackPoint>& path,
                       const vector<Vector2>& xys,
                       const vector<Vector2>& thetas,
                       const vector<vector<float>>& pose_scores) {
      Expects(path.size() == xys.size());
      Expects(path.size() == thetas.size());
      for(auto i = 0u; i < path.size(); ++i) {
         path[i].set_xy(to_vec2f(xys[i]));
         path[i].gaze_direction
             = (thetas[i].is_finite())
                   ? float(std::atan2(thetas[i].y, thetas[i].x))
                   : fNAN;
         path[i].set_pose_scores(pose_scores.at(i));
      }
   };

   auto smooth_track = [&](Track& tt) {
      // Unpack TrackPoint to xy
      vector<Vector2> xys;
      vector<Vector2> thetas;
      vector<vector<float>> pose_scores;
      std::tie(xys, thetas, pose_scores) = unpack_path(tt.path);

      const auto pose_back = pose_scores;

      auto get_vec = [&](const auto& vec, int index) -> std::optional<Vector2> {
         if(size_t(index) >= vec.size()) return {};
         if(!is_finite(vec.at(size_t(index)))) return {};
         return {vec.at(size_t(index))};
      };

      auto get_xy_t
          = [&xys, &get_vec](int index) { return get_vec(xys, index); };

      auto get_theta_t
          = [&thetas, &get_vec](int index) { return get_vec(thetas, index); };

      auto scalar_mult
          = [](const real k, const Vector2& o) -> Vector2 { return o * k; };

      auto plus = [](const Vector2& a, const Vector2& b) { return a + b; };

      auto is_finite = [](const Vector2& o) -> bool { return o.is_finite(); };

      vector<Vector2> o_xys, o_thetas;
      vector<vector<float>> o_pose_scores;
      o_xys.resize(xys.size());
      o_thetas.resize(thetas.size());
      o_pose_scores.resize(pose_scores.size());
      std::fill(begin(o_thetas), end(o_thetas), Vector2::nan());
      std::fill(begin(o_pose_scores),
                end(o_pose_scores),
                vector<float>(n_pose_annotations(), fNAN));

      for(auto i = 0u; i < tt.path.size(); ++i) {
         o_xys[i] = filter.smooth<Vector2>(get_xy_t, int(i), scalar_mult, plus);

         // Smooth omitting 'i'
         const Vector2 theta0 = filter.smooth<Vector2>(
             [&thetas, i](int index) -> std::optional<Vector2> {
                if(unsigned(index) >= thetas.size()) return {};
                if(unsigned(index) == i) return {};
                return {thetas[size_t(index)]};
             },
             int(i),
             scalar_mult,
             plus);

         const auto d0    = std::fabs(1.0 - theta0.dot(thetas[i]));
         const auto d1    = std::fabs(1.0 - theta0.dot(-thetas[i]));
         const auto theta = (d0 < d1) ? thetas[i] : -thetas[i];

         o_thetas[i]
             = filter.smooth<Vector2>(get_theta_t, int(i), scalar_mult, plus);

         for(auto j = 0; j < n_pose_annotations(); ++j) {
            auto get_ij = [&pose_scores, j](int index) -> std::optional<float> {
               const size_t ind = size_t(index);
               if(ind >= pose_scores.size()) return {};
               return pose_scores.at(ind).at(size_t(j));
            };

            o_pose_scores.at(size_t(i)).at(size_t(j))
                = filter.smooth<float>(get_ij, int(i));
         }
      }

      pack_path(tt.path, o_xys, o_thetas, o_pose_scores);

      if(false) {
         Expects(pose_scores.size() == o_pose_scores.size());
         auto f = [&](const float val) { return format("{:8.4f}", val); };
         for(size_t i = 0; i < pose_scores.size(); ++i) {
            cout << format(
                "{:02d}:   [{}]\n", i, rng::implode(pose_scores[i], ", ", f));
            cout << format("smth: [{}]\n",
                           rng::implode(o_pose_scores[i], ", ", f));
            cout << endl;
         }
      }
   };

   FowlkesResult out = result;
   for(auto& tt : out.tracks) smooth_track(tt);

   TRACE(format("END smooth track, {}s", tock(now)));

   return out;
}

// --------------------------------------------------------------- rebase tracks
//
FowlkesResult rebase_tracks(const FowlkesResult& result) noexcept
{
   const auto now = tick();
   TRACE(format("BEGIN rebase tracks"));

   auto out = result;

   const int start_frame = result.start_frame;

   auto tracks = result.tracks;

   for(auto& tt : tracks) {
      vector<TrackPoint> path;
      path.reserve(tt.path.size());
      for(auto& tp : tt.path) {
         if(tp.t >= start_frame) {
            path.push_back(tp);
            path.back().t -= start_frame;
         }
      }
      tt.path = path;
   }

   out.tracks = tracks;

   TRACE(format("END rebase tracks, {}s", tock(now)));

   return out;
}

} // namespace perceive
