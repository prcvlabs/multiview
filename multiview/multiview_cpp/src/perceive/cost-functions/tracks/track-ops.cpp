
#include "track-ops.hpp"

#include "localization-memo.hpp"
#include "perceive/geometry/line-1d.hpp"
#include "perceive/optimization/hungarian-algorithm.hpp"

namespace perceive::track_ops::detail
{
// Cool stuff to move into 'utils'

template<typename T>
constexpr T weighted_average(int n1, const T& a, int n2, const T& b) noexcept
{
   return ((a * T(n1)) + (b * T(n2))) / T(n1 + n2);
}

static void join_tracks(Track& out, const Track& tt)
{
   out.cost += tt.cost;
   out.height = weighted_average(
       int(tt.size()), tt.height, int(out.size()), out.height);
   out.path.insert(end(out.path), cbegin(tt.path), cend(tt.path));
}

static TrackPoint merge_two_track_points(const TrackPoint& tp0,
                                         const TrackPoint& tp1) noexcept
{
   Expects(tp0.t == tp1.t);
   return TrackPoint(0.5f * (tp0.x + tp1.x),
                     0.5f * (tp0.y + tp1.y),
                     tp0.t,
                     angle_average(tp0.gaze_direction, tp1.gaze_direction));
}

// -------------------------------------------------------------------
//
static Line1Di track_overlap_tt(const Track& A,
                                const Track& B,
                                const real overlap_threshold) noexcept
{
   if(A.size() == 0 and B.size() == 0) return Line1Di{};
   if(A.size() == 0) return Line1Di{B.path.front().t, B.path.front().t};
   if(B.size() == 0) return Line1Di{A.path.front().t, A.path.front().t};

   const float thres_sq = float(square(overlap_threshold));

   const auto t0 = A.path_time_range();
   const auto t1 = B.path_time_range();
   const auto tt = intersection_1d(t0, t1);

   if(tt.empty()) return tt;

   const auto A_begin = cbegin(A.path) + (tt.front() - t0.front());
   const auto B_begin = cbegin(B.path) + (tt.front() - t1.front());
   const auto A_end   = A_begin + tt.length();
   auto tp_are_close  = [&](const auto& tp0, const auto& tp1) -> bool {
      return (tp0.xy() - tp1.xy()).quadrance() <= thres_sq;
   };

   const auto is_eq = std::equal(A_begin, A_end, B_begin, tp_are_close);
   return is_eq ? tt : Line1Di{};
}

// ------------------------------------------------------------------- pp-tracks
//
string pp_tracks(const vector<Track>& tts) noexcept
{
   return implode(cbegin(tts), cend(tts), "\n", [&](const auto& a) {
      return indent(a.to_string(), 3);
   });
}

// ------------------------------------------------------ tracks overlap in time
//
bool tracks_overlap_in_time(const Track& A, const Track& B) noexcept
{
   if(A.size() == 0 or B.size() == 0) return false;
   const auto t0 = Line1Di{A.path.front().t, A.path.back().t + 1};
   const auto t1 = Line1Di{B.path.front().t, B.path.back().t + 1};
   return intersection_1d(t0, t1).length() > 0;
}

// ---------------------------------------------------------- track intersection

vector<TrackPoint> track_intersection(const vector<TrackPoint>& A,
                                      const vector<TrackPoint>& B,
                                      const real overlap_threshold) noexcept
{
   auto is_valid_tp_seq = [](const vector<TrackPoint>& A) {
      auto t_not_increasing
          = [](const auto& a, const auto& b) { return !(a.t + 1 == b.t); };
      auto ii = std::adjacent_find(cbegin(A), cend(A), t_not_increasing);
      return ii == cend(A);
   };

   Expects(is_valid_tp_seq(A));
   Expects(is_valid_tp_seq(B));

   if(A.size() == 0 or B.size() == 0) return {};

   const auto t0  = Line1Di{A.front().t, A.back().t + 1};
   const auto t1  = Line1Di{B.front().t, B.back().t + 1};
   const auto t01 = intersection_1d(t0, t1);

   if(t01.empty()) return {};

   const auto A_offset = -t0.a;
   const auto B_offset = -t1.a;

   auto bounds_sanity_check = [&](const auto& vec, const int ind, const int t) {
      const auto in_bounds
          = (size_t(ind) < vec.size()) and (vec[size_t(ind)].t == t);
      if(!in_bounds) {
         LOG_ERR("SANITY bounds check failed");
         cout << format("t0   = {}", str(t0)) << endl;
         cout << format("t1   = {}", str(t1)) << endl;
         cout << format("t01  = {}", str(t01)) << endl;
         cout << format("Aoff = {} - {} = {}", t0.a, t01.a, A_offset) << endl;
         cout << format("Boff = {} - {} = {}", t1.a, t01.a, B_offset) << endl;
         cout << format("t    = {}", t) << endl;
         cout << format("ind  = {}", ind) << endl;

         const auto tt = (size_t(ind) < vec.size()) ? vec[size_t(ind)].t : -1;
         FATAL(format(
             "{} < {}, and vec[ind].t = {}, t = {}", ind, vec.size(), tt, t));
      }
      return in_bounds;
   };

   const float thres_sq = float(square(overlap_threshold));

   auto average_tp = [&](const auto& tp0, const auto& tp1) {
      TrackPoint tp;
      tp.x = 0.5f * (tp0.x + tp1.x);
      tp.y = 0.5f * (tp0.y + tp1.y);
      tp.t = tp0.t;
      Expects(tp1.t == tp.t);
      tp.gaze_direction = 0.5f * (tp0.gaze_direction + tp1.gaze_direction);
      tp.p3d_index      = -1;
      return tp;
   };

   vector<TrackPoint> out;
   out.reserve(size_t(t01.length()));
   for(auto t = t01.a; t < t01.b; ++t) {
      const auto A_ind = A_offset + t;
      const auto B_ind = B_offset + t;
      Expects(bounds_sanity_check(A, A_ind, t));
      Expects(bounds_sanity_check(B, B_ind, t));
      if((A[size_t(A_ind)].xy() - B[size_t(B_ind)].xy()).quadrance()
         <= thres_sq)
         out.push_back(average_tp(A[size_t(A_ind)], B[size_t(B_ind)]));
   }

   // May not be 'valid'

   return out;
}

// ----------------------------------------------------------------- clamp track
// Clips the timepoints of a track to [start_frame, start_frame = n_frames)
void clamp_track(const int start_frame,
                 const int n_frames,
                 Track& track) noexcept
{
   const int end_frame = start_frame + n_frames;

   unsigned write_pos = 0;
   for(auto i = 0u; i < track.path.size(); ++i) {
      const auto& tp = track.path[i];
      if(tp.t >= start_frame and tp.t <= end_frame) {
         if(write_pos != i) track.path[write_pos] = tp;
         ++write_pos;
      }
   }

   if(write_pos < track.path.size()) track.path.resize(write_pos);
}

// ---------------------------------------------------- merge-overlapping-tracks
//
static Track merge_overlapping_tracks(const Track& A,
                                      const Track& B,
                                      const real overlap_threshold) noexcept
{
   Expects((A.size() > 0) and (B.size() > 0));
   const auto ll = track_overlap_tt(A, B, overlap_threshold); // line
   Expects(ll.length() > 0);

   const bool B_is_first = (A.path.front().t == B.path.front().t)
                               ? (B.path.size() > A.path.size())
                               : (ll.front() == A.path.front().t);

   // We're 'appending' V to the end of U
   const auto& U = B_is_first ? B : A;
   const auto& V = B_is_first ? A : B;
   const auto u0 = U.path_time_range();
   const auto v0 = V.path_time_range();
   const auto dd = difference_1d(u0, v0);

   const bool overlap_front = ll.front() < dd.front();

   if(false) {
      INFO(format("u0 = {}", str(u0)));
      INFO(format("v0 = {}", str(v0)));
      INFO(format("dd = {}", str(dd)));
      INFO(format("ll = {}", str(ll)));
   }

   Expects(dd.length() + ll.length() == u0.length());
   if(overlap_front) {
      Expects(u0.front() == ll.front());
      Expects(v0.front() == ll.front());
      Expects(v0.back() == ll.back());
      Expects(dd.front() == v0.back());
   } else {
      Expects(u0.back() == ll.back());
      Expects(dd.back() == v0.front());
   }
   Expects(v0.front() == ll.front());

   Track dst;
   dst.id = U.id;
   dst.height
       = weighted_average(int(U.size()), U.height, int(V.size()), V.height);
   dst.cost = U.cost + V.cost; // WARNING double counts

   // Calculate the new path
   auto U_bb = cbegin(U.path);                           // begin
   auto U_m1 = std::next(U_bb, ll.front() - u0.front()); // mid copy
   auto U_m2 = std::next(U_m1, ll.length());             // mid merge
   auto U_ee = cend(U.path);                             // end

   auto V_bb = cbegin(V.path);               // begin
   auto V_mm = std::next(V_bb, ll.length()); // merge-end
   auto V_ee = cend(V.path);                 // copy-end

   Expects(((U_ee == U_m2) or (V_mm == V_ee)));

   if(false) {
      INFO(format("UU [{}, {}, {}, {}]",
                  std::distance(cbegin(U.path), U_bb),
                  std::distance(cbegin(U.path), U_m1),
                  std::distance(cbegin(U.path), U_m2),
                  std::distance(cbegin(U.path), U_ee)));
      INFO(format("VV [{}, {}, {}]",
                  std::distance(cbegin(V.path), V_bb),
                  std::distance(cbegin(V.path), V_mm),
                  std::distance(cbegin(V.path), V_ee)));
   }

   Expects(std::distance(U_m1, U_m2) == ll.length());
   Expects(std::distance(V_bb, V_mm) == ll.length());
   dst.path.reserve(size_t(dd.length() + ll.length() + v0.length()));

   std::copy(U_bb, U_m1, std::back_inserter(dst.path));
   std::transform(
       U_m1, U_m2, V_bb, std::back_inserter(dst.path), merge_two_track_points);
   if(std::distance(U_m2, U_ee) > 0) {
      Expects(std::distance(V_mm, V_ee) == 0);
      std::copy(U_m2, U_ee, std::back_inserter(dst.path));
   } else {
      std::copy(V_mm, V_ee, std::back_inserter(dst.path));
   }

   Expects(dst.is_valid());
   return dst;
}

// ----------------------------------------------------------------- split track
//
std::pair<Track, Track> split_track(const Track& p, const int t) noexcept
{
   std::pair<Track, Track> uv;

   uv.first.id = uv.second.id = p.id;
   uv.first.cost = uv.second.cost = 0.0f; // has to be recalculated
   uv.first.height = uv.second.height = p.height;
   uv.first.path.reserve(p.size());
   uv.second.path.reserve(p.size());

   for(const auto& x : p.path) {
      if(x.t < t)
         uv.first.path.push_back(x);
      else
         uv.second.path.push_back(x);
   }

   return uv;
}

// ---------------------------------------------------------------- split tracks
//
std::pair<vector<Track>, vector<Track>> split_tracks(const vector<Track>& X,
                                                     const int t) noexcept
{
   std::pair<vector<Track>, vector<Track>> out;

   auto& A = out.first;
   auto& B = out.second;
   A.reserve(X.size());
   B.reserve(X.size());

   for(const Track& tt : X) {
      auto [p, q] = split_track(tt, t);
      if(!p.empty()) A.emplace_back(std::move(p));
      if(!q.empty()) B.emplace_back(std::move(q));
   }

   return out;
}

// ---------------------------------------------------------------- split tracks
//
vector<vector<Track>> split_tracks(const vector<Track>& in,
                                   const int t0,
                                   const int env_size,
                                   const int N) noexcept
{
   vector<Track> tts = in;
   vector<vector<Track>> out((size_t(N)));
   for(auto i = 0; i < N; ++i) {
      const int t    = t0 + (i + 1) * env_size;
      auto [A, B]    = split_tracks(tts, t);
      out[size_t(i)] = std::move(A);
      tts            = std::move(B);
   }
   return out;
}

// ------------------------------------------------------------ merge track sets
//
vector<Track> merge_track_sets(const vector<vector<Track>>& tracks) noexcept
{
   const auto calc_max_id = [&]() {
      int max_id = -1;
      for(const auto& tts : tracks)
         for(const auto& tt : tts)
            if(max_id < tt.id) max_id = tt.id;
      return max_id;
   };

   const int max_id = calc_max_id();
   const int N      = max_id + 1; // N output elements

   // Initialize the output tracks
   vector<Track> out((size_t(N)));
   for(auto i = 0; i < N; ++i) out[size_t(i)].id = i;

   // Merge just a pair of tracks
   auto merge_tts = [](const Track& tt, Track& out) {
      Expects(tt.id == out.id);
      join_tracks(out, tt);
   };

   // Merge the tracks
   auto merge_it = [&](const Track& tt) {
      if(tt.id < 0) return; // track was not valid
      Expects(size_t(tt.id) < out.size());
      merge_tts(tt, out[size_t(tt.id)]);
   };
   for(const auto& tts : tracks)
      for(const auto& tt : tts) merge_it(tt);

   // Finalize the tracks. (Basically ensure the paths are sorted by time.)
   for(auto& tt : out)
      std::sort(begin(tt.path), end(tt.path), [&](auto& a, auto& b) {
         return a.t < b.t;
      });

   return out;
}

// ------------------------------------------------------ merge tracks endpoints
//
vector<Track> merge_track_endpoints(const vector<Track>& in0,
                                    const vector<Track>& in1,
                                    const real merge_xy_dist_threshold) noexcept
{
   vector<Track> out = in0;

   auto match = [merge_xy_dist_threshold](const Track& A, const Track& B) {
      if(A.size() == 0 or B.size() == 0) return false;
      const auto& a = A.path.back();
      const auto& b = B.path.front();
      return (a.t + 1 == b.t)
             and (real((Vector2f(a.x, a.y) - Vector2f(b.x, b.y)).norm())
                  <= merge_xy_dist_threshold);
   };

   auto merge = [&out, &match](const Track& B) {
      auto ii = std::find_if(
          begin(out), end(out), [&](const Track& A) { return match(A, B); });
      if(ii == cend(out))
         out.emplace_back(B);
      else
         join_tracks(*ii, B);
   };

   std::for_each(cbegin(in1), cend(in1), merge);

   return out;
}

// -------------------------------------------------- interpolate-gaps-in-tracks
// A gap of 1 frame in track detection causes the creation of
// a brand new track. Interpolation closes these gaps.
// TODO, change these parameters to take a function of 't'.
//       We need to be able to have a higher level for small 't'
//       As well as a 't' cutoff.
vector<Track> interpolate_gaps_in_tracks(const vector<Track>& in,
                                         const real min_delta_xy,
                                         const real max_delta_xy_per_t,
                                         const int max_delta_t) noexcept
{
   bool deep_debug          = false;
   bool deep_debug2         = false;
   constexpr real max_score = std::numeric_limits<real>::max();

   vector<Track> out;
   out.reserve(in.size());

   auto interpolate_append = [&](vector<TrackPoint>& A, vector<TrackPoint>& B) {
      Expects((A.size() > 0 and B.size() > 0));
      int delta_t = B.front().t - A.back().t - 1;
      Expects(delta_t >= 0);
      A.reserve(A.size() + size_t(delta_t) + B.size());
      const TrackPoint tp0 = A.back();
      const TrackPoint tp1 = B.front();
      const auto dxy
          = to_vec2(Vector2f(tp1.x - tp0.x, tp1.y - tp0.y)) / real(delta_t + 1);
      const auto theta0  = angle_normalise2(tp0.gaze_direction); // [0..2pi)
      const auto theta1  = angle_normalise2(tp1.gaze_direction); // [0..2pi)
      const auto d_theta = angle_diff(theta1, theta0) / float(delta_t + 1);
      // Expects(angle_normalise2(theta0 + d_theta * (delta_t + 1)) - theta1
      //         < 1e-9);

      // Interpolate theta... blergh
      for(auto t = 1; t <= delta_t; ++t) {
         const auto dx = float(dxy.x) * float(t);
         const auto dy = float(dxy.y) * float(t);
         A.emplace_back(tp0.x + dx,
                        tp0.y + dy,
                        tp0.t + t,
                        angle_normalise(theta0 + d_theta * float(t)));
      }
      // Expects(A.back().t + 1 == B.front().t);
      A.insert(end(A), cbegin(B), cend(B));
   };

   auto merge_tracks = [&](Track& tt0, Track&& tt1) {
      if(deep_debug) WARN(format("MERGE tt.{} to tt.{}", tt0.id, tt1.id));
      if(deep_debug) {
         LOG_ERR(format("tt0 = {}", str(tt0)));
         LOG_ERR(format("tt1 = {}", str(tt1)));
      }

      tt0.cost += tt1.cost;
      tt0.height = weighted_average(
          int(tt0.size()), tt0.height, int(tt1.size()), tt1.height);
      if(tt0.before(tt1)) { // add to end
         // Keep the same 'id'
         interpolate_append(tt0.path, tt1.path);
      } else if(tt0.after(tt1)) { // prepend path
         tt0.id = tt1.id;         // keep the second id
         using std::swap;
         swap(tt0.path, tt1.path);
         interpolate_append(tt0.path, tt1.path);
      } else {
         Expects(false);
      }

      if(deep_debug) { cout << str(tt0) << endl; }
   };

   std::function<real(const Track&, const Track&)> track_track_distance
       = [&](const Track& tt0, const Track& tt1) {
            if(tt0.after(tt1)) return track_track_distance(tt1, tt0);
            if(!tt0.before(tt1)) return max_score;
            Expects(tt0.before(tt1));
            Expects((tt0.size() > 0 and tt1.size() > 0));
            const auto& tp0          = tt0.path.back();
            const auto& tp1          = tt1.path.front();
            const auto delta_t       = tp1.t - tp0.t;
            const real calc_delta_xy = max_delta_xy_per_t * delta_t;
            const real delta_xy      = std::max(min_delta_xy, calc_delta_xy);
            const real dist          = real((tp0.xy() - tp1.xy()).norm());

            if(false) {
               WARN(format("({}, {}, {}), ({}, {}, {}), dt = {}, "
                           "dist = {}, deltaxy = "
                           "std::max({} * {} = {}, {}) = {}",
                           tp0.x,
                           tp0.y,
                           tp0.t,
                           tp1.x,
                           tp1.y,
                           tp1.t,
                           (delta_t),
                           dist,
                           max_delta_xy_per_t,
                           delta_t,
                           calc_delta_xy,
                           min_delta_xy,
                           delta_xy));
            }

            if(delta_t > max_delta_t) return max_score;
            if(dist > delta_xy) return max_score;
            return delta_t * 1000.0 + dist;
         };

   auto merge_or_add_track = [&](const Track& tt) {
      auto track_dist_to_tt = [&](const auto& a, const auto& b) {
         return track_track_distance(a, tt) < track_track_distance(b, tt);
      };
      auto ii = std::min_element(begin(out), end(out), track_dist_to_tt);

      deep_debug = false and ((tt.id == 2) or (tt.id == 29))
                   and (tt.path.front().t == 180 or tt.path.front().t == 180);

      if(deep_debug) {
         const bool do_merge
             = (ii != end(out) and track_track_distance(*ii, tt) < max_score);

         INFO("REPORT");
         cout << format("tt.id = {}", tt.id) << endl;
         for(auto jj = begin(out); jj != end(out); ++jj) {
            cout << format("  [{:2d}], id = {:2d} => {} {} {}",
                           std::distance(begin(out), jj),
                           jj->id,
                           track_track_distance(tt, *jj),
                           (ii == jj ? "*" : ""),
                           (ii == jj and do_merge ? "!!!" : ""))
                 << endl;
         }

         LOG_ERR(format("(2) = {}", str(out[2])));
         LOG_ERR(format("tt  = {}", str(tt)));

         deep_debug2 = true;
         track_track_distance(out[2], tt);
      }

      if(ii != end(out) and track_track_distance(*ii, tt) < max_score)
         merge_tracks(*ii, Track(tt)); // do the merge
      else
         out.push_back(tt); // just push the result

      if(deep_debug) FATAL("kBAM!");
   };

   std::for_each(begin(in), end(in), merge_or_add_track);

   return out;
}

// ------------------------------------------------ decompose overlapping tracks
// Two tracks can be in the following states:
// [1] A and B are disjoint (the paths never overlap)
// [2] A and B can cross for a period of time
// In the later case, we want to split A and B into multiple disjoint tracks
vector<Track>
decompose_overlapping_tracks(const vector<Track>& in,
                             const real overlap_threshold) noexcept
{
   // -- (*) -- What's the bounding box for all the tracks?
   auto calc_bounds
       = [&](const vector<Track>& in, const real overlap_threshold) -> AABBi {
      AABB aabb = AABB::minmax();
      for(const auto& tt : in)
         for(const auto& tp : tt.path) aabb.union_point(to_vec2(tp.xy()));
      AABBi out;
      out.left   = int(std::floor(out.left) - std::ceil(overlap_threshold));
      out.top    = int(std::floor(out.top) - std::ceil(overlap_threshold));
      out.right  = int(std::ceil(out.right) + std::ceil(overlap_threshold));
      out.bottom = int(std::ceil(out.bottom) + std::ceil(overlap_threshold));
      return out;
   };
   const auto aabb = calc_bounds(in, overlap_threshold); // inclusive

   // -- (*) -- Create a lookup for the bounds (1 frame)
   Int16Image lookup;
   lookup.resize(aabb.width() + 1, aabb.height() + 1);
   lookup.fill(-1);
   const auto k_label_max
       = std::numeric_limits<decltype(lookup)::value_type>::max();

   vector<int> t_offsets(in.size());
   std::transform(cbegin(in), cend(in), begin(t_offsets), [&](const Track& tt) {
      return (tt.path.size() == 0) ? 0 : tt.path.front().t;
   });

   { // Process one frame
   }

   vector<Track> out;
   out.reserve(in.size());

   auto decompose_with_out = [&](const Track& tt) {

   };

   std::for_each(cbegin(in), cend(in), decompose_with_out);
   return out;
}

// ------------------------------------------ merge beginning-end-overlap tracks
// Sometimes tracks spatially overlap, and can be merged.
// Issue arises because of slight errors in calibration
// GREEDY algorithm
// NOTE: this algorithm will be deprecated (eventually) in favour of
// 'decompose-overlapping-tracks'
vector<Track>
merge_beginning_end_overlap_tracks(const vector<Track>& in,
                                   const real overlap_threshold) noexcept
{
   vector<Track> out;

   auto merge_or_push_track = [&](const Track& A) {
      auto A_overlaps = [&](const Track& B) -> bool {
         const auto overlap = track_overlap_tt(A, B, overlap_threshold);
         if(overlap.empty()) return false;

         const auto ll = overlap;
         const auto& U = (ll.front() == A.path.front().t) ? B : A;
         const auto& V = (ll.front() == A.path.front().t) ? A : B;
         const auto u0 = U.path_time_range();
         const auto v0 = V.path_time_range();
         const auto dd = difference_1d(u0, v0);

         return dd.length() + ll.length() == u0.length();
      };

      auto ii = std::find_if(begin(out), end(out), A_overlaps);
      if(ii == end(out))
         out.push_back(A);
      else
         *ii = merge_overlapping_tracks(A, *ii, overlap_threshold);
   };

   std::for_each(cbegin(in), cend(in), merge_or_push_track);

   return out;
}

// (GREEDILY) removes any tracks that overlap
// The result is sorted by track start-time
vector<Track> remove_overlapping_tracks(const vector<Track>& in,
                                        const real overlap_threshold) noexcept
{
   vector<Track> tmp = in;
   vector<Track> out;
   out.reserve(in.size());

   std::sort(begin(tmp), end(tmp), [](auto& a, auto& b) {
      return a.size() > b.size();
   });

   auto does_overlap = [&](const Track& A, const Track& B) {
      if(!tracks_overlap_in_time(A, B)) return false;
      if(track_intersection(A.path, B.path, overlap_threshold).empty())
         return false;
      return true;
   };

   auto add_or_skip = [&](Track& A) {
      auto A_overlaps = [&](const Track& B) { return does_overlap(A, B); };
      if(std::any_of(cbegin(out), cend(out), A_overlaps)) {
         // skip
      } else {
         out.emplace_back(std::move(A));
      }
   };
   std::for_each(begin(tmp), end(tmp), add_or_skip);

   return out;
}

// --------------------------------------------------------- remove-short-tracks
// Remove tracks whose length is below a threshold
vector<Track> remove_short_tracks(const vector<Track>& in,
                                  const int min_size,
                                  const int start_frame) noexcept
{
   vector<Track> out;
   out.reserve(in.size());

   auto not_a_short_track = [&](const auto& tt) {
      const bool is_long  = int(tt.size()) > min_size;
      const bool at_start = tt.size() > 0 and tt.path.front().t == start_frame;
      return is_long or at_start;
   };

   std::copy_if(
       cbegin(in), cend(in), std::back_inserter(out), not_a_short_track);

   return out;
}

// --------------------------------------------------------------- smooth-tracks
// Smooth the path of each track
vector<Track> smooth_tracks(const vector<Track>& in) noexcept { return in; }

// --------------------------------------------------------- tracklets to tracks
//
Tracks tracklets_to_tracks(const Tracks::Params& p,
                           const Tracklet& tracklet,
                           const int max_id_so_far) noexcept
{
   FATAL("no longer relevant");
   Tracks tts;
   return tts;
}

// ----------------------------------------------------------------- join tracks
//
Tracks join_tracks(const Tracks& A, const Tracks& B) noexcept
{
   Tracks out = A;

   if(false) {
      WARN(format("A [{}, {}] :: B [{}, {}]",
                  A.start_frame,
                  A.start_frame + A.n_frames,
                  B.start_frame,
                  B.start_frame + B.n_frames));
   }

   if(B.start_frame > A.start_frame) {
      Expects(B.start_frame == A.start_frame + A.n_frames);
   } else {
      Expects(A.start_frame == B.start_frame + B.n_frames);
   }

   Expects(A.max_frames_per_track == B.max_frames_per_track);
   Expects(A.w == B.w);
   Expects(A.h == B.h);

   out.seqs.insert(end(out.seqs), cbegin(B.seqs), cend(B.seqs));

   out.max_id               = std::max(A.max_id, B.max_id);
   out.w                    = A.w;
   out.h                    = A.h;
   out.start_frame          = std::min(A.start_frame, B.start_frame);
   out.n_frames             = A.n_frames + B.n_frames;
   out.max_frames_per_track = A.max_frames_per_track;
   out.hist_sz              = A.hist_sz;
   out.aabb                 = A.aabb;

   return out;
}

// ------------------------------------------------------ calc tracks ops method
//
Tracks calc_tracks_ops_method(
    const Tracks::Params& p,
    const real frame_duration,
    const Tracklet* current_tracklet,
    Tracks* previous_track,
    std::function<const LocalizationData*(int t)> get_localization,
    std::function<bool()> is_cancelled,
    const int feedback) noexcept
{
   FATAL("Not relevant");
   Tracks tts;
   return tts;

   // TRACE(format("calc-tracks ops-method"));

   // auto show_vec_tracks = [&](const char* label, const vector<Track>& tts) {
   //    const auto ss = quick_view(tts);
   //    cout << "Computation step: " << label << endl;
   //    cout << indent(ss, 3) << endl;
   // };

   // Tracks out;
   // if(is_cancelled()) return out;

   // auto now = tick();

   // Expects(current_tracklet != nullptr);
   // Tracks this_track = tracklets_to_tracks(
   //     p, *current_tracklet, (previous_track ? previous_track->max_id : -1));

   // Tracks combined = (previous_track) ? join_tracks(*previous_track,
   // this_track)
   //                                    : this_track;

   // const real s0 = tock(now);
   // TRACE(format("converted tracklets to tracks: {}s", s0));
   // now = tick();

   // // Build the localization memo
   // ::perceive::detail::LocalizationMemo loc_memo;
   // const int max_n_frames
   //     = (previous_track ? 2 : 1) * this_track.max_frames_per_track;
   // loc_memo.init(combined.start_frame, max_n_frames, get_localization);
   // auto get_loc = [&](int t) { return loc_memo.get(t); };

   // const real s1 = tock(now);
   // TRACE(format("building localization memo: {}s", s1));
   // now = tick();

   // Expects(loc_memo.size() > 0);
   // const auto loc_ptr0 = loc_memo.front();
   // const real hist_sz  = loc_ptr0->hist_sz;

   // // Update 'n-frames' for this-track
   // combined.n_frames = loc_memo.size();
   // this_track.n_frames
   //     = combined.n_frames - (previous_track ? previous_track->n_frames : 0);

   // if(is_cancelled()) return out;

   // // Join the vectors of Tracks and start munching
   // vector<Track> tts0;
   // {
   //    tts0.reserve(
   //        this_track.tracks.size()
   //        + (!previous_track ? 0 : int(previous_track->tracks.size())));
   //    if(previous_track)
   //       tts0.insert(end(tts0),
   //                   cbegin(previous_track->tracks),
   //                   cend(previous_track->tracks));
   //    tts0.insert(
   //        end(tts0), cbegin(this_track.tracks), cend(this_track.tracks));
   // }

   // if(false and previous_track) {
   //    show_vec_tracks("PREVIOUS", previous_track->tracks);
   //    show_vec_tracks("THIS", this_track.tracks);
   //    show_vec_tracks("COMBINED", tts0);
   //    // FATAL("kBAM!");
   // }

   // const real s_tts0 = tock(now);
   // TRACE(format("combining track vectors: {}s (so far)", s_tts0));

   // // Params
   // const real overlap_threshold = p.person_diameter / hist_sz;
   // const unsigned min_length    = p.min_track_length;
   // const real min_delta_xy      = p.min_interpolate_delta_xy;
   // const real max_delta_xy_per_t
   //     = (p.max_track_speed * frame_duration) / hist_sz;
   // const int max_delta_t = p.stitch_threshold;
   // const int remove_short_track_start_frame
   //     = combined.start_frame == 0 ? -1 : combined.start_frame;

   // if(is_cancelled()) return out;

   // // Merge endpoints-between tracks
   // if(is_cancelled()) return out;
   // now                = tick();
   // vector<Track> tts1 = track_ops::detail::interpolate_gaps_in_tracks(
   //     tts0, min_delta_xy, max_delta_xy_per_t, max_delta_t);
   // const real s_tts1 = tock(now);
   // TRACE(format("merge endpoints: {}s (so far)", s_tts1));

   // if(is_cancelled()) return out;
   // now                = tick();
   // vector<Track> tts2 =
   // track_ops::detail::merge_beginning_end_overlap_tracks(
   //     tts1, overlap_threshold);
   // const real s_tts2 = tock(now);
   // TRACE(format("merge beginning-end-overlap: {}s (so far)", s_tts2));

   // if(is_cancelled()) return out;
   // now = tick();
   // vector<Track> tts3
   //     = track_ops::detail::remove_overlapping_tracks(tts2,
   //     overlap_threshold);
   // const real s_tts3 = tock(now);
   // TRACE(format("remove overlapping-tracks: {}s (so far)", s_tts3));

   // if(is_cancelled()) return out;
   // now = tick();
   // vector<Track> tts4
   //     = (previous_track)
   //           ? track_ops::detail::remove_short_tracks(
   //                 tts3, min_length, remove_short_track_start_frame)
   //           : tts3;
   // const real s_tts4 = tock(now);
   // TRACE(format("remove short-tracks: {}s (so far)", s_tts4));

   // if(is_cancelled()) return out;
   // now                = tick();
   // vector<Track> tts5 = track_ops::detail::smooth_tracks(tts4);
   // const real s_tts5  = tock(now);
   // TRACE(format("smooth tracks: {}s (so far)", s_tts5));

   // if(is_cancelled()) return out;
   // now = tick();
   // vector<Track> prev_tts, this_tts;
   // std::tie(prev_tts, this_tts)
   //     = track_ops::detail::split_tracks(tts5, this_track.start_frame);

   // if(previous_track) {
   //    previous_track->tracks = prev_tts; // update max-id
   //    out.tracks             = this_tts;
   // } else {
   //    Expects(prev_tts.size() == 0);
   //    out.tracks = this_tts; // update max-id
   // }
   // const real s_tts6 = tock(now);
   // TRACE(format("split tracks: {}s (so far)", s_tts6));
   // now = tick();

   // auto calc_out_max_id = [previous_track](const vector<Track>& tracks) {
   //    int max_id = (previous_track == nullptr) ? -1 : previous_track->max_id;
   //    for(const auto& tt : tracks)
   //       if(max_id < tt.id) max_id = tt.id;
   //    return max_id;
   // };

   // out.max_id               = calc_out_max_id(out.tracks);
   // out.w                    = this_track.w;
   // out.h                    = this_track.h;
   // out.start_frame          = this_track.start_frame;
   // out.n_frames             = this_track.n_frames;
   // out.max_frames_per_track = this_track.max_frames_per_track;

   // const real s_out = tock(now);
   // TRACE(format("calculate output track: {}s", s_out));

   // if(feedback > 0) {
   //    INFO(format("TRACK OPS feedback"));
   //    cout << format("tracklt     = [{}..{})/{}\n",
   //                   current_tracklet->start_frame,
   //                   current_tracklet->start_frame +
   //                   current_tracklet->n_frames,
   //                   current_tracklet->max_frames_per_tracklet);

   //    cout << format("this tt     = [{}..{})\n",
   //                   this_track.start_frame,
   //                   this_track.start_frame + this_track.n_frames);
   //    if(previous_track) {
   //       const auto& pt = *previous_track;
   //       cout << format("prev        = [{}..{})\n",
   //                      pt.start_frame,
   //                      pt.start_frame + pt.n_frames);
   //    } else {
   //       cout << format("prev        = nullptr\n");
   //    }
   //    cout << format("combind     = [{}..{})\n",
   //                   combined.start_frame,
   //                   combined.start_frame + combined.n_frames);
   //    cout << format("locmemo     = {}\n", loc_memo.size());
   //    cout << format("max-n-f     = {}\n", max_n_frames);
   //    cout << format("hist-sz     = {}\n", hist_sz);
   //    cout << format("min-len     = {}\n", min_length);
   //    cout << format("max-sped    = {}\n", p.max_track_speed);
   //    cout << format("max-delta-t = {}\n", max_delta_t);
   //    cout << format("max-dxyt    = {}\n", max_delta_xy_per_t);
   //    cout << format("overlp thre = {}\n", overlap_threshold);
   //    cout << format("OUT        --> [{}, {}) / {}, wh = [{}, {}], "
   //                   "max-id = {}\n",
   //                   out.start_frame,
   //                   out.start_frame + out.n_frames,
   //                   out.max_frames_per_track,
   //                   out.w,
   //                   out.h,
   //                   out.max_id);
   //    cout << format("join __ = {}s\n", s0);
   //    cout << format("memo __ = {}s\n", s1);
   //    cout << format("s tts0_ = {}s\n", s_tts0);
   //    cout << format("s tts1_ = {}s\n", s_tts1);
   //    cout << format("s tts2_ = {}s\n", s_tts2);
   //    cout << format("s tts3_ = {}s\n", s_tts3);
   //    cout << format("s tts4_ = {}s\n", s_tts4);
   //    cout << format("s tts5_ = {}s\n", s_tts5);
   //    cout << format("s out__ = {}s\n", s_out);

   //    if(feedback > 1) {
   //       auto show = [&](const char* label, const auto& tts) {
   //          return show_vec_tracks(label, tts);
   //       };

   //       show("TTS0 -- construct", tts0);
   //       show("TTS1 -- interpolate", tts1);
   //       show("TTS2 -- merge overlapping begin/ends", tts2);
   //       show("TTS3 -- remove overlapping tracks", tts3);
   //       show("TTS4 -- remove short tracks", tts4);
   //       show("TTS5 -- smooth tracks", tts5);
   //       show("OUT ", out.tracks);

   //       cout << endl << endl << endl;
   //    }
   // }

   // return out;
}

} // namespace perceive::track_ops::detail

namespace perceive::track_ops
{
// ------------------------------------------------------------- validate-tracks
// Aborts with error message
void validate_tracks_object(const Tracks& tts) noexcept
{
   if(false && multiview_trace_mode()) {
      TRACE(format("tts validation"));
      cout << format(
          R"V0G0N(
Tracks
   start-frame:    {},
   n-frames:       {},
   wh:            [{}x{}],
   max-per-track:  {},
   max-id:         {},
   seq-ids:       [{}]
{})V0G0N",
          tts.start_frame,
          tts.n_frames,
          tts.w,
          tts.h,
          tts.max_frames_per_track,
          tts.max_id,
          implode(cbegin(tts.seqs),
                  cend(tts.seqs),
                  ", ",
                  [](const auto& seq) { return format("{}", seq.id()); }),
          "");
   }

   Expects(tts.start_frame >= 0);
   Expects(tts.n_frames > 0);
   Expects(std::isfinite(tts.hist_sz));
   Expects(tts.aabb.is_finite());
   Expects(tts.w > 0);
   Expects(tts.h > 0);
   Expects(tts.max_frames_per_track > 0);

   int max_id = std::numeric_limits<int>::lowest();
   for(const auto& tt : tts.seqs) {
      if(max_id < tt.id()) max_id = tt.id();
      for(const auto& o : tt.nodes()) {
         Expects(o.t() >= tts.start_frame);
         Expects(o.t() < tts.start_frame + tts.n_frames);
      }
   }

   // This constraint is actually invalid, because, consider:
   // Track epoch A has ids [0, 1, 2, 3], but and track 3 ends.
   // Track epoch B has ids [0, 1, 2], continuing from the previous epoch.
   // We need to have (tts.max_id == 3) for Track epoch B
   Expects(tts.seqs.size() == 0 || tts.max_id >= max_id);
}

} // namespace perceive::track_ops
