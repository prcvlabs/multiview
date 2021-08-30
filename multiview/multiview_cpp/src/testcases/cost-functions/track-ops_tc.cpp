
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/cost-functions/tracks/track-ops.hpp"
#include "perceive/movie/debug-movie.hpp"
#include "perceive/utils/file-system.hpp"

static const bool feedback = false;

namespace perceive
{
using namespace perceive::track_ops::detail;

static Track make_track(int id,
                        float cost,
                        float height,
                        int N,
                        std::function<TrackPoint()> g) noexcept
{
   Expects(N >= 0);
   Track tt;
   tt.id     = id;
   tt.cost   = cost;
   tt.height = height;
   tt.path.resize(size_t(N));
   std::generate(begin(tt.path), end(tt.path), g);
   return tt;
}

static Track make_random_track(int id,
                               float cost,
                               float height,
                               int N,
                               std::mt19937& generator,
                               int t0,
                               Vector2f xy0) noexcept
{
   Expects(t0 >= 0);
   Expects(N >= 0);

   std::uniform_real_distribution<float> dist(-1, 1); // inclusive
   Vector2f xy = xy0;
   int t       = t0;

   auto tt = make_track(id, cost, height, N, [&]() -> TrackPoint {
      auto tp = TrackPoint{xy.x, xy.y, t++};
      xy.x += dist(generator);
      xy.y += dist(generator);
      return tp;
   });

   Expects(tt.path.size() == unsigned(N));
   return tt;
}

static Track make_track(int id,
                        float cost,
                        float height,
                        Point3 x0,
                        float theta0,
                        Point3 x1,
                        float theta1) noexcept
{
   Track tt;
   tt.id       = id;
   tt.cost     = cost;
   tt.height   = height;
   const int N = x1.z - x0.z + 1;
   Expects(N >= 0);
   tt.path.resize(size_t(N));
   if(N == 0) return tt;

   int counter = 0;
   auto dxy    = Vector2(x1.x - x0.x, x1.y - x0.y) / real(N - 1);
   if(!dxy.is_finite()) dxy = Vector2{0.0, 0.0};
   float theta_diff = angle_diff(theta1, theta0) / float(N - 1);
   if(!std::isfinite(theta_diff)) theta_diff = 0.0f;

   if(false) {
      INFO(format(
          "N = {}, theta-dif = {}, dxy = {:s}", N, to_degrees(theta_diff), str(dxy)));
   }

   std::generate(begin(tt.path), end(tt.path), [&]() {
      real t  = counter;
      auto tp = TrackPoint(float(x0.x + dxy.x * t),
                           float(x0.y + dxy.y * t),
                           counter + x0.z,
                           angle_normalise2(theta0 + theta_diff * float(t)));
      ++counter;
      return tp;
   });
   return tt;
}

static vector<Track> make_tracks(const int n_tracks,
                                 const int start_frame,
                                 const int total_frames,
                                 std::mt19937& generator) noexcept
{
   Expects(total_frames > 0);

   std::uniform_int_distribution<int> dist(start_frame,
                                           start_frame + total_frames - 1); // inclusive

   int counter = 0;
   vector<Track> out((size_t(n_tracks)));
   std::generate(begin(out), end(out), [&]() -> Track {
      int N = 0;
      int s = 0;
      while(N == 0) {
         s             = dist(generator);
         int end_frame = dist(generator);
         using std::swap;
         if(end_frame < s) swap(s, end_frame);
         N = end_frame - s;
      }
      Expects(N > 0);
      Vector2f xy0 = Vector2f(float(dist(generator)), float(dist(generator)));
      auto tt      = make_random_track(counter++, 0.0f, 0.0f, N, generator, s, xy0);
      Expects(tt.path.size() == unsigned(N));
      return tt;
   });
   return out;
}

static vector<Track> make_tracks_z(const int n_tracks,
                                   const int start_frame,
                                   const int total_frames,
                                   std::mt19937& generator) noexcept
{
   Expects(total_frames > 0);

   std::uniform_int_distribution<int> dist(start_frame,
                                           start_frame + total_frames - 1); // inclusive

   int counter = 0;
   vector<Track> out((size_t(n_tracks)));
   std::generate(begin(out), end(out), [&]() -> Track {
      int N = 0;
      int s = 0;
      while(N == 0) {
         s             = dist(generator);
         int end_frame = dist(generator);
         using std::swap;
         if(end_frame < s) swap(s, end_frame);
         N = end_frame - s;
      }
      Expects(N > 0);
      Vector2f xy0 = Vector2f(float(counter * 100), float(counter * 100));
      auto tt      = make_random_track(counter++, 0.0f, 0.0f, N, generator, s, xy0);
      Expects(tt.path.size() == unsigned(N));
      return tt;
   });
   return out;
}

static void test_tt(const Track& tt, int t0, int t1) noexcept
{
   CATCH_REQUIRE(tt.is_valid());
   for(const auto& tp : tt.path) CATCH_REQUIRE(((tp.t >= t0) && (tp.t < t1)));
   for(auto i = 1u; i < tt.path.size(); ++i)
      CATCH_REQUIRE(tt.path[i - 1].t + 1 == tt.path[i].t);
};

static Track make_tt0() noexcept
{
   return make_track(0,
                     3.0f,
                     3.4f,
                     Point3(0, 0, 3),
                     to_radians(190.0f),
                     Point3(4, 4, 7),
                     to_radians(180.0f));
}

static Track make_tt1() noexcept
{
   return make_track(1,
                     4.0f,
                     4.4f,
                     Point3(1, 1, 8),
                     to_radians(190.0f),
                     Point3(0, 0, 9),
                     to_radians(180.0f));
}

static Track make_tt2() noexcept

{
   return make_track(2,
                     3.0f,
                     3.1f,
                     Point3(1, 1, 10),
                     to_radians(190.0f),
                     Point3(0, 0, 11),
                     to_radians(180.0f));
}

static Track make_tt3() noexcept

{
   return make_track(3,
                     3.0f,
                     3.1f,
                     Point3(1, 1, 0),
                     to_radians(190.0f),
                     Point3(0, 0, 4),
                     to_radians(180.0f));
}

static Track make_tt4() noexcept

{
   return make_track(4,
                     3.0f,
                     3.1f,
                     Point3(2, 2, 3),
                     to_radians(190.0f),
                     Point3(2, 2, 6),
                     to_radians(180.0f));
}

static Track make_tt5() noexcept

{
   return make_track(5,
                     3.0f,
                     3.1f,
                     Point3(2, 2, 13),
                     to_radians(190.0f),
                     Point3(2, 2, 19),
                     to_radians(180.0f));
}

static Track make_tt6() noexcept

{
   return make_track(6,
                     3.0f,
                     3.1f,
                     Point3(2, 2, 13),
                     to_radians(190.0f),
                     Point3(2, 2, 13),
                     to_radians(180.0f));
}

static Track make_tt7() noexcept

{
   vector<Point3> tcs
       = {{15, 4, 160}, {15, 4, 161}, {15, 4, 162}, {15, 4, 163}, {15, 4, 164},
          {14, 4, 165}, {14, 4, 166}, {15, 4, 167}, {16, 3, 168}, {15, 4, 169},
          {15, 4, 170}, {15, 4, 171}, {15, 4, 172}, {15, 4, 173}, {15, 4, 174},
          {15, 4, 175}, {15, 4, 176}, {15, 4, 177}, {14, 4, 178}, {14, 4, 179}};

   size_t counter = 0;
   const auto tt  = make_track(7, 3.0f, 3.1f, int(tcs.size()), [&]() {
      const auto X = tcs[counter++];
      return TrackPoint{float(X.x), float(X.y), X.z};
   });
   return tt;
}

static Track make_tt8() noexcept

{
   vector<Point3> tcs
       = {{16, 3, 180}, {16, 3, 181}, {15, 4, 182}, {15, 4, 183}, {15, 4, 184},
          {16, 3, 185}, {16, 3, 186}, {15, 4, 187}, {16, 3, 188}, {15, 4, 189},
          {15, 4, 190}, {15, 4, 191}, {14, 4, 192}, {15, 4, 193}, {15, 4, 194},
          {15, 4, 195}, {15, 4, 196}, {15, 4, 197}, {15, 4, 198}, {15, 4, 199}};

   size_t counter = 0;
   const auto tt  = make_track(8, 3.0f, 3.1f, int(tcs.size()), [&]() {
      const auto X = tcs[counter++];
      return TrackPoint{float(X.x), float(X.y), X.z};
   });
   return tt;
}

CATCH_TEST_CASE("TrackOps", "[track-ops]")
{
   std::mt19937 generator;

   // -------------------------------------------------------------- Clamp Track
   CATCH_SECTION("track-ops_clamp-track")
   {
      generator.seed(0);

      auto run_test = [&generator](int start_frame, int N, int t0, int t1) {
         auto tt
             = make_random_track(0, 0.0f, 0.0f, (t1 - t0), generator, t0, {0.0f, 0.0f});
         CATCH_REQUIRE(tt.is_valid());
         clamp_track(start_frame, N, tt);
         test_tt(tt, t0, t1);
      };

      for(auto start = 0; start < 4; ++start)
         for(auto n = 0; n < 5; ++n)
            for(auto t0 = 0; t0 < 5; ++t0)
               for(auto tn = 0; tn < 5; ++tn) run_test(start, n, t0, t0 + tn);
   }

   // -------------------------------------------------------------- Split Track
   CATCH_SECTION("track-ops_split-track")
   {
      generator.seed(0);

      auto run_test = [&generator](int t0, int N, int t) {
         const int t1 = t0 + N;
         std::uniform_int_distribution<int> dist(1, 6); // inclusive
         auto tt = make_random_track(
             dist(generator), 0.0f, 0.0f, N, generator, t0, {0.0f, 0.0f});
         CATCH_REQUIRE(tt.is_valid());
         const auto [A, B] = split_track(tt, t);
         CATCH_REQUIRE(A.id == tt.id);
         CATCH_REQUIRE(B.id == tt.id);
         CATCH_REQUIRE(A.size() + B.size() == tt.size());
         test_tt(A, t0, t);
         test_tt(B, t, t0 + N);

         if(t > t0 and N > 0) {
            CATCH_REQUIRE(A.path.size() > 0);
            CATCH_REQUIRE(A.path.front().t == t0);
            if(t < t1) CATCH_REQUIRE(A.path.back().t + 1 == t);
         }

         if(t < t1 and N > 0) {
            CATCH_REQUIRE(B.path.size() > 0);
            if(t >= t0) CATCH_REQUIRE(B.path.front().t == t);
            CATCH_REQUIRE(B.path.back().t + 1 == t1);
         }
      };

      for(auto t0 = 0; t0 < 10; ++t0)
         for(auto N = 0; N < 10; ++N)
            for(auto t = 0; t < 10; ++t) run_test(t0, N, t);
   }

   // -------------------------------------------------------------- Split Track
   CATCH_SECTION("track-ops_intersection")
   {
      generator.seed(0);

      auto test_it = [&](const Track& tt0,
                         const Track& tt1,
                         const real threshold,
                         const unsigned out_len) {
         const auto& A = tt0.path;
         const auto& B = tt1.path;

         const auto C = track_intersection(A, B, threshold);

         if(false) {
            INFO(format("REPORT, threhold = {}", threshold));
            cout << format("A: [{:s}]\n", implode(cbegin(A), cend(A), ", "));
            cout << format("B: [{:s}]\n", implode(cbegin(B), cend(B), ", "));
            cout << "---- ----\n";
            cout << format("C: [{:s}]\n", implode(cbegin(C), cend(C), ", "));
         }

         CATCH_REQUIRE(out_len == C.size());
      };

      const auto tt0 = make_tt0(); // [3..7]
      const auto tt1 = make_tt1(); // [8..9]
      const auto tt2 = make_tt2(); // [10..11]
      const auto tt3 = make_tt3(); // [0..4]
      const auto tt4 = make_tt4(); // [3..6]

      if(false) {
         test_it(tt0, tt3, 100.0, 2);
         test_it(tt3, tt0, 100.0, 2);
         test_it(tt3, tt0, 0.0, 1);
         test_it(tt0, tt1, 100.0, 0);
      }
   }

   // -------------------------------------------------------- Merge Tracks Sets
   CATCH_SECTION("track-ops_merge-track-sets")
   {
      generator.seed(0);

      auto run_test = [&](const int n_tracks,
                          const int start_frame,
                          const int total_frames,
                          const int env_size) {
         const int remainder = start_frame % env_size;
         const int start0    = start_frame - remainder;
         const int N         = int_div2(total_frames + remainder, env_size);
         Expects(start0 % env_size == 0);
         Expects(start0 <= start_frame);
         Expects(start0 + env_size > start_frame);
         Expects(start0 + N * env_size >= start_frame + total_frames);
         Expects(start0 + (N - 1) * env_size < start_frame + total_frames);

         // Create the tracks
         vector<Track> tts0 = make_tracks(n_tracks, start_frame, total_frames, generator);

         // Split the tracks
         vector<vector<Track>> tts = split_tracks(tts0, start0, env_size, N);

         // Merge the tracks
         vector<Track> tts1 = merge_track_sets(tts);

         // And tts1 should be the same as tts0
         auto cmp = [&](const Track& a, const Track& b) { return a.id < b.id; };
         std::sort(begin(tts0), end(tts0), cmp);
         std::sort(begin(tts1), end(tts1), cmp);
         CATCH_REQUIRE(tts0.size() == tts1.size());
         CATCH_REQUIRE(tts0 == tts1);
      };

      const int start_frame  = 0;
      const int total_frames = 30;
      for(auto i = 0; i < 2; ++i)
         for(auto N = 0; N < 100; ++N)
            for(auto sz = 10; sz < 21; sz += 3)
               run_test(N, start_frame, total_frames, sz);
   }

   // ---------------------------------------------------- Merge Track Endpoints
   CATCH_SECTION("track-ops_merge-track-endpoints")
   {
      generator.seed(0);

      // (*) Create some tracks,
      // (*) split them up at 't',
      // (*) merge-track-endpoints
      // (*) ensure that we have the same tracks

      auto run_test
          = [&](const int n_tracks, const int start_frame, const int total_frames) {
               // Create the tracks
               vector<Track> tts0
                   = make_tracks_z(n_tracks, start_frame, total_frames, generator);

               // Split the tracks
               const auto [A, B] = split_tracks(tts0, start_frame + total_frames / 2);

               // Merge the tracks
               const real threshold = 1.5;
               vector<Track> tts1   = merge_track_endpoints(A, B, threshold);

               // And tts1 should be the same as tts0
               auto cmp = [&](const Track& a, const Track& b) { return a.id < b.id; };
               std::sort(begin(tts0), end(tts0), cmp);
               std::sort(begin(tts1), end(tts1), cmp);

               if(tts0 != tts1) {
                  size_t sz = std::max(tts0.size(), tts1.size());
                  INFO(format("COMPARE tts0, tts1"));
                  for(auto i = 0u; i < sz; ++i) {
                     if(i > 0) cout << "\n";
                     const string s0 = (i < tts0.size()) ? tts0[i].to_string() : ""s;
                     const string s1 = (i < tts1.size()) ? tts1[i].to_string() : ""s;
                     bool eq         = false;
                     if(i < tts0.size() and i < tts1.size()) eq = tts0[i] == tts1[i];
                     if(!eq)
                        cout << format("[{}] = {:s}\n", i, str(eq)) << s0 << s1 << "\n."
                             << endl;
                  }
               }

               CATCH_REQUIRE(tts0.size() == tts1.size());
               CATCH_REQUIRE(tts0 == tts1);
            };

      const int start_frame  = 0;
      const int total_frames = 30;
      for(auto i = 0; i < 20; ++i)
         for(auto N = 0; N < 100; ++N) run_test(N, start_frame, total_frames);
   }

   // -------------------------------------------------------------- Interpolate
   CATCH_SECTION("track-ops_interpolate")
   {
      generator.seed(0);

      auto test_it = [&](const Track& tt0,
                         const Track& tt1,
                         const real min_xy,
                         const real max_xy,
                         const int max_t,
                         const bool should_merge) {
         vector<Track> ttz(2);
         ttz[0]         = tt0;
         ttz[1]         = tt1;
         const auto tts = interpolate_gaps_in_tracks(ttz, min_xy, max_xy, max_t);

         if(false) {
            INFO(format("REPORT, max_xy = {}, max_t = {}", max_xy, max_t));
            for(const auto& tt : ttz) cout << str(tt) << endl << endl;
            cout << " >> >> >> >> >> " << endl;
            for(const auto& tt : tts) cout << str(tt) << endl << endl;
            cout << "." << endl << endl;
         }

         for(const auto& tt : tts) CATCH_REQUIRE(tt.is_valid());
         if(!should_merge) {
            CATCH_REQUIRE(tts.size() == 2);
            CATCH_REQUIRE(std::equal(cbegin(ttz), cend(ttz), cbegin(tts)));
         } else {
            CATCH_REQUIRE(tts.size() == 1);
            const auto& tt = tts[0];
            const auto& t0 = (tt0.before(tt1)) ? tt0 : tt1;
            const auto& t1 = (tt0.before(tt1)) ? tt1 : tt0;

            CATCH_REQUIRE(tt.size() >= t0.size() + t1.size());
            CATCH_REQUIRE(std::equal(cbegin(t0.path), cend(t0.path), cbegin(tt.path)));
            CATCH_REQUIRE(std::equal(cbegin(t1.path),
                                     cend(t1.path),
                                     cbegin(tt.path) + long(tt.size() - t1.size())));
         }
      };

      const auto tt0 = make_tt0();
      const auto tt1 = make_tt1();
      const auto tt2 = make_tt2();

      const auto tt7 = make_tt7();
      const auto tt8 = make_tt8();

      if(false) {
         test_it(tt0, tt1, 0.0, 100.0, 30, true);
         test_it(tt0, tt2, 0.0, 100.0, 30, true);
         test_it(tt1, tt0, 0.0, 100.0, 30, true);
         test_it(tt2, tt1, 0.0, 100.0, 30, true);
         test_it(tt0, tt2, 0.0, 100.0, 1, false);
         test_it(tt0, tt1, 3.0, 100.0, 1, true);
         test_it(tt0, tt1, 3.0, 100.0, 0, false);
         test_it(tt0, tt1, 3.0, 0.0, 1, false);
         test_it(tt0, tt1, 3.0, 4.0, 1, true);

         test_it(tt7, tt8, 2.0, 2.0, 1, true);
         test_it(tt7, tt8, 1.0, 1.0, 1, false);
         test_it(tt7, tt8, 4.0, 1.0, 1, true);
      }
   }

   // -------------------------------------------------------------- merge
   CATCH_SECTION("track-ops_beginning-end-overlap")
   {
      generator.seed(0);

      auto test_it = [&](const Track& tt0,
                         const Track& tt1,
                         const real threshold,
                         const bool should_merge) {
         vector<Track> ttz(2);
         ttz[0] = tt0;
         ttz[1] = tt1;

         const auto tts = merge_beginning_end_overlap_tracks(ttz, threshold);

         if(false) {
            INFO(format("REPORT, threshold = {}", threshold));
            for(const auto& tt : ttz) cout << str(tt) << endl << endl;
            cout << " >> >> >> >> >> " << endl;
            for(const auto& tt : tts) cout << str(tt) << endl << endl;
            cout << "." << endl << endl;
         }

         for(const auto& tt : tts) CATCH_REQUIRE(tt.is_valid());
         if(should_merge)
            CATCH_REQUIRE(tts.size() == 1);
         else
            CATCH_REQUIRE(tts.size() == 2);
      };

      const auto tt0 = make_tt0();
      const auto tt1 = make_tt1();
      const auto tt2 = make_tt2();
      const auto tt3 = make_tt3(); // [0..4]
      const auto tt4 = make_tt4(); // [3..6]
      const auto tt5 = make_tt5(); // [13..20]
      const auto tt6 = make_tt6(); // [13..14]

      {
         test_it(tt0, tt1, 100.0, false);
         test_it(tt0, tt2, 100.0, false);
         test_it(tt1, tt2, 100.0, false);
         test_it(tt3, tt4, 100.0, true);
         test_it(tt4, tt3, 100.0, true);
         test_it(tt0, tt3, 100.0, true);
         test_it(tt0, tt3, 0.01, false);
         test_it(tt5, tt6, 100.0, true);
         test_it(tt6, tt5, 100.0, true);
      }
   }
}

} // namespace perceive
