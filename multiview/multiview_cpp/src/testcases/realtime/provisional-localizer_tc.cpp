
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/cost-functions/tracks/tracks-exec.hpp"
#include "perceive/pipeline/realtime/provisional-localizer.hpp"

namespace perceive::pipeline::realtime
{
namespace provisional_test
{
   constexpr real k_hist_sz = 0.1; // metres per cell

   // A detection whose `prob_xy` peaks at cell `xy`
   static Skeleton2DInfo make_info(Point2 xy, float peak)
   {
      Skeleton2DInfo info;
      info.prob_xy.resize(60, 40);
      info.prob_xy.fill(0.05f);
      info.prob_xy(xy.x, xy.y) = peak;
      return info;
   }

   static vector<vector<int>> cluster(const vector<Skeleton2DInfo>& infos,
                                      real diameter)
   {
      vector<const Skeleton2DInfo*> ptrs;
      for(const auto& info : infos) ptrs.push_back(&info);
      return cluster_detections(ptrs, k_hist_sz, diameter);
   }

   static ProvisionalPosition make_position(real x, real y)
   {
      ProvisionalPosition o;
      o.X = Vector2(x, y);
      return o;
   }

   static LiveTrack make_track(int id, int t, Vector2 X, Vector2 velocity)
   {
      LiveTrack o;
      o.id       = id;
      o.t        = t;
      o.X        = X;
      o.velocity = velocity;
      return o;
   }
} // namespace provisional_test

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("ProvisionalLocalizer", "[realtime_provisional]")
{
   using namespace provisional_test;

   CATCH_SECTION("cluster-detections")
   {
      // Peaks 0.1 m apart: one person seen by two sensors
      {
         vector<Skeleton2DInfo> infos;
         infos.push_back(make_info(Point2{10, 10}, 0.8f));
         infos.push_back(make_info(Point2{11, 10}, 0.9f));
         const auto groups = cluster(infos, 0.6);
         CATCH_REQUIRE(groups.size() == 1);
         CATCH_REQUIRE(groups[0] == vector<int>{1, 0}); // strongest first
      }

      // Peaks 2 m apart: two people
      {
         vector<Skeleton2DInfo> infos;
         infos.push_back(make_info(Point2{10, 10}, 0.8f));
         infos.push_back(make_info(Point2{30, 10}, 0.9f));
         infos.push_back(make_info(Point2{10, 12}, 0.7f));
         const auto groups = cluster(infos, 0.6);
         CATCH_REQUIRE(groups.size() == 2);
         CATCH_REQUIRE(groups[0] == vector<int>{1});
         CATCH_REQUIRE(groups[1] == vector<int>{0, 2});
      }

      // Nothing to cluster
      {
         CATCH_REQUIRE(cluster({}, 0.6).empty());
         vector<Skeleton2DInfo> infos(2); // no prob_xy
         CATCH_REQUIRE(cluster(infos, 0.6).empty());
      }
   }

   CATCH_SECTION("label-nearest-live-track")
   {
      ProvisionalLabeller labeller(0.6, 15);
      labeller.set_tracks(
          {make_track(4, 10, Vector2(1.0, 1.0), Vector2(0.0, 0.0)),
           make_track(7, 10, Vector2(3.0, 1.0), Vector2(0.1, 0.0)),
           make_track(9, -100, Vector2(6.0, 6.0), Vector2(0.0, 0.0))});

      vector<ProvisionalPosition> positions;
      positions.push_back(make_position(3.25, 1.0)); // 7 is expected at 3.2
      positions.push_back(make_position(1.05, 1.0));
      positions.push_back(make_position(10.0, 10.0)); // nobody near
      positions.push_back(make_position(6.0, 6.0));   // 9 is too old
      labeller.label(12, positions);
      CATCH_REQUIRE(positions[0].pid == 7);
      CATCH_REQUIRE(positions[1].pid == 4);
      CATCH_REQUIRE(positions[2].pid == -1);
      CATCH_REQUIRE(positions[3].pid == -1);

      // One to one: two positions near track 4, only the nearer gets it
      vector<ProvisionalPosition> positions2;
      positions2.push_back(make_position(1.3, 1.0));
      positions2.push_back(make_position(1.1, 1.0));
      labeller.label(10, positions2);
      CATCH_REQUIRE(positions2[0].pid == -1);
      CATCH_REQUIRE(positions2[1].pid == 4);

      // No tracks, no labels
      labeller.set_tracks({});
      labeller.label(10, positions2);
      CATCH_REQUIRE(positions2[1].pid == -1);
   }

   CATCH_SECTION("live-tracks")
   {
      Tracks window;
      window.hist_sz = k_hist_sz;
      window.aabb    = AABB(-2.0, -3.0, 4.0, 5.0);

      vector<tracks::Node> nodes;
      for(int t = 20; t < 25; ++t) {
         tracks::Node node;
         node.set_t(t);
         node.set_xy(Vector2f(float(10 + 2 * (t - 20)), 30.0f));
         nodes.push_back(node);
      }
      window.seqs.emplace_back(3, std::move(nodes));

      const auto tracks = live_tracks(window);
      CATCH_REQUIRE(tracks.size() == 1);
      CATCH_REQUIRE(tracks[0].id == 3);
      CATCH_REQUIRE(tracks[0].t == 24);
      CATCH_REQUIRE(std::fabs(tracks[0].X.x - (18 * k_hist_sz - 2.0)) < 1e-9);
      CATCH_REQUIRE(std::fabs(tracks[0].X.y - (30 * k_hist_sz - 3.0)) < 1e-9);
      CATCH_REQUIRE(std::fabs(tracks[0].velocity.x - (2 * k_hist_sz)) < 1e-9);
      CATCH_REQUIRE(std::fabs(tracks[0].velocity.y) < 1e-9);
   }
}

} // namespace perceive::pipeline::realtime
