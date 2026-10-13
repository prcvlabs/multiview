
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/pipeline/realtime/latency-stats.hpp"

namespace perceive::pipeline::realtime
{
using namespace std::chrono_literals;

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("LatencyStats", "[realtime_metrics]")
{
   CATCH_SECTION("percentiles")
   {
      LatencyStats stats;
      CATCH_REQUIRE(stats.percentile(50.0) == 0.0);
      for(int i = 1000; i >= 1; --i) stats.add(double(i));
      CATCH_REQUIRE(stats.count() == 1000);
      CATCH_REQUIRE(stats.percentile(50.0) == 500.0);
      CATCH_REQUIRE(stats.percentile(95.0) == 950.0);
      CATCH_REQUIRE(stats.percentile(100.0) == 1000.0);
      CATCH_REQUIRE(stats.percentile(0.0) == 1.0);
      CATCH_REQUIRE(stats.max() == 1000.0);
      CATCH_REQUIRE(stats.mean() == 500.5);
   }

   CATCH_SECTION("ring-keeps-the-last-samples")
   {
      LatencyStats stats;
      const int n = int(LatencyStats::k_capacity) + 100;
      for(int i = 0; i < n; ++i) stats.add(double(i));
      CATCH_REQUIRE(stats.count() == uint64_t(n));
      CATCH_REQUIRE(stats.n_samples() == LatencyStats::k_capacity);
      const auto xs = stats.samples();
      CATCH_REQUIRE(xs.front() == 100.0); // oldest first
      CATCH_REQUIRE(xs.back() == double(n - 1));
      CATCH_REQUIRE(std::is_sorted(cbegin(xs), cend(xs)));
      CATCH_REQUIRE(stats.percentile(0.0) == 100.0);
      CATCH_REQUIRE(stats.max() == double(n - 1));
   }

   CATCH_SECTION("rate-over-a-fake-clock")
   {
      const auto t0 = std::chrono::steady_clock::time_point{} + 1000s;
      RateCounter rate;
      CATCH_REQUIRE(rate.rate(t0) == 0.0);
      // 15 events per second for 3 seconds
      for(int i = 0; i < 45; ++i) rate.add(t0 + i * 1000ms / 15);
      const auto t_last = t0 + 44 * 1000ms / 15;
      CATCH_REQUIRE(rate.count() == 45);
      CATCH_REQUIRE(rate.rate(t_last) == 15.0);
      CATCH_REQUIRE(rate.rate(t_last + 500ms) == 8.0);
      CATCH_REQUIRE(rate.rate(t_last + 2s) == 0.0);

      RateCounter rate2;
      rate2.set_window(2s);
      for(int i = 0; i < 45; ++i) rate2.add(t0 + i * 1000ms / 15);
      CATCH_REQUIRE(rate2.rate(t_last) == 15.0); // 30 events in 2 seconds
   }
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("RealtimeStats", "[realtime_metrics]")
{
   const auto t0 = std::chrono::steady_clock::time_point{} + 1000s;
   RealtimeStats stats(t0);

   for(int i = 0; i < 30; ++i)
      stats.count_frame(RealtimeStats::Stage::SOURCE, t0 + i * 100ms);
   for(int i = 0; i < 10; ++i)
      stats.count_frame(RealtimeStats::Stage::DAG, t0 + i * 300ms);
   stats.count_drop(DropReason::NO_FREE_SLOT);
   stats.count_drop(DropReason::NO_FREE_SLOT);
   stats.count_drop(DropReason::DAG_ERROR);
   stats.count_drop(DropReason::NONE); // not a drop
   stats.set_queue_depth(RealtimeStats::Queue::LOC, 3);
   stats.set_slots_busy(2);
   stats.set_still_samples(7);
   for(int i = 1; i <= 100; ++i)
      stats.add_latency(RealtimeStats::Latency::PROVISIONAL, double(i));
   stats.add_window_ms(250.0);

   const auto now = t0 + 2900ms;

   CATCH_SECTION("counters")
   {
      CATCH_REQUIRE(stats.n_frames(RealtimeStats::Stage::SOURCE) == 30);
      CATCH_REQUIRE(stats.fps(RealtimeStats::Stage::SOURCE, now) == 10.0);
      CATCH_REQUIRE(stats.n_dropped(DropReason::NO_FREE_SLOT) == 2);
      CATCH_REQUIRE(stats.n_dropped() == 3);
      CATCH_REQUIRE(stats.queue_depth(RealtimeStats::Queue::LOC) == 3);
      CATCH_REQUIRE(stats.slots_busy() == 2);
      CATCH_REQUIRE(stats.still_samples() == 7);
   }

   CATCH_SECTION("to-json")
   {
      const auto o = stats.to_json(now);
      for(const auto key : {"uptime_s",
                            "fps",
                            "frames",
                            "queue_depth",
                            "slots_busy",
                            "drops",
                            "still_samples",
                            "rss_mb",
                            "latency_ms",
                            "window_ms"})
         CATCH_REQUIRE(o.isMember(key));
      CATCH_REQUIRE(o["uptime_s"].asDouble() == 2.9);
      CATCH_REQUIRE(o["fps"]["source"].asDouble() == 10.0);
      CATCH_REQUIRE(o["fps"]["dag"].asDouble() == 3.0);
      CATCH_REQUIRE(o["frames"]["dag"].asUInt64() == 10);
      CATCH_REQUIRE(o["queue_depth"]["loc"].asUInt64() == 3);
      CATCH_REQUIRE(o["queue_depth"]["ring"].asUInt64() == 0);
      CATCH_REQUIRE(o["drops"]["no_free_slot"].asUInt64() == 2);
      CATCH_REQUIRE(o["drops"]["dag_error"].asUInt64() == 1);
      CATCH_REQUIRE(o["drops"]["tracker_behind"].asUInt64() == 0);
      CATCH_REQUIRE(!o["drops"].isMember("none"));
      CATCH_REQUIRE(o["rss_mb"].asDouble() > 0.0);
      const auto& provisional = o["latency_ms"]["provisional"];
      CATCH_REQUIRE(provisional["n"].asUInt64() == 100);
      CATCH_REQUIRE(provisional["p50"].asDouble() == 50.0);
      CATCH_REQUIRE(provisional["p95"].asDouble() == 95.0);
      CATCH_REQUIRE(provisional["max"].asDouble() == 100.0);
      for(const auto key : {"tracked", "revised", "final"})
         CATCH_REQUIRE(o["latency_ms"][key]["n"].asUInt64() == 0);
      CATCH_REQUIRE(o["window_ms"]["max"].asDouble() == 250.0);
   }

   CATCH_SECTION("to-prometheus")
   {
      const auto text = stats.to_prometheus(now);
      for(const auto line :
          {"# TYPE multiview_realtime_frames_total counter\n",
           "multiview_realtime_frames_total{stage=\"source\"} 30\n",
           "multiview_realtime_fps{stage=\"dag\"} 3.0\n",
           "multiview_realtime_queue_depth{queue=\"loc\"} 3\n",
           "multiview_realtime_slots_busy 2\n",
           "multiview_realtime_dropped_frames_total{reason=\"no_free_slot\"} "
           "2\n",
           "multiview_realtime_still_samples_total 7\n",
           "# TYPE multiview_realtime_latency_ms summary\n",
           "multiview_realtime_latency_ms{status=\"provisional\",quantile="
           "\"0.95\"} 95.0\n",
           "multiview_realtime_latency_ms_count{status=\"provisional\"} 100\n",
           "multiview_realtime_window_ms{quantile=\"0.5\"} 250.0\n",
           "multiview_realtime_window_ms_count 1\n"}) {
         CATCH_INFO(line);
         CATCH_REQUIRE(text.find(line) != string::npos);
      }
      // Every line is a comment or "name[{labels}] value"
      std::stringstream ss{text};
      string line;
      while(std::getline(ss, line)) {
         CATCH_INFO(line);
         CATCH_REQUIRE(!line.empty());
         if(line[0] == '#') continue;
         CATCH_REQUIRE(line.rfind("multiview_realtime_", 0) == 0);
         CATCH_REQUIRE(std::count(cbegin(line), cend(line), ' ') == 1);
      }
   }
}

} // namespace perceive::pipeline::realtime
