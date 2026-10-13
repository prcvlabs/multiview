#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/realtime/publisher.hpp"

#include <atomic>
#include <thread>

namespace perceive::pipeline::realtime
{
using namespace std::chrono_literals;

// --------------------------------------------------------------- frame-message
//
static TrackMessage frame_message(int frame_no)
{
   FrameMessage msg;
   msg.frame = frame_no;
   return msg;
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("Publisher", "[realtime_publish]")
{
   CATCH_SECTION("every-line-in-order")
   {
      // A slow sink, and a queue far smaller than the lines: `publish` blocks
      // and nothing is lost
      std::mutex padlock;
      vector<string> lines;
      vector<string> copies;
      auto slow_sink = [&](const string& line) {
         std::this_thread::sleep_for(100us);
         std::lock_guard<std::mutex> lock(padlock);
         lines.push_back(line);
         return true;
      };
      auto copy_sink = [&](const string& line) {
         std::lock_guard<std::mutex> lock(padlock);
         copies.push_back(line);
         return true;
      };

      Publisher::Params params;
      params.capacity = 4;
      Publisher publisher(params, {slow_sink, copy_sink}, {});
      CATCH_REQUIRE(publisher.start());
      constexpr int n = 200;
      for(int i = 0; i < n; ++i)
         CATCH_REQUIRE(publisher.publish(frame_message(i)));
      CATCH_REQUIRE(publisher.publish(EndMessage{}));
      publisher.close();

      CATCH_REQUIRE(lines.size() == n + 1);
      CATCH_REQUIRE(copies == lines);
      for(int i = 0; i < n; ++i)
         CATCH_REQUIRE(lines[size_t(i)] == to_ndjson_line(frame_message(i)));
      CATCH_REQUIRE(lines.back() == to_ndjson_line(EndMessage{}));
      CATCH_REQUIRE(publisher.n_lines() == n + 1);
      CATCH_REQUIRE(publisher.n_errors() == 0);
      CATCH_REQUIRE(!publisher.publish(frame_message(n))); // closed
   }

   CATCH_SECTION("stats-until-end")
   {
      std::mutex padlock;
      vector<string> types;
      auto sink = [&](const string& line) {
         const auto o = parse_json(line);
         std::lock_guard<std::mutex> lock(padlock);
         types.push_back(o["type"].asString());
         return true;
      };
      std::atomic<int> n_stats = 0;
      auto make_stats          = [&](Publisher::time_point) {
         ++n_stats;
         return StatsMessage{};
      };

      Publisher::Params params;
      params.stats_period_s = 0.01;
      Publisher publisher(params, {sink}, make_stats);
      CATCH_REQUIRE(publisher.start());
      CATCH_REQUIRE(publisher.publish(HeaderMessage{}));
      std::this_thread::sleep_for(100ms);
      CATCH_REQUIRE(publisher.publish(EndMessage{}));
      std::this_thread::sleep_for(50ms); // no stats after `end`
      publisher.close();

      CATCH_REQUIRE(n_stats >= 3);
      CATCH_REQUIRE(types.size() == size_t(n_stats) + 2);
      CATCH_REQUIRE(types.front() == "header");
      CATCH_REQUIRE(types.back() == "end");
      for(size_t i = 1; i + 1 < types.size(); ++i)
         CATCH_REQUIRE(types[i] == "stats");
   }

   CATCH_SECTION("sink-errors-are-counted")
   {
      int n_calls = 0;
      auto sink   = [&](const string&) { return ++n_calls % 2 == 0; };
      Publisher publisher(Publisher::Params{}, {sink}, {});
      CATCH_REQUIRE(publisher.start());
      for(int i = 0; i < 10; ++i) publisher.publish(frame_message(i));
      publisher.close();
      CATCH_REQUIRE(n_calls == 10);
      CATCH_REQUIRE(publisher.n_lines() == 10);
      CATCH_REQUIRE(publisher.n_errors() == 5);
   }
}

} // namespace perceive::pipeline::realtime
