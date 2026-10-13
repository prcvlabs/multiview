
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/realtime/latency-stats.hpp"
#include "perceive/pipeline/realtime/track-messages.hpp"

namespace perceive::pipeline::realtime
{
// --------------------------------------------------------------- make-messages
// One message of every type, with awkward values
//
static vector<TrackMessage> make_messages()
{
   vector<TrackMessage> out;

   HeaderMessage header;
   header.scene             = "site04_scene05 \"quoted\"\nnewline";
   header.n_cameras         = 9;
   header.fps               = 15.0;
   header.hist_sz           = 0.1;
   header.top_left_x        = -6.0;
   header.top_left_y        = -3.0 / 7.0;
   header.w                 = 90;
   header.h                 = 130;
   header.start_frame       = 40;
   header.frames_per_window = 10;
   header.delta_t           = 16;
   header.version           = "abc123";
   out.push_back(header);

   FrameMessage frame;
   frame.frame      = 37;
   frame.timestamp  = "1970-01-01T00:00:02.466667";
   frame.latency_ms = 312.4;
   frame.n_p2ds     = 5;
   frame.positions.push_back({1.23, 4.56, 3, 0.81, 0.21, 4});
   frame.positions.push_back({-0.1, 1e-9, 1, 0.5, 0.99, -1});
   out.push_back(frame);

   out.push_back(FrameMessage{}); // no positions

   out.push_back(DroppedMessage{38, str(DropReason::NO_FREE_SLOT)});

   TracksMessage tracks;
   tracks.window   = 3;
   tracks.frame    = 37;
   tracks.revision = 1;
   tracks.status   = "revised";
   tracks.id_remap = {{12, 7}, {13, 8}};
   tracks.tracks.push_back({4, 1.21, 4.50, 1.57, false, true, 0.93});
   tracks.tracks.push_back({7, 2.0, 3.0, -2.5, true, false, 0.1});
   out.push_back(tracks);

   SmoothedMessage smoothed;
   smoothed.id           = 4;
   smoothed.final_upto_t = 21;
   smoothed.points       = {{1.0, 2.0, 20, 0.5}, {1.1, 2.1, 21, 0.6}};
   out.push_back(smoothed);

   out.push_back(TrackEndMessage{12, 44, true});

   RealtimeStats stats;
   stats.count_drop(DropReason::NO_FREE_SLOT);
   stats.add_latency(RealtimeStats::Latency::PROVISIONAL, 12.5);
   out.push_back(StatsMessage{stats.to_json(stats.t0())});

   out.push_back(EndMessage{120, 37, "/tmp/out/pipeline-output.json"});

   return out;
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("TrackMessages", "[realtime_publish]")
{
   const auto messages = make_messages();

   CATCH_SECTION("one-type-each")
   {
      vector<string> types;
      for(const auto& msg : messages) types.push_back(message_type(msg));
      for(const auto type : {"header",
                             "frame",
                             "dropped",
                             "tracks",
                             "smoothed",
                             "track_end",
                             "stats",
                             "end"})
         CATCH_REQUIRE(std::count(cbegin(types), cend(types), type) >= 1);
   }

   CATCH_SECTION("round-trip")
   {
      for(const auto& msg : messages) {
         const string line = to_ndjson_line(msg);
         CATCH_INFO(line);
         CATCH_REQUIRE(!line.empty());
         CATCH_REQUIRE(line.back() == '\n');
         CATCH_REQUIRE(std::count(cbegin(line), cend(line), '\n') == 1);

         const auto o = parse_json(line);
         CATCH_REQUIRE(o.isObject());
         CATCH_REQUIRE(o["type"].asString() == message_type(msg));

         const auto msg2 = track_message_from_json(o);
         CATCH_REQUIRE(msg2.index() == msg.index());
         CATCH_REQUIRE(msg2 == msg);
         CATCH_REQUIRE(to_ndjson_line(msg2) == line);
      }
   }

   CATCH_SECTION("required-keys")
   {
      auto keys_of = [&](size_t index) {
         return to_json(messages.at(index)).getMemberNames();
      };
      auto has_all = [](const vector<string>& keys,
                        std::initializer_list<const char*> required) {
         return std::all_of(cbegin(required), cend(required), [&](auto k) {
            return std::find(cbegin(keys), cend(keys), k) != cend(keys);
         });
      };
      CATCH_REQUIRE(has_all(keys_of(0),
                            {"type",
                             "scene",
                             "n_cameras",
                             "fps",
                             "hist_sz",
                             "top_left",
                             "w",
                             "h",
                             "start_frame",
                             "K",
                             "delta_t",
                             "version"}));
      CATCH_REQUIRE(has_all(
          keys_of(1),
          {"type", "frame", "ts", "latency_ms", "n_p2ds", "positions"}));
      CATCH_REQUIRE(has_all(keys_of(3), {"type", "frame", "reason"}));
      CATCH_REQUIRE(has_all(
          keys_of(4),
          {"type", "window", "frame", "revision", "status", "id_remap"}));
      CATCH_REQUIRE(
          has_all(keys_of(5), {"type", "id", "final_upto_t", "points"}));
      CATCH_REQUIRE(has_all(keys_of(6), {"type", "id", "last_t", "confirmed"}));
      CATCH_REQUIRE(
          has_all(keys_of(7), {"type", "uptime_s", "drops", "latency_ms"}));
      CATCH_REQUIRE(has_all(keys_of(8), {"type", "frames", "dropped"}));

      const auto o = to_json(messages.at(4));
      CATCH_REQUIRE(o["tracks"][0].isMember("interp"));
      CATCH_REQUIRE(o["id_remap"][0][0].asInt() == 12);
      CATCH_REQUIRE(o["id_remap"][0][1].asInt() == 7);
   }

   CATCH_SECTION("non-finite-is-null")
   {
      SmoothedMessage smoothed;
      smoothed.id       = 1;
      smoothed.points   = {{1.0, 2.0, 3, dNAN}};
      const string line = to_ndjson_line(smoothed);
      CATCH_REQUIRE(line.find("null") != string::npos);
      const auto msg = track_message_from_json(parse_json(line));
      const auto& p  = std::get<SmoothedMessage>(msg).points.at(0);
      CATCH_REQUIRE(p.t == 3);
      CATCH_REQUIRE(std::isnan(p.gaze));
   }

   CATCH_SECTION("bad-input")
   {
      Json::Value o{Json::objectValue};
      CATCH_REQUIRE_THROWS(track_message_from_json(o)); // no type
      o["type"] = "nonsense";
      CATCH_REQUIRE_THROWS(track_message_from_json(o));
      o["type"] = "dropped";
      CATCH_REQUIRE_THROWS(track_message_from_json(o)); // no frame
      o["frame"]  = 3;
      o["reason"] = "dag_error";
      CATCH_REQUIRE_NOTHROW(track_message_from_json(o));
   }
}

} // namespace perceive::pipeline::realtime
