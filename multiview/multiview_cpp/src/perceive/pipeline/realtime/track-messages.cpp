
#include "track-messages.hpp"

#include "perceive/io/json-io.hpp"

namespace perceive::pipeline::realtime
{
// -------------------------------------------------------------------- load-key
//
template<typename T> static T load_key(const Json::Value& o, const char* key)
{
   return json_load_key<T>(o, key, "reading a track message");
}

// ------------------------------------------------------------------- array-key
//
static const Json::Value& array_key(const Json::Value& o, const char* key)
{
   if(!has_key(o, key) || !o[key].isArray())
      throw std::runtime_error(format("expected array key '{}'", key));
   return o[key];
}

// ------------------------------------------------------------ to-json (header)
//
static Json::Value to_json_(const HeaderMessage& m)
{
   Json::Value o{Json::objectValue};
   o["scene"]     = m.scene;
   o["n_cameras"] = m.n_cameras;
   o["fps"]       = json_save(m.fps);
   o["hist_sz"]   = json_save(m.hist_sz);
   Json::Value top_left{Json::arrayValue};
   top_left.append(json_save(m.top_left_x));
   top_left.append(json_save(m.top_left_y));
   o["top_left"]    = top_left;
   o["w"]           = m.w;
   o["h"]           = m.h;
   o["start_frame"] = m.start_frame;
   o["K"]           = m.frames_per_window;
   o["delta_t"]     = m.delta_t;
   o["version"]     = m.version;
   return o;
}

// ------------------------------------------------------------- to-json (frame)
//
static Json::Value to_json_(const FrameMessage& m)
{
   Json::Value o{Json::objectValue};
   o["frame"]      = m.frame;
   o["ts"]         = m.timestamp;
   o["latency_ms"] = json_save(m.latency_ms);
   o["n_p2ds"]     = m.n_p2ds;
   Json::Value positions{Json::arrayValue};
   for(const auto& p : m.positions) {
      Json::Value x{Json::objectValue};
      x["x"]            = json_save(p.x);
      x["y"]            = json_save(p.y);
      x["n_detections"] = p.n_detections;
      x["score"]        = json_save(p.score);
      x["prob_fp"]      = json_save(p.prob_fp);
      x["pid"]          = p.pid;
      positions.append(x);
   }
   o["positions"] = positions;
   return o;
}

// ----------------------------------------------------------- to-json (dropped)
//
static Json::Value to_json_(const DroppedMessage& m)
{
   Json::Value o{Json::objectValue};
   o["frame"]  = m.frame;
   o["reason"] = m.reason;
   return o;
}

// ------------------------------------------------------------ to-json (tracks)
//
static Json::Value to_json_(const TracksMessage& m)
{
   Json::Value o{Json::objectValue};
   o["window"]   = m.window;
   o["frame"]    = m.frame;
   o["revision"] = m.revision;
   o["status"]   = m.status;
   Json::Value remap{Json::arrayValue};
   for(const auto& [from, to] : m.id_remap) {
      Json::Value x{Json::arrayValue};
      x.append(from);
      x.append(to);
      remap.append(x);
   }
   o["id_remap"] = remap;
   Json::Value tracks{Json::arrayValue};
   for(const auto& tp : m.tracks) {
      Json::Value x{Json::objectValue};
      x["id"]        = tp.id;
      x["x"]         = json_save(tp.x);
      x["y"]         = json_save(tp.y);
      x["gaze"]      = json_save(tp.gaze);
      x["interp"]    = tp.interp;
      x["confirmed"] = tp.confirmed;
      x["score"]     = json_save(tp.score);
      tracks.append(x);
   }
   o["tracks"] = tracks;
   return o;
}

// ---------------------------------------------------------- to-json (smoothed)
//
static Json::Value to_json_(const SmoothedMessage& m)
{
   Json::Value o{Json::objectValue};
   o["id"]           = m.id;
   o["final_upto_t"] = m.final_upto_t;
   Json::Value points{Json::arrayValue};
   for(const auto& p : m.points) {
      Json::Value x{Json::arrayValue};
      x.append(json_save(p.x));
      x.append(json_save(p.y));
      x.append(p.t);
      x.append(json_save(p.gaze));
      points.append(x);
   }
   o["points"] = points;
   return o;
}

// --------------------------------------------------------- to-json (track-end)
//
static Json::Value to_json_(const TrackEndMessage& m)
{
   Json::Value o{Json::objectValue};
   o["id"]        = m.id;
   o["last_t"]    = m.last_t;
   o["confirmed"] = m.confirmed;
   return o;
}

// ------------------------------------------------------ to-json (window-error)
//
static Json::Value to_json_(const WindowErrorMessage& m)
{
   Json::Value o{Json::objectValue};
   o["window"] = m.window;
   o["reason"] = m.reason;
   return o;
}

// ------------------------------------------------------------- to-json (stats)
//
static Json::Value to_json_(const StatsMessage& m)
{
   return m.stats.isObject() ? m.stats : Json::Value{Json::objectValue};
}

// --------------------------------------------------------------- to-json (end)
//
static Json::Value to_json_(const EndMessage& m)
{
   Json::Value o{Json::objectValue};
   o["frames"]  = m.frames;
   o["dropped"] = m.dropped;
   o["output"]  = m.output;
   return o;
}

// ---------------------------------------------------------- from-json (header)
//
static void from_json_(const Json::Value& o, HeaderMessage& m)
{
   m.scene              = load_key<string>(o, "scene");
   m.n_cameras          = load_key<int>(o, "n_cameras");
   m.fps                = load_key<real>(o, "fps");
   m.hist_sz            = load_key<real>(o, "hist_sz");
   const auto& top_left = array_key(o, "top_left");
   if(top_left.size() != 2)
      throw std::runtime_error("expected 'top_left' to be [x, y]");
   m.top_left_x        = load_numeric(top_left[0]);
   m.top_left_y        = load_numeric(top_left[1]);
   m.w                 = load_key<int>(o, "w");
   m.h                 = load_key<int>(o, "h");
   m.start_frame       = load_key<int>(o, "start_frame");
   m.frames_per_window = load_key<int>(o, "K");
   m.delta_t           = load_key<int>(o, "delta_t");
   m.version           = load_key<string>(o, "version");
}

// ----------------------------------------------------------- from-json (frame)
//
static void from_json_(const Json::Value& o, FrameMessage& m)
{
   m.frame      = load_key<int>(o, "frame");
   m.timestamp  = load_key<string>(o, "ts");
   m.latency_ms = load_key<real>(o, "latency_ms");
   m.n_p2ds     = load_key<int>(o, "n_p2ds");
   m.positions.clear();
   for(const auto& x : array_key(o, "positions")) {
      FramePosition p;
      p.x            = load_key<real>(x, "x");
      p.y            = load_key<real>(x, "y");
      p.n_detections = load_key<int>(x, "n_detections");
      p.score        = load_key<real>(x, "score");
      p.prob_fp      = load_key<real>(x, "prob_fp");
      p.pid          = load_key<int>(x, "pid");
      m.positions.push_back(p);
   }
}

// --------------------------------------------------------- from-json (dropped)
//
static void from_json_(const Json::Value& o, DroppedMessage& m)
{
   m.frame  = load_key<int>(o, "frame");
   m.reason = load_key<string>(o, "reason");
}

// ---------------------------------------------------------- from-json (tracks)
//
static void from_json_(const Json::Value& o, TracksMessage& m)
{
   m.window   = load_key<int>(o, "window");
   m.frame    = load_key<int>(o, "frame");
   m.revision = load_key<int>(o, "revision");
   m.status   = load_key<string>(o, "status");
   m.id_remap.clear();
   for(const auto& x : array_key(o, "id_remap")) {
      if(!x.isArray() || x.size() != 2 || !x[0].isInt() || !x[1].isInt())
         throw std::runtime_error("expected 'id_remap' elements [from, to]");
      m.id_remap.emplace_back(x[0].asInt(), x[1].asInt());
   }
   m.tracks.clear();
   for(const auto& x : array_key(o, "tracks")) {
      TrackPoint tp;
      tp.id        = load_key<int>(x, "id");
      tp.x         = load_key<real>(x, "x");
      tp.y         = load_key<real>(x, "y");
      tp.gaze      = load_key<real>(x, "gaze");
      tp.interp    = load_key<bool>(x, "interp");
      tp.confirmed = load_key<bool>(x, "confirmed");
      tp.score     = load_key<real>(x, "score");
      m.tracks.push_back(tp);
   }
}

// -------------------------------------------------------- from-json (smoothed)
//
static void from_json_(const Json::Value& o, SmoothedMessage& m)
{
   m.id           = load_key<int>(o, "id");
   m.final_upto_t = load_key<int>(o, "final_upto_t");
   m.points.clear();
   for(const auto& x : array_key(o, "points")) {
      if(!x.isArray() || x.size() != 4 || !x[2].isInt())
         throw std::runtime_error("expected 'points' elements [x, y, t, gaze]");
      SmoothedPoint p;
      p.x    = load_numeric(x[0]);
      p.y    = load_numeric(x[1]);
      p.t    = x[2].asInt();
      p.gaze = load_numeric(x[3]);
      m.points.push_back(p);
   }
}

// ------------------------------------------------------- from-json (track-end)
//
static void from_json_(const Json::Value& o, TrackEndMessage& m)
{
   m.id        = load_key<int>(o, "id");
   m.last_t    = load_key<int>(o, "last_t");
   m.confirmed = load_key<bool>(o, "confirmed");
}

// ---------------------------------------------------- from-json (window-error)
//
static void from_json_(const Json::Value& o, WindowErrorMessage& m)
{
   m.window = load_key<int>(o, "window");
   m.reason = load_key<string>(o, "reason");
}

// ----------------------------------------------------------- from-json (stats)
//
static void from_json_(const Json::Value& o, StatsMessage& m)
{
   m.stats = o;
   m.stats.removeMember("type");
}

// ------------------------------------------------------------- from-json (end)
//
static void from_json_(const Json::Value& o, EndMessage& m)
{
   m.frames  = load_key<int>(o, "frames");
   m.dropped = load_key<int>(o, "dropped");
   m.output  = load_key<string>(o, "output");
}

// ---------------------------------------------------------------- message-type
//
const char* message_type(const TrackMessage& msg) noexcept
{
   constexpr std::array<const char*, std::variant_size_v<TrackMessage>> names
       = {"header",
          "frame",
          "dropped",
          "tracks",
          "smoothed",
          "track_end",
          "window_error",
          "stats",
          "end"};
   return names[msg.index()];
}

// --------------------------------------------------------------------- to-json
//
Json::Value to_json(const TrackMessage& msg) noexcept
{
   Json::Value o = std::visit([](const auto& m) { return to_json_(m); }, msg);
   o["type"]     = message_type(msg);
   return o;
}

// ----------------------------------------------------- track-message-from-json
//
template<size_t I = 0>
static TrackMessage from_json_index_(const Json::Value& o, const string& type)
{
   if constexpr(I == std::variant_size_v<TrackMessage>) {
      throw std::runtime_error(format("unknown message type '{}'", type));
   } else {
      TrackMessage msg{std::in_place_index<I>};
      if(type != message_type(msg)) return from_json_index_<I + 1>(o, type);
      from_json_(o, std::get<I>(msg));
      return msg;
   }
}

TrackMessage track_message_from_json(const Json::Value& o) noexcept(false)
{
   if(!o.isObject()) throw std::runtime_error("expected a JSON object");
   return from_json_index_(o, load_key<string>(o, "type"));
}

// -------------------------------------------------------------- to-ndjson-line
//
string to_ndjson_line(const TrackMessage& msg) noexcept
{
   Json::StreamWriterBuilder builder;
   builder["indentation"]  = "";
   builder["commentStyle"] = "None";
   string line             = Json::writeString(builder, to_json(msg));
   line += '\n';
   return line;
}

} // namespace perceive::pipeline::realtime
