
#pragma once

#include "json/json.h"

#include <variant>

namespace perceive::pipeline::realtime
{
// The records of the real-time output stream: one JSON object per line
// (NDJSON), identical in a file and on a WebSocket. Positions are in metres,
// `t` and `frame` are absolute scene frame numbers. A real number that is not
// finite is written as `null`, and read back as NaN.

// --------------------------------------------------------------- HeaderMessage
// First line of a stream.
//
struct HeaderMessage
{
   string scene          = ""s;
   int n_cameras         = 0;
   real fps              = 0.0;
   real hist_sz          = 0.0;
   real top_left_x       = 0.0; // metres
   real top_left_y       = 0.0;
   int w                 = 0; // histogram size
   int h                 = 0;
   int start_frame       = 0;
   int frames_per_window = 0; // "K"
   int delta_t           = 0; // smoothing lag, in frames
   string version        = ""s;

   bool operator==(const HeaderMessage&) const = default;
};

// ---------------------------------------------------------------- FrameMessage
// Provisional positions of one frame, before any tracking window closes.
//
struct FramePosition
{
   real x           = 0.0;
   real y           = 0.0;
   int n_detections = 0;
   real score       = 0.0;
   real prob_fp     = 1.0;
   int pid          = -1; // provisional id, -1 when unmatched

   bool operator==(const FramePosition&) const = default;
};

struct FrameMessage
{
   int frame        = -1;
   string timestamp = ""s; // capture time, "ts"
   real latency_ms  = 0.0;
   int n_p2ds       = 0;
   vector<FramePosition> positions;

   bool operator==(const FrameMessage&) const = default;
};

// -------------------------------------------------------------- DroppedMessage
//
struct DroppedMessage
{
   int frame     = -1;
   string reason = ""s; // str(DropReason)

   bool operator==(const DroppedMessage&) const = default;
};

// --------------------------------------------------------------- TracksMessage
// The tracks of one frame. Revision 0 ("tracked") is written when the frame's
// window closes; revision 1 ("revised") when the next window has rewritten it.
//
struct TrackPoint
{
   int id         = -1;
   real x         = 0.0;
   real y         = 0.0;
   real gaze      = 0.0; // radians
   bool interp    = false;
   bool confirmed = false;
   real score     = 0.0;

   bool operator==(const TrackPoint&) const = default;
};

struct TracksMessage
{
   int window    = -1;
   int frame     = -1;
   int revision  = 0;
   string status = ""s;                  // "tracked" or "revised"
   vector<std::pair<int, int>> id_remap; // {old-id, new-id}
   vector<TrackPoint> tracks;

   bool operator==(const TracksMessage&) const = default;
};

// ------------------------------------------------------------- SmoothedMessage
// Smoothed points of one track. Points with t <= final_upto_t are final.
//
struct SmoothedPoint
{
   real x    = 0.0;
   real y    = 0.0;
   int t     = -1;
   real gaze = 0.0;

   bool operator==(const SmoothedPoint&) const = default;
};

struct SmoothedMessage
{
   int id           = -1;
   int final_upto_t = -1;
   vector<SmoothedPoint> points; // [x, y, t, gaze]

   bool operator==(const SmoothedMessage&) const = default;
};

// ------------------------------------------------------------- TrackEndMessage
//
struct TrackEndMessage
{
   int id         = -1;
   int last_t     = -1;
   bool confirmed = false;

   bool operator==(const TrackEndMessage&) const = default;
};

// ---------------------------------------------------------------- StatsMessage
// `RealtimeStats::to_json`
//
struct StatsMessage
{
   Json::Value stats = Json::Value{Json::objectValue};

   bool operator==(const StatsMessage& o) const { return stats == o.stats; }
};

// ------------------------------------------------------------------ EndMessage
// Last line of a stream.
//
struct EndMessage
{
   int frames    = 0;
   int dropped   = 0;
   string output = ""s; // the pipeline-output.json written at the end

   bool operator==(const EndMessage&) const = default;
};

// ---------------------------------------------------------------- TrackMessage
//
using TrackMessage = std::variant<HeaderMessage,
                                  FrameMessage,
                                  DroppedMessage,
                                  TracksMessage,
                                  SmoothedMessage,
                                  TrackEndMessage,
                                  StatsMessage,
                                  EndMessage>;

// The "type" of the message: "header", "frame", "dropped", "tracks",
// "smoothed", "track_end", "stats", or "end"
const char* message_type(const TrackMessage& msg) noexcept;

Json::Value to_json(const TrackMessage& msg) noexcept;

// Throws std::runtime_error on an unknown "type" or a missing key
TrackMessage track_message_from_json(const Json::Value& o) noexcept(false);

// One line of JSON, terminated by '\n'
string to_ndjson_line(const TrackMessage& msg) noexcept;

} // namespace perceive::pipeline::realtime
