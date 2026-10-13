
#pragma once

#include "bounded-queue.hpp"
#include "track-messages.hpp"

#include <chrono>
#include <thread>

namespace perceive::pipeline::realtime
{
// ------------------------------------------------------------------- Publisher
// One thread that writes the stream: each message is serialised once, as an
// NDJSON line, and handed to every sink in the order it was published. Every
// `stats_period` the thread also publishes the message `make_stats` returns,
// until it has written the `end` message.
//
// `publish` blocks while `capacity` messages wait, so a sink that writes a
// file never loses a line; a sink that must not block (a WebSocket client)
// drops lines itself.
//
class Publisher
{
 public:
   using time_point = std::chrono::steady_clock::time_point;

   // Called on the publisher thread; false on error. Must not throw.
   using sink_fn  = std::function<bool(const string& line)>;
   using stats_fn = std::function<StatsMessage(time_point now)>;

   struct Params
   {
      size_t capacity     = 4096; // messages waiting
      real stats_period_s = 1.0;
   };

 private:
   Params params_;
   vector<sink_fn> sinks_;
   stats_fn make_stats_;
   BoundedQueue<TrackMessage> out_q_;
   std::thread thread_;
   uint64_t n_lines_  = 0; // publisher thread only
   uint64_t n_errors_ = 0; // publisher thread only

   void write_(const TrackMessage& msg) noexcept;
   void run_() noexcept;

 public:
   Publisher(const Params& params, vector<sink_fn> sinks, stats_fn make_stats);
   Publisher(const Publisher&) = delete;
   Publisher(Publisher&&)      = delete;
   ~Publisher(); // `close`
   Publisher& operator=(const Publisher&) = delete;
   Publisher& operator=(Publisher&&)      = delete;

   size_t queue_size() const noexcept { return out_q_.size(); }

   // Lines written and sink errors; valid after `close`
   uint64_t n_lines() const noexcept { return n_lines_; }
   uint64_t n_errors() const noexcept { return n_errors_; }

   bool start() noexcept;

   // Any thread. False once `close` has been called.
   bool publish(TrackMessage&& msg) noexcept;

   // Blocks until every message published before it has been handed to the
   // sinks, then joins the thread. Idempotent.
   void close() noexcept;
};

} // namespace perceive::pipeline::realtime
