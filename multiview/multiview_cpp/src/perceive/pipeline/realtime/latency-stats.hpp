
#pragma once

#include "frame-record.hpp"

#include "json/json.h"

#include <atomic>
#include <chrono>
#include <deque>

namespace perceive::pipeline::realtime
{
// ---------------------------------------------------------------- LatencyStats
// The last `k_capacity` samples (milliseconds) in a ring, and lifetime counts.
// Percentiles are by nearest rank over the ring. Thread-safe.
//
class LatencyStats
{
 public:
   static constexpr size_t k_capacity = 4096;

 private:
   mutable std::mutex padlock_;
   vector<double> ring_;
   size_t next_    = 0;   // where the next sample goes once the ring is full
   uint64_t count_ = 0;   // lifetime
   double max_     = 0.0; // lifetime
   double sum_     = 0.0; // lifetime

 public:
   LatencyStats();
   LatencyStats(const LatencyStats&)            = delete;
   LatencyStats& operator=(const LatencyStats&) = delete;

   uint64_t count() const noexcept;            // lifetime
   size_t n_samples() const noexcept;          // in the ring
   double max() const noexcept;                // lifetime
   double mean() const noexcept;               // lifetime, 0 if empty
   double percentile(double p) const noexcept; // over the ring, 0 if empty
   vector<double> samples() const noexcept;    // the ring, oldest first

   void add(double ms) noexcept;

   // {"n", "p50", "p95", "max"}
   Json::Value to_json() const noexcept;
};

// ----------------------------------------------------------------- RateCounter
// Events per second over a sliding window that ends at `now`. Thread-safe.
//
class RateCounter
{
 public:
   using time_point = std::chrono::steady_clock::time_point;

 private:
   mutable std::mutex padlock_;
   std::deque<time_point> events_;
   std::chrono::nanoseconds window_ = std::chrono::seconds(1);
   uint64_t count_                  = 0;

 public:
   RateCounter()                              = default;
   RateCounter(const RateCounter&)            = delete;
   RateCounter& operator=(const RateCounter&) = delete;

   uint64_t count() const noexcept; // lifetime
   double rate(time_point now) const noexcept;
   void set_window(std::chrono::nanoseconds window) noexcept;

   void add(time_point t) noexcept;
};

// --------------------------------------------------------------- RealtimeStats
// The counters that the `stats` message, `realtime-stats.json`, and the
// metrics endpoint report. Every stage updates its own counters; any thread
// may read them.
//
class RealtimeStats
{
 public:
   using time_point = std::chrono::steady_clock::time_point;

   enum class Stage : int { SOURCE = 0, DAG, PROVISIONAL, TRACKED };
   enum class Queue : int { RING = 0, LOC, TRACKER, OUT };
   enum class Latency : int { PROVISIONAL = 0, TRACKED, REVISED, FINAL };

   static constexpr int k_n_stages    = 4;
   static constexpr int k_n_queues    = 4;
   static constexpr int k_n_latencies = 4;

   // Overloaded: more than this share of the frames of the last window dropped
   static constexpr std::chrono::seconds k_overload_window{5};
   static constexpr double k_overload_share = 0.2;

 private:
   time_point t0_;
   array<RateCounter, k_n_stages> rates_;
   array<std::atomic<uint64_t>, k_n_drop_reasons> drops_;
   array<std::atomic<uint64_t>, k_n_queues> queue_depths_;
   std::atomic<uint64_t> slots_busy_    = 0;
   std::atomic<uint64_t> still_samples_ = 0;
   std::atomic<uint64_t> ws_clients_    = 0;
   std::atomic<uint64_t> ws_dropped_    = 0;
   array<LatencyStats, k_n_latencies> latencies_;
   LatencyStats window_ms_;
   RateCounter recent_frames_; // provisional or dropped, `k_overload_window`
   RateCounter recent_drops_;

 public:
   explicit RealtimeStats(time_point t0 = std::chrono::steady_clock::now());
   RealtimeStats(const RealtimeStats&)            = delete;
   RealtimeStats& operator=(const RealtimeStats&) = delete;

   time_point t0() const noexcept { return t0_; }
   uint64_t n_frames(Stage stage) const noexcept;
   double fps(Stage stage, time_point now) const noexcept;
   uint64_t n_dropped(DropReason reason) const noexcept;
   uint64_t n_dropped() const noexcept; // all reasons
   uint64_t queue_depth(Queue queue) const noexcept;
   uint64_t slots_busy() const noexcept;
   uint64_t still_samples() const noexcept;
   uint64_t ws_clients() const noexcept;
   uint64_t ws_dropped() const noexcept; // lines, all WebSocket clients
   const LatencyStats& latency(Latency which) const noexcept;
   const LatencyStats& window_ms() const noexcept { return window_ms_; }

   // Dropped frames over provisional and dropped ones in the last
   // `k_overload_window`; 0 if there were none
   double drop_share(time_point now) const noexcept;
   bool is_overloaded(time_point now) const noexcept;

   void count_frame(Stage stage, time_point t) noexcept;
   void count_drop(DropReason reason) noexcept; // at `steady_clock::now()`
   void count_drop(DropReason reason, time_point t) noexcept;
   void set_queue_depth(Queue queue, size_t depth) noexcept;
   void set_slots_busy(unsigned n) noexcept;
   void set_still_samples(uint64_t n) noexcept;
   void set_ws_clients(unsigned n_clients, uint64_t n_dropped) noexcept;
   void add_latency(Latency which, double ms) noexcept;
   void add_window_ms(double ms) noexcept;

   // The body of the `stats` message (without "type")
   Json::Value to_json(time_point now) const noexcept;

   // `to_json`, with "status" ("ok" or "overloaded") and "drop_share"
   Json::Value to_health_json(time_point now) const noexcept;

   // Prometheus text exposition format, version 0.0.4
   string to_prometheus(time_point now) const noexcept;
};

const char* str(RealtimeStats::Stage stage) noexcept;
const char* str(RealtimeStats::Queue queue) noexcept;
const char* str(RealtimeStats::Latency latency) noexcept;

// Resident set size of this process, from /proc/self/statm; 0 on error
size_t resident_set_size() noexcept;

} // namespace perceive::pipeline::realtime
