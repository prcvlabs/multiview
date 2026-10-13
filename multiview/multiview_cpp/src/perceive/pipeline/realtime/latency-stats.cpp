
#include "latency-stats.hpp"

#include "perceive/utils/math.hpp"

#include <unistd.h>

namespace perceive::pipeline::realtime
{
// ------------------------------------------------------------------ str(Stage)
//
const char* str(RealtimeStats::Stage stage) noexcept
{
   switch(stage) {
   case RealtimeStats::Stage::SOURCE: return "source";
   case RealtimeStats::Stage::DAG: return "dag";
   case RealtimeStats::Stage::PROVISIONAL: return "provisional";
   case RealtimeStats::Stage::TRACKED: return "tracked";
   }
   return "";
}

// ------------------------------------------------------------------ str(Queue)
//
const char* str(RealtimeStats::Queue queue) noexcept
{
   switch(queue) {
   case RealtimeStats::Queue::RING: return "ring";
   case RealtimeStats::Queue::LOC: return "loc";
   case RealtimeStats::Queue::TRACKER: return "tracker";
   case RealtimeStats::Queue::OUT: return "out";
   }
   return "";
}

// ---------------------------------------------------------------- str(Latency)
//
const char* str(RealtimeStats::Latency latency) noexcept
{
   switch(latency) {
   case RealtimeStats::Latency::PROVISIONAL: return "provisional";
   case RealtimeStats::Latency::TRACKED: return "tracked";
   case RealtimeStats::Latency::REVISED: return "revised";
   case RealtimeStats::Latency::FINAL: return "final";
   }
   return "";
}

// ----------------------------------------------------------- resident-set-size
//
size_t resident_set_size() noexcept
{
   FILE* fp = fopen("/proc/self/statm", "r");
   if(fp == nullptr) return 0;
   unsigned long size     = 0;
   unsigned long resident = 0;
   const int n            = fscanf(fp, "%lu %lu", &size, &resident);
   fclose(fp);
   if(n != 2) return 0;
   return size_t(resident) * size_t(sysconf(_SC_PAGESIZE));
}

// -----------------------------------------------------------------------------
// ---------------------------------------------------------------- LatencyStats
// -----------------------------------------------------------------------------

// ---------------------------------------------------------------- Construction
//
LatencyStats::LatencyStats() { ring_.reserve(k_capacity); }

// ----------------------------------------------------------------------- count
//
uint64_t LatencyStats::count() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return count_;
}

// ------------------------------------------------------------------- n-samples
//
size_t LatencyStats::n_samples() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return ring_.size();
}

// ------------------------------------------------------------------------- max
//
double LatencyStats::max() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return max_;
}

// ------------------------------------------------------------------------ mean
//
double LatencyStats::mean() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return (count_ == 0) ? 0.0 : sum_ / double(count_);
}

// ------------------------------------------------------------------ percentile
// Nearest rank, over the ring
//
double LatencyStats::percentile(double p) const noexcept
{
   vector<double> xs = samples();
   if(xs.empty()) return 0.0;
   return calc_percentile(begin(xs), end(xs), p);
}

// --------------------------------------------------------------------- samples
//
vector<double> LatencyStats::samples() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   vector<double> out;
   out.reserve(ring_.size());
   out.insert(end(out), cbegin(ring_) + long(next_), cend(ring_));
   out.insert(end(out), cbegin(ring_), cbegin(ring_) + long(next_));
   return out;
}

// ------------------------------------------------------------------------- add
//
void LatencyStats::add(double ms) noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   if(ring_.size() < k_capacity) {
      ring_.push_back(ms);
   } else {
      ring_[next_] = ms;
      next_        = (next_ + 1) % k_capacity;
   }
   if(count_ == 0 || ms > max_) max_ = ms;
   sum_ += ms;
   ++count_;
}

// --------------------------------------------------------------------- to-json
//
Json::Value LatencyStats::to_json() const noexcept
{
   Json::Value o{Json::objectValue};
   o["n"]   = Json::Int64(count());
   o["p50"] = percentile(50.0);
   o["p95"] = percentile(95.0);
   o["max"] = max();
   return o;
}

// -----------------------------------------------------------------------------
// ----------------------------------------------------------------- RateCounter
// -----------------------------------------------------------------------------

// ----------------------------------------------------------------------- count
//
uint64_t RateCounter::count() const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   return count_;
}

// ------------------------------------------------------------------------ rate
// Events in (now - window, now], per second
//
double RateCounter::rate(time_point now) const noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   const auto t0  = now - window_;
   const auto n   = std::count_if(cbegin(events_), cend(events_), [&](auto t) {
      return t > t0 and t <= now;
   });
   const double s = std::chrono::duration<double>(window_).count();
   return double(n) / s;
}

// ------------------------------------------------------------------ set-window
//
void RateCounter::set_window(std::chrono::nanoseconds window) noexcept
{
   Expects(window.count() > 0);
   std::lock_guard<std::mutex> lock(padlock_);
   window_ = window;
}

// ------------------------------------------------------------------------- add
//
void RateCounter::add(time_point t) noexcept
{
   std::lock_guard<std::mutex> lock(padlock_);
   events_.push_back(t);
   while(!events_.empty() and events_.front() <= t - window_)
      events_.pop_front();
   ++count_;
}

// -----------------------------------------------------------------------------
// --------------------------------------------------------------- RealtimeStats
// -----------------------------------------------------------------------------

// ---------------------------------------------------------------- Construction
//
RealtimeStats::RealtimeStats(time_point t0)
    : t0_(t0)
{
   for(auto& x : drops_) x = 0;
   for(auto& x : queue_depths_) x = 0;
}

// -------------------------------------------------------------------- n-frames
//
uint64_t RealtimeStats::n_frames(Stage stage) const noexcept
{
   return rates_[size_t(stage)].count();
}

// ------------------------------------------------------------------------- fps
//
double RealtimeStats::fps(Stage stage, time_point now) const noexcept
{
   return rates_[size_t(stage)].rate(now);
}

// ------------------------------------------------------------------- n-dropped
//
uint64_t RealtimeStats::n_dropped(DropReason reason) const noexcept
{
   return drops_[size_t(reason)];
}

// ------------------------------------------------------ n-dropped, all reasons
//
uint64_t RealtimeStats::n_dropped() const noexcept
{
   uint64_t n = 0;
   for(const auto& x : drops_) n += x;
   return n;
}

// ----------------------------------------------------------------- queue-depth
//
uint64_t RealtimeStats::queue_depth(Queue queue) const noexcept
{
   return queue_depths_[size_t(queue)];
}

// ------------------------------------------------------------------ slots-busy
//
uint64_t RealtimeStats::slots_busy() const noexcept { return slots_busy_; }

// --------------------------------------------------------------- still-samples
//
uint64_t RealtimeStats::still_samples() const noexcept
{
   return still_samples_;
}

// --------------------------------------------------------------------- latency
//
const LatencyStats& RealtimeStats::latency(Latency which) const noexcept
{
   return latencies_[size_t(which)];
}

// ----------------------------------------------------------------- count-frame
//
void RealtimeStats::count_frame(Stage stage, time_point t) noexcept
{
   rates_[size_t(stage)].add(t);
}

// ------------------------------------------------------------------ count-drop
//
void RealtimeStats::count_drop(DropReason reason) noexcept
{
   if(reason != DropReason::NONE) ++drops_[size_t(reason)];
}

// ------------------------------------------------------------- set-queue-depth
//
void RealtimeStats::set_queue_depth(Queue queue, size_t depth) noexcept
{
   queue_depths_[size_t(queue)] = depth;
}

// -------------------------------------------------------------- set-slots-busy
//
void RealtimeStats::set_slots_busy(unsigned n) noexcept { slots_busy_ = n; }

// ----------------------------------------------------------- set-still-samples
//
void RealtimeStats::set_still_samples(uint64_t n) noexcept
{
   still_samples_ = n;
}

// ----------------------------------------------------------------- add-latency
//
void RealtimeStats::add_latency(Latency which, double ms) noexcept
{
   latencies_[size_t(which)].add(ms);
}

// --------------------------------------------------------------- add-window-ms
//
void RealtimeStats::add_window_ms(double ms) noexcept { window_ms_.add(ms); }

// --------------------------------------------------------------------- to-json
//
Json::Value RealtimeStats::to_json(time_point now) const noexcept
{
   Json::Value o{Json::objectValue};
   o["uptime_s"] = std::chrono::duration<double>(now - t0_).count();

   Json::Value fps_o{Json::objectValue};
   for(int i = 0; i < k_n_stages; ++i)
      fps_o[str(Stage(i))] = fps(Stage(i), now);
   o["fps"] = fps_o;

   Json::Value frames_o{Json::objectValue};
   for(int i = 0; i < k_n_stages; ++i)
      frames_o[str(Stage(i))] = Json::Int64(n_frames(Stage(i)));
   o["frames"] = frames_o;

   Json::Value queues_o{Json::objectValue};
   for(int i = 0; i < k_n_queues; ++i)
      queues_o[str(Queue(i))] = Json::Int64(queue_depth(Queue(i)));
   o["queue_depth"] = queues_o;

   o["slots_busy"] = Json::Int64(slots_busy());

   Json::Value drops_o{Json::objectValue};
   for(int i = 1; i < k_n_drop_reasons; ++i)
      drops_o[str(DropReason(i))] = Json::Int64(n_dropped(DropReason(i)));
   o["drops"] = drops_o;

   o["still_samples"] = Json::Int64(still_samples());
   o["rss_mb"]        = double(resident_set_size()) / (1024.0 * 1024.0);

   Json::Value latency_o{Json::objectValue};
   for(int i = 0; i < k_n_latencies; ++i)
      latency_o[str(Latency(i))] = latency(Latency(i)).to_json();
   o["latency_ms"] = latency_o;
   o["window_ms"]  = window_ms_.to_json();

   return o;
}

// --------------------------------------------------------------- to-prometheus
//
string RealtimeStats::to_prometheus(time_point now) const noexcept
{
   std::stringstream ss{""};
   const string prefix = "multiview_realtime_"s;

   auto type_line = [&](const string_view name, const string_view type) {
      ss << format("# TYPE {}{} {}\n", prefix, name, type);
   };

   auto summary = [&](const string_view name,
                      const string_view labels,
                      const LatencyStats& stats) {
      const string sep = labels.empty() ? ""s : ","s;
      for(const auto q : {0.5, 0.95, 1.0})
         ss << format("{}{}{{{}{}quantile=\"{}\"}} {}\n",
                      prefix,
                      name,
                      labels,
                      sep,
                      q,
                      (q == 1.0) ? stats.max() : stats.percentile(100.0 * q));
      const string braces = labels.empty() ? ""s : format("{{{}}}", labels);
      ss << format("{}{}_count{} {}\n", prefix, name, braces, stats.count());
      ss << format("{}{}_sum{} {}\n",
                   prefix,
                   name,
                   braces,
                   stats.mean() * double(stats.count()));
   };

   type_line("uptime_seconds", "gauge");
   ss << format("{}uptime_seconds {}\n",
                prefix,
                std::chrono::duration<double>(now - t0_).count());

   type_line("frames_total", "counter");
   for(int i = 0; i < k_n_stages; ++i)
      ss << format("{}frames_total{{stage=\"{}\"}} {}\n",
                   prefix,
                   str(Stage(i)),
                   n_frames(Stage(i)));

   type_line("fps", "gauge");
   for(int i = 0; i < k_n_stages; ++i)
      ss << format("{}fps{{stage=\"{}\"}} {}\n",
                   prefix,
                   str(Stage(i)),
                   fps(Stage(i), now));

   type_line("queue_depth", "gauge");
   for(int i = 0; i < k_n_queues; ++i)
      ss << format("{}queue_depth{{queue=\"{}\"}} {}\n",
                   prefix,
                   str(Queue(i)),
                   queue_depth(Queue(i)));

   type_line("slots_busy", "gauge");
   ss << format("{}slots_busy {}\n", prefix, slots_busy());

   type_line("dropped_frames_total", "counter");
   for(int i = 1; i < k_n_drop_reasons; ++i)
      ss << format("{}dropped_frames_total{{reason=\"{}\"}} {}\n",
                   prefix,
                   str(DropReason(i)),
                   n_dropped(DropReason(i)));

   type_line("still_samples_total", "counter");
   ss << format("{}still_samples_total {}\n", prefix, still_samples());

   type_line("resident_memory_bytes", "gauge");
   ss << format("{}resident_memory_bytes {}\n", prefix, resident_set_size());

   type_line("latency_ms", "summary");
   for(int i = 0; i < k_n_latencies; ++i)
      summary("latency_ms",
              format("status=\"{}\"", str(Latency(i))),
              latency(Latency(i)));

   type_line("window_ms", "summary");
   summary("window_ms", "", window_ms_);

   return ss.str();
}

} // namespace perceive::pipeline::realtime
