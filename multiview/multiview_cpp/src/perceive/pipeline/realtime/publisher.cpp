
#include "publisher.hpp"

namespace perceive::pipeline::realtime
{
using std::chrono::steady_clock;

// ---------------------------------------------------------------- Construction
//
Publisher::Publisher(const Params& params,
                     vector<sink_fn> sinks,
                     stats_fn make_stats)
    : params_(params)
    , sinks_(std::move(sinks))
    , make_stats_(std::move(make_stats))
    , out_q_(params.capacity)
{
   Expects(params_.stats_period_s > 0.0);
}

Publisher::~Publisher() { close(); }

// ----------------------------------------------------------------------- start
//
bool Publisher::start() noexcept
{
   Expects(!thread_.joinable());
   try {
      thread_ = std::thread([this]() { run_(); });
      return true;
   } catch(std::exception& e) {
      LOG_ERR(format("failed to start the publisher thread: {}", e.what()));
   }
   return false;
}

// --------------------------------------------------------------------- publish
//
bool Publisher::publish(TrackMessage&& msg) noexcept
{
   const char* type = message_type(msg);
   try {
      return out_q_.push(std::move(msg));
   } catch(std::exception& e) {
      LOG_ERR(format("failed to publish a '{}' message: {}", type, e.what()));
   }
   return false;
}

// ----------------------------------------------------------------------- close
//
void Publisher::close() noexcept
{
   out_q_.close();
   if(thread_.joinable()) thread_.join();
}

// ---------------------------------------------------------------------- write_
//
void Publisher::write_(const TrackMessage& msg) noexcept
{
   const auto line = to_ndjson_line(msg);
   for(const auto& sink : sinks_) {
      if(sink(line)) continue;
      if(n_errors_++ == 0)
         LOG_ERR(format("failed to write a '{}' line", message_type(msg)));
   }
   ++n_lines_;
}

// ------------------------------------------------------------------------ run_
// Until the queue is closed and empty
//
void Publisher::run_() noexcept
{
   const auto period = std::chrono::duration_cast<steady_clock::duration>(
       std::chrono::duration<double>(params_.stats_period_s));
   auto next_stats = steady_clock::now() + period;
   bool is_ended   = false; // `end` is the last line

   while(true) {
      const auto now  = steady_clock::now();
      const auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(
          next_stats - now);
      auto msg = out_q_.pop_for(std::max(wait, std::chrono::milliseconds(1)));
      if(msg) {
         is_ended = is_ended or std::holds_alternative<EndMessage>(*msg);
         write_(*msg);
      } else if(out_q_.is_closed()) {
         break;
      }

      const auto then = steady_clock::now();
      if(then < next_stats) continue;
      if(make_stats_ and !is_ended) write_(make_stats_(then));
      next_stats += period;
      if(next_stats < then) next_stats = then + period;
   }
}

} // namespace perceive::pipeline::realtime
