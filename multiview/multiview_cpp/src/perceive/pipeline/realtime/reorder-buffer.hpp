
#pragma once

#include "frame-record.hpp"

#include <chrono>
#include <map>
#include <optional>

namespace perceive::pipeline::realtime
{
// --------------------------------------------------------------- ReorderBuffer
// Puts frame records back into frame order. Slots finish frames out of order;
// dropped frames arrive as ranges (`push_drop`), never as records. `pop_ready`
// returns the next frame once its record or drop is in. Every frame number is
// returned once: a record for a frame that was already returned is discarded
// and counted as late.
//
// With a `missing_timeout`, a frame that holds up later frames for that long
// is given up on and returned as a `LATE` drop. Without one (the
// `--drop none` mode) the buffer waits forever. Thread-safe.
//
class ReorderBuffer
{
 public:
   using time_point = std::chrono::steady_clock::time_point;

 private:
   struct DropRange
   {
      int to            = -1; // inclusive
      DropReason reason = DropReason::NONE;
   };

   mutable std::mutex padlock_;
   int next_frame_no_                        = 0;
   std::chrono::nanoseconds missing_timeout_ = std::chrono::nanoseconds(0);
   std::map<int, FrameRecord> records_;
   std::map<int, DropRange> drops_;   // keyed by the first frame of the range
   bool is_blocked_          = false; // `next_frame_no_` holds up later frames
   time_point blocked_since_ = {};
   uint64_t n_late_          = 0;
   uint64_t n_expired_       = 0;

   const DropRange* find_drop_(int frame_no) const noexcept;
   void insert_drop_(int from, int to, DropReason reason) noexcept;
   bool has_later_(int frame_no) const noexcept;
   void update_blocked_(time_point now) noexcept;
   FrameRecord pop_next_(DropReason missing_reason) noexcept;

 public:
   explicit ReorderBuffer(int first_frame_no,
                          std::chrono::nanoseconds missing_timeout
                          = std::chrono::nanoseconds(0));
   ReorderBuffer(const ReorderBuffer&)            = delete;
   ReorderBuffer(ReorderBuffer&&)                 = delete;
   ~ReorderBuffer()                               = default;
   ReorderBuffer& operator=(const ReorderBuffer&) = delete;
   ReorderBuffer& operator=(ReorderBuffer&&)      = delete;

   int next_frame_no() const noexcept;  // the frame `pop_ready` waits for
   size_t n_records() const noexcept;   // records waiting
   uint64_t n_late() const noexcept;    // records discarded as late
   uint64_t n_expired() const noexcept; // frames given up on

   // False if the record is late, and was discarded
   bool push(FrameRecord&& record, time_point now) noexcept;

   // Frames [from, to] were dropped for `reason`. A frame that is already in
   // a dropped range keeps the reason it has.
   void push_drop(int from, int to, DropReason reason, time_point now) noexcept;

   // The next frame in order, if it is in, or has expired at `now`
   std::optional<FrameRecord> pop_ready(time_point now) noexcept;

   // Everything that is in, in order, up to the last frame received; frames
   // missing before it come out as `DAG_ERROR` drops.
   vector<FrameRecord> flush() noexcept;
};

} // namespace perceive::pipeline::realtime
